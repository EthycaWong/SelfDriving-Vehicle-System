#include "planner.h"

#include <algorithm>
#include <cmath>

#include "road_network.h"
#include "vehicle.h"

namespace {

constexpr float kPathMargin = 0.5f;
constexpr double kManeuverCooldown = 1.5;
constexpr double kWaitBeforeUTurn = 6.0;
constexpr double kStartupGrace = 6.0;  // Models take a few seconds to load when the demo starts.
constexpr float kSteerRoomDistance = 5.0f;

bool mustStopFor(ObjectClass cls) { return cls == ObjectClass::Person || cls == ObjectClass::Animal; }

bool blocksTheRoad(ObjectClass cls) {
    return cls == ObjectClass::Person || cls == ObjectClass::Animal || cls == ObjectClass::Vehicle ||
           cls == ObjectClass::Obstruction;
}

}  // namespace

Planner::Planner(MessageBus& bus, Config config) : bus_(bus), config_(config) {
    bus_.subscribe<ObstacleList>(topic::kObstacles, [this](const ObstacleList& list) {
        obstacles_ = list;
        if (list.visionAlive) lastDetections_ = list.time;
    });
    bus_.subscribe<SensorReadings>(topic::kSensors, [this](const SensorReadings& readings) {
        sensors_ = readings;
        lastSensors_ = readings.time;
        hasSensors_ = true;
    });
    bus_.subscribe<VehicleState>(topic::kVehicle, [this](const VehicleState& state) { vehicle_ = state; });
    bus_.subscribe<ManeuverStatus>(topic::kManeuverStatus, [this](const ManeuverStatus& status) { status_ = status; });
    bus_.subscribe<ManeuverPath>(topic::kPath, [this](const ManeuverPath& path) { path_ = path; });
    bus_.subscribe<ManualInput>(topic::kManual, [this](const ManualInput& manual) { manual_ = manual; });
}

std::optional<Planner::Blocker> Planner::firstOnPath() const {
    if (path_.points.size() < 2) return std::nullopt;
    std::optional<Blocker> best;
    for (const Obstacle& obstacle : obstacles_.obstacles) {
        if (!blocksTheRoad(obstacle.cls)) continue;
        const float reach = obstacle.width / 2 + Vehicle::kWidth / 2 + kPathMargin;
        for (size_t i = 0; i + 1 < path_.points.size(); ++i) {
            const Vec2 a = path_.points[i].position;
            const Vec2 b = path_.points[i + 1].position;
            const Vec2 segment = b - a;
            const float segmentLength = length(segment);
            const float t = std::clamp(dot(obstacle.position - a, segment) / std::max(segmentLength * segmentLength, 1e-3f), 0.0f, 1.0f);
            const Vec2 closest = a + segment * t;
            const float gap = length(obstacle.position - closest);
            if (gap < reach) {
                // Measured from the car, not from wherever the path happens to start.
                const float distance = std::max(length(closest - vehicle_.position) - Vehicle::kLength / 2, 0.0f);
                const Vec2 heading = path_.reverse ? fromHeading(vehicle_.heading) * -1.0f : fromHeading(vehicle_.heading);
                if (dot(obstacle.position - vehicle_.position, heading) > 0 && (!best || distance < best->distance)) {
                    best = Blocker{obstacle, distance, gap};
                }
                break;
            }
        }
    }
    return best;
}

bool Planner::laneClear(float lateral, float behind, float ahead) const {
    for (const Obstacle& obstacle : obstacles_.obstacles) {
        if (!blocksTheRoad(obstacle.cls)) continue;
        const Vec2 relative = obstacle.position - status_.road.origin;
        const float along = dot(relative, status_.road.direction) - status_.along;
        const float side = dot(relative, rightOf(status_.road.direction));
        if (along > -behind && along < ahead && std::abs(side - lateral) < RoadNetwork::kLaneWidth / 2 + obstacle.width / 2) {
            return false;
        }
    }
    return true;
}

bool Planner::personInCorridor(float lateral, float ahead) const {
    for (const Obstacle& obstacle : obstacles_.obstacles) {
        if (!mustStopFor(obstacle.cls)) continue;
        const Vec2 relative = obstacle.position - status_.road.origin;
        const float along = dot(relative, status_.road.direction) - status_.along;
        const float side = dot(relative, rightOf(status_.road.direction));
        if (along > 0 && along < ahead && std::abs(side - lateral) < RoadNetwork::kLaneWidth / 2 + obstacle.width / 2 + 0.5f) {
            return true;
        }
    }
    return false;
}

int Planner::sideTowardOtherLane() const { return status_.lane == 0 ? 1 : -1; }

bool Planner::sideSensorClear(int side) const {
    return (side < 0 ? sensors_.leftCm : sensors_.rightCm) >= config_.sensorClearCm;
}

bool Planner::canMoveToOtherLane() const {
    const float target = RoadNetwork::laneOffset(status_.lane == 0 ? 1 : 0);
    return laneClear(target, 8.0f, 40.0f) && !personInCorridor(target, 40.0f) && sideSensorClear(sideTowardOtherLane());
}

bool Planner::sendManeuver(ManeuverType type, const Blocker* blocker) {
    if (status_.type != ManeuverType::LaneKeep) return false;
    // A person or animal in the lane means stop, unless it's the emergency swerve.
    ManeuverRequest request;
    request.type = type;
    if (blocker != nullptr) {
        request.hasTarget = true;
        request.target = blocker->obstacle.position;
        request.targetWidth = blocker->obstacle.width;
    }
    lastManeuverTime_ = bus_.time();
    bus_.publish(topic::kManeuverRequest, request, std::string("Planner chose ") + maneuverName(type));
    return true;
}

void Planner::requestResume() {
    ManeuverRequest request;
    request.resume = true;
    bus_.publish(topic::kManeuverRequest, request, "Rejoining lane");
}

void Planner::requestManeuver(ManeuverType type) {
    const std::optional<Blocker> blocker = firstOnPath();
    switch (type) {
        case ManeuverType::LaneChange:
        case ManeuverType::Overtake:
        case ManeuverType::Swerve:
            if (!canMoveToOtherLane()) {
                bus_.publish(topic::kManeuverRequest, ManeuverRequest{ManeuverType::LaneKeep}, "Refused: other lane isn't clear");
                return;
            }
            break;
        case ManeuverType::UTurn:
            if (!laneClear(-RoadNetwork::laneOffset(1), 6.0f, 25.0f) || !sideSensorClear(-1)) {
                bus_.publish(topic::kManeuverRequest, ManeuverRequest{ManeuverType::LaneKeep}, "Refused: can't turn around here");
                return;
            }
            break;
        case ManeuverType::Reverse:
            if (sensors_.rearCm < config_.sensorClearCm) {
                bus_.publish(topic::kManeuverRequest, ManeuverRequest{ManeuverType::LaneKeep}, "Refused: rear sensor is blocked");
                return;
            }
            break;
        case ManeuverType::PullOver:
            if (!sideSensorClear(1)) {
                bus_.publish(topic::kManeuverRequest, ManeuverRequest{ManeuverType::LaneKeep}, "Refused: right sensor is blocked");
                return;
            }
            break;
        default:
            break;
    }
    sendManeuver(type, blocker ? &*blocker : nullptr);
}

float Planner::stoppingSpeed(float distance) const {
    return std::sqrt(std::max(2 * config_.comfortBrake * (distance - config_.stopMargin), 0.0f));
}

void Planner::decide(double now) {
    PlanTarget plan;
    plan.speed = config_.cruiseSpeed;
    plan.state = DriveState::Cruise;
    plan.reason = "road clear";

    if (startTime_ < 0) startTime_ = now;
    const bool grace = now - startTime_ < kStartupGrace;
    visionHealthy_ = now - lastDetections_ < config_.visionTimeout;
    boardHealthy_ = hasSensors_ && now - lastSensors_ < config_.boardTimeout;

    // Moving is only allowed once the systems that keep it safe have reported in.
    if (!grace) {
        const bool visionLost = config_.requireVision && !visionHealthy_;
        const bool boardLost = config_.requireBoard && !boardHealthy_;
        if (visionLost || boardLost) {
            plan.state = DriveState::Failsafe;
            plan.reason = visionLost && boardLost ? "vision and board silent" : visionLost ? "vision silent" : "board silent";
            plan.speed = 3.0f;
            if (status_.type == ManeuverType::LaneKeep) sendManeuver(ManeuverType::PullOver, nullptr);
            if (status_.type == ManeuverType::PullOver && status_.phase == "stopped on shoulder") plan.speed = 0;
            pulledOver_ = true;
        } else if (pulledOver_) {
            pulledOver_ = false;
            ManeuverRequest request;
            request.resume = true;
            bus_.publish(topic::kManeuverRequest, request, "Systems back: rejoining lane");
        }
    } else if ((config_.requireVision && !visionHealthy_) || (config_.requireBoard && !boardHealthy_)) {
        plan.speed = 0;
        plan.state = DriveState::Stop;
        plan.reason = "waiting for vision and board";
    }

    // The front sensor is the emergency stop. It releases at a larger distance so it can't flicker.
    if (hasSensors_) {
        if (sensors_.frontCm < config_.estopCm) estopLatched_ = true;
        if (sensors_.frontCm >= config_.estopReleaseCm) estopLatched_ = false;
    }
    if (estopLatched_) {
        plan.state = DriveState::EStop;
        plan.speed = 0;
        plan.forceBrake = true;
        plan.reason = "front sensor " + std::to_string(sensors_.frontCm) + " cm";
    }

    if (plan.state == DriveState::Cruise && status_.type == ManeuverType::Swerve && status_.phase == "swerving") {
        // Committed: braking now would leave the car in front of what it's steering around.
        plan.speed = std::abs(vehicle_.speed);
        plan.state = DriveState::Slow;
        plan.reason = "emergency swerve";
    } else if (plan.state == DriveState::Cruise || plan.state == DriveState::Failsafe) {
        const std::optional<Blocker> blocker = firstOnPath();
        const bool canStartManeuver = plan.state == DriveState::Cruise && !manual_.enabled &&
                                      now - lastManeuverTime_ > kManeuverCooldown;
        const bool moving = status_.type != ManeuverType::LaneKeep && status_.type != ManeuverType::PullOver;

        if (blocker) {
            if (blockedSince_ < 0) blockedSince_ = now;
            const Obstacle& obstacle = blocker->obstacle;
            const float speed = std::abs(vehicle_.speed);
            const float hardStop = speed * speed / (2 * config_.hardBrake) + 1.0f;
            plan.speed = std::min(plan.speed, stoppingSpeed(blocker->distance));
            const std::string name = className(obstacle.cls);
            const std::string where = " in path at " + std::to_string(static_cast<int>(blocker->distance)) + " m";

            // Something appeared on the path of a maneuver already running.
            if (moving && status_.type != ManeuverType::Turn && status_.type != ManeuverType::UTurn &&
                status_.type != ManeuverType::Reverse && status_.type != ManeuverType::Swerve && mustStopFor(obstacle.cls)) {
                ManeuverRequest request;
                request.abort = true;
                bus_.publish(topic::kManeuverRequest, request, "Aborting: " + name + where);
            }

            if (mustStopFor(obstacle.cls)) {
                plan.state = DriveState::Stop;
                plan.reason = name + where;
                if (blocker->distance < hardStop) {
                    if (canStartManeuver && status_.type == ManeuverType::LaneKeep && speed > 4.0f && canMoveToOtherLane()) {
                        sendManeuver(ManeuverType::Swerve, &*blocker);
                        plan.reason = "swerving past " + name;
                    } else {
                        plan.forceBrake = speed > 0.1f;
                    }
                }
            } else {
                plan.state = plan.speed < 0.5f ? DriveState::Stop : DriveState::Slow;
                plan.reason = name + where;
                if (blocker->distance < hardStop) plan.forceBrake = speed > 0.1f;

                if (canStartManeuver && status_.type == ManeuverType::LaneKeep && blocker->distance < 30.0f) {
                    const bool slowTraffic = obstacle.speed < 1.5f;
                    if (obstacle.cls == ObjectClass::Obstruction) {
                        if (blocker->distance < kSteerRoomDistance && speed < 0.5f && sensors_.rearCm >= config_.sensorClearCm) {
                            sendManeuver(ManeuverType::Reverse, &*blocker);
                        } else if (blocker->distance >= kSteerRoomDistance && canMoveToOtherLane()) {
                            sendManeuver(ManeuverType::GoAround, &*blocker);
                        }
                    } else if (slowTraffic) {
                        if (blocker->distance < kSteerRoomDistance && speed < 0.5f && sensors_.rearCm >= config_.sensorClearCm) {
                            sendManeuver(ManeuverType::Reverse, &*blocker);
                        } else if (blocker->distance >= kSteerRoomDistance && canMoveToOtherLane()) {
                            sendManeuver(ManeuverType::Overtake, &*blocker);
                        }
                    }
                    // Stuck with no way past: turn around instead of waiting forever.
                    if (speed < 0.3f && now - blockedSince_ > kWaitBeforeUTurn &&
                        laneClear(-RoadNetwork::laneOffset(1), 6.0f, 25.0f) && sideSensorClear(-1)) {
                        sendManeuver(ManeuverType::UTurn, nullptr);
                    }
                }
            }
        } else {
            blockedSince_ = -1;
            if (canStartManeuver && status_.type == ManeuverType::LaneKeep && status_.wantLane >= 0 &&
                status_.wantLane != status_.lane) {
                const float toJunction = status_.road.length - status_.along;
                if (toJunction < 70.0f && toJunction > 25.0f && canMoveToOtherLane()) {
                    sendManeuver(ManeuverType::LaneChange, nullptr);
                }
            }
        }
    }

    // Moves the car can't reverse out of still need the rear sensor to stay clear.
    if (path_.reverse && sensors_.rearCm < config_.sensorClearCm) {
        plan.forceBrake = true;
        plan.speed = 0;
        plan.reason = "rear sensor " + std::to_string(sensors_.rearCm) + " cm";
        ManeuverRequest request;
        request.abort = true;
        if (status_.type == ManeuverType::Reverse) bus_.publish(topic::kManeuverRequest, request, "Aborting reverse: rear sensor");
    }

    const bool changed = plan.state != state_ || plan.reason != reason_;
    state_ = plan.state;
    reason_ = plan.reason;
    bus_.publish(topic::kPlan, plan, changed ? std::string(driveStateName(plan.state)) + ": " + plan.reason : std::string());
}

void Planner::tick(double now) { decide(now); }
