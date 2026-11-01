#include "controller.h"

#include <algorithm>
#include <cmath>

#include "vehicle.h"

namespace {

constexpr float kSpeedGain = 0.6f;
constexpr float kIntegralGain = 0.15f;
constexpr float kPi = 3.14159265f;

float wrapAngle(float angle) {
    while (angle > kPi) angle -= 2 * kPi;
    while (angle < -kPi) angle += 2 * kPi;
    return angle;
}

}  // namespace

Controller::Controller(MessageBus& bus) : bus_(bus) {
    bus_.subscribe<VehicleState>(topic::kVehicle, [this](const VehicleState& state) { vehicle_ = state; });
    bus_.subscribe<ManeuverPath>(topic::kPath, [this](const ManeuverPath& path) { path_ = path; });
    bus_.subscribe<PlanTarget>(topic::kPlan, [this](const PlanTarget& plan) { plan_ = plan; });
    bus_.subscribe<ManeuverStatus>(topic::kManeuverStatus, [this](const ManeuverStatus& status) { status_ = status; });
    bus_.subscribe<ManualInput>(topic::kManual, [this](const ManualInput& manual) { manual_ = manual; });
}

void Controller::tick(float dt) {
    ControlCommand command;
    command.state = plan_.state;
    command.maneuver = status_.type;

    if (manual_.enabled) {
        command.manual = true;
        command.steer = manual_.steer * Vehicle::kMaxSteer;
        command.throttle = manual_.throttle;
        command.brake = manual_.brake;
        command.reverse = manual_.reverse;
        // Safety stops still win over the driver.
        if (plan_.forceBrake) {
            command.throttle = 0;
            command.brake = 1;
        }
        bus_.publish(topic::kControl, command);
        return;
    }

    // Steer toward a point on the path a little ahead of the nearest one.
    if (path_.points.size() >= 2) {
        size_t nearest = 0;
        float best = 1e9f;
        for (size_t i = 0; i < path_.points.size(); ++i) {
            const float distance = length(path_.points[i].position - vehicle_.position);
            if (distance < best) {
                best = distance;
                nearest = i;
            }
        }
        const float lookahead = std::max(3.0f, 0.6f * std::abs(vehicle_.speed));
        size_t target = nearest;
        float travelled = 0;
        while (target + 1 < path_.points.size() && travelled < lookahead) {
            travelled += length(path_.points[target + 1].position - path_.points[target].position);
            ++target;
        }
        const Vec2 toTarget = path_.points[target].position - vehicle_.position;
        const float distance = std::max(length(toTarget), 0.5f);
        // Going backward, the car acts like one driving forward with its steering mirrored.
        const float facing = path_.reverse ? vehicle_.heading + kPi : vehicle_.heading;
        const float alpha = wrapAngle(std::atan2(toTarget.y, toTarget.x) - facing);
        const float steer = std::atan2(2 * Vehicle::kWheelbase * std::sin(alpha), distance);
        command.steer = std::clamp(path_.reverse ? -steer : steer, -Vehicle::kMaxSteer, Vehicle::kMaxSteer);
    }

    const float wanted = std::min(plan_.speed, path_.speedCap);
    const float signedWanted = path_.reverse ? -wanted : wanted;
    const bool wrongDirection = (path_.reverse && vehicle_.speed > 0.05f) || (!path_.reverse && vehicle_.speed < -0.05f);

    if (plan_.forceBrake) {
        command.brake = 1;
        speedIntegral_ = 0;
    } else if (wrongDirection) {
        command.brake = 0.7f;
        command.reverse = path_.reverse;
        speedIntegral_ = 0;
    } else {
        command.reverse = path_.reverse;
        const float error = std::abs(signedWanted) - std::abs(vehicle_.speed);
        speedIntegral_ = std::clamp(speedIntegral_ + error * dt, -3.0f, 3.0f);
        const float output = kSpeedGain * error + kIntegralGain * speedIntegral_;
        if (wanted < 0.05f && std::abs(vehicle_.speed) < 0.3f) {
            command.brake = 1;
            speedIntegral_ = 0;
        } else if (output >= 0) {
            command.throttle = std::min(output, 1.0f);
        } else {
            command.brake = std::min(-output, 1.0f);
        }
    }
    bus_.publish(topic::kControl, command);
}
