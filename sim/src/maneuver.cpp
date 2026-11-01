#include "maneuver.h"

#include <algorithm>
#include <cmath>

#include "vehicle.h"

namespace {

constexpr float kPi = 3.14159265f;
constexpr float kPathLength = 45.0f;
constexpr float kPathStep = 1.5f;
constexpr float kTurnSpeed = 4.0f;
constexpr float kUTurnSpeed = 2.5f;
constexpr float kReverseDistance = 8.0f;
constexpr float kReverseSpeed = 1.5f;
constexpr float kPassClearance = 9.0f;
constexpr float kArrived = 0.3f;
constexpr float kMinUTurnRadius = 5.0f;
constexpr float kThreePointRadius = 4.5f;

float smooth(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * (3 - 2 * t);
}

// Points along a circular arc. Positive turnSign curves right. backward drives the arc in reverse.
std::vector<PathPoint> arc(Vec2& position, float& heading, float radius, float angle, int turnSign, bool backward) {
    std::vector<PathPoint> points;
    const int steps = std::max(6, static_cast<int>(radius * angle / 0.4f));
    const float stepAngle = angle / static_cast<float>(steps);
    const float stepLength = radius * stepAngle;
    points.push_back({position});
    for (int i = 0; i < steps; ++i) {
        // Reversing with a given steering angle swings the nose the opposite way.
        heading += (backward ? -1.0f : 1.0f) * static_cast<float>(turnSign) * stepAngle;
        position = position + fromHeading(heading) * (backward ? -stepLength : stepLength);
        points.push_back({position});
    }
    return points;
}

}  // namespace

ManeuverModule::ManeuverModule(MessageBus& bus, const RoadNetwork& roads, unsigned seed)
    : bus_(bus), roads_(roads), rng_(seed) {
    bus_.subscribe<VehicleState>(topic::kVehicle, [this](const VehicleState& state) { vehicle_ = state; });
    bus_.subscribe<ManeuverRequest>(topic::kManeuverRequest, [this](const ManeuverRequest& r) { handleRequest(r); });
}

RoadFrame ManeuverModule::frameOf(const Edge& edge) const {
    const Vector2 a = roads_.node(edge.fromColumn, edge.fromRow);
    const Vector2 b = roads_.node(edge.toColumn, edge.toRow);
    const Vec2 delta{b.x - a.x, b.y - a.y};
    const float distance = length(delta);
    return {{a.x, a.y}, delta * (1 / distance), distance};
}

bool ManeuverModule::validEdge(const Edge& edge) const {
    return edge.toColumn >= 0 && edge.toRow >= 0 && edge.toColumn < roads_.columns() && edge.toRow < roads_.rows();
}

ManeuverModule::Edge ManeuverModule::edgeAfter(const Edge& edge, NextMove move) const {
    const int dx = edge.toColumn - edge.fromColumn;
    const int dy = edge.toRow - edge.fromRow;
    // Facing +X, right is +Z, which is +row in the grid.
    int nx = dx, ny = dy;
    if (move == NextMove::Right) {
        nx = -dy;
        ny = dx;
    } else if (move == NextMove::Left) {
        nx = dy;
        ny = -dx;
    }
    return {edge.toColumn, edge.toRow, edge.toColumn + nx, edge.toRow + ny};
}

void ManeuverModule::chooseNextMove() {
    std::vector<NextMove> options;
    for (NextMove move : {NextMove::Straight, NextMove::Straight, NextMove::Left, NextMove::Right}) {
        if (validEdge(edgeAfter(edge_, move))) options.push_back(move);
    }
    if (!randomRoute_ && options.front() == NextMove::Straight) {
        next_ = NextMove::Straight;
        return;
    }
    next_ = options[std::uniform_int_distribution<size_t>(0, options.size() - 1)(rng_)];
}

void ManeuverModule::start(int fromColumn, int fromRow, int toColumn, int toRow, int lane) {
    edge_ = {fromColumn, fromRow, toColumn, toRow};
    frame_ = frameOf(edge_);
    lane_ = lane;
    homeOffset_ = offsetStart_ = offsetEnd_ = RoadNetwork::laneOffset(lane);
    status_ = {};
    status_.type = ManeuverType::LaneKeep;
    status_.lane = lane;
    chooseNextMove();
}

void ManeuverModule::relocate(Vec2 position, float heading) {
    const Vec2 facing = fromHeading(heading);
    float best = 1e9f;
    Edge bestEdge = edge_;
    for (int column = 0; column < roads_.columns(); ++column) {
        for (int row = 0; row < roads_.rows(); ++row) {
            const int steps[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
            for (const auto& step : steps) {
                const Edge edge{column, row, column + step[0], row + step[1]};
                if (!validEdge(edge)) continue;
                const RoadFrame road = frameOf(edge);
                if (dot(road.direction, facing) < 0.3f) continue;
                const Vec2 relative = position - road.origin;
                const float along = dot(relative, road.direction);
                const float beyond = along < 0 ? -along : along > road.length ? along - road.length : 0;
                const float score = std::abs(dot(relative, rightOf(road.direction)) - RoadNetwork::laneOffset(1)) + beyond;
                if (score < best) {
                    best = score;
                    bestEdge = edge;
                }
            }
        }
    }
    edge_ = bestEdge;
    frame_ = frameOf(edge_);
    const Vec2 relative = position - frame_.origin;
    status_.along = dot(relative, frame_.direction);
    status_.lateral = dot(relative, rightOf(frame_.direction));
    status_.type = ManeuverType::LaneKeep;
    status_.phase.clear();
    status_.signal = 0;
    segments_.clear();
    lane_ = status_.lateral < RoadNetwork::kLaneWidth ? 0 : 1;
    status_.lane = lane_;
    homeOffset_ = RoadNetwork::laneOffset(lane_);
    blendTo(homeOffset_, std::max(std::abs(status_.lateral - homeOffset_) * 5.0f, 8.0f));
    chooseNextMove();
}

float ManeuverModule::offsetAt(float along) const {
    if (blendEnd_ <= blendStart_) return along >= blendEnd_ ? offsetEnd_ : offsetStart_;
    return offsetStart_ + (offsetEnd_ - offsetStart_) * smooth((along - blendStart_) / (blendEnd_ - blendStart_));
}

void ManeuverModule::blendTo(float lateral, float distance) {
    offsetStart_ = status_.lateral;
    offsetEnd_ = lateral;
    blendStart_ = status_.along;
    blendEnd_ = status_.along + distance;
}

void ManeuverModule::finish() {
    status_.type = ManeuverType::LaneKeep;
    status_.phase.clear();
    status_.signal = 0;
    phase_ = 0;
    ++status_.completed;
    bus_.publish(topic::kManeuverStatus, status_, "finished maneuver");
}

void ManeuverModule::abortManeuver() {
    if (status_.type == ManeuverType::LaneKeep || status_.type == ManeuverType::Turn) return;
    const bool fixedPath = status_.type == ManeuverType::UTurn || status_.type == ManeuverType::Reverse;
    if (!fixedPath) blendTo(homeOffset_, std::max(std::abs(vehicle_.speed) * 2.5f, 10.0f));
    lane_ = std::abs(homeOffset_ - RoadNetwork::laneOffset(0)) < std::abs(homeOffset_ - RoadNetwork::laneOffset(1)) ? 0 : 1;
    status_.lane = lane_;
    status_.type = ManeuverType::LaneKeep;
    status_.signal = 0;
    status_.phase.clear();
    ++status_.aborted;
    phase_ = 0;
    bus_.publish(topic::kManeuverStatus, status_, "aborted maneuver");
}

void ManeuverModule::handleRequest(const ManeuverRequest& request) {
    if (request.abort) {
        abortManeuver();
        return;
    }
    if (request.resume) {
        if (status_.type == ManeuverType::PullOver) {
            status_.type = ManeuverType::LaneKeep;
            status_.phase.clear();
            blendTo(RoadNetwork::laneOffset(1), 20.0f);
            lane_ = 1;
            status_.lane = 1;
        }
        return;
    }
    if (status_.type != ManeuverType::LaneKeep) return;

    const float speedDistance = std::max(std::abs(vehicle_.speed) * 3.0f, 14.0f);
    const int otherLane = lane_ == 0 ? 1 : 0;
    homeOffset_ = RoadNetwork::laneOffset(lane_);
    phase_ = 0;

    switch (request.type) {
        case ManeuverType::LaneChange:
            status_.type = ManeuverType::LaneChange;
            status_.phase = "moving over";
            status_.signal = otherLane > lane_ ? 1 : -1;
            lane_ = otherLane;
            status_.lane = lane_;
            blendTo(RoadNetwork::laneOffset(lane_), speedDistance);
            break;
        case ManeuverType::Overtake:
        case ManeuverType::Swerve: {
            status_.type = request.type;
            status_.phase = request.type == ManeuverType::Swerve ? "swerving" : "pulling out";
            status_.signal = otherLane > lane_ ? 1 : -1;
            passAlong_ = status_.along + 25;
            if (request.hasTarget) {
                passAlong_ = dot(request.target - frame_.origin, frame_.direction) + kPassClearance;
            }
            passOffset_ = RoadNetwork::laneOffset(otherLane);
            blendTo(passOffset_, request.type == ManeuverType::Swerve ? std::max(std::abs(vehicle_.speed), 7.0f)
                                                                       : speedDistance);
            break;
        }
        case ManeuverType::GoAround: {
            status_.type = ManeuverType::GoAround;
            status_.phase = "steering around";
            const float clearance = request.targetWidth / 2 + Vehicle::kWidth / 2 + 0.9f;
            float centre = status_.lateral + 3.0f;
            passAlong_ = status_.along + 25;
            if (request.hasTarget) {
                centre = dot(request.target - frame_.origin, rightOf(frame_.direction));
                passAlong_ = dot(request.target - frame_.origin, frame_.direction) + kPassClearance;
            }
            // Pass on the requested side, or else the side that keeps the car on its own half.
            int side = request.side;
            if (side == 0) side = centre - clearance > 0.9f ? -1 : 1;
            passOffset_ = std::clamp(centre + static_cast<float>(side) * clearance, 0.95f,
                                     2 * RoadNetwork::kLaneWidth + RoadNetwork::kShoulderWidth - 1.0f);
            status_.signal = side;
            blendTo(passOffset_, speedDistance);
            break;
        }
        case ManeuverType::PullOver:
            status_.type = ManeuverType::PullOver;
            status_.phase = "moving to shoulder";
            status_.signal = 1;
            blendTo(RoadNetwork::shoulderOffset(), std::max(speedDistance, 20.0f));
            break;
        case ManeuverType::UTurn:
            beginUTurn();
            break;
        case ManeuverType::Reverse:
            beginReverse();
            break;
        case ManeuverType::Turn:
            // Prefer a real turn at the next junction if the grid allows one.
            for (NextMove move : {NextMove::Right, NextMove::Left}) {
                if (validEdge(edgeAfter(edge_, move))) {
                    next_ = move;
                    break;
                }
            }
            break;
        case ManeuverType::LaneKeep:
            break;
    }
    if (status_.type != ManeuverType::LaneKeep) {
        bus_.publish(topic::kManeuverStatus, status_, std::string("started ") + maneuverName(status_.type));
    }
}

void ManeuverModule::beginTurn() {
    const Edge after = edgeAfter(edge_, next_);
    const RoadFrame out = frameOf(after);
    const float entryAlong = frame_.length - RoadNetwork::kHalfRoad - 1.0f;
    const float inLateral = status_.lateral;
    const float outLateral = RoadNetwork::laneOffset(lane_);
    const Vec2 entry = frame_.origin + frame_.direction * entryAlong + rightOf(frame_.direction) * inLateral;
    const Vec2 exit = out.origin + out.direction * (RoadNetwork::kHalfRoad + 1.0f) + rightOf(out.direction) * outLateral;
    // The two lane lines cross at the curve's control point.
    const Vec2 corner = frame_.origin + frame_.direction * (frame_.length + dot(rightOf(out.direction) * outLateral, frame_.direction)) +
                        rightOf(frame_.direction) * inLateral;

    std::vector<PathPoint> curve;
    curve.push_back({status_.along < entryAlong ? frame_.origin + frame_.direction * status_.along + rightOf(frame_.direction) * inLateral : entry});
    constexpr int kSteps = 24;
    for (int i = 0; i <= kSteps; ++i) {
        const float t = static_cast<float>(i) / kSteps;
        const float u = 1 - t;
        curve.push_back({entry * (u * u) + corner * (2 * u * t) + exit * (t * t)});
    }
    // Some straight road after the exit, so steering has something to aim at.
    for (float extra = 2.0f; extra <= 8.0f; extra += 2.0f) {
        curve.push_back({exit + out.direction * extra});
    }

    segments_ = {curve};
    segmentReverse_ = {false};
    segment_ = 0;
    afterFixed_ = after;
    laneAfterFixed_ = lane_;
    status_.type = ManeuverType::Turn;
    status_.phase = next_ == NextMove::Left ? "turning left" : "turning right";
    status_.signal = next_ == NextMove::Left ? -1 : 1;
    bus_.publish(topic::kManeuverStatus, status_, std::string("started TURN: ") + status_.phase);
}

void ManeuverModule::beginUTurn() {
    Vec2 position = vehicle_.position;
    float heading = std::atan2(frame_.direction.y, frame_.direction.x);
    const float targetLateral = -RoadNetwork::laneOffset(1);
    const float span = status_.lateral - targetLateral;

    segments_.clear();
    segmentReverse_.clear();
    segment_ = 0;
    status_.type = ManeuverType::UTurn;
    status_.signal = -1;
    if (span >= 2 * kMinUTurnRadius) {
        status_.phase = "turning around";
        segments_.push_back(arc(position, heading, span / 2, kPi, -1, false));
        segmentReverse_.push_back(false);
        // Straighten out along the new direction.
        std::vector<PathPoint>& last = segments_.back();
        for (float extra = 2.0f; extra <= 10.0f; extra += 2.0f) {
            last.push_back({position + fromHeading(heading) * extra});
        }
    } else {
        // Not enough width for one sweep: forward, back, forward again.
        status_.phase = "three-point turn";
        const float first = 1.2f;
        segments_.push_back(arc(position, heading, kThreePointRadius, first, -1, false));
        segmentReverse_.push_back(false);
        segments_.push_back(arc(position, heading, kThreePointRadius, first, +1, true));
        segmentReverse_.push_back(true);
        std::vector<PathPoint> last = arc(position, heading, kThreePointRadius, kPi - 2 * first, -1, false);
        for (float extra = 2.0f; extra <= 10.0f; extra += 2.0f) {
            last.push_back({position + fromHeading(heading) * extra});
        }
        segments_.push_back(std::move(last));
        segmentReverse_.push_back(false);
    }
    afterFixed_ = {edge_.toColumn, edge_.toRow, edge_.fromColumn, edge_.fromRow};
    laneAfterFixed_ = 1;
}

void ManeuverModule::beginReverse() {
    const Vec2 backward = fromHeading(vehicle_.heading) * -1.0f;
    std::vector<PathPoint> line;
    for (float d = 0; d <= kReverseDistance + 6.0f; d += 1.0f) {
        line.push_back({vehicle_.position + backward * d});
    }
    segments_ = {line};
    segmentReverse_ = {true};
    segment_ = 0;
    reverseFrom_ = status_.along;
    status_.type = ManeuverType::Reverse;
    status_.phase = "backing up";
    status_.signal = 0;
    afterFixed_ = edge_;
    laneAfterFixed_ = lane_;
}

void ManeuverModule::updateLaneFollowing() {
    switch (status_.type) {
        case ManeuverType::LaneChange:
            if (std::abs(status_.lateral - offsetEnd_) < kArrived) {
                homeOffset_ = offsetEnd_;
                finish();
            }
            break;
        case ManeuverType::Overtake:
        case ManeuverType::Swerve:
        case ManeuverType::GoAround:
            if (phase_ == 0 && status_.along >= passAlong_ && std::abs(status_.lateral - offsetEnd_) < 0.6f) {
                phase_ = 1;
                status_.phase = "returning to lane";
                status_.signal = homeOffset_ > status_.lateral ? 1 : -1;
                blendTo(homeOffset_, std::max(std::abs(vehicle_.speed) * 3.0f, 14.0f));
            } else if (phase_ == 0 && status_.along >= blendEnd_) {
                status_.phase = "passing";
            } else if (phase_ == 1 && status_.along >= blendEnd_) {
                finish();
            }
            break;
        case ManeuverType::PullOver:
            if (std::abs(status_.lateral - RoadNetwork::shoulderOffset()) < 0.4f) status_.phase = "stopped on shoulder";
            break;
        default:
            break;
    }

    // Move to the lane the next turn needs, if there's time.
    status_.wantLane = next_ == NextMove::Left ? 0 : next_ == NextMove::Right ? 1 : -1;

    const float junction = frame_.length - RoadNetwork::kHalfRoad;
    if (status_.along > frame_.length) {
        // Rolled through the junction: the next road's frame takes over.
        const float passed = frame_.length;
        edge_ = edgeAfter(edge_, NextMove::Straight);
        frame_ = frameOf(edge_);
        status_.along -= passed;
        blendStart_ -= passed;
        blendEnd_ -= passed;
        passAlong_ -= passed;
        chooseNextMove();
    } else if (next_ != NextMove::Straight && status_.along >= junction - 1.0f && status_.type == ManeuverType::LaneKeep) {
        beginTurn();
        return;
    }

    std::vector<PathPoint> points;
    points.reserve(static_cast<size_t>(kPathLength / kPathStep) + 1);
    const Vec2 right = rightOf(frame_.direction);
    for (float s = status_.along; s < status_.along + kPathLength; s += kPathStep) {
        points.push_back({frame_.origin + frame_.direction * s + right * offsetAt(s)});
    }
    path_.points = std::move(points);
    path_.reverse = false;
    path_.speedCap = 100;
    if (next_ != NextMove::Straight) {
        const float toJunction = std::max(junction - status_.along, 0.0f);
        path_.speedCap = std::sqrt(kTurnSpeed * kTurnSpeed + 2 * 1.8f * toJunction);
    }
    if (status_.type == ManeuverType::PullOver) {
        const float remaining = std::abs(status_.lateral - RoadNetwork::shoulderOffset());
        path_.speedCap = remaining < 0.4f ? 0.0f : std::min(path_.speedCap, 5.0f);
    }
}

void ManeuverModule::updateFixedPath() {
    const std::vector<PathPoint>& points = segments_[segment_];
    size_t nearest = 0;
    float best = 1e9f;
    for (size_t i = 0; i < points.size(); ++i) {
        const float distance = length(points[i].position - vehicle_.position);
        if (distance < best) {
            best = distance;
            nearest = i;
        }
    }

    path_.points = points;
    path_.reverse = segmentReverse_[segment_];
    path_.speedCap = status_.type == ManeuverType::Turn      ? kTurnSpeed
                     : status_.type == ManeuverType::Reverse ? kReverseSpeed
                                                             : kUTurnSpeed;

    bool done = nearest + 2 >= points.size();
    if (status_.type == ManeuverType::Reverse) {
        done = length(vehicle_.position - points.front().position) >= kReverseDistance;
    } else if (status_.type == ManeuverType::Turn) {
        // Finished once the car is clear of the junction on the new road.
        const RoadFrame out = frameOf(afterFixed_);
        done = dot(vehicle_.position - out.origin, out.direction) > RoadNetwork::kHalfRoad + 2.0f;
    } else if (status_.type == ManeuverType::UTurn && segment_ + 1 == segments_.size()) {
        const RoadFrame out = frameOf(afterFixed_);
        done = dot(fromHeading(vehicle_.heading), out.direction) > 0.96f;
    }
    if (!done) return;

    if (segment_ + 1 < segments_.size()) {
        ++segment_;
        return;
    }

    const ManeuverType finished = status_.type;
    edge_ = afterFixed_;
    frame_ = frameOf(edge_);
    lane_ = laneAfterFixed_;
    status_.lane = lane_;
    const Vec2 relative = vehicle_.position - frame_.origin;
    status_.along = dot(relative, frame_.direction);
    status_.lateral = dot(relative, rightOf(frame_.direction));
    homeOffset_ = RoadNetwork::laneOffset(lane_);
    blendTo(homeOffset_, std::max(std::abs(status_.lateral - homeOffset_) * 5.0f, 8.0f));
    if (finished != ManeuverType::Reverse) chooseNextMove();
    segments_.clear();
    finish();
}

void ManeuverModule::tick() {
    const Vec2 relative = vehicle_.position - frame_.origin;
    status_.along = dot(relative, frame_.direction);
    status_.lateral = dot(relative, rightOf(frame_.direction));
    status_.road = frame_;

    const bool fixed = !segments_.empty() && (status_.type == ManeuverType::Turn || status_.type == ManeuverType::UTurn ||
                                              status_.type == ManeuverType::Reverse);
    if (fixed) {
        updateFixedPath();
    } else {
        segments_.clear();
        updateLaneFollowing();
    }
    publish();
}

void ManeuverModule::publish() {
    status_.road = frame_;
    bus_.publish(topic::kManeuverStatus, status_);
    bus_.publish(topic::kPath, path_);
}
