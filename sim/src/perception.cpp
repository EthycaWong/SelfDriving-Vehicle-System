#include "perception.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr float kPi = 3.14159265f;
constexpr float kMatchDistance = 3.5f;
constexpr float kMaxRange = 60.0f;
constexpr int kConfirmHits = 2;
constexpr double kForgetInView = 0.9;     // It should be visible, so losing it means it's gone.
constexpr double kForgetOutOfView = 6.0;  // Beside or behind the car, so keep it in mind.

// Trees, buildings and signs are reported but don't get ground positions. Their boxes don't sit on
// the road surface the way the distance estimate assumes. Segmentation regions have the same issue.
bool isGroundObject(const Detection& detection) {
    if (detection.source == "segmenter") return false;
    return detection.cls == ObjectClass::Person || detection.cls == ObjectClass::Animal ||
           detection.cls == ObjectClass::Vehicle || detection.cls == ObjectClass::Obstruction;
}

}  // namespace

float CameraGeometry::focalPixels() const {
    return static_cast<float>(height) / 2 / std::tan(fovYDegrees * kPi / 360);
}

Perception::Perception(MessageBus& bus, CameraGeometry camera) : bus_(bus), camera_(camera) {
    bus_.subscribe<VehicleState>(topic::kVehicle, [this](const VehicleState& state) { vehicle_ = state; });
    bus_.subscribe<DetectionFrame>(topic::kDetections, [this](const DetectionFrame& frame) { onDetections(frame); });
}

bool Perception::inCameraView(Vec2 position) const {
    const Vec2 forward = fromHeading(vehicle_.heading);
    const Vec2 relative = position - vehicle_.position;
    const float ahead = dot(relative, forward);
    if (ahead < 2.0f) return false;
    const float halfWidth = ahead * static_cast<float>(camera_.width) / 2 / camera_.focalPixels();
    return std::abs(dot(relative, rightOf(forward))) < halfWidth;
}

void Perception::onDetections(const DetectionFrame& frame) {
    lastDetection_ = frame.time;
    dirty_ = true;
    classMask_ = 0;

    const float focal = camera_.focalPixels();
    const float centerX = static_cast<float>(camera_.width) / 2;
    const float horizon = static_cast<float>(camera_.height) / 2;
    const Vec2 forward = fromHeading(vehicle_.heading);
    const Vec2 right = rightOf(forward);
    const Vec2 cameraPosition = vehicle_.position + forward * camera_.mountForward;

    for (Track& track : trackStates_) track.obstacle.inView = inCameraView(track.obstacle.position);

    std::vector<bool> matched(trackStates_.size(), false);
    for (const Detection& detection : frame.detections) {
        classMask_ |= 1u << static_cast<int>(detection.cls);
        if (!isGroundObject(detection)) continue;
        const float below = detection.y1 - horizon;
        if (below < 2.0f) continue;

        // The box's bottom edge is where the object meets the road, which gives its distance.
        const float distance = focal * camera_.mountHeight / below;
        if (distance > kMaxRange) continue;
        const float centre = (detection.x0 + detection.x1) / 2;
        const float sideways = (centre - centerX) * distance / focal;
        const Vec2 position = cameraPosition + forward * distance + right * sideways;
        const float width = std::max((detection.x1 - detection.x0) * distance / focal, 0.4f);

        size_t bestIndex = trackStates_.size();
        float bestGap = kMatchDistance;
        for (size_t i = 0; i < trackStates_.size(); ++i) {
            const Track& track = trackStates_[i];
            if (matched[i] || track.obstacle.cls != detection.cls) continue;
            const float gap = length(track.obstacle.position - position);
            if (gap < bestGap) {
                bestGap = gap;
                bestIndex = i;
            }
        }
        if (bestIndex < trackStates_.size()) {
            matched[bestIndex] = true;
            Track* best = &trackStates_[bestIndex];
            const double elapsed = std::max(frame.time - best->lastUpdate, 0.05);
            const float moved = length(position - best->lastPosition) / static_cast<float>(elapsed);
            // Heavy smoothing: the distance estimate is noisy from frame to frame.
            best->obstacle.speed += 0.2f * (moved - best->obstacle.speed);
            best->obstacle.position = best->obstacle.position * 0.5f + position * 0.5f;
            best->obstacle.width = best->obstacle.width * 0.5f + width * 0.5f;
            best->obstacle.lastSeen = frame.time;
            best->obstacle.inView = true;
            best->lastPosition = position;
            best->lastUpdate = frame.time;
            ++best->hits;
        } else {
            Track track;
            track.obstacle.trackId = nextTrack_++;
            track.obstacle.cls = detection.cls;
            track.obstacle.position = position;
            track.obstacle.width = width;
            track.obstacle.lastSeen = frame.time;
            track.lastPosition = position;
            track.lastUpdate = frame.time;
            track.hits = 1;
            trackStates_.push_back(track);
            matched.push_back(true);
        }
    }
}

void Perception::tick(double now) {
    std::erase_if(trackStates_, [&](const Track& track) {
        const double age = now - track.obstacle.lastSeen;
        return age > (track.obstacle.inView ? kForgetInView : kForgetOutOfView);
    });

    tracks_.clear();
    for (const Track& track : trackStates_) {
        // Two sightings in a row before a detection can affect driving, which filters flicker.
        if (track.hits >= kConfirmHits) tracks_.push_back(track.obstacle);
    }

    ObstacleList list;
    list.obstacles = tracks_;
    list.classMask = classMask_;
    list.visionAlive = dirty_;
    list.time = now;
    dirty_ = false;
    bus_.publish(topic::kObstacles, std::move(list));
}
