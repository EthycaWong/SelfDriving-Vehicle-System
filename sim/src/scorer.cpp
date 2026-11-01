#include "scorer.h"

#include <algorithm>

namespace {

constexpr float kMaxDistance = 30.0f;

bool counts(const VisibleObject& object) {
    if (object.distance > kMaxDistance) return false;
    if (object.cls == ObjectClass::Tree || object.cls == ObjectClass::Building) return object.visiblePixels >= 1500;
    return object.fullyInFrame && object.visiblePixels >= 100 && object.visibleFraction >= 0.6f;
}

bool isHit(const VisibleObject& object, const Detection& detection) {
    if (detection.cls != object.cls) return false;
    const Rectangle& box = object.visibleBox;
    const float ix = std::max(0.0f, std::min(box.x + box.width, detection.x1) - std::max(box.x, detection.x0));
    const float iy = std::max(0.0f, std::min(box.y + box.height, detection.y1) - std::max(box.y, detection.y0));
    const float inter = ix * iy;
    const float boxArea = box.width * box.height;
    const float detectionArea = (detection.x1 - detection.x0) * (detection.y1 - detection.y0);
    const float unionArea = boxArea + detectionArea - inter;
    if (unionArea > 0 && inter / unionArea >= 0.3f) return true;
    return detection.source == "segmenter" && inter / std::max(boxArea, 1.0f) >= 0.5f;
}

}  // namespace

Scorer::Scorer(MessageBus& bus) {
    bus.subscribe<DetectionFrame>(topic::kDetections, [this](const DetectionFrame& frame) { onDetections(frame); });
}

void Scorer::addGroundTruth(uint32_t frameId, std::vector<VisibleObject> objects) {
    pending_[frameId] = std::move(objects);
    while (pending_.size() > 16) pending_.erase(pending_.begin());
}

void Scorer::onDetections(const DetectionFrame& frame) {
    const auto found = pending_.find(frame.frameId);
    if (found == pending_.end()) return;
    for (const Detection& detection : frame.detections) {
        ++scores_[static_cast<size_t>(detection.cls)].detections;
    }
    for (const VisibleObject& object : found->second) {
        if (!counts(object)) continue;
        ClassScore& score = scores_[static_cast<size_t>(object.cls)];
        ++score.objects;
        if (std::any_of(frame.detections.begin(), frame.detections.end(),
                        [&](const Detection& d) { return isHit(object, d); })) {
            ++score.hits;
        }
    }
    pending_.erase(found);
}
