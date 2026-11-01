#pragma once

#include <array>
#include <map>
#include <vector>

#include "bus.h"
#include "messages.h"
#include "renderer.h"

// Compares the classifier's answers with what the simulation really placed, using the same rules
// as tools/score_recognition.py. It only feeds the dashboard. Nothing it knows reaches the Planner.
class Scorer {
public:
    struct ClassScore {
        int objects = 0;
        int hits = 0;
        int detections = 0;
    };

    explicit Scorer(MessageBus& bus);

    void addGroundTruth(uint32_t frameId, std::vector<VisibleObject> objects);
    const std::array<ClassScore, kClassCount>& scores() const { return scores_; }

private:
    void onDetections(const DetectionFrame& frame);

    std::map<uint32_t, std::vector<VisibleObject>> pending_;
    std::array<ClassScore, kClassCount> scores_{};
};
