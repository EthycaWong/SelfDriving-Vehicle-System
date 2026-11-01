#pragma once

#include <vector>

#include "bus.h"
#include "messages.h"

struct CameraGeometry {
    int width = 960;
    int height = 540;
    float fovYDegrees = 40.0f;
    float mountHeight = 1.4f;
    float mountForward = 1.5f;

    float focalPixels() const;
};

// Turns image boxes into positions on the ground, and tracks them between frames. Objects that
// have slipped out of the camera's view are remembered for a while, because the car is about to
// pass them.
class Perception {
public:
    Perception(MessageBus& bus, CameraGeometry camera);
    void tick(double now);

    const std::vector<Obstacle>& tracks() const { return tracks_; }
    double lastDetectionTime() const { return lastDetection_; }

private:
    struct Track {
        Obstacle obstacle;
        int hits = 0;
        Vec2 lastPosition{};
        double lastUpdate = 0;
    };

    void onDetections(const DetectionFrame& frame);
    bool inCameraView(Vec2 position) const;

    MessageBus& bus_;
    CameraGeometry camera_;
    VehicleState vehicle_{};
    std::vector<Track> trackStates_;
    std::vector<Obstacle> tracks_;
    uint32_t classMask_ = 0;
    int nextTrack_ = 1;
    double lastDetection_ = -100;
    bool dirty_ = false;
};
