#pragma once

#include <optional>

#include "bus.h"
#include "messages.h"

// Decides how fast to drive and which maneuver to use. It only knows what Perception and the
// board's sensors report, never the simulation's own object list.
class Planner {
public:
    struct Config {
        float cruiseSpeed = 8.0f;
        float comfortBrake = 3.5f;
        float hardBrake = 6.5f;
        float stopMargin = 4.5f;
        int sensorClearCm = 100;     // A side or rear sensor below this blocks moves that way.
        int estopCm = 60;            // The front sensor below this is an emergency stop.
        int estopReleaseCm = 80;
        double visionTimeout = 2.0;
        double boardTimeout = 2.0;
        bool requireBoard = true;
        bool requireVision = true;
    };

    Planner(MessageBus& bus, Config config);
    void tick(double now);

    // Keyboard requests. These pass the same safety checks as the Planner's own choices.
    void requestManeuver(ManeuverType type);
    void requestResume();

    DriveState state() const { return state_; }
    const std::string& reason() const { return reason_; }
    bool visionHealthy() const { return visionHealthy_; }
    bool boardHealthy() const { return boardHealthy_; }

private:
    struct Blocker {
        Obstacle obstacle;
        float distance = 0;
        float lateral = 0;
    };

    std::optional<Blocker> firstOnPath() const;
    bool laneClear(float lateral, float behind, float ahead) const;
    bool personInCorridor(float lateral, float ahead) const;
    int sideTowardOtherLane() const;
    bool sideSensorClear(int side) const;
    bool canMoveToOtherLane() const;
    bool sendManeuver(ManeuverType type, const Blocker* blocker);
    float stoppingSpeed(float distance) const;
    void decide(double now);

    MessageBus& bus_;
    Config config_;
    ObstacleList obstacles_{};
    SensorReadings sensors_{};
    LinkStatus link_{};
    VehicleState vehicle_{};
    ManeuverStatus status_{};
    ManeuverPath path_{};
    ManualInput manual_{};

    DriveState state_ = DriveState::Cruise;
    std::string reason_;
    bool estopLatched_ = false;
    bool pulledOver_ = false;
    bool visionHealthy_ = false;
    bool boardHealthy_ = false;
    double lastDetections_ = -100;
    double lastSensors_ = -100;
    double startTime_ = -1;
    double lastManeuverTime_ = -100;
    double blockedSince_ = -1;
    bool hasSensors_ = false;
};
