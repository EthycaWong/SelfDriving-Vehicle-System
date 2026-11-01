#pragma once

#include "bus.h"
#include "messages.h"

// Bicycle-model car. Steering angle and steering rate are limited, so the car can't move sideways
// or turn on the spot.
class Vehicle {
public:
    static constexpr float kWheelbase = 2.7f;
    static constexpr float kMaxSteer = 0.61f;      // 35 degrees.
    static constexpr float kSteerRate = 1.05f;     // 60 degrees per second.
    static constexpr float kMaxAccel = 3.0f;
    static constexpr float kMaxBrake = 7.0f;
    static constexpr float kMaxForwardSpeed = 14.0f;
    static constexpr float kMaxReverseSpeed = 3.0f;
    static constexpr float kLength = 4.5f;
    static constexpr float kWidth = 1.9f;

    explicit Vehicle(MessageBus& bus);

    void reset(Vec2 position, float heading);
    void step(double now, float dt);
    const VehicleState& state() const { return state_; }

private:
    MessageBus& bus_;
    VehicleState state_{};
    ControlCommand command_{};
};
