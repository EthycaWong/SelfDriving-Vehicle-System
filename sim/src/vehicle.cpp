#include "vehicle.h"

#include <algorithm>
#include <cmath>

Vehicle::Vehicle(MessageBus& bus) : bus_(bus) {
    bus_.subscribe<ControlCommand>(topic::kControl, [this](const ControlCommand& command) { command_ = command; });
}

void Vehicle::reset(Vec2 position, float heading) {
    state_ = {};
    state_.position = position;
    state_.heading = heading;
}

void Vehicle::step(double now, float dt) {
    const float wanted = std::clamp(command_.steer, -kMaxSteer, kMaxSteer);
    const float maxChange = kSteerRate * dt;
    state_.steer += std::clamp(wanted - state_.steer, -maxChange, maxChange);

    // Brakes oppose motion and can't push the car the other way. The gear can only pull one way,
    // so changing direction always passes through a stop.
    float speed = state_.speed;
    const float drive = command_.throttle * kMaxAccel;
    speed += (command_.reverse ? -drive : drive) * dt;
    const float braking = command_.brake * kMaxBrake * dt + 0.05f * dt;
    if (std::abs(speed) <= braking) {
        speed = 0;
    } else {
        speed -= std::copysign(braking, speed);
    }
    state_.speed = std::clamp(speed, -kMaxReverseSpeed, kMaxForwardSpeed);

    state_.heading += state_.speed * std::tan(state_.steer) / kWheelbase * dt;
    state_.position = state_.position + fromHeading(state_.heading) * (state_.speed * dt);
    state_.time = now;
    bus_.publish(topic::kVehicle, state_);
}
