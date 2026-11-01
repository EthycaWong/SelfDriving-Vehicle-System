#pragma once

#include "bus.h"
#include "messages.h"

// Follows the current path with pure-pursuit steering and PID speed control, in either direction.
class Controller {
public:
    explicit Controller(MessageBus& bus);
    void tick(float dt);

private:
    MessageBus& bus_;
    VehicleState vehicle_{};
    ManeuverPath path_{};
    PlanTarget plan_{};
    ManeuverStatus status_{};
    ManualInput manual_{};
    float speedIntegral_ = 0;
};
