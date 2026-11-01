#pragma once

#include <raylib.h>

#include <map>
#include <string>
#include <vector>

#include "bus.h"
#include "messages.h"
#include "scorer.h"

struct DashboardData {
    DriveState state = DriveState::Cruise;
    std::string reason;
    ManeuverStatus maneuver{};
    VehicleState vehicle{};
    ControlCommand command{};
    SensorReadings sensors{};
    LinkStatus link{};
    bool visionConnected = false;
    bool visionHealthy = false;
    bool boardHealthy = false;
    double visionRoundTripMs = 0;
    float detectorMs = 0;
    float segmenterMs = 0;
    int tracks = 0;
    int collisions = 0;
    bool manual = false;
    double now = 0;
    const std::array<Scorer::ClassScore, kClassCount>* scores = nullptr;
};

void drawDashboard(MessageBus& bus, const DashboardData& data, Rectangle area);
void drawBusLog(MessageBus& bus, Rectangle area);
