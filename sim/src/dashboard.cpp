#include "dashboard.h"

#include <cmath>
#include <cstdio>

namespace {

constexpr Color kPanel{24, 27, 33, 255};
constexpr Color kText{222, 226, 232, 255};
constexpr Color kDim{135, 143, 155, 255};
constexpr Color kGood{88, 200, 130, 255};
constexpr Color kWarn{240, 190, 70, 255};
constexpr Color kBad{240, 95, 90, 255};
constexpr int kBody = 15;
constexpr int kHeading = 16;
constexpr int kRow = 20;

void heading(const char* text, float x, float& y) {
    y += 8;
    DrawText(text, static_cast<int>(x), static_cast<int>(y), kHeading, kDim);
    y += kRow + 2;
}

void row(const std::string& label, const std::string& value, Color color, float x, float& y, float valueX) {
    DrawText(label.c_str(), static_cast<int>(x), static_cast<int>(y), kBody, kText);
    DrawText(value.c_str(), static_cast<int>(valueX), static_cast<int>(y), kBody, color);
    y += kRow;
}

Color stateColor(DriveState state) {
    switch (state) {
        case DriveState::Cruise: return kGood;
        case DriveState::Slow: return kWarn;
        default: return kBad;
    }
}

std::string percent(float value) {
    char text[16];
    std::snprintf(text, sizeof(text), "%.0f%%", static_cast<double>(value) * 100);
    return text;
}

}  // namespace

void drawDashboard(MessageBus& bus, const DashboardData& data, Rectangle area) {
    DrawRectangleRec(area, kPanel);
    const float x = area.x + 14;
    const float valueX = area.x + 170;
    float y = area.y + 8;
    char text[96];

    DrawText(driveStateName(data.state), static_cast<int>(x), static_cast<int>(y), 28, stateColor(data.state));
    y += 32;
    DrawText(data.reason.c_str(), static_cast<int>(x), static_cast<int>(y), kBody, kText);
    y += kRow;
    std::string maneuver = maneuverName(data.maneuver.type);
    if (!data.maneuver.phase.empty()) maneuver += ": " + data.maneuver.phase;
    if (data.manual) maneuver = "MANUAL DRIVING";
    DrawText(maneuver.c_str(), static_cast<int>(x), static_cast<int>(y), kBody, kWarn);
    y += kRow;
    std::snprintf(text, sizeof(text), "%.1f m/s   steer %+.0f deg   lane %d   collisions %d", std::abs(static_cast<double>(data.vehicle.speed)),
                  static_cast<double>(data.vehicle.steer) * 57.3, data.maneuver.lane, data.collisions);
    DrawText(text, static_cast<int>(x), static_cast<int>(y), kBody, data.collisions > 0 ? kBad : kText);
    y += kRow;

    heading("MODULES  (messages per second)", x, y);
    const std::map<std::string, TopicStats> stats = bus.stats();
    const struct {
        const char* module;
        const char* topic;
    } modules[] = {
        {"Renderer", topic::kCameraFrame}, {"Classifier link", topic::kDetections}, {"Perception", topic::kObstacles},
        {"Planner", topic::kPlan},         {"Maneuver", topic::kPath},              {"Controller", topic::kControl},
        {"Vehicle", topic::kVehicle},      {"Board link", topic::kSensors},
    };
    for (const auto& module : modules) {
        const auto found = stats.find(module.topic);
        const bool active = found != stats.end() && data.now - found->second.lastTime < 1.5;
        std::snprintf(text, sizeof(text), "%5.1f  %s", found != stats.end() ? found->second.rate : 0.0, module.topic);
        row(module.module, text, active ? kGood : kBad, x, y, valueX);
    }

    heading("LINKS", x, y);
    std::snprintf(text, sizeof(text), "%s  %.0f ms (detect %.0f, segment %.0f)", data.visionHealthy ? "ok" : "SILENT", data.visionRoundTripMs,
                  static_cast<double>(data.detectorMs), static_cast<double>(data.segmenterMs));
    row("Classifier", text, data.visionHealthy ? kGood : kBad, x, y, valueX);
    std::snprintf(text, sizeof(text), "%s  ack %.0f ms, bad lines %d", data.boardHealthy ? "ok" : "SILENT", data.link.ackRoundTripMs, data.link.badLines);
    row("Wokwi board", text, data.boardHealthy ? kGood : kBad, x, y, valueX);
    std::snprintf(text, sizeof(text), "%d tracked objects", data.tracks);
    row("Perception", text, kText, x, y, valueX);

    heading("BOARD SENSORS  (cm)", x, y);
    const auto sensorRow = [&](const char* name, int cm, int limit) {
        const Color color = cm < limit ? kBad : kGood;
        DrawText(name, static_cast<int>(x), static_cast<int>(y), kBody, kText);
        const float barX = valueX;
        const float barWidth = area.width - (valueX - area.x) - 70;
        DrawRectangle(static_cast<int>(barX), static_cast<int>(y + 2), static_cast<int>(barWidth), 11, Color{50, 54, 62, 255});
        DrawRectangle(static_cast<int>(barX), static_cast<int>(y + 2), static_cast<int>(barWidth * std::min(cm, 400) / 400.0f), 11, color);
        DrawText(std::to_string(cm).c_str(), static_cast<int>(barX + barWidth + 10), static_cast<int>(y), kBody, color);
        y += kRow;
    };
    sensorRow("Front (e-stop)", data.sensors.frontCm, 60);
    sensorRow("Rear", data.sensors.rearCm, 100);
    sensorRow("Left", data.sensors.leftCm, 100);
    sensorRow("Right", data.sensors.rightCm, 100);

    heading("CLASSIFIER SCORE  (live, within 30 m)", x, y);
    if (data.scores != nullptr) {
        for (int i = 0; i < kClassCount; ++i) {
            const Scorer::ClassScore& score = (*data.scores)[static_cast<size_t>(i)];
            const float recall = score.objects > 0 ? static_cast<float>(score.hits) / static_cast<float>(score.objects) : 0;
            std::snprintf(text, sizeof(text), "%s  (%d of %d)", score.objects > 0 ? percent(recall).c_str() : "-", score.hits, score.objects);
            row(className(static_cast<ObjectClass>(i)), text, score.objects == 0 ? kDim : recall >= 0.7f ? kGood : kWarn, x, y, valueX);
        }
    }

    heading("KEYS", x, y);
    const char* keys[] = {
        "M manual (WASD, R reverse gear)",
        "1 lane change  2 overtake  3 go around",
        "4 swerve  5 turn  6 U-turn  7 reverse",
        "8 pull over  9 rejoin lane",
        "P person  N animal  V vehicle  O obstacle  C clear",
    };
    for (const char* line : keys) {
        DrawText(line, static_cast<int>(x), static_cast<int>(y), 14, kDim);
        y += 18;
    }
}

void drawBusLog(MessageBus& bus, Rectangle area) {
    DrawRectangleRec(area, kPanel);
    DrawText("MESSAGE BUS LOG", static_cast<int>(area.x + 10), static_cast<int>(area.y + 8), kHeading, kDim);
    const std::vector<BusLogEntry> log = bus.recentLog();
    const int rows = static_cast<int>((area.height - 34) / 17);
    int y = static_cast<int>(area.y + 30);
    int shown = 0;
    for (auto entry = log.rbegin(); entry != log.rend() && shown < rows; ++entry) {
        if (entry->topic == topic::kDetections && shown > 2) continue;
        char text[120];
        std::snprintf(text, sizeof(text), "%6.1f  %s", entry->time, entry->summary.c_str());
        DrawText(text, static_cast<int>(area.x + 10), y, 14, shown == 0 ? kText : kDim);
        y += 17;
        ++shown;
    }
}
