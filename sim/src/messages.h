#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "assets.h"

// Message types that travel over the bus, plus the small math type they share.
// World axes: +X east, +Z south, so a heading of 0 faces +X and positive headings turn right.

struct Vec2 {
    float x = 0;
    float y = 0;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, float k) { return {a.x * k, a.y * k}; }
inline float dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline float length(Vec2 a) { return std::sqrt(dot(a, a)); }
inline Vec2 rightOf(Vec2 direction) { return {-direction.y, direction.x}; }
inline Vec2 fromHeading(float heading) { return {std::cos(heading), std::sin(heading)}; }

namespace topic {
inline constexpr const char* kCameraFrame = "camera.frame";
inline constexpr const char* kDetections = "vision.detections";
inline constexpr const char* kObstacles = "world.obstacles";
inline constexpr const char* kSensors = "arduino.sensors";
inline constexpr const char* kLink = "arduino.link";
inline constexpr const char* kPlan = "plan.target";
inline constexpr const char* kManeuverRequest = "maneuver.request";
inline constexpr const char* kPath = "maneuver.path";
inline constexpr const char* kManeuverStatus = "maneuver.status";
inline constexpr const char* kControl = "control.command";
inline constexpr const char* kManual = "manual.input";
inline constexpr const char* kVehicle = "vehicle.state";
}  // namespace topic

enum class DriveState : int { Cruise, Slow, Stop, EStop, Failsafe };

enum class ManeuverType : int { LaneKeep, LaneChange, Overtake, GoAround, Swerve, Turn, UTurn, Reverse, PullOver };

inline const char* driveStateName(DriveState state) {
    static const char* names[] = {"CRUISE", "SLOW", "STOP", "ESTOP", "FAILSAFE"};
    return names[static_cast<int>(state)];
}

inline const char* maneuverName(ManeuverType type) {
    static const char* names[] = {"LANE KEEP", "LANE CHANGE", "OVERTAKE", "GO AROUND", "SWERVE",
                                  "TURN",      "U-TURN",      "REVERSE",  "PULL OVER"};
    return names[static_cast<int>(type)];
}

struct VehicleState {
    Vec2 position{};
    float heading = 0;
    float speed = 0;  // Negative when reversing.
    float steer = 0;  // Radians. Positive steers right.
    double time = 0;
};

// Pixels only: frames never carry scene labels.
struct CameraFrame {
    uint32_t id = 0;
    int width = 0;
    int height = 0;
    std::shared_ptr<std::vector<uint8_t>> rgb;
    double time = 0;
};

struct Detection {
    ObjectClass cls = ObjectClass::Obstruction;
    float score = 0;
    float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    std::string source;  // "detector", "segmenter" or "road_check".
};

struct DetectionFrame {
    uint32_t frameId = 0;
    std::vector<Detection> detections;
    float detectorMs = 0;
    float segmenterMs = 0;
    double roundTripMs = 0;
    double time = 0;
};

struct Obstacle {
    int trackId = 0;
    ObjectClass cls = ObjectClass::Obstruction;
    Vec2 position{};
    float width = 1;
    float speed = 0;
    double lastSeen = 0;
    bool inView = true;  // False when remembered from earlier frames.
};

struct ObstacleList {
    std::vector<Obstacle> obstacles;
    uint32_t classMask = 0;  // Every class seen this frame, including trees and buildings.
    bool visionAlive = false;
    double time = 0;
};

// Distances from the four HC-SR04 sensors in the Wokwi circuit.
struct SensorReadings {
    int frontCm = 400;
    int rearCm = 400;
    int leftCm = 400;
    int rightCm = 400;
    bool watchdogTripped = false;
    double time = 0;
};

struct LinkStatus {
    bool connected = false;
    double lastReceived = 0;
    double ackRoundTripMs = 0;
    int badLines = 0;
    std::string lastBoardLine;
};

struct ManeuverRequest {
    ManeuverType type = ManeuverType::LaneKeep;
    bool hasTarget = false;
    Vec2 target{};
    float targetWidth = 1;
    int side = 0;  // -1 left, +1 right, 0 lets the module choose.
    bool resume = false;  // Rejoin the lane after a pull over.
    bool abort = false;
};

struct PlanTarget {
    DriveState state = DriveState::Cruise;
    float speed = 0;
    bool forceBrake = false;
    std::string reason;
};

struct PathPoint {
    Vec2 position{};
};

struct ManeuverPath {
    std::vector<PathPoint> points;
    bool reverse = false;
    float speedCap = 100;
};

struct RoadFrame {
    Vec2 origin{};
    Vec2 direction{1, 0};
    float length = 0;
};

struct ManeuverStatus {
    ManeuverType type = ManeuverType::LaneKeep;
    std::string phase;
    int lane = 1;
    float along = 0;
    float lateral = 0;
    RoadFrame road{};
    int wantLane = -1;  // The route needs this lane for its next turn.
    int completed = 0;
    int aborted = 0;
    int signal = 0;  // -1 left, +1 right.
};

struct ControlCommand {
    float steer = 0;
    float throttle = 0;
    float brake = 0;
    bool reverse = false;
    DriveState state = DriveState::Cruise;
    ManeuverType maneuver = ManeuverType::LaneKeep;
    bool manual = false;
};

struct ManualInput {
    bool enabled = false;
    float steer = 0;  // -1 to 1.
    float throttle = 0;
    float brake = 0;
    bool reverse = false;
};
