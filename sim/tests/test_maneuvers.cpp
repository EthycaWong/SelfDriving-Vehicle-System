// Drives the real Planner, Maneuver, Controller and Vehicle through scripted scenes. Obstacles here
// come straight from the scene, standing in for Perception, so these tests check driving logic and
// not recognition. Every run also checks that the car never hits anything or leaves the road.
#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "bus.h"
#include "check.h"
#include "controller.h"
#include "maneuver.h"
#include "planner.h"
#include "road_network.h"
#include "vehicle.h"

namespace {

constexpr float kDt = 1.0f / 30.0f;
constexpr float kPi = 3.14159265f;

struct Thing {
    ObjectClass cls;
    Vec2 position;
    float width;
};

struct Run {
    MessageBus bus;
    RoadNetwork roads{5, 5, 130.0f};
    Vehicle vehicle{bus};
    Controller controller{bus};
    ManeuverModule maneuver{bus, roads, 3};
    Planner planner;
    std::vector<Thing> things;
    SensorReadings sensors{};
    bool visionOn = true;
    double now = 0;
    int collisions = 0;
    float maxOffRoad = 0;
    std::vector<ManeuverType> seen;
    std::vector<DriveState> states;

    static Planner::Config config(float cruise) {
        Planner::Config c;
        c.requireBoard = true;
        c.cruiseSpeed = cruise;
        return c;
    }

    // Starts on the road heading east along row 2, some way before the first junction.
    explicit Run(int lane = 1, float along = 15.0f, float cruise = 8.0f) : planner(bus, config(cruise)) {
        maneuver.setRandomRoute(false);
        const Vector2 from = roads.node(1, 2);
        const Vec2 start{from.x + along, from.y + RoadNetwork::laneOffset(lane)};
        maneuver.start(1, 2, 2, 2, lane);
        vehicle.reset(start, 0);
        // Wait out the Planner's start-up period, then begin the scene from standing.
        for (int i = 0; i < 200; ++i) step();
        maneuver.start(1, 2, 2, 2, lane);
        vehicle.reset(start, 0);
        // Two steps, so every module has seen the new position before a test places anything.
        step();
        step();
        seen.clear();
        states.clear();
    }

    Vec2 ahead(float distance, float lateral) const {
        const auto& s = maneuver.status();
        return s.road.origin + s.road.direction * (s.along + distance) + rightOf(s.road.direction) * lateral;
    }

    void step() {
        now += kDt;
        bus.setTime(now);
        ObstacleList list;
        list.time = now;
        list.visionAlive = visionOn;
        const VehicleState& state = vehicle.state();
        for (size_t i = 0; i < things.size(); ++i) {
            if (length(things[i].position - state.position) < 45.0f) {
                Obstacle o;
                o.trackId = static_cast<int>(i) + 1;
                o.cls = things[i].cls;
                o.position = things[i].position;
                o.width = things[i].width;
                list.obstacles.push_back(o);
            }
        }
        bus.publish(topic::kObstacles, list);
        sensors.time = now;
        bus.publish(topic::kSensors, sensors);
        bus.dispatch();
        maneuver.tick();
        planner.tick(now);
        bus.dispatch();
        controller.tick(kDt);
        bus.dispatch();
        vehicle.step(now, kDt);
        bus.dispatch();
        checkSafety();
        const ManeuverType type = maneuver.status().type;
        if (seen.empty() || seen.back() != type) seen.push_back(type);
        if (states.empty() || states.back() != planner.state()) states.push_back(planner.state());
    }

    void checkSafety() {
        const VehicleState& state = vehicle.state();
        const Vec2 forward = fromHeading(state.heading);
        for (const Thing& thing : things) {
            const Vec2 relative = thing.position - state.position;
            const float along = std::abs(dot(relative, forward));
            const float side = std::abs(dot(relative, rightOf(forward)));
            if (along < Vehicle::kLength / 2 + thing.width / 2 && side < Vehicle::kWidth / 2 + thing.width / 2) {
                ++collisions;
            }
        }
        // Distance beyond the asphalt: on no east-west or north-south road.
        const float spacing = roads.spacing();
        const float dz = std::abs(std::remainder(state.position.y, spacing));
        const float dx = std::abs(std::remainder(state.position.x, spacing));
        maxOffRoad = std::max(maxOffRoad, std::min(dx, dz) - RoadNetwork::kHalfRoad);
    }

    void run(float seconds) {
        const int steps = static_cast<int>(seconds / kDt);
        for (int i = 0; i < steps; ++i) step();
    }

    bool runUntil(const std::function<bool()>& condition, float seconds) {
        const int steps = static_cast<int>(seconds / kDt);
        for (int i = 0; i < steps; ++i) {
            step();
            if (condition()) return true;
        }
        return false;
    }

    bool sawManeuver(ManeuverType type) const { return std::find(seen.begin(), seen.end(), type) != seen.end(); }
    bool sawState(DriveState state) const { return std::find(states.begin(), states.end(), state) != states.end(); }
    float lateral() const { return maneuver.status().lateral; }
    bool safe() const { return collisions == 0 && maxOffRoad <= 0.05f; }
};

const float kLane0 = RoadNetwork::laneOffset(0);
const float kLane1 = RoadNetwork::laneOffset(1);

void report(const char* name, const Run& run) {
    std::printf("%-28s collisions=%d offroad=%.2f completed=%d aborted=%d\n", name, run.collisions,
                static_cast<double>(run.maxOffRoad), run.maneuver.status().completed, run.maneuver.status().aborted);
}

void testLaneKeep() {
    Run run;
    run.run(9);
    CHECK(std::abs(run.lateral() - kLane1) < 0.3f);
    CHECK(run.vehicle.state().speed > 7.0f);
    CHECK(run.safe());
    report("lane keep", run);
}

void testLaneChange() {
    Run run;
    run.run(2);
    run.planner.requestManeuver(ManeuverType::LaneChange);
    CHECK(run.runUntil([&] { return run.maneuver.status().completed >= 1; }, 10));
    CHECK(run.sawManeuver(ManeuverType::LaneChange));
    CHECK(std::abs(run.lateral() - kLane0) < 0.4f);
    CHECK(run.safe());
    report("lane change", run);
}

void testOvertakeStoppedVehicle() {
    Run run;
    run.things.push_back({ObjectClass::Vehicle, run.ahead(40, kLane1), 1.9f});
    CHECK(run.runUntil([&] { return run.maneuver.status().completed >= 1; }, 20));
    CHECK(run.sawManeuver(ManeuverType::Overtake));
    CHECK(std::abs(run.lateral() - kLane1) < 0.5f);
    CHECK(run.safe());
    report("overtake", run);
}

void testGoAroundObstruction() {
    Run run;
    run.things.push_back({ObjectClass::Obstruction, run.ahead(40, kLane1 + 0.5f), 1.2f});
    CHECK(run.runUntil([&] { return run.maneuver.status().completed >= 1; }, 20));
    CHECK(run.sawManeuver(ManeuverType::GoAround));
    CHECK(run.safe());
    report("go around", run);
}

void testStopsForPerson() {
    Run run;
    run.things.push_back({ObjectClass::Person, run.ahead(38, kLane1), 0.6f});
    run.run(14);
    CHECK(std::abs(run.vehicle.state().speed) < 0.1f);
    CHECK(run.sawState(DriveState::Stop));
    CHECK(!run.sawManeuver(ManeuverType::Overtake));
    CHECK(!run.sawManeuver(ManeuverType::Swerve));
    CHECK(run.safe());
    report("stop for person", run);
}

// Swerving only beats braking at higher speed: at 8 m/s the car stops in less distance than it
// takes to clear the lane sideways. So this scene runs at 12 m/s.
void testEmergencySwerve() {
    Run run(1, 5.0f, 12.0f);
    CHECK(run.runUntil([&] { return run.vehicle.state().speed > 11.5f; }, 12));
    run.things.push_back({ObjectClass::Animal, run.ahead(11.0f, kLane1), 0.8f});
    run.run(7);
    CHECK(run.sawManeuver(ManeuverType::Swerve));
    CHECK(run.safe());
    report("emergency swerve", run);
}

void testBrakesWhenNoRoomToSwerve() {
    Run run;
    run.run(5);
    run.sensors.leftCm = 50;
    run.things.push_back({ObjectClass::Animal, run.ahead(9.0f, kLane1), 0.8f});
    run.run(6);
    CHECK(!run.sawManeuver(ManeuverType::Swerve));
    CHECK(std::abs(run.vehicle.state().speed) < 0.1f);
    CHECK(run.collisions == 0);
    report("no room: brakes straight", run);
}

void testTurn(int lane, const char* name) {
    Run run(lane, 70.0f);
    run.planner.requestManeuver(ManeuverType::Turn);
    const RoadFrame before = run.maneuver.status().road;
    CHECK(run.runUntil([&] { return run.maneuver.status().completed >= 1 && run.sawManeuver(ManeuverType::Turn); }, 40));
    run.run(4);
    const RoadFrame after = run.maneuver.status().road;
    CHECK(std::abs(dot(before.direction, after.direction)) < 0.1f);
    CHECK(run.safe());
    report(name, run);
}

void testUTurnPlain() {
    Run run(1);
    run.run(1);
    const float startHeading = run.vehicle.state().heading;
    run.planner.requestManeuver(ManeuverType::UTurn);
    CHECK(run.runUntil([&] { return run.maneuver.status().completed >= 1; }, 30));
    run.run(3);
    CHECK(std::cos(run.vehicle.state().heading - startHeading) < -0.95f);
    CHECK(run.lateral() > 0);
    CHECK(run.safe());
    report("u-turn (one sweep)", run);
}

void testUTurnThreePoint() {
    Run run(0);
    run.run(1);
    const float startHeading = run.vehicle.state().heading;
    run.planner.requestManeuver(ManeuverType::UTurn);
    run.step();
    bool reversed = false;
    CHECK(run.runUntil([&] {
        reversed = reversed || run.vehicle.state().speed < -0.2f;
        return run.maneuver.status().completed >= 1;
    }, 45));
    CHECK(reversed);
    run.run(3);
    CHECK(std::cos(run.vehicle.state().heading - startHeading) < -0.95f);
    CHECK(run.safe());
    report("u-turn (three-point)", run);
}

void testReverse() {
    Run run;
    run.run(3);
    const Vec2 start = run.vehicle.state().position;
    run.planner.requestManeuver(ManeuverType::Reverse);
    bool wentBack = false;
    CHECK(run.runUntil([&] {
        wentBack = wentBack || run.vehicle.state().speed < -0.5f;
        return run.maneuver.status().completed >= 1;
    }, 30));
    CHECK(wentBack);
    CHECK(run.safe());
    (void)start;
    report("reverse", run);
}

void testReverseOutOfTightSpot() {
    Run run;
    // Too close to steer around, with the other lane free.
    run.things.push_back({ObjectClass::Obstruction, run.ahead(Vehicle::kLength / 2 + 2.0f, kLane1), 2.0f});
    CHECK(run.runUntil([&] { return run.sawManeuver(ManeuverType::Reverse); }, 8));
    CHECK(run.runUntil([&] { return run.sawManeuver(ManeuverType::GoAround); }, 15));
    CHECK(run.collisions == 0);
    report("reverse then go around", run);
}

void testPullOverAndResume() {
    Run run;
    run.run(2);
    run.planner.requestManeuver(ManeuverType::PullOver);
    CHECK(run.runUntil([&] { return run.maneuver.status().phase == "stopped on shoulder"; }, 20));
    run.run(3);
    CHECK(std::abs(run.vehicle.state().speed) < 0.1f);
    CHECK(std::abs(run.lateral() - RoadNetwork::shoulderOffset()) < 0.5f);
    CHECK(run.safe());
    report("pull over", run);
}

void testAbortsWhenPathBlocked() {
    Run run;
    run.run(4);
    run.planner.requestManeuver(ManeuverType::LaneChange);
    run.run(0.6f);
    CHECK(run.maneuver.status().type == ManeuverType::LaneChange);
    run.things.push_back({ObjectClass::Person, run.ahead(16, kLane0), 0.6f});
    CHECK(run.runUntil([&] { return run.maneuver.status().aborted >= 1; }, 3));
    run.run(6);
    CHECK(run.collisions == 0);
    CHECK(std::abs(run.lateral() - kLane1) < 0.8f);
    report("abort on blocked path", run);
}

void testSensorsBlockManeuvers() {
    Run run;
    run.run(1);
    run.sensors.leftCm = 40;
    run.step();
    run.planner.requestManeuver(ManeuverType::LaneChange);
    run.run(0.5f);
    CHECK(run.maneuver.status().type == ManeuverType::LaneKeep);

    run.sensors.leftCm = 400;
    run.sensors.rearCm = 40;
    run.step();
    run.planner.requestManeuver(ManeuverType::Reverse);
    run.run(0.5f);
    CHECK(run.maneuver.status().type == ManeuverType::LaneKeep);
    report("sensors block maneuvers", run);
}

void testFrontSensorEmergencyStop() {
    Run run;
    run.run(5);
    run.sensors.frontCm = 40;
    run.run(4);
    CHECK(run.planner.state() == DriveState::EStop);
    CHECK(std::abs(run.vehicle.state().speed) < 0.05f);
    run.sensors.frontCm = 400;
    run.run(4);
    CHECK(run.vehicle.state().speed > 2.0f);
    report("front sensor e-stop", run);
}

void testFailsafeAndRecovery() {
    Run run;
    run.run(5);
    run.visionOn = false;
    CHECK(run.runUntil([&] { return run.planner.state() == DriveState::Failsafe; }, 4));
    CHECK(run.runUntil([&] { return run.maneuver.status().phase == "stopped on shoulder"; }, 20));
    run.run(2);
    CHECK(std::abs(run.vehicle.state().speed) < 0.1f);
    run.visionOn = true;
    run.run(6);
    CHECK(run.planner.state() != DriveState::Failsafe);
    CHECK(run.vehicle.state().speed > 2.0f);
    CHECK(run.safe());
    report("failsafe and recovery", run);
}

void testPhysicsLimits() {
    MessageBus bus;
    Vehicle vehicle(bus);
    vehicle.reset({0, 0}, 0);
    ControlCommand command;
    command.steer = 5;
    command.throttle = 0;
    bus.publish(topic::kControl, command);
    bus.dispatch();
    // Standing still, full steering can't turn the car on the spot.
    for (int i = 0; i < 60; ++i) vehicle.step(i * kDt, kDt);
    CHECK(std::abs(vehicle.state().heading) < 1e-4f);
    CHECK(vehicle.state().steer <= Vehicle::kMaxSteer + 1e-4f);
    // The steering takes time to move.
    vehicle.reset({0, 0}, 0);
    vehicle.step(0, kDt);
    CHECK(vehicle.state().steer <= Vehicle::kSteerRate * kDt + 1e-4f);
    // Sideways speed is impossible: motion always follows the heading.
    command.steer = 0;
    command.throttle = 1;
    bus.publish(topic::kControl, command);
    bus.dispatch();
    vehicle.reset({0, 0}, kPi / 2);
    for (int i = 0; i < 30; ++i) vehicle.step(i * kDt, kDt);
    CHECK(std::abs(vehicle.state().position.x) < 0.01f);
    // Reverse gear has its own speed limit.
    command.reverse = true;
    bus.publish(topic::kControl, command);
    bus.dispatch();
    vehicle.reset({0, 0}, 0);
    for (int i = 0; i < 300; ++i) vehicle.step(i * kDt, kDt);
    CHECK(vehicle.state().speed >= -Vehicle::kMaxReverseSpeed);
    CHECK(vehicle.state().speed < -1.0f);
}

}  // namespace

int main() {
    testPhysicsLimits();
    testLaneKeep();
    testLaneChange();
    testOvertakeStoppedVehicle();
    testGoAroundObstruction();
    testStopsForPerson();
    testEmergencySwerve();
    testBrakesWhenNoRoomToSwerve();
    testTurn(1, "turn from right lane");
    testTurn(0, "turn from left lane");
    testUTurnPlain();
    testUTurnThreePoint();
    testReverse();
    testReverseOutOfTightSpot();
    testPullOverAndResume();
    testAbortsWhenPathBlocked();
    testSensorsBlockManeuvers();
    testFrontSensorEmergencyStop();
    testFailsafeAndRecovery();
    std::printf("maneuver tests: %d failures\n", failureCount());
    return failureCount() == 0 ? 0 : 1;
}
