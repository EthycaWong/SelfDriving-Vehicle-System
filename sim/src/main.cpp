// The simulation: renders the world, runs the driving modules and connects to the classifier and
// the Wokwi board. Start the classifier and the board first, or the car waits and then pulls over.
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "arduino_link.h"
#include "bus.h"
#include "controller.h"
#include "dashboard.h"
#include "maneuver.h"
#include "perception.h"
#include "planner.h"
#include "protocol.h"
#include "renderer.h"
#include "scene.h"
#include "scorer.h"
#include "vehicle.h"
#include "vision_link.h"

namespace {

constexpr int kWindowWidth = 1440;
constexpr int kWindowHeight = 810;
constexpr int kChaseWidth = 960;
constexpr int kChaseHeight = 540;
constexpr double kCapturePeriod = 0.12;
constexpr double kScorePeriod = 1.0;
constexpr double kLogPeriod = 0.25;

struct Options {
    std::string root = ".";
    int visionPort = 5555;
    int boardPort = 4000;
    bool requireBoard = true;
    bool hidden = false;
    double duration = 0;
    unsigned seed = 11;
    std::string logPath;
    std::string shotPrefix;
    struct Event {
        double at;
        std::string action;
    };
    std::vector<Event> events;
};

Options parseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto value = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
        if (arg == "--root") options.root = value();
        else if (arg == "--vision-port") options.visionPort = std::stoi(value());
        else if (arg == "--board-port") options.boardPort = std::stoi(value());
        else if (arg == "--no-board") options.requireBoard = false;
        else if (arg == "--hidden") options.hidden = true;
        else if (arg == "--duration") options.duration = std::stod(value());
        else if (arg == "--seed") options.seed = static_cast<unsigned>(std::stoul(value()));
        else if (arg == "--log") options.logPath = value();
        else if (arg == "--shots") options.shotPrefix = value();
        else if (arg == "--events") {
            // Comma-separated time:action pairs, for example 12:person,30:shot.
            std::stringstream list(value());
            std::string item;
            while (std::getline(list, item, ',')) {
                const size_t colon = item.find(':');
                options.events.push_back({std::stod(item.substr(0, colon)), item.substr(colon + 1)});
            }
        }
    }
    return options;
}

// People, animals, parked cars and obstacles spread along the streets, in fixed spots for a seed.
void populateTraffic(Scene& scene, const AssetLibrary& assets, const RoadNetwork& roads, unsigned seed) {
    std::mt19937 rng(seed);
    const auto pick = [&](const std::vector<int>& list) {
        return list[std::uniform_int_distribution<size_t>(0, list.size() - 1)(rng)];
    };
    const std::vector<int> people = assets.cutoutsOfClass(ObjectClass::Person);
    const std::vector<int> animals = assets.cutoutsOfClass(ObjectClass::Animal);
    const std::vector<int> vehicles = assets.cutoutsOfClass(ObjectClass::Vehicle);
    const std::vector<int> signs = assets.cutoutsOfClass(ObjectClass::TrafficControl);
    std::uniform_real_distribution<float> chance(0, 1);

    const float inset = RoadNetwork::kHalfRoad + 25.0f;
    const float span = roads.spacing() - 2 * inset;
    const auto street = [&](Vec2 origin, Vec2 direction) {
        const Vec2 right = rightOf(direction);
        const auto at = [&](float along, float side) {
            const Vec2 p = origin + direction * along + right * side;
            return Vector3{p.x, 0, p.y};
        };
        for (const float side : {1.0f, -1.0f}) {
            scene.addCutout(pick(signs), at(inset - 12, side * (RoadNetwork::kHalfRoad + 0.4f)));
            scene.addCutout(pick(people), at(inset + chance(rng) * span, side * (RoadNetwork::kHalfRoad + 1.5f)));
            if (chance(rng) < 0.5f) scene.addCutout(pick(animals), at(inset + chance(rng) * span, side * (RoadNetwork::kHalfRoad + 1.5f)));
            if (chance(rng) < 0.6f) scene.addModel(3 + static_cast<int>(chance(rng) * 3) % 3, at(inset + chance(rng) * span, side * (RoadNetwork::kHalfRoad + 0.8f)), 90);
        }
        // Hazards in the outer lanes, far enough apart that the car has room to deal with each.
        const float hazard = chance(rng);
        const float along = inset + span * (0.3f + 0.4f * chance(rng));
        const float lane = RoadNetwork::laneOffset(1);
        if (hazard < 0.45f) {
            scene.addCutout(pick(vehicles), at(along, lane));
        } else if (hazard < 0.75f) {
            scene.addModel(static_cast<int>(chance(rng) * 3) % 3, at(along, lane), 90);
        }
    };
    for (int column = 0; column < roads.columns(); ++column) {
        for (int row = 0; row < roads.rows(); ++row) {
            const Vector2 n = roads.node(column, row);
            if (column + 1 < roads.columns()) {
                street({n.x, n.y}, {1, 0});
                street({n.x + roads.spacing(), n.y}, {-1, 0});
            }
            if (row + 1 < roads.rows()) {
                street({n.x, n.y}, {0, 1});
                street({n.x, n.y + roads.spacing()}, {0, -1});
            }
        }
    }
}

void drawCar(const VehicleState& state, const ManeuverStatus& status, double now) {
    rlPushMatrix();
    rlTranslatef(state.position.x, 0, state.position.y);
    rlRotatef(-state.heading * RAD2DEG, 0, 1, 0);
    DrawCube({0, 0.62f, 0}, Vehicle::kLength, 0.75f, Vehicle::kWidth, Color{196, 42, 48, 255});
    DrawCube({-0.25f, 1.2f, 0}, 2.2f, 0.6f, 1.7f, Color{48, 56, 70, 255});
    DrawCube({-0.25f, 1.53f, 0}, 2.0f, 0.08f, 1.6f, Color{176, 38, 44, 255});
    for (const float x : {1.45f, -1.45f}) {
        for (const float z : {0.95f, -0.95f}) {
            DrawCube({x, 0.33f, z}, 0.7f, 0.66f, 0.22f, Color{22, 22, 24, 255});
        }
    }
    const bool blink = std::fmod(now, 0.7) < 0.35;
    if (status.signal != 0 && blink) {
        const float z = status.signal > 0 ? 0.8f : -0.8f;
        DrawCube({2.28f, 0.75f, z}, 0.06f, 0.18f, 0.28f, ORANGE);
        DrawCube({-2.28f, 0.75f, z}, 0.06f, 0.18f, 0.28f, ORANGE);
    }
    if (state.speed < -0.1f) {
        DrawCube({-2.28f, 0.8f, 0.6f}, 0.06f, 0.14f, 0.3f, WHITE);
        DrawCube({-2.28f, 0.8f, -0.6f}, 0.06f, 0.14f, 0.3f, WHITE);
    }
    rlPopMatrix();
}

void drawPath(const ManeuverPath& path, bool reverse) {
    const Color color = reverse ? Color{255, 255, 255, 255} : Color{70, 220, 255, 255};
    for (size_t i = 0; i + 1 < path.points.size(); ++i) {
        const Vec2 a = path.points[i].position;
        const Vec2 b = path.points[i + 1].position;
        DrawCylinderEx({a.x, 0.06f, a.y}, {b.x, 0.06f, b.y}, 0.09f, 0.09f, 4, color);
    }
}

}  // namespace

int main(int argc, char** argv) {
    const Options options = parseOptions(argc, argv);

    if (options.hidden) SetConfigFlags(FLAG_WINDOW_HIDDEN);
    SetTraceLogLevel(LOG_ERROR);
    InitWindow(kWindowWidth, kWindowHeight, "SelfDrive");
    SetTargetFPS(30);

    const CameraGeometry geometry{};
    Renderer renderer(geometry.width, geometry.height);
    AssetLibrary assets;
    if (!assets.load(options.root, renderer.shader())) return 1;

    RoadNetwork roads(4, 3, 130.0f);
    Scene scene(roads, assets, options.seed);
    scene.populateStreets();
    populateTraffic(scene, assets, roads, options.seed + 1);
    renderer.buildStatic(scene);

    MessageBus bus;
    Vehicle vehicle(bus);
    Controller controller(bus);
    ManeuverModule maneuver(bus, roads, options.seed);
    Planner::Config plannerConfig;
    plannerConfig.requireBoard = options.requireBoard;
    Planner planner(bus, plannerConfig);
    Perception perception(bus, geometry);
    Scorer scorer(bus);
    VisionLink visionLink(bus, "127.0.0.1", options.visionPort);
    std::unique_ptr<ArduinoLink> boardLink;
    if (options.requireBoard) boardLink = std::make_unique<ArduinoLink>(bus, "127.0.0.1", options.boardPort);

    // These only feed the dashboard and the driver's view overlay.
    ManeuverPath path{};
    SensorReadings sensors{};
    ControlCommand command{};
    DetectionFrame lastDetections{};
    bus.subscribe<ManeuverPath>(topic::kPath, [&](const ManeuverPath& p) { path = p; });
    bus.subscribe<SensorReadings>(topic::kSensors, [&](const SensorReadings& s) { sensors = s; });
    bus.subscribe<ControlCommand>(topic::kControl, [&](const ControlCommand& c) { command = c; });
    bus.subscribe<DetectionFrame>(topic::kDetections, [&](const DetectionFrame& d) { lastDetections = d; });

    const Vector2 startNode = roads.node(0, 1);
    maneuver.start(0, 1, 1, 1, 0);
    vehicle.reset({startNode.x + 15, startNode.y + RoadNetwork::laneOffset(0)}, 0);

    RenderTexture2D chaseTarget = LoadRenderTexture(kChaseWidth, kChaseHeight);
    RenderTexture2D uiTarget = LoadRenderTexture(kWindowWidth, kWindowHeight);

    Camera3D driverCamera{};
    driverCamera.up = {0, 1, 0};
    driverCamera.fovy = geometry.fovYDegrees;
    driverCamera.projection = CAMERA_PERSPECTIVE;
    Camera3D chaseCamera{};
    chaseCamera.up = {0, 1, 0};
    chaseCamera.fovy = 55;
    chaseCamera.projection = CAMERA_PERSPECTIVE;
    Vec2 chaseFollow = vehicle.state().position - fromHeading(0) * 9.0f;

    std::ofstream log;
    if (!options.logPath.empty()) log.open(options.logPath);

    double now = 0;
    double nextCapture = 0;
    double nextScore = 0;
    double nextLog = 0;
    uint32_t frameId = 0;
    int collisions = 0;
    std::vector<int> touching;
    std::vector<int> spawned;
    std::vector<Options::Event> events = options.events;
    std::sort(events.begin(), events.end(), [](const auto& a, const auto& b) { return a.at < b.at; });
    size_t nextEvent = 0;
    bool manual = false;
    bool manualReverse = false;
    int shot = 0;

    const auto spawnAhead = [&](const std::string& what) {
        const ManeuverStatus& status = maneuver.status();
        const Vec2 spot = status.road.origin + status.road.direction * (status.along + 22.0f) +
                          rightOf(status.road.direction) * RoadNetwork::laneOffset(status.lane);
        const Vector3 position{spot.x, 0, spot.y};
        const auto first = [](const std::vector<int>& list) { return list[static_cast<size_t>(GetRandomValue(0, static_cast<int>(list.size()) - 1))]; };
        if (what == "person") spawned.push_back(scene.addCutout(first(assets.cutoutsOfClass(ObjectClass::Person)), position));
        else if (what == "animal") spawned.push_back(scene.addCutout(first(assets.cutoutsOfClass(ObjectClass::Animal)), position));
        else if (what == "vehicle") spawned.push_back(scene.addCutout(first(assets.cutoutsOfClass(ObjectClass::Vehicle)), position));
        else if (what == "obstacle") spawned.push_back(scene.addModel(GetRandomValue(0, 2), position, 90));
    };
    const auto clearSpawned = [&] {
        for (const int id : spawned) scene.removeObject(id);
        spawned.clear();
    };

    while (!WindowShouldClose() && (options.duration <= 0 || now < options.duration)) {
        const float dt = std::min(GetFrameTime(), 0.05f);
        now += dt;
        bus.setTime(now);

        // Keyboard.
        if (IsKeyPressed(KEY_M)) {
            manual = !manual;
            if (!manual) maneuver.relocate(vehicle.state().position, vehicle.state().heading);
        }
        if (IsKeyPressed(KEY_R)) manualReverse = !manualReverse;
        const struct {
            int key;
            ManeuverType type;
        } maneuverKeys[] = {{KEY_ONE, ManeuverType::LaneChange}, {KEY_TWO, ManeuverType::Overtake}, {KEY_THREE, ManeuverType::GoAround},
                            {KEY_FOUR, ManeuverType::Swerve},    {KEY_FIVE, ManeuverType::Turn},    {KEY_SIX, ManeuverType::UTurn},
                            {KEY_SEVEN, ManeuverType::Reverse},  {KEY_EIGHT, ManeuverType::PullOver}};
        for (const auto& item : maneuverKeys) {
            if (IsKeyPressed(item.key) && !manual) planner.requestManeuver(item.type);
        }
        if (IsKeyPressed(KEY_NINE)) planner.requestResume();
        if (IsKeyPressed(KEY_P)) spawnAhead("person");
        if (IsKeyPressed(KEY_N)) spawnAhead("animal");
        if (IsKeyPressed(KEY_V)) spawnAhead("vehicle");
        if (IsKeyPressed(KEY_O)) spawnAhead("obstacle");
        if (IsKeyPressed(KEY_C)) clearSpawned();

        // Scripted events, for automated runs.
        bool takeShot = false;
        while (nextEvent < events.size() && events[nextEvent].at <= now) {
            const std::string& action = events[nextEvent++].action;
            if (action == "shot") takeShot = true;
            else if (action == "clear") clearSpawned();
            else if (action == "pullover") planner.requestManeuver(ManeuverType::PullOver);
            else if (action == "uturn") planner.requestManeuver(ManeuverType::UTurn);
            else spawnAhead(action);
        }

        ManualInput input;
        input.enabled = manual;
        if (manual) {
            input.throttle = IsKeyDown(KEY_W) || IsKeyDown(KEY_UP) ? 1.0f : 0.0f;
            input.brake = IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN) ? 1.0f : 0.0f;
            input.steer = (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT) ? 1.0f : 0.0f) - (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT) ? 1.0f : 0.0f);
            input.reverse = manualReverse;
        }
        bus.publish(topic::kManual, input);

        // One pass of the driving modules.
        bus.dispatch();
        perception.tick(now);
        bus.dispatch();
        maneuver.tick();
        planner.tick(now);
        bus.dispatch();
        controller.tick(dt);
        bus.dispatch();
        vehicle.step(now, dt);
        bus.dispatch();

        const VehicleState& state = vehicle.state();
        const Vec2 forward = fromHeading(state.heading);

        // Collisions are counted from the real scene, purely as a report card.
        std::vector<int> nowTouching;
        for (const SceneObject& object : scene.objects()) {
            if (object.kind == ObjectKind::Building || object.cls == ObjectClass::Tree) continue;
            const Vec2 relative{object.position.x - state.position.x, object.position.z - state.position.y};
            if (length(relative) > 8) continue;
            const float radius = std::min((object.boundsMax.x - object.boundsMin.x) / 2, 1.5f);
            if (std::abs(dot(relative, forward)) < Vehicle::kLength / 2 + radius &&
                std::abs(dot(relative, rightOf(forward))) < Vehicle::kWidth / 2 + radius) {
                nowTouching.push_back(object.id);
                if (std::find(touching.begin(), touching.end(), object.id) == touching.end()) {
                    ++collisions;
                    bus.publish(topic::kVehicle, state, std::string("COLLISION with ") + className(object.cls));
                }
            }
        }
        touching = nowTouching;

        // Camera frame for the classifier.
        const Vec2 cameraSpot = state.position + forward * geometry.mountForward;
        driverCamera.position = {cameraSpot.x, geometry.mountHeight, cameraSpot.y};
        driverCamera.target = {cameraSpot.x + forward.x * 10, geometry.mountHeight, cameraSpot.y + forward.y * 10};
        if (now >= nextCapture) {
            nextCapture = now + kCapturePeriod;
            renderer.renderCamera(scene, driverCamera);
            Image image = renderer.readCameraPixels();
            ImageFormat(&image, PIXELFORMAT_UNCOMPRESSED_R8G8B8);
            CameraFrame frame;
            frame.id = ++frameId;
            frame.width = image.width;
            frame.height = image.height;
            const auto* bytes = static_cast<const uint8_t*>(image.data);
            frame.rgb = std::make_shared<std::vector<uint8_t>>(bytes, bytes + static_cast<size_t>(image.width * image.height * 3));
            frame.time = now;
            UnloadImage(image);
            bus.publish(topic::kCameraFrame, std::move(frame));
            if (now >= nextScore) {
                nextScore = now + kScorePeriod;
                scorer.addGroundTruth(frameId, renderer.renderGroundTruth(scene, driverCamera));
            }
        }

        // Chase view.
        const Vec2 wanted = state.position - forward * 10.0f;
        chaseFollow = chaseFollow + (wanted - chaseFollow) * std::min(dt * 3.0f, 1.0f);
        chaseCamera.position = {chaseFollow.x, 4.8f, chaseFollow.y};
        chaseCamera.target = {state.position.x + forward.x * 6, 1.0f, state.position.y + forward.y * 6};
        BeginTextureMode(chaseTarget);
        ClearBackground(Color{150, 180, 210, 255});
        renderer.drawWorld(scene, chaseCamera);
        BeginMode3D(chaseCamera);
        drawPath(path, path.reverse);
        drawCar(state, maneuver.status(), now);
        EndMode3D();
        EndTextureMode();

        DashboardData data;
        data.state = planner.state();
        data.reason = planner.reason();
        data.maneuver = maneuver.status();
        data.vehicle = state;
        data.command = command;
        data.sensors = sensors;
        if (boardLink) data.link = boardLink->status();
        data.visionConnected = visionLink.connected();
        data.visionHealthy = planner.visionHealthy();
        data.boardHealthy = planner.boardHealthy();
        data.visionRoundTripMs = visionLink.lastRoundTripMs();
        data.detectorMs = lastDetections.detectorMs;
        data.segmenterMs = lastDetections.segmenterMs;
        data.tracks = static_cast<int>(perception.tracks().size());
        data.collisions = collisions;
        data.manual = manual;
        data.now = now;
        data.scores = &scorer.scores();

        // The whole screen goes into one image so automated runs can save it too.
        BeginTextureMode(uiTarget);
        ClearBackground(BLACK);
        DrawTextureRec(chaseTarget.texture, {0, 0, kChaseWidth, -kChaseHeight}, {0, 0}, WHITE);
        const Texture2D driverTexture = renderer.cameraTexture();
        const Rectangle driverArea{0, kChaseHeight, 480, 270};
        DrawTexturePro(driverTexture, {0, 0, static_cast<float>(driverTexture.width), -static_cast<float>(driverTexture.height)}, driverArea, {0, 0}, 0, WHITE);
        const float scale = driverArea.width / static_cast<float>(geometry.width);
        for (const Detection& d : lastDetections.detections) {
            if (d.source == "segmenter") continue;
            const Rectangle box{driverArea.x + d.x0 * scale, driverArea.y + d.y0 * scale, (d.x1 - d.x0) * scale, (d.y1 - d.y0) * scale};
            DrawRectangleLinesEx(box, 1.5f, YELLOW);
            DrawText(className(d.cls), static_cast<int>(box.x), static_cast<int>(box.y) - 12, 11, YELLOW);
        }
        DrawText("CAMERA (what the classifier sees)", 8, kChaseHeight + 6, 14, WHITE);
        drawBusLog(bus, {480, kChaseHeight, 480, 270});
        drawDashboard(bus, data, {kChaseWidth, 0, kWindowWidth - kChaseWidth, kWindowHeight});
        EndTextureMode();

        BeginDrawing();
        DrawTextureRec(uiTarget.texture, {0, 0, kWindowWidth, -kWindowHeight}, {0, 0}, WHITE);
        EndDrawing();

        if (takeShot && !options.shotPrefix.empty()) {
            Image image = LoadImageFromTexture(uiTarget.texture);
            ImageFlipVertical(&image);
            ExportImage(image, (options.shotPrefix + std::to_string(shot++) + ".png").c_str());
            UnloadImage(image);
        }

        if (log.is_open() && now >= nextLog) {
            nextLog = now + kLogPeriod;
            char line[420];
            std::snprintf(line, sizeof(line),
                          "{\"t\":%.2f,\"state\":\"%s\",\"maneuver\":\"%s\",\"x\":%.1f,\"z\":%.1f,\"speed\":%.2f,\"lateral\":%.2f,"
                          "\"collisions\":%d,\"vision\":%s,\"board\":%s,\"tracks\":%d,\"front\":%d,\"frames\":%u,\"detections\":%zu}\n",
                          now, driveStateName(planner.state()), maneuverName(maneuver.status().type),
                          static_cast<double>(state.position.x), static_cast<double>(state.position.y), static_cast<double>(state.speed),
                          static_cast<double>(maneuver.status().lateral), collisions, planner.visionHealthy() ? "true" : "false",
                          planner.boardHealthy() ? "true" : "false", static_cast<int>(perception.tracks().size()), sensors.frontCm, frameId,
                          lastDetections.detections.size());
            log << line;
            log.flush();
        }
    }

    if (log.is_open()) {
        const auto& scores = scorer.scores();
        log << "{\"final\":true,\"collisions\":" << collisions << ",\"scores\":{";
        for (int i = 0; i < kClassCount; ++i) {
            log << (i ? "," : "") << "\"" << className(static_cast<ObjectClass>(i)) << "\":[" << scores[static_cast<size_t>(i)].hits << ","
                << scores[static_cast<size_t>(i)].objects << "]";
        }
        log << "}}\n";
    }
    CloseWindow();
    return 0;
}
