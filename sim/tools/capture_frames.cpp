// Renders a fixed drive down one road and writes frames plus ground truth, so the classifier can
// be scored on exactly what the simulation shows.
//
// Usage: capture_frames <project root> <output folder> [frame count] [seed]
#include <raylib.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "renderer.h"
#include "scene.h"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kCameraHeight = 1.4f;
constexpr float kCameraFovY = 40.0f;

// Lays out test objects along the road at row 1, travelling east. Each class gets the same share
// and a mix of positions: in the driving lane, in other lanes, on the shoulder and on the sidewalk.
void placeTestObjects(Scene& scene, const AssetLibrary& assets, float zRoad, float xStart, float xEnd, unsigned seed) {
    std::mt19937 rng(seed);
    const auto pickFrom = [&](const std::vector<int>& list) {
        return list[std::uniform_int_distribution<size_t>(0, list.size() - 1)(rng)];
    };
    const std::vector<int> people = assets.cutoutsOfClass(ObjectClass::Person);
    const std::vector<int> animals = assets.cutoutsOfClass(ObjectClass::Animal);
    const std::vector<int> vehicles = assets.cutoutsOfClass(ObjectClass::Vehicle);
    const std::vector<int> signs = assets.cutoutsOfClass(ObjectClass::TrafficControl);
    std::uniform_real_distribution<float> jitter(-1.0f, 1.0f);
    std::uniform_real_distribution<float> yaw(-25.0f, 25.0f);

    const float laneNear = RoadNetwork::laneOffset(0);
    const float laneFar = RoadNetwork::laneOffset(1);
    const float shoulder = RoadNetwork::shoulderOffset();
    const float sidewalk = RoadNetwork::kHalfRoad + 1.5f;
    const float oncoming = -RoadNetwork::laneOffset(0);

    int slot = 0;
    for (float x = xStart; x < xEnd; x += 9.0f, ++slot) {
        const float along = x + jitter(rng);
        switch (slot % 6) {
            case 0: {
                const float offsets[] = {laneFar, shoulder, sidewalk, laneNear};
                scene.addCutout(pickFrom(people), {along, 0, zRoad + offsets[(slot / 6) % 4]});
                break;
            }
            case 1: {
                const float offsets[] = {shoulder, laneFar, laneNear, sidewalk};
                scene.addCutout(pickFrom(animals), {along, 0, zRoad + offsets[(slot / 6) % 4]});
                break;
            }
            case 2: {
                const float offsets[] = {laneFar, oncoming, shoulder, laneNear};
                scene.addCutout(pickFrom(vehicles), {along, 0, zRoad + offsets[(slot / 6) % 4]});
                break;
            }
            case 3:
                scene.addCutout(pickFrom(signs), {along, 0, zRoad + RoadNetwork::kHalfRoad + 0.4f});
                break;
            case 4: {
                // Barrier, boulder, barrel, hydrant, bench and lamp, in turn.
                const int model = (slot / 6) % static_cast<int>(assets.models.size());
                const float offsets[] = {shoulder, laneFar};
                const float placed = model < 3 ? offsets[(slot / 6) % 2] : sidewalk - 1.0f;
                scene.addModel(model, {along, 0, zRoad + placed}, 90.0f + yaw(rng));
                break;
            }
            default:
                break;  // Trees and buildings already line the street.
        }
    }
}

const char* rectJson(const Rectangle& r, char* buffer, size_t size) {
    std::snprintf(buffer, size, "[%.1f,%.1f,%.1f,%.1f]", static_cast<double>(r.x), static_cast<double>(r.y),
                  static_cast<double>(r.x + r.width), static_cast<double>(r.y + r.height));
    return buffer;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: capture_frames <project root> <output folder> [frame count]\n");
        return 2;
    }
    const std::string root = argv[1];
    const std::filesystem::path out = argv[2];
    const int frames = argc > 3 ? std::stoi(argv[3]) : 200;
    const unsigned seed = argc > 4 ? static_cast<unsigned>(std::stoul(argv[4])) : 7;
    std::filesystem::create_directories(out);

    SetConfigFlags(FLAG_WINDOW_HIDDEN);
    SetTraceLogLevel(LOG_WARNING);
    InitWindow(64, 64, "capture");

    Renderer renderer(kWidth, kHeight);
    AssetLibrary assets;
    if (!assets.load(root, renderer.shader())) return 1;

    RoadNetwork roads(4, 3, 130.0f);
    Scene scene(roads, assets, seed);
    scene.populateStreets();
    const float zRoad = roads.node(0, 1).y;
    const float xEnd = roads.width();
    placeTestObjects(scene, assets, zRoad, 20.0f, xEnd - 20.0f, seed + 1);
    renderer.buildStatic(scene);

    Camera3D camera{};
    camera.up = {0, 1, 0};
    camera.fovy = kCameraFovY;
    camera.projection = CAMERA_PERSPECTIVE;

    std::ofstream truth(out / "ground_truth.jsonl");
    const float laneZ = zRoad + RoadNetwork::laneOffset(0);
    const float step = (xEnd - 40.0f) / static_cast<float>(frames);
    for (int frame = 0; frame < frames; ++frame) {
        const float x = 5.0f + static_cast<float>(frame) * step;
        camera.position = {x, kCameraHeight, laneZ};
        camera.target = {x + 10.0f, kCameraHeight, laneZ};

        renderer.renderCamera(scene, camera);
        const std::vector<VisibleObject> visible = renderer.renderGroundTruth(scene, camera);

        Image image = renderer.readCameraPixels();
        ImageFormat(&image, PIXELFORMAT_UNCOMPRESSED_R8G8B8);
        char name[32];
        std::snprintf(name, sizeof(name), "%04d.png", frame);
        ExportImage(image, (out / name).string().c_str());
        UnloadImage(image);

        truth << "{\"frame\":\"" << name << "\",\"objects\":[";
        for (size_t i = 0; i < visible.size(); ++i) {
            const VisibleObject& v = visible[i];
            char a[64], b[64];
            truth << (i ? "," : "") << "{\"id\":" << v.id << ",\"class\":\"" << className(v.cls)
                  << "\",\"distance\":" << v.distance << ",\"visible_px\":" << v.visiblePixels
                  << ",\"box\":" << rectJson(v.visibleBox, a, sizeof(a))
                  << ",\"projected\":" << rectJson(v.projectedBox, b, sizeof(b))
                  << ",\"in_frame\":" << (v.fullyInFrame ? "true" : "false")
                  << ",\"visible_fraction\":" << v.visibleFraction << "}";
        }
        truth << "]}\n";
    }
    CloseWindow();
    std::printf("Wrote %d frames to %s\n", frames, out.string().c_str());
    return 0;
}
