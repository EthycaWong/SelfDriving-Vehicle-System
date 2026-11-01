#pragma once

#include <raylib.h>

#include <random>
#include <vector>

#include "assets.h"
#include "road_network.h"

enum class ObjectKind { Cutout, Model, Building };

// One thing placed in the world. `id` is what the ground-truth pass draws, and it stays on the
// simulation side: it's never sent to the classifier or the Planner.
struct SceneObject {
    int id = 0;
    ObjectClass cls = ObjectClass::Obstruction;
    ObjectKind kind = ObjectKind::Cutout;
    int asset = 0;        // Index into cutouts or models; for buildings, the building mesh index.
    Vector3 position{};   // On the ground, at the object's center.
    float scale = 1.0f;   // Cutouts: multiplies the asset's real height. Models: uniform scale.
    float yawDegrees = 0; // Models only. Cutouts always face the camera.
    Vector3 boundsMin{};
    Vector3 boundsMax{};
};

struct BuildingMesh {
    Mesh walls{};
    Mesh roof{};
    int facade = 0;
};

class Scene {
public:
    Scene(const RoadNetwork& roads, const AssetLibrary& assets, unsigned seed);

    const RoadNetwork& roads() const { return roads_; }
    const AssetLibrary& assets() const { return assets_; }
    const std::vector<SceneObject>& objects() const { return objects_; }
    const std::vector<BuildingMesh>& buildings() const { return buildings_; }

    // Lines every block with buildings and street trees.
    void populateStreets();

    int addCutout(int cutoutIndex, Vector3 position, float scale = 1.0f);
    int addModel(int modelIndex, Vector3 position, float yawDegrees, float scale = 1.0f);
    int addBuilding(Vector3 minCorner, Vector3 maxCorner, int facade);
    void removeObject(int id);
    void moveObject(int id, Vector3 position);

    const SceneObject* find(int id) const;

private:
    void addBlockSide(Vector2 start, Vector2 along, Vector2 outward, float length);

    const RoadNetwork& roads_;
    const AssetLibrary& assets_;
    std::vector<SceneObject> objects_;
    std::vector<BuildingMesh> buildings_;
    std::mt19937 rng_;
    int nextId_ = 1;
};

// Collects textured triangles in world space, with texture coordinates measured in metres.
class MeshBuilder {
public:
    // Corners go counter-clockwise when seen from the side that should be visible.
    void quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector2 uvA, Vector2 uvB, Vector2 uvC, Vector2 uvD);
    // A rectangle on the ground at height y, with the texture repeating every `tile` metres.
    void groundRect(float x0, float z0, float x1, float z1, float y, float tile);
    Mesh build();

private:
    std::vector<float> vertices_;
    std::vector<float> texcoords_;
    std::vector<float> normals_;
};
