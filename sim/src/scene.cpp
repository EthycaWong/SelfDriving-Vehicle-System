#include "scene.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include <raymath.h>

namespace {

constexpr float kFloorHeight = 3.5f;
constexpr float kFacadeTileWidth = 7.0f;

template <typename T>
T* copyToRaylib(const std::vector<T>& data) {
    // UnloadMesh frees these with raylib's allocator.
    T* memory = static_cast<T*>(MemAlloc(static_cast<unsigned int>(data.size() * sizeof(T))));
    std::memcpy(memory, data.data(), data.size() * sizeof(T));
    return memory;
}

}  // namespace

void MeshBuilder::quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector2 uvA, Vector2 uvB, Vector2 uvC, Vector2 uvD) {
    for (const Vector3& p : {a, b, c, a, c, d}) {
        vertices_.insert(vertices_.end(), {p.x, p.y, p.z});
    }
    for (const Vector2& t : {uvA, uvB, uvC, uvA, uvC, uvD}) {
        texcoords_.insert(texcoords_.end(), {t.x, t.y});
    }
    const Vector3 normal = Vector3Normalize(Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(c, a)));
    for (int i = 0; i < 6; ++i) {
        normals_.insert(normals_.end(), {normal.x, normal.y, normal.z});
    }
}

void MeshBuilder::groundRect(float x0, float z0, float x1, float z1, float y, float tile) {
    quad({x0, y, z0}, {x0, y, z1}, {x1, y, z1}, {x1, y, z0}, {x0 / tile, z0 / tile}, {x0 / tile, z1 / tile},
         {x1 / tile, z1 / tile}, {x1 / tile, z0 / tile});
}

Mesh MeshBuilder::build() {
    Mesh mesh{};
    mesh.vertexCount = static_cast<int>(vertices_.size() / 3);
    mesh.triangleCount = mesh.vertexCount / 3;
    mesh.vertices = copyToRaylib(vertices_);
    mesh.texcoords = copyToRaylib(texcoords_);
    mesh.normals = copyToRaylib(normals_);
    UploadMesh(&mesh, false);
    return mesh;
}

Scene::Scene(const RoadNetwork& roads, const AssetLibrary& assets, unsigned seed)
    : roads_(roads), assets_(assets), rng_(seed) {}

int Scene::addCutout(int cutoutIndex, Vector3 position, float scale) {
    const CutoutAsset& asset = assets_.cutouts[static_cast<size_t>(cutoutIndex)];
    const float height = asset.heightM * scale;
    const float halfWidth = height * asset.aspect / 2;
    SceneObject object;
    object.id = nextId_++;
    object.cls = asset.cls;
    object.kind = ObjectKind::Cutout;
    object.asset = cutoutIndex;
    object.position = position;
    object.scale = scale;
    object.boundsMin = {position.x - halfWidth, position.y + asset.mountM, position.z - halfWidth};
    object.boundsMax = {position.x + halfWidth, position.y + asset.mountM + height, position.z + halfWidth};
    objects_.push_back(object);
    return object.id;
}

int Scene::addModel(int modelIndex, Vector3 position, float yawDegrees, float scale) {
    const ModelAsset& asset = assets_.models[static_cast<size_t>(modelIndex)];
    const float radius = scale * std::max({std::abs(asset.bounds.min.x), std::abs(asset.bounds.max.x),
                                           std::abs(asset.bounds.min.z), std::abs(asset.bounds.max.z)});
    SceneObject object;
    object.id = nextId_++;
    object.cls = asset.cls;
    object.kind = ObjectKind::Model;
    object.asset = modelIndex;
    object.position = position;
    object.scale = scale;
    object.yawDegrees = yawDegrees;
    object.boundsMin = {position.x - radius, position.y + asset.bounds.min.y * scale, position.z - radius};
    object.boundsMax = {position.x + radius, position.y + asset.bounds.max.y * scale, position.z + radius};
    objects_.push_back(object);
    return object.id;
}

int Scene::addBuilding(Vector3 minCorner, Vector3 maxCorner, int facade) {
    const float x0 = minCorner.x, x1 = maxCorner.x, z0 = minCorner.z, z1 = maxCorner.z, y1 = maxCorner.y;
    // The tile holds one floor, so the walls show whole storeys and whole window bays.
    const float floors = y1 / kFloorHeight;
    const float bayX = std::round((x1 - x0) / kFacadeTileWidth);
    const float bayZ = std::round((z1 - z0) / kFacadeTileWidth);

    MeshBuilder walls;
    walls.quad({x0, y1, z0}, {x0, 0, z0}, {x0, 0, z1}, {x0, y1, z1}, {0, 0}, {0, floors}, {bayZ, floors}, {bayZ, 0});
    walls.quad({x0, y1, z1}, {x0, 0, z1}, {x1, 0, z1}, {x1, y1, z1}, {0, 0}, {0, floors}, {bayX, floors}, {bayX, 0});
    walls.quad({x1, y1, z1}, {x1, 0, z1}, {x1, 0, z0}, {x1, y1, z0}, {0, 0}, {0, floors}, {bayZ, floors}, {bayZ, 0});
    walls.quad({x1, y1, z0}, {x1, 0, z0}, {x0, 0, z0}, {x0, y1, z0}, {0, 0}, {0, floors}, {bayX, floors}, {bayX, 0});
    MeshBuilder roof;
    roof.groundRect(x0, z0, x1, z1, y1, 4.0f);

    BuildingMesh mesh;
    mesh.walls = walls.build();
    mesh.roof = roof.build();
    mesh.facade = facade;
    buildings_.push_back(mesh);

    SceneObject object;
    object.id = nextId_++;
    object.cls = ObjectClass::Building;
    object.kind = ObjectKind::Building;
    object.asset = static_cast<int>(buildings_.size()) - 1;
    object.position = {(x0 + x1) / 2, 0, (z0 + z1) / 2};
    object.boundsMin = {x0, 0, z0};
    object.boundsMax = {x1, y1, z1};
    objects_.push_back(object);
    return object.id;
}

void Scene::removeObject(int id) {
    std::erase_if(objects_, [id](const SceneObject& o) { return o.id == id; });
}

void Scene::moveObject(int id, Vector3 position) {
    for (SceneObject& object : objects_) {
        if (object.id != id) continue;
        const Vector3 delta{position.x - object.position.x, position.y - object.position.y,
                            position.z - object.position.z};
        object.position = position;
        object.boundsMin = {object.boundsMin.x + delta.x, object.boundsMin.y + delta.y, object.boundsMin.z + delta.z};
        object.boundsMax = {object.boundsMax.x + delta.x, object.boundsMax.y + delta.y, object.boundsMax.z + delta.z};
        return;
    }
}

const SceneObject* Scene::find(int id) const {
    for (const SceneObject& object : objects_) {
        if (object.id == id) return &object;
    }
    return nullptr;
}

void Scene::addBlockSide(Vector2 start, Vector2 along, Vector2 outward, float length) {
    std::uniform_real_distribution<float> width(21.0f, 35.0f);
    std::uniform_int_distribution<int> floors(3, 9);
    std::uniform_int_distribution<int> facade(0, static_cast<int>(assets_.facades.size()) - 1);
    std::uniform_real_distribution<float> depth(14.0f, 22.0f);
    std::uniform_real_distribution<float> gap(2.0f, 8.0f);
    std::uniform_real_distribution<float> treeScale(0.9f, 1.5f);
    const std::vector<int> trees = assets_.cutoutsOfClass(ObjectClass::Tree);
    std::uniform_int_distribution<int> treePick(0, static_cast<int>(trees.size()) - 1);

    const float setback = RoadNetwork::kHalfStreet + 0.5f;
    float cursor = gap(rng_);
    while (cursor < length - 12.0f) {
        const float w = std::min(width(rng_), length - cursor);
        const float d = depth(rng_);
        const Vector2 a{start.x + along.x * cursor + outward.x * setback,
                        start.y + along.y * cursor + outward.y * setback};
        const Vector2 b{a.x + along.x * w + outward.x * d, a.y + along.y * w + outward.y * d};
        addBuilding({std::min(a.x, b.x), 0, std::min(a.y, b.y)},
                    {std::max(a.x, b.x), static_cast<float>(floors(rng_)) * kFloorHeight, std::max(a.y, b.y)},
                    facade(rng_));
        cursor += w + gap(rng_);
    }

    const float treeOffset = RoadNetwork::kHalfRoad + 1.0f;
    for (float t = 14.0f; t < length - 6.0f; t += 24.0f) {
        addCutout(trees[static_cast<size_t>(treePick(rng_))],
                  {start.x + along.x * t + outward.x * treeOffset, 0, start.y + along.y * t + outward.y * treeOffset},
                  treeScale(rng_));
    }
}

void Scene::populateStreets() {
    const float inset = RoadNetwork::kHalfStreet;
    const float length = roads_.spacing() - 2 * inset;
    for (int column = 0; column < roads_.columns(); ++column) {
        for (int row = 0; row < roads_.rows(); ++row) {
            const Vector2 n = roads_.node(column, row);
            if (column + 1 < roads_.columns()) {
                addBlockSide({n.x + inset, n.y}, {1, 0}, {0, 1}, length);
                addBlockSide({n.x + inset, n.y}, {1, 0}, {0, -1}, length);
            }
            if (row + 1 < roads_.rows()) {
                addBlockSide({n.x, n.y + inset}, {0, 1}, {1, 0}, length);
                addBlockSide({n.x, n.y + inset}, {0, 1}, {-1, 0}, length);
            }
        }
    }
}
