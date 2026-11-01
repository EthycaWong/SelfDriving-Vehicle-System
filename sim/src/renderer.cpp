#include "renderer.h"

#include <raymath.h>
#include <rlgl.h>

#include <algorithm>
#include <cmath>
#include <map>

namespace {

constexpr int kSuperSample = 2;
constexpr float kFarPlane = 2500.0f;

const char* kVertexShader = R"(#version 330
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec4 vertexColor;
uniform mat4 mvp;
out vec2 fragTexCoord;
out vec4 fragColor;
void main() {
    fragTexCoord = vertexTexCoord;
    fragColor = vertexColor;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)";

const char* kFragmentShader = R"(#version 330
in vec2 fragTexCoord;
in vec4 fragColor;
uniform sampler2D texture0;
uniform vec4 colDiffuse;
uniform vec4 idColor;
uniform int idMode;
out vec4 finalColor;
void main() {
    vec4 texel = texture(texture0, fragTexCoord);
    if (idMode == 1) {
        if (texel.a < 0.5) discard;
        finalColor = idColor;
    } else {
        if (texel.a < 0.05) discard;
        finalColor = texel * colDiffuse * fragColor;
    }
}
)";

// The ID pass stores an object's ID in the red and green channels.
Color idToColor(int id) {
    return {static_cast<unsigned char>(id & 0xFF), static_cast<unsigned char>((id >> 8) & 0xFF), 200, 255};
}

Mesh buildUnitQuad() {
    MeshBuilder builder;
    // x runs from -0.5 to 0.5 and y from 0 to 1. Faces +Z so it can be turned toward the camera.
    builder.quad({-0.5f, 1, 0}, {-0.5f, 0, 0}, {0.5f, 0, 0}, {0.5f, 1, 0}, {0, 0}, {0, 1}, {1, 1}, {1, 0});
    return builder.build();
}

}  // namespace

Renderer::Renderer(int width, int height) : width_(width), height_(height) {
    shader_ = LoadShaderFromMemory(kVertexShader, kFragmentShader);
    idColorLoc_ = GetShaderLocation(shader_, "idColor");
    idModeLoc_ = GetShaderLocation(shader_, "idMode");
    superTarget_ = LoadRenderTexture(width * kSuperSample, height * kSuperSample);
    SetTextureFilter(superTarget_.texture, TEXTURE_FILTER_BILINEAR);
    cameraTarget_ = LoadRenderTexture(width, height);
    idTarget_ = LoadRenderTexture(width, height);
    SetTextureFilter(idTarget_.texture, TEXTURE_FILTER_POINT);
    material_ = LoadMaterialDefault();
    material_.shader = shader_;
    skyMesh_ = GenMeshSphere(kFarPlane * 0.8f, 32, 48);
    quadMesh_ = buildUnitQuad();
}

Renderer::~Renderer() {
    UnloadRenderTexture(superTarget_);
    UnloadRenderTexture(cameraTarget_);
    UnloadRenderTexture(idTarget_);
    UnloadShader(shader_);
}

void Renderer::buildStatic(const Scene& scene) {
    const RoadNetwork& roads = scene.roads();
    const float halfRoad = RoadNetwork::kHalfRoad;
    const float halfStreet = RoadNetwork::kHalfStreet;
    const float width = roads.width(), depth = roads.depth();

    MeshBuilder grass, asphalt, sidewalk, whiteLines, yellowLines;
    grass.groundRect(-kFarPlane, -kFarPlane, width + kFarPlane, depth + kFarPlane, -0.03f, 4.0f);

    const auto dashedAlongX = [&](float x0, float x1, float z) {
        for (float x = x0; x < x1 - 3.0f; x += 9.0f) whiteLines.groundRect(x, z - 0.08f, x + 3.0f, z + 0.08f, 0.012f, 1);
    };
    const auto dashedAlongZ = [&](float z0, float z1, float x) {
        for (float z = z0; z < z1 - 3.0f; z += 9.0f) whiteLines.groundRect(x - 0.08f, z, x + 0.08f, z + 3.0f, 0.012f, 1);
    };

    for (int row = 0; row < roads.rows(); ++row) {
        const float z = roads.node(0, row).y;
        for (int column = 0; column + 1 < roads.columns(); ++column) {
            const float x0 = roads.node(column, row).x + halfRoad, x1 = roads.node(column + 1, row).x - halfRoad;
            asphalt.groundRect(x0, z - halfRoad, x1, z + halfRoad, 0.0f, 6.0f);
            for (float side : {-1.0f, 1.0f}) {
                const float a = z + side * halfRoad, b = z + side * halfStreet;
                sidewalk.groundRect(x0, std::min(a, b), x1, std::max(a, b), 0.02f, 3.0f);
                dashedAlongX(x0 + 3, x1 - 3, z + side * RoadNetwork::kLaneWidth);
                const float edge = z + side * 2 * RoadNetwork::kLaneWidth;
                whiteLines.groundRect(x0, edge - 0.08f, x1, edge + 0.08f, 0.012f, 1);
                yellowLines.groundRect(x0, z + side * 0.12f - 0.06f, x1, z + side * 0.12f + 0.06f, 0.012f, 1);
            }
        }
    }
    for (int column = 0; column < roads.columns(); ++column) {
        const float x = roads.node(column, 0).x;
        for (int row = 0; row + 1 < roads.rows(); ++row) {
            const float z0 = roads.node(column, row).y + halfRoad, z1 = roads.node(column, row + 1).y - halfRoad;
            asphalt.groundRect(x - halfRoad, z0, x + halfRoad, z1, 0.0f, 6.0f);
            for (float side : {-1.0f, 1.0f}) {
                const float a = x + side * halfRoad, b = x + side * halfStreet;
                sidewalk.groundRect(std::min(a, b), z0, std::max(a, b), z1, 0.02f, 3.0f);
                dashedAlongZ(z0 + 3, z1 - 3, x + side * RoadNetwork::kLaneWidth);
                const float edge = x + side * 2 * RoadNetwork::kLaneWidth;
                whiteLines.groundRect(edge - 0.08f, z0, edge + 0.08f, z1, 0.012f, 1);
                yellowLines.groundRect(x + side * 0.12f - 0.06f, z0, x + side * 0.12f + 0.06f, z1, 0.012f, 1);
            }
        }
    }
    for (int column = 0; column < roads.columns(); ++column) {
        for (int row = 0; row < roads.rows(); ++row) {
            const Vector2 n = roads.node(column, row);
            asphalt.groundRect(n.x - halfRoad, n.y - halfRoad, n.x + halfRoad, n.y + halfRoad, 0.0f, 6.0f);
        }
    }

    grassMesh_ = grass.build();
    asphaltMesh_ = asphalt.build();
    sidewalkMesh_ = sidewalk.build();
    whiteLineMesh_ = whiteLines.build();
    yellowLineMesh_ = yellowLines.build();
}

void Renderer::setIdUniform(int id, bool idPass) {
    const int mode = idPass ? 1 : 0;
    SetShaderValue(shader_, idModeLoc_, &mode, SHADER_UNIFORM_INT);
    const Color c = idToColor(id);
    const float color[4] = {static_cast<float>(c.r) / 255.0f, static_cast<float>(c.g) / 255.0f,
                            static_cast<float>(c.b) / 255.0f, 1.0f};
    SetShaderValue(shader_, idColorLoc_, color, SHADER_UNIFORM_VEC4);
}

void Renderer::drawSky(const Scene& scene, const Camera3D& camera) {
    material_.maps[MATERIAL_MAP_DIFFUSE].texture = scene.assets().sky;
    material_.maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
    rlDisableBackfaceCulling();
    rlDisableDepthMask();
    DrawMesh(skyMesh_, material_, MatrixTranslate(camera.position.x, 0, camera.position.z));
    rlEnableDepthMask();
    rlEnableBackfaceCulling();
}

void Renderer::drawGround(const Scene& scene) {
    const AssetLibrary& assets = scene.assets();
    const auto draw = [&](const Mesh& mesh, Texture2D texture, Color tint) {
        material_.maps[MATERIAL_MAP_DIFFUSE].texture = texture;
        material_.maps[MATERIAL_MAP_DIFFUSE].color = tint;
        DrawMesh(mesh, material_, MatrixIdentity());
    };
    draw(grassMesh_, assets.grass, WHITE);
    draw(asphaltMesh_, assets.asphalt, Color{170, 170, 170, 255});
    draw(sidewalkMesh_, assets.sidewalk, WHITE);
    draw(whiteLineMesh_, assets.white, Color{235, 235, 230, 255});
    draw(yellowLineMesh_, assets.white, Color{225, 180, 40, 255});
}

void Renderer::drawObjects(const Scene& scene, const Camera3D& camera, bool idPass) {
    const AssetLibrary& assets = scene.assets();

    // Solid things first, then cutouts from far to near so their soft edges blend correctly.
    std::vector<const SceneObject*> cutouts;
    for (const SceneObject& object : scene.objects()) {
        setIdUniform(object.id, idPass);
        if (object.kind == ObjectKind::Building) {
            const BuildingMesh& mesh = scene.buildings()[static_cast<size_t>(object.asset)];
            material_.maps[MATERIAL_MAP_DIFFUSE].texture = assets.facades[static_cast<size_t>(mesh.facade)];
            material_.maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
            rlDisableBackfaceCulling();
            DrawMesh(mesh.walls, material_, MatrixIdentity());
            material_.maps[MATERIAL_MAP_DIFFUSE].texture = assets.sidewalk;
            material_.maps[MATERIAL_MAP_DIFFUSE].color = Color{120, 120, 120, 255};
            DrawMesh(mesh.roof, material_, MatrixIdentity());
            rlEnableBackfaceCulling();
        } else if (object.kind == ObjectKind::Model) {
            const Model& model = assets.models[static_cast<size_t>(object.asset)].model;
            DrawModelEx(model, object.position, {0, 1, 0}, object.yawDegrees,
                        {object.scale, object.scale, object.scale}, WHITE);
        } else {
            cutouts.push_back(&object);
        }
    }

    std::sort(cutouts.begin(), cutouts.end(), [&](const SceneObject* a, const SceneObject* b) {
        return Vector3DistanceSqr(a->position, camera.position) > Vector3DistanceSqr(b->position, camera.position);
    });

    rlDisableBackfaceCulling();
    if (!idPass) BeginBlendMode(BLEND_ALPHA);
    for (const SceneObject* object : cutouts) {
        const CutoutAsset& asset = assets.cutouts[static_cast<size_t>(object->asset)];
        const float height = asset.heightM * object->scale;
        const float width = height * asset.aspect;
        const float base = object->position.y + asset.mountM;

        material_.maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
        if (asset.mountM > 0) {
            // The pole is scenery with no ID, so it isn't scored as part of the sign.
            setIdUniform(0, idPass);
            material_.maps[MATERIAL_MAP_DIFFUSE].texture = assets.white;
            material_.maps[MATERIAL_MAP_DIFFUSE].color = Color{90, 92, 95, 255};
            const float yaw = std::atan2(camera.position.x - object->position.x,
                                         camera.position.z - object->position.z);
            const Matrix pole = MatrixMultiply(MatrixMultiply(MatrixScale(0.09f, base + height * 0.1f, 1), MatrixRotateY(yaw)),
                                               MatrixTranslate(object->position.x, object->position.y, object->position.z));
            DrawMesh(quadMesh_, material_, pole);
            material_.maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
        }

        setIdUniform(object->id, idPass);
        material_.maps[MATERIAL_MAP_DIFFUSE].texture = asset.texture;
        const float yaw = std::atan2(camera.position.x - object->position.x, camera.position.z - object->position.z);
        const Matrix transform = MatrixMultiply(MatrixMultiply(MatrixScale(width, height, 1), MatrixRotateY(yaw)),
                                                MatrixTranslate(object->position.x, base, object->position.z));
        DrawMesh(quadMesh_, material_, transform);
    }
    if (!idPass) EndBlendMode();
    rlEnableBackfaceCulling();
}

void Renderer::drawWorld(const Scene& scene, const Camera3D& camera) {
    rlSetClipPlanes(0.1, kFarPlane);
    BeginMode3D(camera);
    setIdUniform(0, false);
    drawSky(scene, camera);
    drawGround(scene);
    drawObjects(scene, camera, false);
    EndMode3D();
}

void Renderer::renderCamera(const Scene& scene, const Camera3D& camera) {
    BeginTextureMode(superTarget_);
    ClearBackground(Color{150, 180, 210, 255});
    drawWorld(scene, camera);
    EndTextureMode();

    // Averaging the larger image down keeps cutout and building edges from looking jagged.
    BeginTextureMode(cameraTarget_);
    const Rectangle source{0, 0, static_cast<float>(superTarget_.texture.width),
                           -static_cast<float>(superTarget_.texture.height)};
    DrawTexturePro(superTarget_.texture, source, {0, 0, static_cast<float>(width_), static_cast<float>(height_)},
                   {0, 0}, 0, WHITE);
    EndTextureMode();
}

Image Renderer::readCameraPixels() const {
    Image image = LoadImageFromTexture(cameraTarget_.texture);
    ImageFlipVertical(&image);
    return image;
}

std::vector<VisibleObject> Renderer::renderGroundTruth(const Scene& scene, const Camera3D& camera) {
    BeginTextureMode(idTarget_);
    ClearBackground(BLANK);
    rlSetClipPlanes(0.1, kFarPlane);
    BeginMode3D(camera);
    BeginBlendMode(BLEND_CUSTOM);
    rlSetBlendFactors(RL_ONE, RL_ZERO, RL_FUNC_ADD);
    rlSetBlendMode(RL_BLEND_CUSTOM);
    drawObjects(scene, camera, true);
    EndBlendMode();
    EndMode3D();
    EndTextureMode();
    setIdUniform(0, false);

    Image image = LoadImageFromTexture(idTarget_.texture);
    ImageFlipVertical(&image);
    Color* pixels = LoadImageColors(image);

    struct Extent {
        int count = 0, x0 = 1 << 30, y0 = 1 << 30, x1 = -1, y1 = -1;
    };
    std::map<int, Extent> extents;
    for (int y = 0; y < image.height; ++y) {
        for (int x = 0; x < image.width; ++x) {
            const Color& p = pixels[y * image.width + x];
            if (p.a == 0 || p.b != 200) continue;
            const int id = p.r | (p.g << 8);
            if (id == 0) continue;
            Extent& e = extents[id];
            ++e.count;
            e.x0 = std::min(e.x0, x);
            e.y0 = std::min(e.y0, y);
            e.x1 = std::max(e.x1, x);
            e.y1 = std::max(e.y1, y);
        }
    }
    UnloadImageColors(pixels);
    UnloadImage(image);

    const Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
    std::vector<VisibleObject> result;
    for (const auto& [id, e] : extents) {
        const SceneObject* object = scene.find(id);
        if (object == nullptr) continue;

        VisibleObject visible;
        visible.id = id;
        visible.cls = object->cls;
        visible.visiblePixels = e.count;
        visible.visibleBox = {static_cast<float>(e.x0), static_cast<float>(e.y0), static_cast<float>(e.x1 - e.x0 + 1),
                              static_cast<float>(e.y1 - e.y0 + 1)};
        // Distance to the nearest point of the object's box, so a long building counts when
        // its near end is close even though its center is far away.
        const Vector3 nearest{Clamp(camera.position.x, object->boundsMin.x, object->boundsMax.x),
                              Clamp(camera.position.y, object->boundsMin.y, object->boundsMax.y),
                              Clamp(camera.position.z, object->boundsMin.z, object->boundsMax.z)};
        visible.distance = Vector3Distance(camera.position, nearest);

        float minX = 1e9f, minY = 1e9f, maxX = -1e9f, maxY = -1e9f;
        bool behind = false;
        for (int corner = 0; corner < 8; ++corner) {
            const Vector3 p{corner & 1 ? object->boundsMax.x : object->boundsMin.x,
                            corner & 2 ? object->boundsMax.y : object->boundsMin.y,
                            corner & 4 ? object->boundsMax.z : object->boundsMin.z};
            if (Vector3DotProduct(Vector3Subtract(p, camera.position), forward) < 0.2f) {
                behind = true;
                continue;
            }
            const Vector2 s = GetWorldToScreenEx(p, camera, width_, height_);
            minX = std::min(minX, s.x);
            minY = std::min(minY, s.y);
            maxX = std::max(maxX, s.x);
            maxY = std::max(maxY, s.y);
        }
        visible.projectedBox = {minX, minY, maxX - minX, maxY - minY};
        visible.fullyInFrame = !behind && minX >= 0 && minY >= 0 && maxX < static_cast<float>(width_) &&
                               maxY < static_cast<float>(height_);
        result.push_back(visible);
    }
    return result;
}
