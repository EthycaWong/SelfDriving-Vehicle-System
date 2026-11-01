#include "assets.h"

#include <array>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace {

constexpr std::array<const char*, kClassCount> kClassNames = {
    "person", "animal", "vehicle", "traffic_control", "tree", "building", "obstruction"};

Texture2D loadTiled(const std::string& path) {
    Texture2D texture = LoadTexture(path.c_str());
    GenTextureMipmaps(&texture);
    SetTextureFilter(texture, TEXTURE_FILTER_TRILINEAR);
    SetTextureWrap(texture, TEXTURE_WRAP_REPEAT);
    return texture;
}

Texture2D loadCutout(const std::string& path) {
    Texture2D texture = LoadTexture(path.c_str());
    GenTextureMipmaps(&texture);
    SetTextureFilter(texture, TEXTURE_FILTER_TRILINEAR);
    SetTextureWrap(texture, TEXTURE_WRAP_CLAMP);
    return texture;
}

bool readCutoutManifest(const std::string& root, const std::string& file, std::vector<CutoutAsset>& out) {
    std::ifstream in(root + "/" + file);
    if (!in) {
        std::fprintf(stderr, "Missing %s/%s\n", root.c_str(), file.c_str());
        return false;
    }
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::stringstream row(line);
        std::string path, cls, category, height, aspect, mount;
        std::getline(row, path, '\t');
        std::getline(row, cls, '\t');
        std::getline(row, category, '\t');
        std::getline(row, height, '\t');
        std::getline(row, aspect, '\t');
        std::getline(row, mount, '\t');
        CutoutAsset asset;
        asset.texture = loadCutout(root + "/" + path);
        asset.cls = classFromName(cls);
        asset.category = category;
        asset.heightM = std::stof(height);
        asset.aspect = std::stof(aspect);
        asset.mountM = std::stof(mount);
        out.push_back(std::move(asset));
    }
    return true;
}

}  // namespace

const char* className(ObjectClass cls) { return kClassNames[static_cast<size_t>(cls)]; }

ObjectClass classFromName(const std::string& name) {
    for (int i = 0; i < kClassCount; ++i) {
        if (name == kClassNames[static_cast<size_t>(i)]) return static_cast<ObjectClass>(i);
    }
    return ObjectClass::Obstruction;
}

bool AssetLibrary::load(const std::string& root, Shader shader) {
    if (!readCutoutManifest(root, "assets/cutouts/manifest.tsv", cutouts)) return false;
    if (!readCutoutManifest(root, "assets/cutouts/trees.tsv", cutouts)) return false;

    const std::array<const char*, 6> modelNames = {"concrete_road_barrier", "boulder_01", "Barrel_01",
                                                   "fire_hydrant", "painted_wooden_bench", "street_lamp_01"};
    for (const char* name : modelNames) {
        ModelAsset asset;
        asset.name = name;
        const std::string path = root + "/assets/models/" + name + "/" + name + ".gltf";
        asset.model = LoadModel(path.c_str());
        if (asset.model.meshCount == 0) {
            std::fprintf(stderr, "Could not load %s\n", path.c_str());
            return false;
        }
        for (int i = 0; i < asset.model.materialCount; ++i) {
            asset.model.materials[i].shader = shader;
            Texture2D& diffuse = asset.model.materials[i].maps[MATERIAL_MAP_DIFFUSE].texture;
            GenTextureMipmaps(&diffuse);
            SetTextureFilter(diffuse, TEXTURE_FILTER_TRILINEAR);
        }
        asset.bounds = GetModelBoundingBox(asset.model);
        models.push_back(std::move(asset));
    }

    for (int i = 0; i < 4; ++i) {
        facades.push_back(loadTiled(root + "/assets/textures/facade_" + std::to_string(i) + ".png"));
    }
    asphalt = loadTiled(root + "/assets/textures/asphalt_02.jpg");
    sidewalk = loadTiled(root + "/assets/textures/concrete_pavement.jpg");
    grass = loadTiled(root + "/assets/textures/sparse_grass.jpg");
    sky = LoadTexture((root + "/assets/textures/sky.jpg").c_str());
    SetTextureFilter(sky, TEXTURE_FILTER_BILINEAR);
    Image pixel = GenImageColor(1, 1, WHITE);
    white = LoadTextureFromImage(pixel);
    UnloadImage(pixel);
    return true;
}

std::vector<int> AssetLibrary::cutoutsOfClass(ObjectClass cls) const {
    std::vector<int> result;
    for (size_t i = 0; i < cutouts.size(); ++i) {
        if (cutouts[i].cls == cls) result.push_back(static_cast<int>(i));
    }
    return result;
}

std::vector<int> AssetLibrary::cutoutsOfCategory(const std::string& category) const {
    std::vector<int> result;
    for (size_t i = 0; i < cutouts.size(); ++i) {
        if (cutouts[i].category == category) result.push_back(static_cast<int>(i));
    }
    return result;
}
