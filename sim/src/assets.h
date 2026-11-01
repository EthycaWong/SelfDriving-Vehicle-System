#pragma once

#include <raylib.h>

#include <string>
#include <vector>

// Order must match CLASSES in vision/mapping.py.
enum class ObjectClass : int { Person, Animal, Vehicle, TrafficControl, Tree, Building, Obstruction, Count };

constexpr int kClassCount = static_cast<int>(ObjectClass::Count);

const char* className(ObjectClass cls);
ObjectClass classFromName(const std::string& name);

// A flat image drawn on a quad that turns to face the camera.
struct CutoutAsset {
    Texture2D texture{};
    ObjectClass cls = ObjectClass::Person;
    std::string category;
    float heightM = 1.0f;
    float aspect = 1.0f;  // Width divided by height.
    float mountM = 0.0f;  // Height of the image's bottom edge above the ground.
};

struct ModelAsset {
    Model model{};
    ObjectClass cls = ObjectClass::Obstruction;
    std::string name;
    BoundingBox bounds{};
};

struct AssetLibrary {
    std::vector<CutoutAsset> cutouts;
    std::vector<ModelAsset> models;
    std::vector<Texture2D> facades;
    Texture2D asphalt{};
    Texture2D sidewalk{};
    Texture2D grass{};
    Texture2D sky{};
    Texture2D white{};

    // Needs an open window. `root` is the project folder. Models get `shader` as their material.
    bool load(const std::string& root, Shader shader);

    std::vector<int> cutoutsOfClass(ObjectClass cls) const;
    std::vector<int> cutoutsOfCategory(const std::string& category) const;
};
