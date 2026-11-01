#pragma once

#include <raylib.h>

#include <vector>

#include "scene.h"

// What the ground-truth pass found for one object in one frame. Used for scoring only.
struct VisibleObject {
    int id = 0;
    ObjectClass cls = ObjectClass::Obstruction;
    float distance = 0;
    int visiblePixels = 0;
    Rectangle visibleBox{};    // Pixels of the object that are actually on screen.
    Rectangle projectedBox{};  // Where the whole object would be if nothing blocked it.
    bool fullyInFrame = false;
    float visibleFraction = 0; // Share of the object's own pixels that nothing covers.
};

class Renderer {
public:
    Renderer(int width, int height);
    ~Renderer();
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    Shader shader() const { return shader_; }

    // Builds road, sidewalk and marking meshes. Call once after the scene's roads are known.
    void buildStatic(const Scene& scene);

    // Draws the scene into the camera target. Pixels can then be read with readCameraPixels.
    void renderCamera(const Scene& scene, const Camera3D& camera);
    Image readCameraPixels() const;
    Texture2D cameraTexture() const { return cameraTarget_.texture; }

    std::vector<VisibleObject> renderGroundTruth(const Scene& scene, const Camera3D& camera);

    // Draws the scene into whatever target is active, for views the classifier doesn't see.
    void drawWorld(const Scene& scene, const Camera3D& camera);

    int width() const { return width_; }
    int height() const { return height_; }

private:
    void drawSky(const Scene& scene, const Camera3D& camera);
    void drawGround(const Scene& scene);
    void drawObjects(const Scene& scene, const Camera3D& camera, bool idPass);
    void drawObject(const Scene& scene, const SceneObject& object, const Camera3D& camera, bool idPass);
    void drawShadows(const Scene& scene);
    void setIdUniform(int id, bool idPass);
    void setLit(bool lit);
    int countPixelsAlone(const Scene& scene, const SceneObject& object, const Camera3D& camera);

    int width_;
    int height_;
    Shader shader_{};
    int idColorLoc_ = -1;
    int idModeLoc_ = -1;
    int litLoc_ = -1;
    RenderTexture2D superTarget_{};
    RenderTexture2D cameraTarget_{};
    RenderTexture2D idTarget_{};
    Mesh grassMesh_{};
    Mesh asphaltMesh_{};
    Mesh sidewalkMesh_{};
    Mesh whiteLineMesh_{};
    Mesh yellowLineMesh_{};
    Mesh skyMesh_{};
    Mesh quadMesh_{};
    Mesh groundQuadMesh_{};
    Texture2D shadowTexture_{};
    Material material_{};
};
