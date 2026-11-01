// Renders each prepared tree model once from several angles into a transparent image. The models
// have 1.5-4 million triangles, far too many to draw live for a whole street.
//
// Usage: bake_trees <project root>   (run tools/prep_trees.py first)
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr int kSize = 1536;

const char* kVertex = R"(#version 330
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec3 vertexNormal;
uniform mat4 mvp;
out vec2 uv;
out vec3 normal;
void main() {
    uv = vertexTexCoord;
    normal = vertexNormal;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)";

const char* kFragment = R"(#version 330
in vec2 uv;
in vec3 normal;
uniform sampler2D texture0;
out vec4 color;
void main() {
    vec3 sun = normalize(vec3(0.4, 0.8, 0.45));
    float light = 0.55 + 0.6 * abs(dot(normalize(normal), sun));
    color = vec4(texture(texture0, uv).rgb * light, 1.0);
}
)";

struct Part {
    Mesh mesh{};
    Material material{};
};

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: bake_trees <project root>\n");
        return 2;
    }
    const std::string root = argv[1];
    const std::string prep = root + "/assets/raw/trees";

    SetConfigFlags(FLAG_WINDOW_HIDDEN);
    SetTraceLogLevel(LOG_WARNING);
    InitWindow(64, 64, "bake");
    Shader shader = LoadShaderFromMemory(kVertex, kFragment);
    RenderTexture2D target = LoadRenderTexture(kSize, kSize);

    std::ifstream index(prep + "/index.tsv");
    if (!index) {
        std::fprintf(stderr, "Missing %s/index.tsv\n", prep.c_str());
        return 1;
    }

    // Rows: tree name, height in metres, center x, center z, then raw file and texture per part.
    std::ofstream manifest(root + "/assets/cutouts/trees.tsv");
    std::filesystem::create_directories(root + "/assets/cutouts/tree");
    std::string line;
    while (std::getline(index, line)) {
        std::stringstream row(line);
        std::string name, heightText, cxText, czText, radiusText;
        std::getline(row, name, '\t');
        std::getline(row, heightText, '\t');
        std::getline(row, cxText, '\t');
        std::getline(row, czText, '\t');
        std::getline(row, radiusText, '\t');
        const float height = std::stof(heightText);
        const float cx = std::stof(cxText), cz = std::stof(czText), radius = std::stof(radiusText);

        std::vector<Part> parts;
        std::string rawFile, textureFile;
        while (std::getline(row, rawFile, '\t') && std::getline(row, textureFile, '\t')) {
            std::ifstream raw(prep + "/" + rawFile, std::ios::binary | std::ios::ate);
            const auto bytes = static_cast<size_t>(raw.tellg());
            raw.seekg(0);
            const int vertexCount = static_cast<int>(bytes / (8 * sizeof(float)));
            std::vector<float> data(bytes / sizeof(float));
            raw.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(bytes));

            Part part;
            part.mesh.vertexCount = vertexCount;
            part.mesh.triangleCount = vertexCount / 3;
            part.mesh.vertices = static_cast<float*>(MemAlloc(static_cast<unsigned>(vertexCount) * 3 * sizeof(float)));
            part.mesh.normals = static_cast<float*>(MemAlloc(static_cast<unsigned>(vertexCount) * 3 * sizeof(float)));
            part.mesh.texcoords = static_cast<float*>(MemAlloc(static_cast<unsigned>(vertexCount) * 2 * sizeof(float)));
            for (int v = 0; v < vertexCount; ++v) {
                const float* source = &data[static_cast<size_t>(v) * 8];
                std::memcpy(&part.mesh.vertices[v * 3], source, 3 * sizeof(float));
                std::memcpy(&part.mesh.normals[v * 3], source + 3, 3 * sizeof(float));
                std::memcpy(&part.mesh.texcoords[v * 2], source + 6, 2 * sizeof(float));
            }
            UploadMesh(&part.mesh, false);
            part.material = LoadMaterialDefault();
            part.material.shader = shader;
            Texture2D texture = LoadTexture((root + "/" + textureFile).c_str());
            GenTextureMipmaps(&texture);
            SetTextureFilter(texture, TEXTURE_FILTER_TRILINEAR);
            part.material.maps[MATERIAL_MAP_DIFFUSE].texture = texture;
            parts.push_back(part);
        }

        for (int view = 0; view < 3; ++view) {
            const float angle = static_cast<float>(view) * 2.1f;
            const float span = std::max(height, radius * 2) * 1.04f;
            Camera3D camera{};
            camera.position = {cx + std::sin(angle) * span * 3, height / 2, cz + std::cos(angle) * span * 3};
            camera.target = {cx, height / 2, cz};
            camera.up = {0, 1, 0};
            camera.fovy = span;
            camera.projection = CAMERA_ORTHOGRAPHIC;

            BeginTextureMode(target);
            ClearBackground(BLANK);
            rlSetClipPlanes(0.1, span * 8);
            BeginMode3D(camera);
            rlDisableBackfaceCulling();
            for (const Part& part : parts) DrawMesh(part.mesh, part.material, MatrixIdentity());
            EndMode3D();
            EndTextureMode();

            Image image = LoadImageFromTexture(target.texture);
            ImageFlipVertical(&image);
            const Rectangle border = GetImageAlphaBorder(image, 0.1f);
            ImageCrop(&image, border);
            const std::string relative = "assets/cutouts/tree/" + name + "_" + std::to_string(view) + ".png";
            ExportImage(image, (root + "/" + relative).c_str());
            const float bakedHeight = border.height / kSize * span;
            manifest << relative << "\ttree\t" << name << '\t' << bakedHeight << '\t'
                     << border.width / border.height << "\t0.0\n";
            std::printf("%s view %d: %dx%d, %.1f m tall\n", name.c_str(), view, image.width, image.height,
                        static_cast<double>(bakedHeight));
            UnloadImage(image);
        }
        for (Part& part : parts) UnloadMesh(part.mesh);
    }
    CloseWindow();
    return 0;
}
