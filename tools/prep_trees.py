#!/usr/bin/env python3
"""Unpacks the Poly Haven tree models into flat vertex files for sim/tools/bake_trees.cpp.

raylib stores mesh indices as 16-bit values, so it can't load models this large directly. Writing
every triangle's vertices out in full avoids indices altogether.
"""
import json
import pathlib

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
TREES = ["tree_small_02", "island_tree_01", "jacaranda_tree"]
COMPONENT_TYPES = {5121: np.uint8, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
COMPONENT_COUNTS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}


def read_accessor(gltf, binary, index):
    accessor = gltf["accessors"][index]
    view = gltf["bufferViews"][accessor["bufferView"]]
    offset = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
    dtype = COMPONENT_TYPES[accessor["componentType"]]
    components = COMPONENT_COUNTS[accessor["type"]]
    item = np.dtype(dtype).itemsize * components
    stride = view.get("byteStride", item)
    if stride == item:
        data = np.frombuffer(binary, dtype, accessor["count"] * components, offset)
    else:
        raw = np.frombuffer(binary, np.uint8, accessor["count"] * stride, offset)
        data = raw.reshape(-1, stride)[:, :item].copy().view(dtype)
    return data.reshape(accessor["count"], components) if components > 1 else data


def main():
    out = ROOT / "assets" / "raw" / "trees"
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for name in TREES:
        folder = ROOT / "assets" / "models" / name
        gltf = json.loads((folder / f"{name}.gltf").read_text())
        binary = (folder / gltf["buffers"][0]["uri"]).read_bytes()
        node = gltf["nodes"][0]
        if "matrix" in node or "rotation" in node or "scale" in node:
            raise SystemExit(f"{name}: node transforms are not handled")

        parts, minimum, maximum = [], np.full(3, np.inf), np.full(3, -np.inf)
        for i, primitive in enumerate(gltf["meshes"][node["mesh"]]["primitives"]):
            indices = read_accessor(gltf, binary, primitive["indices"])
            positions = read_accessor(gltf, binary, primitive["attributes"]["POSITION"])
            normals = read_accessor(gltf, binary, primitive["attributes"]["NORMAL"])
            uvs = read_accessor(gltf, binary, primitive["attributes"]["TEXCOORD_0"])
            minimum = np.minimum(minimum, positions.min(axis=0))
            maximum = np.maximum(maximum, positions.max(axis=0))
            flat = np.hstack([positions[indices], normals[indices], uvs[indices]]).astype(np.float32)
            raw_name = f"{name}_{i}.raw"
            flat.tofile(out / raw_name)

            material = gltf["materials"][primitive["material"]]
            texture = material["pbrMetallicRoughness"]["baseColorTexture"]["index"]
            uri = gltf["images"][gltf["textures"][texture]["source"]]["uri"]
            parts += [raw_name, str((folder / uri).relative_to(ROOT))]
            print(f"{name} part {i}: {len(indices) // 3} triangles")

        center = (minimum + maximum) / 2
        radius = max(maximum[0] - minimum[0], maximum[2] - minimum[2]) / 2
        rows.append("\t".join([name, f"{maximum[1]:.3f}", f"{center[0]:.3f}", f"{center[2]:.3f}", f"{radius:.3f}", *parts]))
    (out / "index.tsv").write_text("\n".join(rows) + "\n")


if __name__ == "__main__":
    main()
