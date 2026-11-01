"""Maps model labels onto the simulation's object classes."""

# Order must match enum ObjectClass in sim/src/assets.h.
CLASSES = ("person", "animal", "vehicle", "traffic_control", "tree", "building", "obstruction")

COCO_TO_CLASS = {
    "person": "person",
    **dict.fromkeys(
        ("bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe"), "animal"
    ),
    **dict.fromkeys(("bicycle", "car", "motorcycle", "bus", "truck", "train"), "vehicle"),
    **dict.fromkeys(("traffic light", "stop sign"), "traffic_control"),
    **dict.fromkeys(("bench", "fire hydrant", "parking meter", "suitcase", "chair"), "obstruction"),
}

# ADE20K names, as they appear in the SegFormer config. Some entries list several synonyms
# separated by semicolons, so the first word of each entry is what gets matched.
ADE_TO_CLASS = {
    "signboard": "traffic_control",
    "tree": "tree",
    "plant": "tree",
    "building": "building",
    "skyscraper": "building",
    "wall": "obstruction",
    "fence": "obstruction",
    "rock": "obstruction",
    "pole": "obstruction",
    "streetlight": "obstruction",
    "barrel": "obstruction",
    "bench": "obstruction",
}
