"""Runtime-light contracts and immutable artifact helpers."""
from pathlib import Path
import hashlib
import json
import math

CONTRACT = "clothforge_garment_v1"
def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()

def canonical_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()

def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as f:
        json.dump(value, f, indent=2, ensure_ascii=False, allow_nan=False, sort_keys=True)
        f.write("\n")

def read_json(path):
    return json.loads(Path(path).read_text())

def validate_material(material):
    if material.get("calibration_status") != "synthetic_unmeasured":
        raise ValueError("pilot materials must not claim real calibration")
    if material.get("mass_per_area_unit") != "kg/m2":
        raise ValueError("explicit area-density units required")
    for key in ["mass_per_area", "tri_ke", "tri_ka", "edge_ke", "friction"]:
        value = material[key]
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError("positive finite material value required: " + key)

def validate_manifest(m):
    if m.get("contract") != CONTRACT or m.get("units") != "m":
        raise ValueError("unsupported garment contract or units")
    if m.get("category") not in ["tshirt", "trousers", "open_coat", "towel", "napkin", "long_sleeve", "shorts", "skirt"]:
        raise ValueError("unsupported pilot garment category")
    if not isinstance(m.get("seed"), int):
        raise ValueError("explicit random seed required")
    if not m.get("pattern_sha256") or not m.get("mesh_sha256"):
        raise ValueError("artifact fingerprints required")
    if m.get("physical_validation") != "not_run":
        raise ValueError("geometry generation cannot certify dynamics")
    if not m.get("upstream_revision"):
        raise ValueError("upstream source revision required")
    json.dumps(m, allow_nan=False)
    return m
