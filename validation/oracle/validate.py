"""Independent NumPy checks. Imports no motors and has no robot connection."""
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

repo = Path(__file__).resolve().parents[2]
results = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(repo / "validation/oracle"))
from arm_planner import Model, validate_mathnet_path

model = Model(repo / "validation/fixtures/axol_arms.urdf")
summary = json.loads((results / "summary.json").read_text())
assert model.id == summary["ModelSha256"]

def quaternion_matrix(w, x, y, z):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def angle(rotation):
    sine = .5*np.linalg.norm([rotation[2,1]-rotation[1,2], rotation[0,2]-rotation[2,0], rotation[1,0]-rotation[0,1]])
    return math.atan2(sine, np.clip((np.trace(rotation)-1)/2, -1, 1))

checks, max_position, max_orientation = 0, 0., 0.
for row in json.loads((results / "cases.json").read_text()):
    if not row["Success"]:
        continue
    q = np.zeros(14)
    side = "left" if row["Side"] == "Left" else "right"
    q[:7] = row["Angles"] if side == "left" else np.zeros(7)
    q[7:] = row["Angles"] if side == "right" else np.zeros(7)
    model.validate(q)
    frame = model.transforms(q)[f"{side}_gripper"]
    position_error = np.linalg.norm(frame[:3, 3] - row["TargetPosition"])
    orientation_error = angle(frame[:3, :3] @ quaternion_matrix(*row["TargetQuaternion"]).T)
    assert position_error <= summary["PositionToleranceMm"] / 1000 + 1e-9
    assert math.degrees(orientation_error) <= summary["OrientationToleranceDegrees"] + 1e-7
    assert abs(position_error*1000-row["PositionErrorMm"]) < 1e-7
    assert abs(math.degrees(orientation_error)-row["OrientationErrorDegrees"]) < 1e-7
    checks += 1
    max_position = max(max_position, position_error*1000)
    max_orientation = max(max_orientation, math.degrees(orientation_error))

paths = []
for path in summary["Paths"]:
    assert path["Success"], path["Detail"]
    data = (results / path["File"]).read_bytes()
    action = {"kind": "translate", "side": path["Side"].lower(), "delta": path["Delta"]}
    samples, tips = validate_mathnet_path(json.loads(data), model, np.asarray(path["Start"]), action)
    for q in samples:
        model.cross_arm_clearance(q)
    paths.append({"name": path["Name"], "frames": len(samples), "validated": True,
                  "sha256": hashlib.sha256(data).hexdigest()})

report = {"independent_endpoint_checks": checks,
          "max_position_error_mm": max_position, "max_orientation_error_deg": max_orientation,
          "paths": paths, "model_sha256": model.id, "hardware_connected": False,
          "sdk_collision_checks_included": False}
(results / "independent-validation.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report))
