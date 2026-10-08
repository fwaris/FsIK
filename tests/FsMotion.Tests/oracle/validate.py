"""Independent analytical planar kinematics; no FsIK/FsMotion imports."""
import json
from pathlib import Path
import numpy as np

path = Path(__file__).resolve().parents[3] / 'artifacts/motion-example/detour.json'
plan = json.loads(path.read_text())
samples = plan['samples']
q = np.array([[s['shoulder'], s['elbow']] for s in samples])
assert all(s['otherJoint'] == 0.37 for s in samples)
assert np.array_equal(q[0], [-0.8, 0.])
assert np.allclose(q[-1], [0.8, 0.], atol=1e-12, rtol=0)
assert np.all(np.abs(q[:, 0]) <= 2.) and np.all(np.abs(q[:, 1]) <= 2.5)
elbow = .3*np.column_stack([np.cos(q[:, 0]), np.sin(q[:, 0])])
tip = elbow + .25*np.column_stack([np.cos(q.sum(axis=1)), np.sin(q.sum(axis=1))])
zero = np.zeros_like(tip)
fixed = np.tile([-1., 0.], (len(q), 1))

def point_segment(p, a, b):
    d = b-a
    den = np.sum(d*d, axis=1)
    along = np.divide(np.sum((p-a)*d, axis=1), den, out=np.zeros(len(den)), where=den>0)
    along = np.clip(along, 0, 1)
    return np.linalg.norm(p-(a+along[:, None]*d), axis=1)

obstacle = np.tile([.55, 0.], (len(q), 1))
clearances = [
    point_segment(obstacle, zero, elbow)-.015-.055,
    point_segment(obstacle, elbow, tip)-.015-.055,
    np.linalg.norm(obstacle-tip, axis=1)-.025-.055,
    point_segment(zero, elbow, tip)-.04-.015,
    np.linalg.norm(tip, axis=1)-.04-.025,
    point_segment(tip, zero, elbow)-.025-.015,
    point_segment(fixed, zero, elbow)-.03-.015,
    point_segment(fixed, elbow, tip)-.03-.015,
    np.linalg.norm(fixed-tip, axis=1)-.03-.025,
]
minimum = min(c.min() for c in clearances)
assert minimum > .005, minimum
rate = plan['rate']
velocity = np.diff(q, axis=0)*rate
acceleration = np.diff(np.vstack([np.zeros((1,2)), velocity, np.zeros((1,2))]), axis=0)*rate
tip_speed = np.linalg.norm(np.diff(tip, axis=0), axis=1)*rate
assert np.abs(velocity).max() <= .100001
assert np.abs(acceleration).max() <= .200001
assert tip_speed.max() <= .020001
assert abs(plan['duration_s']-(len(q)-1)/rate) < 1e-10
print(f'NumPy oracle passed: {len(q)} samples; minimum clearance {minimum:.6f} m; '
      f'maximum joint speed {np.abs(velocity).max():.6f} rad/s; '
      f'acceleration {np.abs(acceleration).max():.6f} rad/s²; tip speed {tip_speed.max():.6f} m/s')
