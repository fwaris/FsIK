"""Pure pose planning worker. Imports no motor drivers and never enables hardware.

Simulation FK uses the MIT-licensed Axol URDF bundled beside this file. Hardware
planning uses the installed SDK IK/path planners plus independent validation.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
import numpy as np

RATE = 120
JOINTS = ("shoulder_1", "shoulder_2", "shoulder_3", "elbow", "wrist_1", "wrist_2", "wrist_3")
SUFFIXES = ("s1_0", "s2_0", "s3_0", "e1_0", "e2_0", "w1_0", "w2_0")
MAX_VEL, MAX_ACCEL, MAX_TIP_SPEED = .1, .2, .02
MAX_FRAMES = 18000
# Application allowance for encoder readings at a model boundary. Planning
# projects these readings to the same legal boundary used by SDK target clamping.
# The allowance does not expand commanded positions or trajectory limits.
MEASURED_LIMIT_TOLERANCE = math.radians(.2)


def vector(value, count):
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f"Expected {count} values")
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in value):
        raise ValueError("Finite numbers required")
    return np.asarray(value, dtype=float)


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * (cross @ cross)


class Model:
    def __init__(self, path=None):
        path = Path(path or Path(__file__).with_name("axol_arms.urdf"))
        self.path = path.resolve()
        self.id = hashlib.sha256(path.read_bytes()).hexdigest()
        tree = ET.parse(path).getroot()
        self.names = [f"{side}_{name}" for side in ("left", "right") for name in JOINTS]
        actuated = [f"{side}_{name}" for side in ("left", "right") for name in SUFFIXES]
        self.joints = []
        limits = {}
        for joint in tree.findall("joint"):
            origin = joint.find("origin")
            xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
            rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
            transform = np.eye(4)
            transform[:3, :3] = rotation([0, 0, 1], rpy[2]) @ rotation([0, 1, 0], rpy[1]) @ rotation([1, 0, 0], rpy[0])
            transform[:3, 3] = xyz
            index = actuated.index(joint.get("name")) if joint.get("name") in actuated else None
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
            self.joints.append((joint.find("parent").get("link"), joint.find("child").get("link"), transform, axis, index))
            if index is not None:
                limits[index] = (float(joint.find("limit").get("lower")), float(joint.find("limit").get("upper")))
        if len(limits) != 14:
            raise ValueError("Unexpected Axol joint layout")
        self.limits = np.asarray([limits[index] for index in range(14)])

    def transforms(self, q):
        result = {"root": np.eye(4)}
        pending = list(self.joints)
        while pending:
            ready = [joint for joint in pending if joint[0] in result]
            if not ready:
                raise ValueError("Invalid URDF tree")
            for parent, child, origin, axis, index in ready:
                turn = np.eye(4)
                if index is not None:
                    turn[:3, :3] = rotation(axis, q[index])
                result[child] = result[parent] @ origin @ turn
                pending.remove(next(item for item in pending if item[1] == child))
        return result

    def tips(self, q):
        frames = self.transforms(q)
        return [(frames[f"{side}_gripper"] @ np.array([0, 0, -.145, 1]))[:3] for side in ("left", "right")]

    def validate_vector(self, q):
        if q.shape != (14,) or not np.all(np.isfinite(q)):
            raise ValueError("Invalid joint vector")

    def _validate_limits(self, q, tolerance):
        self.validate_vector(q)
        invalid = np.flatnonzero((q < self.limits[:, 0] - tolerance) | (q > self.limits[:, 1] + tolerance))
        if invalid.size:
            details = "; ".join(
                f"{self.names[index]} {math.degrees(q[index]):.3f} degrees "
                f"(allowed {math.degrees(self.limits[index, 0]):.1f} to {math.degrees(self.limits[index, 1]):.1f})"
                for index in invalid)
            raise ValueError("Joint limit exceeded: " + details)

    def validate(self, q):
        self._validate_limits(q, 1e-5)

    def validate_measured(self, q):
        self._validate_limits(q, MEASURED_LIMIT_TOLERANCE)

    def planning_pose(self, q, source):
        self.validate_measured(q)
        bounded = np.clip(q, self.limits[:, 0], self.limits[:, 1])
        adjustments = [
            {"joint": self.names[index], "source": source,
             "measured_deg": math.degrees(q[index]), "planned_deg": math.degrees(bounded[index])}
            for index in np.flatnonzero(bounded != q)]
        return bounded, adjustments

    def cross_arm_clearance(self, q):
        # Conservative swept centerlines supplement the SDK's torso-only model.
        frames = self.transforms(q)
        chains = []
        for side in ("left", "right"):
            points = [frames[f"{side}_{name}"][:3, 3] for name in ("s2", "s3", "e1", "e2", "w0", "w1", "w2", "gripper")]
            points.append((frames[f"{side}_gripper"] @ np.array([0, 0, -.145, 1]))[:3])
            samples = [a + (b - a) * t for a, b in zip(points, points[1:]) for t in np.linspace(0, 1, 12)]
            chains.append(np.asarray(samples))
        distance = np.linalg.norm(chains[0][:, None] - chains[1][None, :], axis=2).min()
        # 35 mm radius per arm and 30 mm sampling allowance; external obstacles
        # and actual link meshes still require a separate environment model.
        if distance < .100:
            raise ValueError("Conservative cross-arm clearance violated")


def simulate_ik(model, q, side, delta):
    index = 0 if side == "left" else 1
    selected = slice(index * 7, (index + 1) * 7)
    initial = model.transforms(q)[f"{side}_gripper"]
    target = initial[:3, 3] + delta
    desired_rotation = initial[:3, :3]
    result = q.copy()
    for _ in range(350):
        frame = model.transforms(result)[f"{side}_gripper"]
        error_rot = sum(np.cross(frame[:3, k], desired_rotation[:, k]) for k in range(3)) / 2
        error = np.concatenate((target - frame[:3, 3], error_rot * .2))
        if np.linalg.norm(error) < 1e-5:
            break
        columns = []
        for joint in range(index * 7, (index + 1) * 7):
            perturbed = result.copy()
            perturbed[joint] += 1e-5
            moved = model.transforms(perturbed)[f"{side}_gripper"]
            angular = sum(np.cross(frame[:3, k], moved[:3, k]) for k in range(3)) / (2e-5)
            columns.append(np.concatenate(((moved[:3, 3] - frame[:3, 3]) / 1e-5, angular * .2)))
        jac = np.asarray(columns).T
        step = jac.T @ np.linalg.solve(jac @ jac.T + np.eye(6) * 1e-5, error)
        step *= min(1, .025 / max(np.linalg.norm(step), 1e-9))
        result[selected] = np.clip(result[selected] + step, model.limits[selected, 0], model.limits[selected, 1])
    final = model.transforms(result)[f"{side}_gripper"]
    angle = math.acos(np.clip((np.trace(final[:3, :3].T @ desired_rotation) - 1) / 2, -1, 1))
    if np.linalg.norm(final[:3, 3] - target) > .005 or angle > math.radians(5):
        raise ValueError("Target cannot be reached with the requested orientation")
    return result


def retime(path, model):
    path = np.asarray(path, dtype=float)
    for _ in range(12):
        if len(path) > MAX_FRAMES:
            raise ValueError("Plan exceeds 150-second session limit")
        velocity = np.diff(path, axis=0) * RATE
        accel = np.diff(np.vstack((np.zeros((1, 14)), velocity, np.zeros((1, 14)))), axis=0) * RATE
        tips = np.asarray([model.tips(q) for q in path])
        tip_speed = np.linalg.norm(np.diff(tips, axis=0), axis=2).max(initial=0) * RATE
        stretch = max(velocity.max(initial=0) / MAX_VEL, -velocity.min(initial=0) / MAX_VEL,
                      math.sqrt(np.abs(accel).max(initial=0) / MAX_ACCEL), tip_speed / MAX_TIP_SPEED)
        if stretch <= 1.00001:
            return path, tips
        count = int(math.ceil((len(path) - 1) * stretch * 1.1)) + 1
        if count > MAX_FRAMES:
            raise ValueError("Plan cannot meet conservative motion limits")
        grid = np.linspace(0, len(path) - 1, count)
        path = np.column_stack([np.interp(grid, np.arange(len(path)), path[:, joint]) for joint in range(14)])
    raise ValueError("Trajectory could not be retimed within limits")


def mathnet_path(model, start, action):
    """F# supplies a timed path; Python independently validates every sample."""
    executable = os.environ.get("AXOL_MATHNET_PLANNER")
    if executable:
        args = [executable]
    elif os.name == "nt":
        dll = Path(__file__).resolve().parent.parent / "src/Axol.MathNetPlanner/bin/Release/net10.0/axol-mathnet-planner.dll"
        args = ["dotnet", str(dll)]
    else:
        args = [str(Path(__file__).with_name("axol-mathnet-planner"))]
    request = {"model_path": str(model.path), "model_id": model.id,
               "start": start.tolist(), "action": action}
    result = subprocess.run(args, input=json.dumps(request), capture_output=True,
                            text=True, timeout=120, check=False)
    if len(result.stdout) > 8*1024*1024:
        raise ValueError("MathNet response exceeded bounds")
    response = json.loads(result.stdout)
    if result.returncode != 0 or response.get("ok") is not True:
        raise ValueError(response.get("error", "MathNet planner failed"))
    value = response["plan"]
    if value.get("model_id") != model.id or value.get("rate") != RATE:
        raise ValueError("MathNet model or sample rate mismatch")
    if not np.array_equal(vector(value.get("start"), 14), start):
        raise ValueError("MathNet planning start mismatch")
    return validate_mathnet_path(value.get("path"), model, start, action)


def validate_mathnet_path(raw, model, start, action):
    if not isinstance(raw, list) or not 2 <= len(raw) <= MAX_FRAMES:
        raise ValueError("MathNet trajectory size invalid")
    path = np.asarray([vector(q, 14) for q in raw])
    if not np.array_equal(path[0], start):
        raise ValueError("MathNet trajectory does not begin at the planning start")
    if np.any(path < model.limits[:, 0]) or np.any(path > model.limits[:, 1]):
        raise ValueError("MathNet trajectory exceeds joint limits")
    if action.get("kind") != "translate" or action.get("side") not in ("left", "right"):
        raise ValueError("MathNet supports hand translations only")
    side = 0 if action["side"] == "left" else 1
    other = slice((1-side)*7, (2-side)*7)
    if not np.all(path[:, other] == start[other]):
        raise ValueError("MathNet moved the unrequested arm")
    velocity = np.diff(path, axis=0) * RATE
    accel = np.diff(np.vstack((np.zeros((1, 14)), velocity, np.zeros((1, 14)))), axis=0) * RATE
    tips = np.asarray([model.tips(q) for q in path])
    tip_speed = np.linalg.norm(np.diff(tips, axis=0), axis=2).max(initial=0) * RATE
    if (np.abs(velocity).max(initial=0) > MAX_VEL * 1.00001
            or np.abs(accel).max(initial=0) > MAX_ACCEL * 1.00001
            or tip_speed > MAX_TIP_SPEED * 1.00001):
        raise ValueError("MathNet trajectory exceeds speed/acceleration limits")
    # Much tighter commissioning checks than the SDK planner's general tolerance.
    initial_rotation = model.transforms(start)[f"{action['side']}_gripper"][:3, :3]
    delta = vector(action.get("delta"), 3)
    if not 0 < np.linalg.norm(delta) <= .100001:
        raise ValueError("Translation must be between zero and 10 cm")
    for q, tip in zip(path, tips[:, side]):
        rotation_now = model.transforms(q)[f"{action['side']}_gripper"][:3, :3]
        angle = math.acos(np.clip((np.trace(rotation_now.T @ initial_rotation)-1)/2, -1, 1))
        along = np.clip(np.dot(tip-tips[0, side], delta) / np.dot(delta, delta), 0, 1)
        if angle > math.radians(.1) or np.linalg.norm(tip-(tips[0, side]+along*delta)) > .0001:
            raise ValueError("MathNet straight-line/orientation tolerance exceeded")
    if np.linalg.norm(tips[-1, side]-tips[0, side]-delta) > .0001:
        raise ValueError("MathNet did not reach the requested gripper-tip translation")
    return path, tips


def plan(request, simulate=False):
    planner = request.get("planner", "sdk")
    if planner not in ("sdk", "mathnet"):
        raise ValueError("Unknown arm planner")
    if planner == "mathnet" and request.get("action", {}).get("kind") != "translate":
        raise ValueError("MathNet supports hand translations only")
    if simulate:
        model = Model()
        solver = None
    else:
        # Keep JAX compilation/IK off the cores reserved for motor traffic.
        from almond_axol.utils.affinity import core_groups
        groups = core_groups() or {}
        ik = groups.get("ik", set())
        background = ik if ik.isdisjoint(groups.get("realtime", set())) else groups.get("background", set())
        if background and hasattr(os, "sched_setaffinity"):
            os.sched_setaffinity(0, background)
        from almond_axol.constants import URDF_PATH
        from almond_axol.kinematics import KinematicsSolver
        from almond_axol.kinematics.model import collision_cost_params
        model = Model(URDF_PATH)
        solver = KinematicsSolver()
    start, boundary_adjustments = model.planning_pose(vector(request["start"], 14), "start")
    action = request["action"]
    kind = action["kind"]
    goal = start.copy()
    mathnet = None
    if kind == "saved":
        if action.get("model_id") != model.id:
            raise ValueError("Saved pose belongs to a different robot model")
        # Captured poses contain encoder readings too. A boundary offset is
        # projected before planning, never sent as an out-of-range target.
        goal, target_adjustments = model.planning_pose(vector(action["target"], 14), "saved target")
        boundary_adjustments += target_adjustments
    elif kind == "joint":
        side, joint, delta = action.get("side"), action.get("joint"), action.get("delta_rad")
        if side not in ("left", "right") or joint not in JOINTS:
            raise ValueError("Choose a valid arm and joint")
        if type(delta) not in (int, float) or not math.isfinite(delta) or not 0 < abs(delta) <= math.radians(5):
            raise ValueError("Joint step must be finite, nonzero, and at most 5 degrees")
        index = (0 if side == "left" else 7) + JOINTS.index(joint)
        goal[index] += delta
    elif kind == "translate":
        side = action.get("side")
        if side not in ("left", "right"):
            raise ValueError("Choose left or right hand")
        delta = vector(action["delta"], 3)
        if not 0 < np.linalg.norm(delta) <= .100001:
            raise ValueError("Translation must be between zero and 10 cm")
        if planner == "mathnet":
            mathnet = mathnet_path(model, start, action)
            goal = mathnet[0][-1].copy()
        elif simulate:
            goal = simulate_ik(model, start, side, delta)
        else:
            poses = solver.fk(start)
            target = (poses[0 if side == "left" else 1][0] + delta, poses[0 if side == "left" else 1][1])
            goal = start.copy()
            for _ in range(250):
                goal = np.asarray(solver.ik(goal, **{f"{side}_pose": target}))
            index = 0 if side == "left" else 1
            goal[(1-index)*7:(2-index)*7] = start[(1-index)*7:(2-index)*7]
            achieved = solver.fk(goal)[index]
            angle = math.acos(np.clip((np.trace(achieved[1].T @ target[1]) - 1) / 2, -1, 1))
            if np.linalg.norm(achieved[0] - target[0]) > .005 or angle > math.radians(5):
                raise ValueError("IK target unreachable or soft-clamped")
    elif kind != "hold":
        raise ValueError("Unsupported arm action")
    model.validate(goal)
    if mathnet is not None:
        path = mathnet[0]
    elif kind == "hold":
        path = np.asarray([start, start])
    elif simulate or kind == "joint":
        duration = max(1.0, np.max(np.abs(goal - start)) * 1.875 / MAX_VEL,
                       math.sqrt(np.max(np.abs(goal-start)) * 6 / MAX_ACCEL))
        t = np.linspace(0, 1, math.ceil(duration * RATE) + 1)
        ease = 10*t**3 - 15*t**4 + 6*t**5
        path = start[None] + (goal-start)[None] * ease[:, None]
    elif kind == "translate":
        from almond_axol.kinematics.path import plan_linear_segment, warmup
        warmup(solver)
        path = np.vstack((start, list(plan_linear_segment(solver, start, goal, speed=.01, ang_speed=.05, rate=RATE))))
        index = 0 if action["side"] == "left" else 1
        path[:, (1-index)*7:(2-index)*7] = start[(1-index)*7:(2-index)*7]
    else:
        from almond_axol.teleop.trajectory import plan_collision_aware_trajectory
        path = np.vstack((start, plan_collision_aware_trajectory(solver, start, goal, speed=.05, rate=RATE, min_duration=1.0)))
    if np.max(np.abs(path[-1] - goal)) > math.radians(.5):
        raise ValueError("Planner did not reach the requested joint target")
    path, tips = mathnet if mathnet is not None else retime(path, model)
    if kind == "translate":
        side = action["side"]
        index = 0 if side == "left" else 1
        initial = model.transforms(start)[f"{side}_gripper"]
        delta = vector(action["delta"], 3)
        for q, tip in zip(path, tips[:, index]):
            rotation_now = model.transforms(q)[f"{side}_gripper"][:3, :3]
            angle = math.acos(np.clip((np.trace(rotation_now.T @ initial[:3, :3]) - 1) / 2, -1, 1))
            along = np.clip(np.dot(tip-tips[0, index], delta) / np.dot(delta, delta), 0, 1)
            if angle > math.radians(5) or np.linalg.norm(tip - (tips[0, index] + along*delta)) > .005:
                raise ValueError("Translation path cannot preserve orientation/straight-line tolerance")
        if np.linalg.norm(tips[-1, index] - tips[0, index] - delta) > .005:
            raise ValueError("Gripper-tip translation did not reach target")
    if kind != "hold":
        for q in path:
            model.validate(q)
            model.cross_arm_clearance(q)
        if solver is not None:
            import jax.numpy as jnp
            # Per-pair calibrated floors are needed: conservative SDK capsules
            # overlap at some known-safe home configurations.
            floors, widths = collision_cost_params(solver.robot, solver.robot_coll, .025)
            floors = np.minimum(floors, np.asarray(solver.robot_coll.compute_self_collision_distance(solver.robot, jnp.zeros(14))))
            import jax
            check = jax.jit(jax.vmap(lambda q: solver.robot_coll.compute_self_collision_distance(solver.robot, q)))
            # Fixed-size batches bound compilation/memory for variable path lengths.
            for offset in range(0, len(path), 128):
                batch = np.asarray([solver.to_pyroki_order(q) for q in path[offset:offset+128]])
                batch = np.pad(batch, ((0, 128-len(batch)), (0, 0)), mode="edge")
                distances = np.asarray(check(jnp.asarray(batch)))
                if np.any(distances < floors - .001):
                    raise ValueError("SDK torso/arm clearance violated")
    return {"planner": planner, "model_id": model.id, "start": start.tolist(), "target": path[-1].tolist(),
            "boundary_adjustments": boundary_adjustments,
            "duration": (len(path)-1)/RATE, "tip_start": tips[0].tolist(), "tip_target": tips[-1].tolist(),
            "path": path.tolist(), "rate": RATE}


if __name__ == "__main__":
    import os
    # SDK/JAX native prints must never corrupt the worker response pipe.
    protocol = os.fdopen(os.dup(sys.stdout.fileno()), "w")
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    try:
        request = json.loads(sys.stdin.readline(65536))
        response = {"ok": True, "plan": plan(request, "--simulate" in sys.argv)}
    except Exception as error:
        response = {"ok": False, "error": str(error)}
    print(json.dumps(response, allow_nan=False), file=protocol, flush=True)
