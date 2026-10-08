# FsMotion — offline collision-aware arm planning

FsMotion is a separate F#/.NET 10 component beside FsIK. It does not enable motors,
connect to a robot, or change the controller's planner/package selection.

## First supported workflow

1. Load the complete bounded robot joint tree and its collision geometry from URDF.
2. Supply all measured starting joints, a fixed scene snapshot and explicit allowed
   collision pairs. Choose one arm's ordered active joints; other joints stay fixed.
3. Supply target joints, or use `Planner.planToPose` with a model-owned FsIK chain
   and a target position/orientation in the URDF root frame.
4. Check the direct joint-space route. If blocked, a deterministic seeded RRT
   searches bounded joint space for a detour. Return a complete route or typed
   failure; no partial route is executable.
5. `Planner.retime` adds conservative stop-at-waypoint timing and independently
   rechecks every resulting swept interval. Operator review and an independent
   execution validator remain downstream responsibilities.

Joint-space detours do not promise a straight hand path or preserved orientation
during transit. The pose goal constrains the endpoint only. Cartesian path
constraints and alternative collision-free IK endpoint enumeration remain future
work. One IK endpoint can collide even when another endpoint solution is possible.

## API sketch

```fsharp
let model = Model.load urdfPath resolveMesh allowedPairs |> unwrap
let chain = Model.chain model "left_gripper" |> unwrap
let scene = { Id = "scene-revision-1"; Obstacles = obstacles }
let options = Planner.defaults
let checker = Collision.create model scene options.Margin 20 |> unwrap
let path =
    Planner.planToPose options model scene.Id checker chain
        fullStartingJoints targetPose cancellation
    |> unwrap
let timed =
    Planner.retime Trajectory.conservative model chain checker path cancellation
    |> unwrap
```

`unwrap` is illustrative. Applications should display typed errors rather than
throw. A package consumer needs both FsMotion and its FsIK dependency from the
local feed. `scripts/MotionExample.fsx` runs an actual obstacle-detour example and
exports the plan as JSON without any controller or hardware interface.

## Collision implementation and contracts

- URDF sphere, cylinder and box geometry and **binary STL** meshes are supported.
  Meshes and boxes become conservative enclosing capsules. Origins and mesh scales
  are applied. Missing meshes, malformed shapes, mimic joints, prismatic and
  continuous joints fail explicitly; they are not silently skipped.
- Directly joint-connected link pairs and same-link shapes are automatically
  excluded. Other exclusions are supplied by the caller and contribute to model
  identity. Collisionless links remain kinematic frames: the caller must ensure
  every physical body requiring protection has collision geometry.
- The scene contains static world-space capsules (a sphere is a zero-length
  capsule). The immutable model and scene are fixed for the whole plan. Root pose
  is identity; transform world obstacles into this frame before calling.
- Configuration checking computes signed capsule clearance against the whole
  robot and scene. Both moving and inactive bodies participate.
- Swept checking uses a conservative angular-displacement bound derived from the
  complete model's reach. It recursively bisects until clearance is certified,
  collision is found, cancellation occurs or its depth budget is exhausted.
  Exhausted certification is rejected. Endpoints alone never certify an edge.
- Model/scene IDs and requested margin bind the checker, route and retiming.
  Another backend can implement `CollisionChecker`; it must obey its whole-edge
  certification and cooperative cancellation contract. It must not return
  `ClearEdge` merely because sampled configurations were clear.
- Search node/attempt and time budgets are local to a call. Time budget applies to
  search; the bounded FsIK solve precedes it. RRT is not a guarantee of finding a
  route, and failure does not prove physical unreachability.
- Timing uses the existing FsIK retimer and also independently caps speed of the
  selected URDF tip origin. FsIK's additional Axol tool-offset speed check can
  make timing more conservative for other robots. Tool/gripper offsets should be
  represented as fixed tip links. Runtime tracking and dynamic obstacles require
  a downstream controller; this offline component cannot monitor them.

## Axol fixture and current limitation

Tests vendor 18 official Axol STL files and the matching arm URDF at upstream
commit `5a66c86ef04acab79499ba3edc277da7a9c0b57c`, with SHA-256 manifest and Almond's
MIT license. The URDF hash matches the controller's
`9451ddfb18721f2027b77430fcbc8935d4928d989e2faba21741cf7fed792664` model.

An offline probe of the VR rest pose with the default generic pair policy reports
overlap between `left_s3` and `s1` (approximately -92 mm envelope clearance).
The SDK excludes shoulder-mount pairs and calibrates some torso/arm thresholds
relative to home. **The generic capsule model is not calibrated for physical Axol
execution.** We do not automatically copy the SDK's broad within-arm/cross-arm
exclusions or relax margins to force a plan through. A validated Axol-specific
profile or a tighter collision backend must resolve false positives first.
The arm-only URDF also cannot protect unmodelled head/base hardware or surroundings.

## Validation

The new suite covers a clear route, a blocked direct route with deterministic
detour, an interior swept collision, joint limits, unchanged targets, endpoint
collisions, inactive-joint preservation, retiming speed/acceleration/tip speed,
model/scene/margin binding, cancellation, cooperative timeout, exhausted budgets,
checker errors, missing assets, retained-result ownership, concurrency and real
Axol asset loading. FsIK's existing tests and frozen numerical validation remain
unchanged.

Build/check with `dotnet fsi scripts/Build-Motion.fsx`; add `pack` to build a new
immutable local package. All work is offline. Future coordinated-arm planning can
reuse full configurations and collision checking, with composite pose goals and a
shared multi-chain retimer added separately.
