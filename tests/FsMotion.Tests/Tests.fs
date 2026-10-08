module FsMotion.Tests

open System
open System.IO
open System.Threading
open FsIK
open FsMotion
open Xunit

let unwrap =
    function
    | Ok value -> value
    | Error error -> failwithf "%A" error

let fixture () =
    let path = Path.Combine(AppContext.BaseDirectory, "arm.urdf")
    let model = Model.load path (fun _ -> failwith "No meshes") [] |> unwrap
    let chain = Geometry.load path "base" "tip"
    model, chain

let start = Map.ofList [ "shoulder", -0.8; "elbow", 0.; "otherJoint", 0.37 ]
let goal = Map.add "shoulder" 0.8 start
let active = [ "shoulder"; "elbow" ]

let scene obstacle =
    { Id = "fixture"; Obstacles = obstacle }

let sphere name x y r : Obstacle =
    { Name = name
      Shape =
        { A = Geometry.v x y 0.
          B = Geometry.v x y 0.
          Radius = r } }

let checker model obstacles =
    Collision.create model (scene obstacles) 0.005 20 |> unwrap

let options =
    { Planner.defaults with
        Margin = 0.005
        MaxSeconds = 30. }

[<Fact>]
let ``free direct route preserves inactive joints and retiming stays certified`` () =
    let model, chain = fixture ()
    let check = checker model []

    let path =
        Planner.plan options model "fixture" check active start goal CancellationToken.None
        |> unwrap

    Assert.Equal(2, path.Configurations.Length)

    let timed =
        Planner.retime Trajectory.conservative model chain check path CancellationToken.None
        |> unwrap

    Assert.True((start = timed.Samples.Head))
    Assert.True((goal = List.last timed.Samples))
    Assert.True(timed.Samples |> List.forall (fun q -> q["otherJoint"] = 0.37))

    let maxStep =
        timed.Samples
        |> List.pairwise
        |> List.collect (fun (a, b) -> active |> List.map (fun name -> abs (b[name] - a[name]) * float timed.Rate))
        |> List.max

    Assert.InRange(maxStep, 0., Trajectory.conservative.MaxJointVelocity * 1.00001)

    let velocities =
        timed.Samples
        |> List.pairwise
        |> List.map (fun (a, b) -> active |> List.map (fun n -> (b[n] - a[n]) * float timed.Rate))

    let accelerations =
        ([ List.replicate active.Length 0. ]
         @ velocities
         @ [ List.replicate active.Length 0. ])
        |> List.pairwise
        |> List.collect (fun (a, b) -> List.map2 (fun x y -> abs (y - x) * float timed.Rate) a b)

    Assert.InRange(List.max accelerations, 0., Trajectory.conservative.MaxJointAcceleration * 1.00001)

    let tipSpeed =
        timed.Samples
        |> List.map (fun q -> (Geometry.forward chain (active |> List.map (fun n -> q[n]))).Position)
        |> List.pairwise
        |> List.map (fun (a, b) -> Geometry.norm (Geometry.subtract b a) * float timed.Rate)
        |> List.max

    Assert.InRange(tipSpeed, 0., Trajectory.conservative.MaxTipSpeed * 1.00001)

[<Fact>]
let ``blocked direct route obtains deterministic detour with every edge certified`` () =
    let model, _ = fixture ()
    let check = checker model [ sphere "obstacle" 0.55 0. 0.055 ]
    Assert.NotEqual(Ok ClearEdge, check.Edge start goal CancellationToken.None)

    let run () =
        Planner.plan options model "fixture" check active start goal CancellationToken.None

    let path = run () |> unwrap
    Assert.True(path.Configurations.Length > 2)
    Assert.True((start = path.Configurations.Head))
    Assert.True((goal = List.last path.Configurations))

    for a, b in List.pairwise path.Configurations do
        Assert.Equal(Ok ClearEdge, check.Edge a b CancellationToken.None)

    Assert.Equal(Ok path, run ())
    let retained = path.Configurations
    let results = [| 1..4 |] |> Array.Parallel.map (fun _ -> run ())
    Assert.True(results |> Array.forall ((=) (Ok path)))
    Assert.True((retained = path.Configurations))

[<Fact>]
let ``mismatched scene model margin and retiming checker are rejected`` () =
    let model, chain = fixture ()
    let clear = checker model []

    for changed in
        [ { clear with ModelId = "other" }
          { clear with SceneId = "old-scene" }
          { clear with Margin = 0.001 } ] do
        match Planner.plan options model "fixture" changed active start goal CancellationToken.None with
        | Error(InvalidRequest _) -> ()
        | result -> failwithf "%A" result

    let path =
        Planner.plan options model "fixture" clear active start goal CancellationToken.None
        |> unwrap

    Assert.True(
        Planner.retime
            Trajectory.conservative
            model
            chain
            { clear with SceneId = "new-scene" }
            path
            CancellationToken.None
        |> Result.isError
    )

    let forged =
        { path with
            Configurations = [ start; Map.add "otherJoint" 0. goal ] }

    Assert.True(
        Planner.retime Trajectory.conservative model chain clear forged CancellationToken.None
        |> Result.isError
    )

[<Fact>]
let ``time budget interrupts a cooperative slow edge checker`` () =
    let model, _ = fixture ()
    let clear = checker model []

    let slow =
        { clear with
            Edge =
                fun _ _ token ->
                    token.WaitHandle.WaitOne(1000) |> ignore
                    Ok InterruptedEdge }

    Assert.Equal(
        Error PlanningTimeout,
        Planner.plan { options with MaxSeconds = 0.02 } model "fixture" slow active start goal CancellationToken.None
    )

[<Fact>]
let ``unchanged target remains stationary and goal collision is identified`` () =
    let model, chain = fixture ()
    let clear = checker model []

    let path =
        Planner.plan options model "fixture" clear active start start CancellationToken.None
        |> unwrap

    Assert.True(path.Configurations |> List.forall ((=) start))
    let tip = (Geometry.forward chain [ goal["shoulder"]; goal["elbow"] ]).Position

    let obstacle: Obstacle =
        { Name = "goal-obstacle"
          Shape = { A = tip; B = tip; Radius = 0.03 } }

    match
        Planner.plan options model "fixture" (checker model [ obstacle ]) active start goal CancellationToken.None
    with
    | Error(GoalCollision _) -> ()
    | result -> failwithf "%A" result

[<Fact>]
let ``URDF without collision shapes and unresolved meshes cannot load`` () =
    let root = Path.Combine(Path.GetTempPath(), Guid.NewGuid().ToString("N"))
    Directory.CreateDirectory root |> ignore

    try
        let file = Path.Combine(root, "missing.urdf")
        File.WriteAllText(file, "<robot><link name='base'/></robot>")
        Assert.True(Model.load file id [] |> Result.isError)

        File.WriteAllText(
            file,
            "<robot><link name='base'><collision><geometry><mesh filename='missing.stl'/></geometry></collision></link></robot>"
        )

        Assert.True(Model.load file id [] |> Result.isError)
    finally
        Directory.Delete(root, true)

[<Fact>]
let ``swept check finds interior collision despite clear endpoints`` () =
    let model, _ = fixture ()
    let check = checker model [ sphere "obstacle" 0.55 0. 0.055 ]
    Assert.True((check.Configuration start |> unwrap).MinimumClearance > 0.005)
    Assert.True((check.Configuration goal |> unwrap).MinimumClearance > 0.005)

    match check.Edge start goal CancellationToken.None with
    | Ok(BlockedEdge _) -> ()
    | unexpected -> failwithf "Missed interior collision: %A" unexpected

[<Fact>]
let ``colliding endpoints invalid requests and cancellation return no path`` () =
    let model, _ = fixture ()
    let check = checker model [ sphere "cover" 0. 0. 3. ]

    match Planner.plan options model "fixture" check active start goal CancellationToken.None with
    | Error(StartCollision _) -> ()
    | result -> failwithf "%A" result

    let clear = checker model []

    for invalid in
        [ Map.remove "elbow" start
          Map.add "elbow" Double.NaN start
          Map.add "elbow" 3. start ] do
        match Planner.plan options model "fixture" clear active invalid goal CancellationToken.None with
        | Error(InvalidRequest _) -> ()
        | result -> failwithf "%A" result

    match
        Planner.plan options model "fixture" clear active start (Map.add "otherJoint" 0. goal) CancellationToken.None
    with
    | Error(InvalidRequest _) -> ()
    | result -> failwithf "%A" result

    use cancel = new CancellationTokenSource()
    cancel.Cancel()
    Assert.Equal(Error PlanningCancelled, Planner.plan options model "fixture" clear active start goal cancel.Token)

[<Fact>]
let ``bounded search exhausts without returning a partial trajectory`` () =
    let model, _ = fixture ()
    let clear = checker model []

    let blocked =
        { clear with
            Edge = fun _ _ _ -> Ok UncertifiedEdge }

    match
        Planner.plan { options with MaxNodes = 8 } model "fixture" blocked active start goal CancellationToken.None
    with
    | Error(SearchExhausted 1) -> ()
    | result -> failwithf "%A" result

    let broken =
        { clear with
            Configuration = fun _ -> Error "checker unavailable" }

    Assert.Equal(
        Error(CheckerFailure "checker unavailable"),
        Planner.plan options model "fixture" broken active start goal CancellationToken.None
    )

[<Fact>]
let ``FsIK target endpoint is connected by collision aware planner`` () =
    let model, chain = fixture ()
    let check = checker model []
    let target = Geometry.forward chain [ -0.5; 0.3 ]

    let result =
        Planner.planToPose options model "fixture" check chain start target CancellationToken.None
        |> unwrap

    let last = List.last result.Configurations

    let position, rotation =
        Geometry.poseError target (Geometry.forward chain (active |> List.map (fun n -> last[n])))

    Assert.InRange(position, 0., Solver.defaults.PositionTolerance)
    Assert.InRange(rotation, 0., Solver.defaults.OrientationTolerance)

[<Fact>]
let ``capsule distances handle crossing parallel and degenerate segments`` () =
    let capsule a b r = { A = a; B = b; Radius = r }
    let first = capsule (Geometry.v -1. 0. 0.) (Geometry.v 1. 0. 0.) 0.1
    let crossing = capsule (Geometry.v 0. -1. 0.) (Geometry.v 0. 1. 0.) 0.2
    Assert.Equal(-0.3, Collision.distance first crossing, 10)
    let parallelShape = capsule (Geometry.v -1. 1. 0.) (Geometry.v 1. 1. 0.) 0.1
    Assert.Equal(0.8, Collision.distance first parallelShape, 10)
    let point = capsule (Geometry.v 2. 0. 0.) (Geometry.v 2. 0. 0.) 0.1
    Assert.Equal(0.8, Collision.distance first point, 10)

[<Fact>]
let ``uncertified edges and invalid scene fail closed`` () =
    let model, _ = fixture ()
    let check = Collision.create model (scene []) 0.005 0 |> unwrap
    Assert.Equal(Ok UncertifiedEdge, check.Edge start goal CancellationToken.None)

    Assert.True(
        Collision.create model (scene [ sphere "bad" 0. 0. -1. ]) 0.005 10
        |> Result.isError
    )

[<Fact>]
let ``Axol collision meshes resolve and full robot geometry loads`` () =
    let root = Path.Combine(AppContext.BaseDirectory, "axol")

    let resolve (uri: string) =
        Path.Combine(root, "meshes", Path.GetFileName uri |> Option.ofObj |> Option.defaultValue "")

    let model = Model.load (Path.Combine(root, "axol.urdf")) resolve [] |> unwrap
    Assert.Equal(18, Model.shapeCount model)
    Assert.Equal(14, (Model.joints model).Count)

    let q =
        Model.joints model
        |> Map.map (fun _ bounds -> (bounds.Lower + bounds.Upper) / 2.)

    let check = checker model []
    Assert.True(check.Configuration q |> Result.isOk)
// Successful loading is not evidence that conservative envelopes fit hardware.


[<Fact>]
let ``node budget includes the root and cannot grow beyond its limit`` () =
    let model, _ = fixture ()
    let clear = checker model []

    let noGoal =
        { clear with
            Edge = fun _ b _ -> if b = goal then Ok UncertifiedEdge else Ok ClearEdge }

    match
        Planner.plan
            { options with
                MaxNodes = 2
                GoalBias = 0. }
            model
            "fixture"
            noGoal
            active
            start
            goal
            CancellationToken.None
    with
    | Error(SearchExhausted 2) -> ()
    | result -> failwithf "%A" result
