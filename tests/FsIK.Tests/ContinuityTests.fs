module FsIK.ContinuityTests

open System
open System.IO
open FsIK
open Xunit

let single =
    Geometry.load (Path.Combine(AppContext.BaseDirectory, "planar.urdf")) "base" "tip"

let multi =
    Geometry.load (Path.Combine(AppContext.BaseDirectory, "branched.urdf")) "base" "tip"

let tight =
    { Solver.defaults with
        PositionTolerance = 1e-5
        OrientationTolerance = 1e-4 }

let solved =
    function
    | Solved solution -> solution
    | failure -> failwithf "%A" failure

let complete =
    function
    | Ok value -> value
    | Error error -> failwith error

let changes a b = List.map2 (fun x y -> abs (y - x)) a b

let assertPath bound path =
    path
    |> List.pairwise
    |> List.iter (fun (a, b) ->
        Assert.True(changes a b |> List.forall (fun value -> value <= bound), sprintf "%A" (changes a b)))

[<Fact>]
let ``local solver cannot use a distant endpoint solution`` () =
    let target = Geometry.forward single [ 0.4 ]
    Solver.solve tight single target [ 0. ] |> solved |> ignore

    match Solver.solveNear tight 0.1 single target [ 0. ] with
    | Failed(NoConvergence result) -> Assert.InRange(result.Angles.Head, -0.1, 0.1)
    | unexpected -> failwithf "%A" unexpected

[<Fact>]
let ``local solver reaches nearby targets and keeps unchanged boundary poses`` () =
    for start, goal in [ 0., 0.05; 0., -0.05; 0.7, 0.75; -0.7, -0.75; 0.75, 0.75 ] do
        let target = Geometry.forward single [ goal ]
        let result = Solver.solveNear tight 0.1 single target [ start ] |> solved
        Assert.True(abs (result.Angles.Head - start) <= 0.1)
        Assert.InRange(result.Angles.Head, -0.75, 0.75)

        let position, rotation =
            Geometry.poseError target (Geometry.forward single result.Angles)

        Assert.True(position <= tight.PositionTolerance && rotation <= tight.OrientationTolerance)

[<Fact>]
let ``invalid local requests are rejected`` () =
    let target = Geometry.forward single [ 0.05 ]

    let requests =
        [ for radius in [ 0.; -0.1; Double.NaN; Double.PositiveInfinity ] do
              yield Solver.solveNear tight radius single target [ 0. ]
          yield Solver.solveNear tight 0.1 single target []
          yield Solver.solveNear tight 0.1 single target [ 0.8 ]
          yield Solver.solveNear tight 0.1 single target [ Double.NaN ]
          yield
              Solver.solveNear
                  tight
                  0.1
                  single
                  { target with
                      Orientation = { W = 2.; X = 0.; Y = 0.; Z = 0. } }
                  [ 0. ]
          yield Solver.solveNear { tight with MaxIterations = 0 } 0.1 single target [ 0. ] ]

    for request in requests do
        match request with
        | Failed(InvalidInput _) -> ()
        | unexpected -> failwithf "%A" unexpected

[<Fact>]
let ``mixed local and global solves retain independent concurrent results`` () =
    let targets =
        [ -0.06; 0.; 0.07 ] |> List.map (fun angle -> Geometry.forward single [ angle ])

    let expected =
        targets
        |> List.map (fun target -> Solver.solveNear tight 0.1 single target [ 0. ])

    let retained = sprintf "%A" expected

    for _ in 1..4 do
        let actual =
            targets
            |> List.toArray
            |> Array.Parallel.map (fun target ->
                Solver.solveNear tight 0.000001 single (Geometry.forward single [ 0.4 ]) [ 0. ]
                |> ignore

                Solver.solve tight single (Geometry.forward single [ 0.6 ]) [ 0. ]
                |> solved
                |> ignore

                Solver.solveNear tight 0.1 single target [ 0. ])
            |> Array.toList

        Assert.True((expected = actual))

    Assert.Equal(retained, sprintf "%A" expected)

[<Fact>]
let ``two elbow configurations keep their local translation branch`` () =
    for seed in [ [ 0.6; -1.2; 0.6 ]; [ -0.6; 1.2; -0.6 ] ] do
        let path =
            Translation.waypoints Translation.defaults multi seed (Geometry.v -0.005 0. 0.)
            |> complete

        assertPath 0.1 path
        Assert.True(path |> List.forall (fun angles -> angles[1] * seed[1] > 0.))
        let initial = Geometry.forward multi seed

        let target =
            { initial with
                Position = Geometry.add initial.Position (Geometry.v -0.005 0. 0.) }

        let position, rotation =
            Geometry.poseError target (Geometry.forward multi (List.last path))

        Assert.True(position <= tight.PositionTolerance && rotation <= tight.OrientationTolerance)

[<Fact>]
let ``subdivision recovers a locally bounded movement from a straight chain`` () =
    let options =
        { Translation.defaults with
            MaxJointChangeRadians = 0.025 }

    let delta = Geometry.v -0.001 0. 0.
    let seed = [ 0.; 0.; 0. ]

    let noSplit =
        Translation.waypoints { options with MaxSubdivisionDepth = 0 } multi seed delta

    Assert.True(Result.isError noSplit)
    let path = Translation.waypoints options multi seed delta |> complete
    Assert.True(path.Length > 3)
    assertPath options.MaxJointChangeRadians path

    let target =
        { Geometry.forward multi seed with
            Position = Geometry.add (Geometry.forward multi seed).Position delta }

    let position, rotation =
        Geometry.poseError target (Geometry.forward multi (List.last path))

    Assert.True(position <= tight.PositionTolerance && rotation <= tight.OrientationTolerance)

[<Fact>]
let ``exhausted budgets cannot return partial trajectories`` () =
    for options, expected in
        [ { Translation.defaults with
              MaxWaypoints = 2 },
          "Waypoint limit"
          { Translation.defaults with
              MaxJointChangeRadians = 0.000001
              MaxSubdivisionDepth = 0 },
          "Subdivision depth" ] do
        match Translation.planWithContinuity options multi [ 0.; 0.; 0. ] (Geometry.v -0.001 0. 0.) with
        | Error error ->
            Assert.Contains(expected, error)
            Assert.Contains("joint-change bound", error)
        | Ok _ -> failwith "Partial plan accepted"

[<Fact>]
let ``invalid continuity options and starts do not produce a trajectory`` () =
    for options in
        [ { Translation.defaults with
              MaxJointChangeRadians = Double.NaN }
          { Translation.defaults with
              MaxJointChangeRadians = 0. }
          { Translation.defaults with
              MaxSubdivisionDepth = -1 }
          { Translation.defaults with
              MaxSubdivisionDepth = 17 }
          { Translation.defaults with
              MaxWaypoints = 1 } ] do
        Assert.True(
            Translation.planWithContinuity options multi [ 0.; 0.; 0. ] (Geometry.v -0.001 0. 0.)
            |> Result.isError
        )

    for seed in [ []; [ Double.NaN; 0.; 0. ]; [ 4.; 0.; 0. ] ] do
        Assert.True(Translation.plan multi seed (Geometry.v -0.001 0. 0.) |> Result.isError)

[<Theory>]
[<InlineData(false, 0.01)>]
[<InlineData(false, 0.0508)>]
[<InlineData(true, 0.01)>]
[<InlineData(true, 0.0508)>]
let ``recorded Axol paths respect the actual waypoint bound`` right distance =
    let side = if right then "right" else "left"

    let chain =
        Geometry.load (Path.Combine(AppContext.BaseDirectory, "model.urdf")) "root" (side + "_gripper")

    let seed =
        if right then
            [ -0.1179007292
              -0.0163885057
              0.005143642426
              -0.07373118401
              -0.119107008
              -0.08132505417
              0.1460343599 ]
        else
            [ -0.04086410999
              -0.2739132643
              0.1320786476
              0.
              0.4081912041
              0.06568455696
              -0.1403121948 ]

    let path =
        Translation.waypoints Translation.defaults chain seed (Geometry.v 0. 0. distance)
        |> complete

    assertPath 0.1 path
    Assert.Equal<float list>(seed, path.Head)

[<Fact>]
let ``parallel translation calls preserve deterministic complete waypoints`` () =
    let seed = [ 0.6; -1.2; 0.6 ]

    let expected =
        Translation.waypoints Translation.defaults multi seed (Geometry.v -0.005 0. 0.)

    let concurrent =
        [| 1..8 |]
        |> Array.Parallel.map (fun _ -> Translation.waypoints Translation.defaults multi seed (Geometry.v -0.005 0. 0.))

    Assert.True(concurrent |> Array.forall ((=) expected))


[<Fact>]
let ``floating point rounding cannot widen the local joint bound`` () =
    let result =
        Solver.solveNear tight 0.1 single (Geometry.forward single [ 0.4 ]) [ 0.3 ]
        |> solved

    Assert.True(abs (result.Angles.Head - 0.3) <= 0.1)

    let tiny =
        Solver.solveNear tight System.Double.Epsilon single (Geometry.forward single [ 0.75 ]) [ 0.75 ]
        |> solved

    Assert.Equal<float list>([ 0.75 ], tiny.Angles)

[<Fact>]
let ``straight Axol arms solve nearby legal boundary targets locally`` () =
    for side in [ "left"; "right" ] do
        let chain =
            Geometry.load (Path.Combine(AppContext.BaseDirectory, "model.urdf")) "root" (side + "_gripper")

        let bounds = Geometry.limits chain

        let goal =
            bounds
            |> List.mapi (fun index limit ->
                if index = 3 then
                    System.Math.Clamp((if side = "left" then 0.01 else -0.01), limit.Lower, limit.Upper)
                else
                    0.)

        let seed = List.replicate 7 0.

        let result =
            Solver.solveNear tight 0.1 chain (Geometry.forward chain goal) seed |> solved

        Assert.True(changes seed result.Angles |> List.forall (fun value -> value <= 0.1))
