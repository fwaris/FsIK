module FsIK.GenericTests

open System
open System.IO
open FsIK
open Xunit

let chain =
    Geometry.load (Path.Combine(AppContext.BaseDirectory, "planar.urdf")) "base" "tip"

[<Fact>]
let ``one revolute chain solves without Axol joint assumptions`` () =
    let target = Geometry.forward chain [ 0.4 ]

    match Solver.solve Solver.defaults chain target [ 0. ] with
    | Solved solution ->
        Assert.Single(solution.Angles) |> ignore

        let position, orientation =
            Geometry.poseError target (Geometry.forward chain solution.Angles)

        Assert.True(position <= Solver.defaults.PositionTolerance)
        Assert.True(orientation <= Solver.defaults.OrientationTolerance)
        Assert.InRange(solution.Angles.Head, -0.75, 0.75)
    | Failed failure -> failwithf "%A" failure

[<Fact>]
let ``standalone assembly rejects illegal seed and preserves parallel determinism`` () =
    Assert.Equal("FsIK", typeof<Chain>.Assembly.GetName().Name)
    let target = Geometry.forward chain [ 0.4 ]

    match Solver.solve Solver.defaults chain target [ 0.8 ] with
    | Failed(InvalidInput _) -> ()
    | unexpected -> failwithf "Illegal seed accepted: %A" unexpected

    let sequential = Solver.solve Solver.defaults chain target [ 0. ]

    let concurrentResults =
        [| 1..16 |]
        |> Array.Parallel.map (fun _ -> Solver.solve Solver.defaults chain target [ 0. ])

    Assert.True(concurrentResults |> Array.forall ((=) sequential))


[<Fact>]
let ``public geometry retains reference records and double precision`` () =
    Assert.False(typeof<Vec3>.IsValueType)
    Assert.False(typeof<Quaternion>.IsValueType)
    Assert.False(typeof<Pose>.IsValueType)
    let delta = Geometry.v 0.00000001 0. 0.
    let position = Geometry.v 16. 0. 0.
    let result = Geometry.add position delta
    Assert.Equal(16.00000001, result.X)
    Assert.True(result.X > position.X)
    let opposite: Quaternion = { W = -1.; X = 0.; Y = 0.; Z = 0. }
    Assert.Equal(0., Geometry.norm (Geometry.rotationError Geometry.identity.Orientation opposite))

[<Fact>]
let ``generic forward and jacobian match analytic planar chain at both bounds`` () =
    for angle in [ -0.75; -0.4; 0.; 0.4; 0.75 ] do
        let pose = Geometry.forward chain [ angle ]
        Assert.InRange(abs (pose.Position.X - 0.3 * cos angle), 0., 1e-14)
        Assert.InRange(abs (pose.Position.Y - 0.3 * sin angle), 0., 1e-14)
        let rows = Geometry.jacobianRows chain [ angle ]
        let expected = [ -0.3 * sin angle; 0.3 * cos angle; 0.; 0.; 0.; 1. ]
        List.iter2 (fun (row: float list) value -> Assert.InRange(abs (row.Head - value), 0., 1e-14)) rows expected

[<Fact>]
let ``value traversal rejects missing and excess joint angles`` () =
    for angles in [ []; [ 0.; 0. ] ] do
        Assert.Throws<ArgumentException>(fun () -> Geometry.forward chain angles |> ignore)
        |> ignore

        Assert.Throws<ArgumentException>(fun () -> Geometry.jacobianRows chain angles |> ignore)
        |> ignore


[<Fact>]
let ``retiming scratch is isolated across segments calls and concurrent plans`` () =
    let input = [ [ 0. ]; [ 0.2 ]; [ 0.2 ]; [ -0.3 ]; [ 0. ] ]
    let expectedInput = [ [ 0. ]; [ 0.2 ]; [ 0.2 ]; [ -0.3 ]; [ 0. ] ]

    let trajectory =
        match Trajectory.retime Trajectory.conservative chain input with
        | Ok result -> result
        | Error failure -> failwithf "%A" failure

    let retainedFrames = trajectory.Samples |> List.map List.toArray
    Assert.Equal<float list list>(expectedInput, input)
    Assert.Equal<float list>([ 0. ], trajectory.Samples.Head)
    Assert.Equal<float list>([ 0. ], List.last trajectory.Samples)
    Assert.True(trajectory.Samples |> List.exists (fun angles -> angles.Head >= 0.19))
    Assert.True(trajectory.Samples |> List.exists (fun angles -> angles.Head <= -0.29))

    let concurrent =
        [| 1..8 |]
        |> Array.Parallel.map (fun _ -> Trajectory.retime Trajectory.conservative chain input)

    Assert.True(concurrent |> Array.forall ((=) (Ok trajectory)))
    Trajectory.retime Trajectory.conservative chain [ [ -0.5 ]; [ 0.5 ] ] |> ignore
    Assert.Equal<float list list>(expectedInput, input)

    List.iter2
        (fun angles (retained: float array) -> Assert.Equal<float>(retained, List.toArray angles))
        trajectory.Samples
        retainedFrames


[<Fact>]
let ``solver workspace never overwrites retained results or mixes concurrent targets`` () =
    let targets =
        [| -0.7; -0.4; 0.; 0.2; 0.65; 0.75 |]
        |> Array.map (fun angle -> Geometry.forward chain [ angle ])

    let expected =
        targets
        |> Array.map (fun target -> Solver.solve Solver.defaults chain target [ 0. ])

    let retained = expected |> Array.map (fun result -> sprintf "%A" result)

    for target in targets do
        match
            Solver.solve
                { Solver.defaults with
                    MaxIterations = 1 }
                chain
                target
                [ -0.7 ]
        with
        | Solved _
        | Failed(NoConvergence _) -> ()
        | failure -> failwithf "%A" failure

    for _ in 1..4 do
        let concurrent =
            targets
            |> Array.Parallel.map (fun target -> Solver.solve Solver.defaults chain target [ 0. ])

        Assert.True(Array.forall2 (=) expected concurrent)

    Assert.Equal<string>(retained, expected |> Array.map (fun result -> sprintf "%A" result))

    for target in targets do
        match Solver.solve Solver.defaults chain target [ 0.8 ] with
        | Failed(InvalidInput _) -> ()
        | failure -> failwithf "%A" failure

    Assert.True(
        Array.forall2
            (=)
            expected
            (targets
             |> Array.map (fun target -> Solver.solve Solver.defaults chain target [ 0. ]))
    )
