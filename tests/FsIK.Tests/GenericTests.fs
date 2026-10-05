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
    let opposite = { W = -1.; X = 0.; Y = 0.; Z = 0. }
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
