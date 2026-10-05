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
