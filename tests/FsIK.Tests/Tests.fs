module FsIK.Tests

open System
open System.IO
open FsIK
open Xunit

let model = Path.Combine(AppContext.BaseDirectory, "model.urdf")
let left = Geometry.load model "root" "left_gripper"

let seed =
    [ -0.04086410999
      -0.2739132643
      0.1320786476
      0.
      0.4081912041
      0.06568455696
      -0.1403121948 ]

[<Theory>]
[<InlineData(0.01)>]
[<InlineData(0.0508)>]
let ``translation reaches requested tip height and holds orientation`` distance =
    let initial = Geometry.forward left seed
    let result = Translation.plan left seed (Geometry.v 0. 0. distance)

    match result with
    | Error error -> failwith error
    | Ok trajectory ->
        let last = Geometry.forward left (List.last trajectory.Samples)

        let target =
            { initial with
                Position = Geometry.add initial.Position (Geometry.v 0. 0. distance) }

        let positionError, orientationError = Geometry.poseError target last
        Assert.True(positionError < 1e-5)
        Assert.True(orientationError < 1e-4)

        let velocity =
            trajectory.Samples
            |> List.pairwise
            |> List.map (fun (a, b) -> List.map2 (fun x y -> (y - x) * 120.) a b)

        Assert.True(velocity |> List.collect id |> List.forall (fun v -> abs v <= 0.100001))

        let acceleration =
            (List.replicate 7 0. :: velocity) @ [ List.replicate 7 0. ]
            |> List.pairwise
            |> List.collect (fun (a, b) -> List.map2 (fun x y -> abs (y - x) * 120.) a b)

        Assert.True(acceleration |> List.forall (fun v -> v <= 0.200001))

[<Fact>]
let ``invalid translations cannot generate motion`` () =
    for delta in [ Geometry.v 0. 0. 0.; Geometry.v 0. 0. 0.2; Geometry.v Double.NaN 0. 0. ] do
        Assert.True(Translation.plan left seed delta |> Result.isError)

[<Theory>]
[<InlineData(0.01)>]
[<InlineData(0.0508)>]
let ``continuous timing preserves the Cartesian path and sampled motion limits`` distance =
    let delta = Geometry.v 0. 0. distance

    let get =
        function
        | Ok value -> value
        | Error error -> failwithf "%A" error

    let old = Translation.plan left seed delta |> get
    let path = Translation.planContinuous left seed delta |> get

    Assert.True(
        path.DurationSeconds < old.DurationSeconds,
        sprintf "old %.3f s; continuous %.3f s" old.DurationSeconds path.DurationSeconds
    )

    Assert.Equal<float list>(seed, path.Samples.Head)
    Assert.Equal<float list>(List.last old.Samples, List.last path.Samples)
    let initial = Geometry.forward left seed

    for angles in path.Samples do
        let frame = Geometry.forward left angles
        let displacement = Geometry.subtract frame.Position initial.Position
        Assert.True(abs displacement.X < 0.0001 && abs displacement.Y < 0.0001)

        let _, error =
            Geometry.poseError
                { frame with
                    Orientation = initial.Orientation }
                frame

        Assert.True(error < 0.00174533)

    let velocity =
        path.Samples
        |> List.pairwise
        |> List.map (fun (a, b) -> List.map2 (fun x y -> (y - x) * 120.) a b)

    Assert.True(velocity |> List.collect id |> List.forall (fun value -> abs value <= 0.100001))
    let padded = (List.replicate 7 0. :: velocity) @ [ List.replicate 7 0. ]

    Assert.True(
        padded
        |> List.pairwise
        |> List.collect (fun (a, b) -> List.map2 (fun x y -> abs (y - x) * 120.) a b)
        |> List.forall (fun value -> value <= 0.200001)
    )

[<Fact>]
let ``continuous retiming is deterministic concurrent and rejects invalid paths`` () =
    let target =
        seed |> List.mapi (fun i angle -> if i = 0 then angle + 0.01 else angle)

    let input = [ seed; target; seed ]

    let run () =
        Trajectory.retimeContinuous Trajectory.conservative left input

    let expected = run ()

    let results =
        [| for _ in 1..8 -> System.Threading.Tasks.Task.Run(fun () -> run ()) |]

    System.Threading.Tasks.Task.WaitAll(results |> Array.map (fun task -> task :> System.Threading.Tasks.Task))

    for task in results do
        Assert.Equal(expected, task.Result)

    Assert.True(
        Trajectory.retimeContinuous Trajectory.conservative left [ seed ]
        |> Result.isError
    )

    Assert.True(
        Trajectory.retimeContinuous
            { Trajectory.conservative with
                MaxJointVelocity = 0. }
            left
            input
        |> Result.isError
    )
