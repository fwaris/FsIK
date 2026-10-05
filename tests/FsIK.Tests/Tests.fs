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
