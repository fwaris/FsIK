#r "../src/FsIK/bin/Release/net10.0/FsIK.dll"
#r "../src/FsMotion/bin/Release/net10.0/FsMotion.dll"

open System
open System.IO
open System.Text.Json
open System.Threading
open FsIK
open FsMotion

let unwrap =
    function
    | Ok value -> value
    | Error error -> failwithf "%A" error

let root = Path.GetFullPath(Path.Combine(__SOURCE_DIRECTORY__, ".."))

let model =
    Model.load (Path.Combine(root, "tests/FsMotion.Tests/arm.urdf")) (fun _ -> failwith "No mesh expected") []
    |> unwrap

let chain = Model.chain model "tip" |> unwrap

let obstacle: Obstacle =
    { Name = "blocking-object"
      Shape =
        { A = Geometry.v 0.55 0. 0.
          B = Geometry.v 0.55 0. 0.
          Radius = 0.055 } }

let scene =
    { Id = "offline-example-1"
      Obstacles = [ obstacle ] }

let options = { Planner.defaults with Margin = 0.005 }
let checker = Collision.create model scene options.Margin 20 |> unwrap
let start = Map.ofList [ "shoulder", -0.8; "elbow", 0.; "otherJoint", 0.37 ]
let goal = Map.add "shoulder" 0.8 start

let route =
    Planner.plan options model scene.Id checker (Geometry.names chain) start goal CancellationToken.None
    |> unwrap

let timed =
    Planner.retime Trajectory.conservative model chain checker route CancellationToken.None
    |> unwrap

let directory = Path.Combine(root, "artifacts/motion-example")
Directory.CreateDirectory directory |> ignore

let payload =
    {| model_id = route.ModelId
       scene_id = route.SceneId
       active_joints = route.ActiveJoints
       waypoints = route.Configurations |> List.toArray
       samples = timed.Samples |> List.toArray
       rate = timed.Rate
       duration_s = timed.DurationSeconds
       nodes = route.NodesExplored |}

File.WriteAllText(
    Path.Combine(directory, "detour.json"),
    JsonSerializer.Serialize(payload, JsonSerializerOptions(WriteIndented = true))
)

printfn
    "Offline obstacle detour: %d waypoints, %d nodes, %d timed samples, %.3f s movement duration"
    route.Configurations.Length
    route.NodesExplored
    timed.Samples.Length
    timed.DurationSeconds

printfn "No robot transport or motors used. Plan: %s" (Path.Combine(directory, "detour.json"))
