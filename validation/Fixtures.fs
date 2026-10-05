namespace FsIK.Validation

open System
open System.IO
open System.Text.Json
open FsIK

type Side =
    | Left
    | Right

type CaseGroup =
    | Named
    | NearReachable
    | WideReachable
    | CartesianProbe

type EndpointCase =
    { Name: string
      Group: CaseGroup
      Side: Side
      Chain: Chain
      Seed: float list
      Target: Pose }

type Fixture =
    { ModelPath: string
      Recorded: float list
      LeftChain: Chain
      RightChain: Chain }

module Fixtures =
    let options =
        { Solver.defaults with
            PositionTolerance = 1e-5
            OrientationTolerance = 1e-4 }

    let private file envName bundled source =
        [ Environment.GetEnvironmentVariable envName |> Option.ofObj
          Some(Path.Combine(AppContext.BaseDirectory, bundled))
          Some(Path.Combine(__SOURCE_DIRECTORY__, source)) ]
        |> List.choose id
        |> List.tryFind File.Exists
        |> Option.map Path.GetFullPath
        |> Option.defaultWith (fun () -> failwith ("Benchmark fixture missing: " + bundled))

    let load () : Fixture =
        let model = file "AXOL_BENCH_MODEL" "model.urdf" "fixtures/axol_arms.urdf"
        let left = Geometry.load model "root" "left_gripper"
        let right = Geometry.load model "root" "right_gripper"

        use pose =
            JsonDocument.Parse(
                File.ReadAllText(file "AXOL_BENCH_POSE" "recorded-pose.json" "fixtures/recorded-pose.json")
            )

        let measured =
            pose.RootElement.GetProperty("measured_start").EnumerateArray()
            |> Seq.map _.GetDouble()
            |> Seq.toList

        let bounds = Geometry.limits left @ Geometry.limits right

        if measured.Length <> 14 then
            failwith "Fourteen measured joints required"

        let recorded =
            List.map2
                (fun limit value ->
                    let projected = Math.Clamp(value, limit.Lower, limit.Upper)

                    if not (Double.IsFinite value) || abs (projected - value) > Math.PI / 900. then
                        failwith "Recorded encoder reading exceeds approved 0.2 degree allowance"

                    projected)
                bounds
                measured

        { ModelPath = model
          Recorded = recorded
          LeftChain = left
          RightChain = right }

    let chain side data =
        if side = Left then data.LeftChain else data.RightChain

    let angles side values =
        if side = Left then
            List.take 7 values
        else
            List.skip 7 values

    let join side values selected =
        if side = Left then
            selected @ List.skip 7 values
        else
            List.take 7 values @ selected

    let endpoint name group data side start delta =
        let selected = chain side data
        let seed = angles side start
        let pose = Geometry.forward selected seed

        { Name = name
          Group = group
          Side = side
          Chain = selected
          Seed = seed
          Target =
            { pose with
                Position = Geometry.add pose.Position delta } }

    let named data =
        [ endpoint "left-up-10mm" Named data Left data.Recorded (Geometry.v 0. 0. 0.01)
          endpoint "left-up-50.8mm" Named data Left data.Recorded (Geometry.v 0. 0. 0.0508)
          endpoint "right-up-50.8mm" Named data Right data.Recorded (Geometry.v 0. 0. 0.0508)
          endpoint "left-straight-up-50.8mm" Named data Left (List.replicate 14 0.) (Geometry.v 0. 0. 0.0508)
          endpoint "right-straight-up-50.8mm" Named data Right (List.replicate 14 0.) (Geometry.v 0. 0. 0.0508)
          endpoint "left-unchanged" Named data Left data.Recorded (Geometry.v 0. 0. 0.) ]

    let reachable data =
        // Randomness belongs only to this deterministic fixture, never the solver.
        let random = Random 20261005

        [ for index in 1..1000 do
              let side = if index % 2 = 0 then Left else Right
              let selected = chain side data
              let initial = angles side data.Recorded
              let group = if index <= 800 then NearReachable else WideReachable
              let bounds = Geometry.limits selected

              let seed =
                  if group = NearReachable then
                      initial
                  else
                      bounds
                      |> List.map (fun bound -> bound.Lower + random.NextDouble() * (bound.Upper - bound.Lower))

              let target =
                  List.map2
                      (fun bound q ->
                          if group = NearReachable then
                              Math.Clamp(q + (random.NextDouble() * 2. - 1.) * 0.8, bound.Lower, bound.Upper)
                          else
                              bound.Lower + random.NextDouble() * (bound.Upper - bound.Lower))
                      bounds
                      seed

              yield
                  { Name = sprintf "%A-%A-%04d" group side index
                    Group = group
                    Side = side
                    Chain = selected
                    Seed = seed
                    Target = Geometry.forward selected target } ]

    let cartesian data =
        let starts =
            [ "recorded", data.Recorded
              "straight", List.replicate 14 0.
              "rest", [ -0.2; -0.4; 0.; 0.7; 0.; -0.3; 0.; 0.2; 0.4; 0.; -0.7; 0.; 0.3; 0. ] ]

        let directions =
            [ "forward", Geometry.v 1. 0. 0.
              "backward", Geometry.v -1. 0. 0.
              "left", Geometry.v 0. 1. 0.
              "right", Geometry.v 0. -1. 0.
              "up", Geometry.v 0. 0. 1.
              "down", Geometry.v 0. 0. -1. ]

        [ for poseName, start in starts do
              for side in [ Left; Right ] do
                  for directionName, direction in directions do
                      for distance in [ 0.01; 0.0254; 0.0508 ] do
                          yield
                              endpoint
                                  (sprintf "%s-%A-%s-%gmm" poseName side directionName (distance * 1000.))
                                  CartesianProbe
                                  data
                                  side
                                  start
                                  (Geometry.scale distance direction) ]

    let paths data =
        [ for side in [ Left; Right ] do
              for distance in [ 0.01; 0.0508 ] do
                  yield sprintf "%A-up-%gmm" side (distance * 1000.), side, Geometry.v 0. 0. distance ]
