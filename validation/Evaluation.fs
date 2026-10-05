namespace FsIK.Validation

open System
open System.Diagnostics
open System.IO
open System.Runtime.InteropServices
open System.Security.Cryptography
open System.Text.Json
open FsIK

type CaseReport =
    { Name: string
      Group: string
      Side: string
      Success: bool
      Milliseconds: float
      Iterations: int
      PositionErrorMm: float
      OrientationErrorDegrees: float
      Seed: float list
      Angles: float list
      TargetPosition: float list
      TargetQuaternion: float list }

module Evaluation =
    let private write path value =
        File.WriteAllText(path, JsonSerializer.Serialize(value, JsonSerializerOptions(WriteIndented = true)))

    let private percentile p values =
        let sorted = values |> List.sort

        if sorted.IsEmpty then
            0.
        else
            sorted[int (ceil (p * float sorted.Length)) - 1]

    let private solve (sample: EndpointCase) =
        let beginTime = Stopwatch.GetTimestamp()
        let result = Solver.solve Fixtures.options sample.Chain sample.Target sample.Seed
        let milliseconds = Stopwatch.GetElapsedTime(beginTime).TotalMilliseconds

        let success, info =
            match result with
            | Solved solution -> true, solution
            | Failed(NoConvergence solution) -> false, solution
            | Failed(InvalidInput error) -> failwith error

        { Name = sample.Name
          Group = sprintf "%A" sample.Group
          Side = sprintf "%A" sample.Side
          Success = success
          Milliseconds = milliseconds
          Iterations = info.Iterations
          PositionErrorMm = info.PositionError * 1000.
          OrientationErrorDegrees = info.OrientationError * 180. / Math.PI
          Seed = sample.Seed
          Angles = info.Angles
          TargetPosition = [ sample.Target.Position.X; sample.Target.Position.Y; sample.Target.Position.Z ]
          TargetQuaternion =
            [ sample.Target.Orientation.W
              sample.Target.Orientation.X
              sample.Target.Orientation.Y
              sample.Target.Orientation.Z ] }

    let private jacobianCheck chain seed =
        let analytic = Geometry.jacobianRows chain seed
        let epsilon = 1e-6

        [ for column in 0 .. seed.Length - 1 do
              let shifted direction =
                  seed
                  |> List.mapi (fun i q -> if i = column then q + direction * epsilon else q)
                  |> Geometry.forward chain

              let plus, minus = shifted 1., shifted -1.

              let dp =
                  Geometry.subtract plus.Position minus.Position
                  |> Geometry.scale (1. / (2. * epsilon))

              let dr =
                  Geometry.rotationError plus.Orientation minus.Orientation
                  |> Geometry.scale (1. / (2. * epsilon))

              for row, value in List.indexed [ dp.X; dp.Y; dp.Z; dr.X; dr.Y; dr.Z ] do
                  yield abs (analytic[row][column] - value) ]
        |> List.max

    let run (outputPath: string) =
        let output = Path.GetFullPath outputPath
        Directory.CreateDirectory output |> ignore
        let data = Fixtures.load ()

        let first =
            Fixtures.named data |> List.find (fun sample -> sample.Name = "left-up-50.8mm")

        let cold = solve first
        let warm = [ for _ in 1..300 -> solve first ]
        let corpus = Fixtures.named data @ Fixtures.reachable data @ Fixtures.cartesian data

        let reports =
            corpus
            |> List.mapi (fun index sample ->
                let result = solve sample

                if index % 100 = 0 then
                    printfn "Correctness corpus: %d/%d" (index + 1) corpus.Length

                result)

        let groups =
            reports
            |> List.groupBy _.Group
            |> List.map (fun (group, rows) ->
                let passed = rows |> List.filter _.Success

                {| Group = group
                   Total = rows.Length
                   Solved = passed.Length
                   MedianMs = percentile 0.5 (rows |> List.map _.Milliseconds)
                   P95Ms = percentile 0.95 (rows |> List.map _.Milliseconds)
                   MaxPositionErrorMm = passed |> List.map _.PositionErrorMm |> List.fold max 0.
                   MaxOrientationErrorDegrees = passed |> List.map _.OrientationErrorDegrees |> List.fold max 0. |})

        let seed = first.Seed
        let target = first.Target

        let malformed =
            [ Solver.solve Fixtures.options first.Chain target [ 0. ]
              Solver.solve
                  Fixtures.options
                  first.Chain
                  target
                  (seed |> List.mapi (fun i q -> if i = 0 then Double.NaN else q))
              Solver.solve
                  Fixtures.options
                  first.Chain
                  { target with
                      Position = Geometry.v Double.NaN 0. 0. }
                  seed
              Solver.solve
                  Fixtures.options
                  first.Chain
                  { target with
                      Orientation = { W = 2.; X = 0.; Y = 0.; Z = 0. } }
                  seed
              Solver.solve
                  Fixtures.options
                  first.Chain
                  target
                  (seed |> List.mapi (fun i q -> if i = 3 then -0.01 else q))
              Solver.solve
                  { Fixtures.options with
                      MaxStep = Double.NaN }
                  first.Chain
                  target
                  seed
              Solver.solve
                  { Fixtures.options with
                      MaxIterations = 0 }
                  first.Chain
                  target
                  seed ]

        let malformedRejected =
            malformed
            |> List.filter (function
                | Failed(InvalidInput _) -> true
                | _ -> false)
            |> List.length

        let distant =
            solve
                { first with
                    Name = "unreachable-3m"
                    Target =
                        { target with
                            Position = Geometry.add target.Position (Geometry.v 0. 0. 3.) } }

        let jacobianError =
            [ jacobianCheck data.LeftChain (List.take 7 data.Recorded)
              jacobianCheck data.RightChain (List.skip 7 data.Recorded)
              jacobianCheck data.LeftChain (List.replicate 7 0.)
              jacobianCheck data.RightChain (List.replicate 7 0.) ]
            |> List.max

        let paths =
            Fixtures.paths data
            |> List.map (fun (name, side, delta) ->
                let start = Fixtures.angles side data.Recorded
                let beginTime = Stopwatch.GetTimestamp()
                let result = Translation.plan (Fixtures.chain side data) start delta
                let milliseconds = Stopwatch.GetElapsedTime(beginTime).TotalMilliseconds

                match result with
                | Error error ->
                    {| Name = name
                       Side = sprintf "%A" side
                       Success = false
                       Detail = error
                       PlanningMs = milliseconds
                       DurationSeconds = 0.
                       Frames = 0
                       Start = data.Recorded
                       Delta = [ delta.X; delta.Y; delta.Z ]
                       File = "" |}
                | Ok trajectory ->
                    let file = name + ".path.json"

                    write
                        (Path.Combine(output, file))
                        (trajectory.Samples |> List.map (Fixtures.join side data.Recorded))

                    {| Name = name
                       Side = sprintf "%A" side
                       Success = true
                       Detail = "Timed path generated; collision checking excluded"
                       PlanningMs = milliseconds
                       DurationSeconds = trajectory.DurationSeconds
                       Frames = trajectory.Samples.Length
                       Start = data.Recorded
                       Delta = [ delta.X; delta.Y; delta.Z ]
                       File = file |})

        let summary =
            {| CreatedUtc = DateTimeOffset.UtcNow
               ModelSha256 = Convert.ToHexStringLower(SHA256.HashData(File.ReadAllBytes data.ModelPath))
               Runtime = RuntimeInformation.FrameworkDescription
               OS = RuntimeInformation.OSDescription
               Architecture = RuntimeInformation.ProcessArchitecture.ToString()
               LogicalProcessors = Environment.ProcessorCount
               Provider = MathNet.Numerics.Providers.LinearAlgebra.LinearAlgebraControl.Provider.ToString()
               SolverAssemblySha256 =
                Convert.ToHexStringLower(SHA256.HashData(File.ReadAllBytes(typeof<Options>.Assembly.Location)))
               PositionToleranceMm = Fixtures.options.PositionTolerance * 1000.
               OrientationToleranceDegrees = Fixtures.options.OrientationTolerance * 180. / Math.PI
               ColdFirstIkMs = cold.Milliseconds
               WarmTrials = warm.Length
               WarmMedianMs = percentile 0.5 (warm |> List.map _.Milliseconds)
               WarmP95Ms = percentile 0.95 (warm |> List.map _.Milliseconds)
               Groups = groups
               MalformedTotal = malformed.Length
               MalformedRejected = malformedRejected
               DistantTargetRejected = not distant.Success
               JacobianMaxAbsoluteError = jacobianError
               Paths = paths
               HardwareConnected = false
               SdkCollisionChecksIncluded = false |}

        write (Path.Combine(output, "summary.json")) summary
        write (Path.Combine(output, "cases.json")) reports
        write (Path.Combine(output, "warm-trials.json")) warm

        printfn
            "First IK call %.3f ms; warm median %.3f ms; p95 %.3f ms"
            summary.ColdFirstIkMs
            summary.WarmMedianMs
            summary.WarmP95Ms

        for group in groups do
            printfn
                "%s: %d/%d solved; median %.3f ms; p95 %.3f ms"
                group.Group
                group.Solved
                group.Total
                group.MedianMs
                group.P95Ms

        for path in paths do
            printfn
                "%s: success=%b, planning %.3f ms, %d frames, duration %.3f s"
                path.Name
                path.Success
                path.PlanningMs
                path.Frames
                path.DurationSeconds

        printfn
            "Malformed %d/%d rejected; distant target rejected=%b; Jacobian error %.3g"
            malformedRejected
            malformed.Length
            summary.DistantTargetRejected
            jacobianError

        if
            cold.Success
            && List.forall _.Success warm
            && (groups
                |> List.exists (fun group -> group.Group = "Named" && group.Solved = group.Total))
            && malformedRejected = malformed.Length
            && summary.DistantTargetRejected
            && jacobianError < 1e-7
            && (paths |> List.forall _.Success)
        then
            0
        else
            1
