open System
open System.Diagnostics
open System.IO
open System.Text.Json
open FsIK
open FsIK.Validation

let write path value =
    File.WriteAllText(path, JsonSerializer.Serialize(value, JsonSerializerOptions(WriteIndented = true)))

let values name (row: JsonElement) =
    row.GetProperty(name: string).EnumerateArray()
    |> Seq.map _.GetDouble()
    |> Seq.toList

let text name (row: JsonElement) =
    row.GetProperty(name: string).GetString()
    |> Option.ofObj
    |> Option.defaultWith (fun () -> failwith ("Missing string: " + name))

let evaluateFrozen data (row: JsonElement) =
    let side =
        if row.GetProperty("Side").GetString() = "Left" then
            Left
        else
            Right

    let p, q = values "TargetPosition" row, values "TargetQuaternion" row
    let seed = values "Seed" row

    let target =
        { Position = Geometry.v p[0] p[1] p[2]
          Orientation =
            { W = q[0]
              X = q[1]
              Y = q[2]
              Z = q[3] } }

    let start = Stopwatch.GetTimestamp()
    let result = Solver.solve Fixtures.options (Fixtures.chain side data) target seed

    let success, solution =
        match result with
        | Solved solution -> true, solution
        | Failed(NoConvergence solution) -> false, solution
        | Failed(InvalidInput reason) -> failwith reason

    { Name = text "Name" row
      Group = text "Group" row
      Side = text "Side" row
      Success = success
      Milliseconds = Stopwatch.GetElapsedTime(start).TotalMilliseconds
      Iterations = solution.Iterations
      PositionErrorMm = solution.PositionError * 1000.
      OrientationErrorDegrees = solution.OrientationError * 180. / Math.PI
      Seed = seed
      Angles = solution.Angles
      TargetPosition = p
      TargetQuaternion = q }

let measure name iterations (operation: unit -> int) =
    for _ in 1..10 do
        operation () |> ignore
    // Allocation counters surround the complete synchronous public operation.
    let rounds =
        [ for _ in 1..3 do
              GC.Collect()
              GC.WaitForPendingFinalizers()
              GC.Collect()
              let before = GC.GetAllocatedBytesForCurrentThread()
              let clock = Stopwatch.GetTimestamp()
              let mutable consumed = 0

              for _ in 1..iterations do
                  consumed <- consumed + operation ()

              let elapsed = Stopwatch.GetElapsedTime(clock).TotalMilliseconds / float iterations

              let allocated =
                  float (GC.GetAllocatedBytesForCurrentThread() - before) / float iterations

              if consumed = 0 then
                  failwith "Measurement did not consume valid results"

              yield allocated, elapsed ]

    {| Name = name
       Bytes = rounds |> List.map fst |> List.sort |> List.item 1
       Milliseconds = rounds |> List.map snd |> List.sort |> List.item 1 |}

[<EntryPoint>]
let main args =
    let output = Path.GetFullPath args[0]
    let corpusPath = Path.GetFullPath args[1]
    Directory.CreateDirectory output |> ignore
    let code = Evaluation.run output
    let data = Fixtures.load ()
    use corpus = JsonDocument.Parse(File.ReadAllText corpusPath)

    let reports =
        corpus.RootElement.EnumerateArray()
        |> Seq.map (evaluateFrozen data)
        |> Seq.toArray

    write (Path.Combine(output, "frozen-cases.json")) reports

    let named = Fixtures.named data

    let endpoints =
        [ "left-up-10mm"
          "left-up-50.8mm"
          "right-up-50.8mm"
          "left-straight-up-50.8mm" ]
        |> List.map (fun name ->
            let case = named |> List.find (fun sample -> sample.Name = name)

            measure name 40 (fun () ->
                match Solver.solve Fixtures.options case.Chain case.Target case.Seed with
                | Solved result -> result.Angles.Length
                | _ -> failwith ("Named endpoint failed: " + name)))

    let paths =
        [ 0.01; 0.0508 ]
        |> List.map (fun distance ->
            measure (sprintf "left-path-%gmm" (distance * 1000.)) 8 (fun () ->
                match
                    Translation.plan data.LeftChain (Fixtures.angles Left data.Recorded) (Geometry.v 0. 0. distance)
                with
                | Ok path -> path.Samples.Length
                | Error reason -> failwith reason))

    write (Path.Combine(output, "allocation.json")) (endpoints @ paths)

    // Repeated and concurrent calls must produce the same immutable output.
    let samples =
        corpus.RootElement.EnumerateArray()
        |> Seq.filter (fun row -> row.GetProperty("Group").GetString() = "HeldOutReachable")
        |> Seq.truncate 12
        |> Seq.toArray

    let sequential = samples |> Array.map (evaluateFrozen data)
    let concurrent = samples |> Array.Parallel.map (evaluateFrozen data)

    let deterministic =
        Array.forall2 (fun a b -> a.Success = b.Success && a.Angles = b.Angles) sequential concurrent

    write (Path.Combine(output, "determinism.json")) {| Passed = deterministic |}
    if code = 0 && deterministic then 0 else 1
