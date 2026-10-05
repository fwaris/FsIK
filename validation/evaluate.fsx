open System
open System.Diagnostics
open System.Globalization
open System.IO
open System.Security.Cryptography
open System.Text
open System.Text.Json
open System.Xml.Linq

type CheckResult =
    | Passed
    | Failed of string

type Evaluation =
    { Checks: Map<string, CheckResult>
      Metrics: Map<string, decimal>
      Evidence: string list }

type CommandResult =
    | Completed of int * string
    | TimedOut of string

let root = Path.GetFullPath(Path.Combine(__SOURCE_DIRECTORY__, ".."))
let control = Path.Combine(root, "validation")

let resultPath =
    (if fsi.CommandLineArgs.Length > 1 then
         Some fsi.CommandLineArgs[1]
     else
         Environment.GetEnvironmentVariable "FSHARNESS_RESULT_PATH" |> Option.ofObj)
    |> Option.defaultValue (Path.Combine(root, "artifacts/validation/latest/evaluation.json"))
    |> Path.GetFullPath

let output = Path.Combine(Path.GetDirectoryName resultPath, "ik-evidence")
Directory.CreateDirectory output |> ignore

let python =
    Environment.GetEnvironmentVariable "FSIK_PYTHON"
    |> Option.ofObj
    |> Option.defaultValue "python"

let initial =
    { Checks =
        [ "build"
          "tests"
          "numerical_correctness"
          "regressions"
          "reachability"
          "independent_validation"
          "latency"
          "determinism"
          "contract" ]
        |> List.map (fun name -> name, Failed "Not run")
        |> Map.ofList
      Metrics = Map.ofList [ ("priority_score", 999999999M) ]
      Evidence = [] }

let check name passed reason state =
    { state with
        Checks = state.Checks |> Map.add name (if passed then Passed else Failed reason) }

let tail (value: string) =
    value.Substring(max 0 (value.Length - 2400))

let command label executable args timeoutSeconds =
    let settings = ProcessStartInfo(executable)
    settings.UseShellExecute <- false
    settings.CreateNoWindow <- true
    settings.WorkingDirectory <- root
    settings.RedirectStandardOutput <- true
    settings.RedirectStandardError <- true
    settings.StandardOutputEncoding <- UTF8Encoding(false)
    settings.StandardErrorEncoding <- UTF8Encoding(false)
    args |> List.iter settings.ArgumentList.Add
    use child = new Process(StartInfo = settings)
    child.Start() |> ignore
    let stdout = child.StandardOutput.ReadToEndAsync()
    let stderr = child.StandardError.ReadToEndAsync()

    if child.WaitForExit(timeoutSeconds * 1000) then
        let text = stdout.Result + stderr.Result
        File.WriteAllText(Path.Combine(output, label + ".log"), text)
        printfn "%s: exit %d" label child.ExitCode
        Completed(child.ExitCode, text)
    else
        child.Kill(true)
        child.WaitForExit()
        let text = stdout.Result + stderr.Result
        File.WriteAllText(Path.Combine(output, label + ".log"), text)
        TimedOut(label + " exceeded its bounded timeout")

let executeCheck name executable args timeout state =
    match command name executable args timeout with
    | Completed(0, _) -> check name true "" state
    | Completed(_, text) ->
        let failed = check name false (tail text) state

        { failed with
            Evidence = state.Evidence @ [ name + ": " + tail text ] }
    | TimedOut reason ->
        let failed = check name false reason state

        { failed with
            Evidence = state.Evidence @ [ reason ] }

let passed name state = Map.find name state.Checks = Passed

let readJson path =
    use document = JsonDocument.Parse(File.ReadAllText path)
    document.RootElement.Clone()

let load name = readJson (Path.Combine(output, name))
let property name (row: JsonElement) = row.GetProperty(name: string)
let number name row = (property name row).GetDouble()
let flag name row = (property name row).GetBoolean()

let text name row =
    (property name row).GetString()
    |> Option.ofObj
    |> Option.defaultWith (fun () -> failwith name)

let elements (row: JsonElement) = row.EnumerateArray() |> Seq.toList

let numbers name row =
    property name row |> elements |> List.map _.GetDouble()

let identity row =
    text "Name" row,
    text "Group" row,
    text "Side" row,
    numbers "Seed" row,
    numbers "TargetPosition" row,
    numbers "TargetQuaternion" row

let outputJson path value =
    File.WriteAllText(path, JsonSerializer.Serialize(value, JsonSerializerOptions(WriteIndented = true)))

let modelLimits () =
    let xml = XDocument.Load(Path.Combine(root, "validation/fixtures/axol_arms.urdf"))

    let joints =
        xml.Descendants(XName.Get "joint")
        |> Seq.filter (fun joint -> joint.Attribute(XName.Get "type").Value = "revolute")
        |> Seq.map (fun joint ->
            let limit = joint.Element(XName.Get "limit")

            let parse name =
                Double.Parse(limit.Attribute(XName.Get name).Value, CultureInfo.InvariantCulture)

            joint.Attribute(XName.Get "name").Value, (parse "lower", parse "upper"))
        |> Map.ofSeq

    [ "Left"; "Right" ]
    |> List.map (fun side ->
        side,
        [ "s1_0"; "s2_0"; "s3_0"; "e1_0"; "e2_0"; "w1_0"; "w2_0" ]
        |> List.map (fun suffix -> Map.find (side.ToLowerInvariant() + "_" + suffix) joints))
    |> Map.ofList

let quality state =
    try
        let summary = load "summary.json"
        let rows = load "frozen-cases.json" |> elements
        let inputs = readJson (Path.Combine(control, "corpus.json")) |> elements
        let byName = rows |> List.map (fun row -> text "Name" row, row) |> Map.ofList
        let sameInputs = List.map identity inputs = List.map identity rows

        let baseline =
            readJson (Path.Combine(control, "baseline-passed.json"))
            |> elements
            |> List.map _.GetString()

        let regressions =
            sameInputs
            && (baseline |> List.forall (fun name -> flag "Success" (Map.find name byName)))

        let known =
            rows
            |> List.filter (fun row ->
                List.contains (text "Group" row) [ "NearReachable"; "WideReachable"; "HeldOutReachable" ])

        let missing = known |> List.filter (flag "Success" >> not)
        let inputByName = inputs |> List.map (fun row -> text "Name" row, row) |> Map.ofList
        let generated = load "cases.json" |> elements

        let stableTargets =
            generated
            |> List.forall (fun row -> identity row = identity (Map.find (text "Name" row) inputByName))

        let limits = modelLimits ()

        let bounded =
            rows
            |> List.forall (fun row ->
                let q, bounds = numbers "Angles" row, Map.find (text "Side" row) limits

                q.Length = 7
                && List.forall2
                    (fun value (low, high) -> Double.IsFinite value && value >= low - 1e-12 && value <= high + 1e-12)
                    q
                    bounds)

        let numeric =
            passed "numerical_correctness" state
            && bounded
            && number "MalformedRejected" summary = 7.
            && number "MalformedTotal" summary = 7.
            && flag "DistantTargetRejected" summary
            && number "JacobianMaxAbsoluteError" summary < 1e-7
            && number "PositionToleranceMm" summary = 0.01
            && number "OrientationToleranceDegrees" summary <= 1e-4 * 180. / Math.PI + 1e-12
            && (property "Paths" summary |> elements |> List.forall (flag "Success"))

        let verified =
            state
            |> check
                "contract"
                (passed "contract" state && stableTargets && sameInputs)
                "Frozen target contract differs"
            |> check "regressions" regressions "A previously passing endpoint regressed"
            |> check
                "reachability"
                (missing.IsEmpty && known.Length = 1100)
                "All 1100 known reachable targets must remain solved"
            |> check "numerical_correctness" numeric "Numerical invariants or strict output limits failed"

        // The F# evaluator orchestrates the frozen independent numerical oracle.
        // All endpoint outputs, including held-out targets, are checked by NumPy.
        outputJson (Path.Combine(output, "cases.json")) rows

        let independentlyVerified =
            verified
            |> executeCheck "independent_validation" python [ "validation/oracle/validate.py"; output ] 180

        let allocation = load "allocation.json" |> elements

        let referenceRows =
            readJson (Path.Combine(control, "allocation-contract.json"))
            |> property "References"
            |> elements

        let references = referenceRows |> List.map (number "Bytes")

        if List.map (text "Name") allocation <> List.map (text "Name") referenceRows then
            failwith "Allocation fixture identity changed"

        if allocation.Length <> references.Length then
            failwith "Allocation case count changed"

        let ratio =
            List.map2 (fun row reference -> number "Bytes" row / reference) allocation references
            |> List.average

        let times = known |> List.map (number "Milliseconds") |> List.sort
        let p95 = times[int (ceil (float times.Length * 0.95)) - 1]

        let latency =
            p95 <= 500.
            && (allocation
                |> List.forall (fun row ->
                    number "Milliseconds" row
                    <= (if (text "Name" row).Contains "path" then 500. else 10.)))

        if not (Double.IsFinite ratio) || ratio < 0. then
            failwith "Invalid allocation metric"

        let metrics =
            [ "priority_score", decimal missing.Length * 1000M + min 100M (decimal ratio)
              "known_reachable_missing", decimal missing.Length
              "known_reachable_total", decimal known.Length
              "allocation_ratio", decimal ratio
              "reachable_p95_ms", decimal p95 ]
            @ (allocation
               |> List.map (fun row -> text "Name" row + "_bytes", decimal (number "Bytes" row)))
            |> Map.ofList

        let completed =
            independentlyVerified
            |> check "latency" latency "Latency ceiling exceeded"
            |> check "determinism" (load "determinism.json" |> flag "Passed") "Repeated/concurrent outputs differ"

        let findings =
            [ sprintf
                  "Known reachable: %d/%d; allocation ratio %.6f; p95 %.3f ms"
                  (known.Length - missing.Length)
                  known.Length
                  ratio
                  p95
              "Failed cases (first 30): "
              + (missing |> List.truncate 30 |> List.map (text "Name") |> String.concat ", ")
              "Detailed targets, angles, timings, paths: " + output
              "Allocation bytes: " + JsonSerializer.Serialize allocation
              "Offline only; SDK collision checks and physical execution are separate." ]

        { completed with
            Metrics = metrics
            Evidence = independentlyVerified.Evidence @ findings }
    with error ->
        File.WriteAllText(Path.Combine(output, "exception.log"), error.ToString())
        let failed = state |> check "numerical_correctness" false error.Message

        { failed with
            Evidence = state.Evidence @ [ error.Message ] }

let evaluate () =
    try
        let manifest = readJson (Path.Combine(control, "protected-hashes.json"))

        let frozen =
            manifest.EnumerateObject()
            |> Seq.forall (fun entry ->
                let actual =
                    SHA256.HashData(File.ReadAllBytes(Path.Combine(root, entry.Name)))
                    |> Convert.ToHexStringLower

                actual = entry.Value.GetString())

        let contract =
            initial
            |> check "contract" frozen "Frozen evaluator/model/input contract changed"

        if not frozen then
            contract
        else
            let built =
                contract
                |> executeCheck
                    "build"
                    "dotnet"
                    [ "build"
                      "validation/FsIK.Validation.fsproj"
                      "-c"
                      "Release"
                      "-m:1"
                      "--nologo" ]
                    180

            if not (passed "build" built) then
                built
            else
                let tested =
                    built
                    |> executeCheck
                        "tests"
                        "dotnet"
                        [ "test"
                          "tests/FsIK.Tests/FsIK.Tests.fsproj"
                          "-c"
                          "Release"
                          "-m:1"
                          "--nologo" ]
                        180

                tested
                |> executeCheck
                    "numerical_correctness"
                    "dotnet"
                    [ "validation/bin/Release/net10.0/FsIK.Validation.dll"
                      output
                      Path.Combine(control, "corpus.json") ]
                    240
                |> quality
    with error ->
        File.WriteAllText(Path.Combine(output, "exception.log"), error.ToString())

        { initial with
            Evidence = [ error.Message ] }

let evaluated = evaluate ()

let payload =
    {| schemaVersion = 1
       constraints = evaluated.Checks |> Map.map (fun _ result -> result = Passed)
       metrics = evaluated.Metrics
       summary =
        sprintf
            "Missing reachable targets %s; allocation ratio %s"
            (evaluated.Metrics
             |> Map.tryFind "known_reachable_missing"
             |> Option.map string
             |> Option.defaultValue "unknown")
            (evaluated.Metrics
             |> Map.tryFind "allocation_ratio"
             |> Option.map string
             |> Option.defaultValue "unknown")
       evidence = evaluated.Evidence |}

Directory.CreateDirectory(Path.GetDirectoryName resultPath) |> ignore
let temporary = resultPath + ".tmp"
outputJson temporary payload
File.Move(temporary, resultPath, true)
printfn "%s" (JsonSerializer.Serialize(payload, JsonSerializerOptions(WriteIndented = true)))

if evaluated.Checks |> Map.forall (fun _ check -> check = Passed) |> not then
    exit 1
