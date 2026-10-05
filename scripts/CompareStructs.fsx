open System
open System.IO
open System.IO.Compression
open System.Reflection
open System.Runtime.Loader
open System.Security.Cryptography
open System.Text.Json

let root = Path.GetFullPath(Path.Combine(__SOURCE_DIRECTORY__, ".."))
let evidence = Path.Combine(root, "artifacts/validation/structs")

let read path =
    use document = JsonDocument.Parse(File.ReadAllText path)
    document.RootElement.Clone()

let rows (json: JsonElement) = json.EnumerateArray() |> Seq.toList
let property (name: string) (json: JsonElement) = json.GetProperty name
let number name json = property name json |> _.GetDouble()
let name json = property "Name" json |> _.GetString()
let median values = values |> List.sort |> List.item 1

let load label round file =
    read (Path.Combine(evidence, label, string round, file))

let validate label round =
    let result = load label round "evaluation.json"
    let gates = property "constraints" result |> _.EnumerateObject() |> Seq.toList

    if
        gates.Length <> 9
        || gates |> List.exists (fun gate -> not (gate.Value.GetBoolean()))
    then
        failwithf "%s round %d failed validation" label round

    let metrics = property "metrics" result

    if
        number "known_reachable_missing" metrics <> 0.
        || number "known_reachable_total" metrics <> 1100.
    then
        failwith "Reachable target count changed"

    let successful = load label round "ik-evidence/frozen-cases.json" |> rows
    let byName = successful |> List.map (fun row -> name row, row) |> Map.ofList
    let prior = read (Path.Combine(root, "validation/baseline-passed.json")) |> rows

    if
        prior.Length <> 1164
        || prior
           |> List.exists (fun row -> not (property "Success" byName[row.GetString()] |> _.GetBoolean()))
    then
        failwith "Previously passing case regressed"

for label in [ "baseline"; "candidate" ] do
    for round in 1..3 do
        validate label round

let runs label =
    [ 1..3 ]
    |> List.map (fun round -> load label round "ik-evidence/allocation.json" |> rows)

let baseline = runs "baseline"
let candidate = runs "candidate"

let measurements =
    baseline.Head
    |> List.mapi (fun index original ->
        let before = baseline |> List.map (fun run -> run[index])
        let after = candidate |> List.map (fun run -> run[index])

        if after |> List.exists (fun row -> name row <> name original) then
            failwith "Benchmark identity changed"

        let baselineBytes = before |> List.map (number "Bytes") |> median
        let candidateBytes = after |> List.map (number "Bytes") |> median
        let baselineMs = before |> List.map (number "Milliseconds") |> median
        let candidateMs = after |> List.map (number "Milliseconds") |> median

        {| Name = name original
           BaselineBytes = baselineBytes
           CandidateBytes = candidateBytes
           AllocationReductionPercent = 100. * (1. - candidateBytes / baselineBytes)
           BaselineMilliseconds = baselineMs
           CandidateMilliseconds = candidateMs
           TimeChangePercent = 100. * (candidateMs / baselineMs - 1.) |})

let normalized =
    measurements
    |> List.averageBy (fun row -> row.CandidateBytes / row.BaselineBytes)

if
    normalized >= 1.
    || measurements |> List.exists (fun row -> row.CandidateBytes > row.BaselineBytes)
then
    failwith "Allocation acceptance failed"

// Use separate load contexts to compare the two immutable packages' public API.
let api version =
    let directory = Path.Combine(evidence, "api", version)
    Directory.CreateDirectory directory |> ignore
    let dll = Path.Combine(directory, "FsIK.dll")

    use archive =
        ZipFile.OpenRead(Path.Combine(root, "artifacts/packages/FsIK." + version + ".nupkg"))

    archive.GetEntry("lib/net10.0/FsIK.dll").ExtractToFile(dll, true)
    let context = new AssemblyLoadContext(version, true)

    context.add_Resolving (fun _ dependency ->
        if dependency.Name = "FSharp.Core" then
            typeof<int list>.Assembly
        else
            AssemblyLoadContext.Default.LoadFromAssemblyName dependency)

    let assembly = context.LoadFromAssemblyPath dll

    let flags =
        BindingFlags.Public
        ||| BindingFlags.Static
        ||| BindingFlags.Instance
        ||| BindingFlags.DeclaredOnly

    let signatures =
        assembly.GetExportedTypes()
        |> Array.collect (fun t ->
            [| yield sprintf "type %s value=%b" t.FullName t.IsValueType
               for memberInfo in t.GetMembers flags do
                   yield t.FullName + ": " + memberInfo.ToString() |])
        |> Array.sort

    File.WriteAllLines(Path.Combine(directory, "public-api.txt"), signatures)
    context.Unload()
    signatures

let sameApi = api "0.1.0-local.1" = api "0.1.0-local.2"

if not sameApi then
    failwith "Public API differs"

let endpointDifferences =
    List.map2
        (fun before after ->
            [ "Name"
              "Success"
              "Angles"
              "PositionErrorMm"
              "OrientationErrorDegrees"
              "Iterations" ]
            |> List.exists (fun field -> (property field before).GetRawText() <> (property field after).GetRawText()))
        (load "baseline" 1 "ik-evidence/frozen-cases.json" |> rows)
        (load "candidate" 1 "ik-evidence/frozen-cases.json" |> rows)
    |> List.filter id
    |> List.length

let samePaths =
    [ "Left-up-10mm"; "Left-up-50.8mm"; "Right-up-10mm"; "Right-up-50.8mm" ]
    |> List.forall (fun name ->
        let before = (load "baseline" 1 ("ik-evidence/" + name + ".path.json")).GetRawText()
        let after = (load "candidate" 1 ("ik-evidence/" + name + ".path.json")).GetRawText()
        before = after)

let report =
    {| BaselineVersion = "0.1.0-local.1"
       CandidateVersion = "0.1.0-local.2"
       CompleteRunsPerVersion = 3
       ValidationGatesPassedPerRun = 9
       KnownReachableTargets = 1100
       PreviouslyPassingCases = 1164
       PublicApiUnchanged = sameApi
       EndpointOutputDifferences = endpointDifferences
       PathSamplesIdentical = samePaths
       MeanNormalizedAllocation = normalized
       MeanAllocationReductionPercent = 100. * (1. - normalized)
       Measurements = measurements
       RequiresTimingInvestigation = measurements |> List.exists (fun row -> row.TimeChangePercent > 10.) |}

File.WriteAllText(
    Path.Combine(evidence, "comparison.json"),
    JsonSerializer.Serialize(report, JsonSerializerOptions(WriteIndented = true))
)

printfn "%s" (File.ReadAllText(Path.Combine(evidence, "comparison.json")))
