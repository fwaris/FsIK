open System
open System.Diagnostics
open System.IO
open System.Xml.Linq

let root = Path.GetFullPath(Path.Combine(__SOURCE_DIRECTORY__, ".."))

let run args =
    let settings = ProcessStartInfo "dotnet"
    settings.WorkingDirectory <- root
    settings.UseShellExecute <- false
    settings.CreateNoWindow <- true
    settings.RedirectStandardOutput <- true
    settings.RedirectStandardError <- true

    for arg in args do
        settings.ArgumentList.Add arg

    use child = Process.Start settings
    let output = child.StandardOutput.ReadToEndAsync()
    let errors = child.StandardError.ReadToEndAsync()
    child.WaitForExit()
    printf "%s" output.Result
    eprintf "%s" errors.Result

    if child.ExitCode <> 0 then
        failwithf "dotnet %A failed" args

let pack =
    match fsi.CommandLineArgs |> Array.skip 1 |> Array.toList with
    | [] -> false
    | [ "pack" ] -> true
    | _ -> failwith "Usage: dotnet fsi scripts/Build-Motion.fsx [pack]"

let package =
    let xml = XDocument.Load(Path.Combine(root, "src/FsMotion/FsMotion.fsproj"))
    let version = xml.Descendants(XName.Get "Version") |> Seq.exactlyOne |> _.Value
    Path.Combine(root, "artifacts/packages", "FsMotion." + version + ".nupkg")

if pack && File.Exists package then
    failwith "Package already exists; increment FsMotion's version before packaging changed code."

run
    [ "fantomas"
      "src/FsMotion/Model.fs"
      "src/FsMotion/Collision.fs"
      "src/FsMotion/Planner.fs"
      "tests/FsMotion.Tests/Tests.fs"
      "scripts/Build-Motion.fsx"
      "scripts/MotionExample.fsx"
      "--check" ]

run [ "test"; "FsIK.slnx"; "-c"; "Release"; "-m:1" ]
run [ "fsi"; "scripts/MotionExample.fsx" ]

match Environment.GetEnvironmentVariable "FSIK_PYTHON" |> Option.ofObj with
| None -> printfn "Set FSIK_PYTHON to run the independent NumPy motion oracle."
| Some python ->
    let settings = ProcessStartInfo python
    settings.WorkingDirectory <- root
    settings.UseShellExecute <- false
    settings.CreateNoWindow <- true
    settings.RedirectStandardOutput <- true
    settings.RedirectStandardError <- true
    settings.ArgumentList.Add "tests/FsMotion.Tests/oracle/validate.py"
    use child = Process.Start settings
    let output = child.StandardOutput.ReadToEndAsync()
    let errors = child.StandardError.ReadToEndAsync()
    child.WaitForExit()
    printf "%s" output.Result
    eprintf "%s" errors.Result

    if child.ExitCode <> 0 then
        failwith "Independent motion oracle failed"

if pack then
    run
        [ "pack"
          "src/FsMotion/FsMotion.fsproj"
          "-c"
          "Release"
          "--no-build"
          "-o"
          "artifacts/packages" ]
