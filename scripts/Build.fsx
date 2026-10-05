open System
open System.Diagnostics
open System.IO
open System.Xml.Linq

type Operation =
    | Pack
    | Validate

let root = Path.GetFullPath(Path.Combine(__SOURCE_DIRECTORY__, ".."))

let operation =
    match fsi.CommandLineArgs |> Array.skip 1 |> Array.toList with
    | [ "pack" ] -> Pack
    | [ "validate" ] -> Validate
    | _ -> failwith "Usage: dotnet fsi scripts/Build.fsx pack|validate"

let run args =
    let settings = ProcessStartInfo("dotnet")
    settings.UseShellExecute <- false
    settings.CreateNoWindow <- true
    settings.WorkingDirectory <- root
    settings.RedirectStandardOutput <- true
    settings.RedirectStandardError <- true
    args |> List.iter settings.ArgumentList.Add
    use child = Process.Start settings
    let output = child.StandardOutput.ReadToEndAsync()
    let errors = child.StandardError.ReadToEndAsync()
    child.WaitForExit()
    Console.Write output.Result
    Console.Error.Write errors.Result

    if child.ExitCode <> 0 then
        failwithf "dotnet %A failed with code %d" args child.ExitCode

let pack () =
    let properties = XDocument.Load(Path.Combine(root, "Directory.Build.props"))

    let version =
        properties.Descendants(XName.Get "FsIKVersion") |> Seq.exactlyOne |> _.Value

    let feed = Path.Combine(root, "artifacts/packages")
    Directory.CreateDirectory feed |> ignore
    let package = Path.Combine(feed, "FsIK." + version + ".nupkg")

    if File.Exists package then
        failwith "Package already exists: bump FsIKVersion before packing again"

    run [ "tool"; "restore" ]
    run [ "fantomas"; "src"; "tests"; "validation"; "scripts"; "--check" ]
    run [ "test"; "FsIK.slnx"; "-c"; "Release"; "-m:1"; "--nologo" ]

    run
        [ "pack"
          "src/FsIK/FsIK.fsproj"
          "-c"
          "Release"
          "--no-build"
          "-o"
          feed
          "--nologo" ]

    printfn "Local NuGet package: %s" package

match operation with
| Pack -> pack ()
| Validate -> run [ "fsi"; "validation/evaluate.fsx" ]
