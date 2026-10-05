namespace FsIK

open System

module Translation =
    /// Build a straight Cartesian path, preserving orientation and legal joints.
    /// Other-arm positions and independent collision checks belong to the adapter.
    let plan chain seed delta : Result<Trajectory, string> =
        let distance = Geometry.norm delta

        if not (Double.IsFinite distance) || distance <= 0. || distance > 0.100001 then
            Error "Translation must be finite, nonzero, and at most 10 cm"
        else
            let initial = Geometry.forward chain seed
            let count = max 2 (int (ceil (distance / 0.000508)))

            let options =
                { Solver.defaults with
                    PositionTolerance = 1e-5
                    OrientationTolerance = 1e-4 }

            let rec build index previous reversed =
                if index > count then
                    Ok(List.rev reversed)
                else
                    let target =
                        { initial with
                            Position = Geometry.add initial.Position (Geometry.scale (float index / float count) delta) }

                    match Solver.solve options chain target previous with
                    | Solved result -> build (index + 1) result.Angles (result.Angles :: reversed)
                    | Failed failure -> Error(sprintf "MathNet IK failed at waypoint %d/%d: %A" index count failure)

            build 1 seed [ seed ]
            |> Result.bind (fun path ->
                Trajectory.retime Trajectory.conservative chain path
                |> Result.mapError (sprintf "MathNet trajectory rejected: %A"))
