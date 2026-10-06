namespace FsIK

open System
open System.Runtime.CompilerServices

[<assembly: InternalsVisibleTo("FsIK.Tests")>]
do ()

type TranslationContinuityOptions =
    { MaxJointChangeRadians: float
      MaxSubdivisionDepth: int
      MaxWaypoints: int }

module Translation =
    let defaults =
        { MaxJointChangeRadians = 0.1
          MaxSubdivisionDepth = 8
          MaxWaypoints = 4096 }

    // Internal seam lets tests inspect the actual accepted waypoints before retiming.
    let internal waypoints (continuity: TranslationContinuityOptions) chain (seed: float list) delta =
        let distance = Geometry.norm delta
        let bounds = Geometry.limits chain

        if
            not (Double.IsFinite continuity.MaxJointChangeRadians)
            || continuity.MaxJointChangeRadians <= 0.
            || continuity.MaxSubdivisionDepth < 0
            || continuity.MaxSubdivisionDepth > 16
            || continuity.MaxWaypoints < 2
        then
            Error
                "Continuity requires a finite positive joint-change bound, subdivision depth 0..16, and at least two waypoints"
        elif not (Double.IsFinite distance) || distance <= 0. || distance > 0.100001 then
            Error "Translation must be finite, nonzero, and at most 10 cm"
        elif
            List.isEmpty bounds
            || seed.Length <> bounds.Length
            || not (List.forall Double.IsFinite seed)
            || not (List.forall2 (fun limit angle -> angle >= limit.Lower && angle <= limit.Upper) bounds seed)
        then
            Error "Finite, legal planning-start joints are required"
        else
            let initial = Geometry.forward chain seed
            let count = max 2 (int (ceil (distance / 0.000508)))

            let options =
                { Solver.defaults with
                    PositionTolerance = 1e-5
                    OrientationTolerance = 1e-4 }

            let target fraction =
                { initial with
                    Position = Geometry.add initial.Position (Geometry.scale fraction delta) }

            let failure low high reason =
                Error(
                    sprintf
                        "Path continuity failed at interval %.9f..%.9f (joint-change bound %.6g rad): %s"
                        low
                        high
                        continuity.MaxJointChangeRadians
                        reason
                )

            let rec interval depth low high previous reversed accepted =
                if accepted >= continuity.MaxWaypoints then
                    failure
                        low
                        high
                        (sprintf "Waypoint limit %d exhausted; no partial trajectory returned" continuity.MaxWaypoints)
                else
                    match Solver.solveNear options continuity.MaxJointChangeRadians chain (target high) previous with
                    | Solved result when
                        List.forall2 (fun a b -> abs (b - a) <= continuity.MaxJointChangeRadians) previous result.Angles
                        ->
                        Ok(result.Angles, result.Angles :: reversed, accepted + 1)
                    | Failed(InvalidInput error) -> failure low high error
                    | result ->
                        if depth >= continuity.MaxSubdivisionDepth then
                            failure
                                low
                                high
                                (sprintf
                                    "Subdivision depth %d exhausted; %A; no partial trajectory returned"
                                    depth
                                    result)
                        else
                            let middle = low + (high - low) / 2.

                            interval (depth + 1) low middle previous reversed accepted
                            |> Result.bind (fun (midJoints, midPath, midCount) ->
                                interval (depth + 1) middle high midJoints midPath midCount)

            let rec build index previous reversed accepted =
                if index > count then
                    Ok(List.rev reversed)
                else
                    interval 0 (float (index - 1) / float count) (float index / float count) previous reversed accepted
                    |> Result.bind (fun (joints, path, total) -> build (index + 1) joints path total)

            build 1 seed [ seed ] 1

    /// Build a complete straight Cartesian path using bounded local IK and subdivision.
    /// Other-arm positions and independent collision checks belong to the adapter.
    let planWithContinuity continuity chain seed delta : Result<Trajectory, string> =
        waypoints continuity chain seed delta
        |> Result.bind (fun path ->
            Trajectory.retime Trajectory.conservative chain path
            |> Result.mapError (sprintf "MathNet trajectory rejected: %A"))

    let plan chain seed delta : Result<Trajectory, string> =
        planWithContinuity defaults chain seed delta
