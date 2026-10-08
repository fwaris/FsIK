namespace FsMotion

open System
open System.Threading
open FsIK

type Contact =
    { First: string
      Second: string
      Clearance: float }

type CollisionReport =
    { MinimumClearance: float
      Closest: Contact option }

type EdgeResult =
    | ClearEdge
    | BlockedEdge of Contact option
    | UncertifiedEdge
    | InterruptedEdge

/// Inject another checker without changing search. Edge checks must cover the entire interpolation.
type CollisionChecker =
    { ModelId: string
      SceneId: string
      Margin: float
      Configuration: Map<string, float> -> Result<CollisionReport, string>
      Edge: Map<string, float> -> Map<string, float> -> CancellationToken -> Result<EdgeResult, string> }

module Collision =
    let private dot (a: Vec3) (b: Vec3) = a.X * b.X + a.Y * b.Y + a.Z * b.Z
    let private clamp x = max 0. (min 1. x)

    /// Segment distance including degenerate spheres and parallel segments.
    let distance (first: Capsule) (second: Capsule) =
        let d1, d2, r =
            Geometry.subtract first.B first.A, Geometry.subtract second.B second.A, Geometry.subtract first.A second.A

        let a, e, f = dot d1 d1, dot d2 d2, dot d2 r

        let s, t =
            if a <= 1e-24 && e <= 1e-24 then
                0., 0.
            elif a <= 1e-24 then
                0., clamp (f / e)
            else
                let c = dot d1 r

                if e <= 1e-24 then
                    clamp (-c / a), 0.
                else
                    let b = dot d1 d2
                    let denominator = a * e - b * b

                    let s =
                        if denominator > 1e-15 * a * e then
                            clamp ((b * f - c * e) / denominator)
                        else
                            0.

                    let t = (b * s + f) / e

                    if t < 0. then clamp (-c / a), 0.
                    elif t > 1. then clamp ((b - c) / a), 1.
                    else s, t

        let p = Geometry.add first.A (Geometry.scale s d1)
        let q = Geometry.add second.A (Geometry.scale t d2)
        Geometry.norm (Geometry.subtract p q) - first.Radius - second.Radius

    /// Conservative capsule envelopes and a Lipschitz bound certify swept edges.
    /// Uncertifiable intervals fail closed; checking only samples is never sufficient.
    let create model scene margin maxDepth : Result<CollisionChecker, string> =
        if not (Double.IsFinite margin) || margin < 0. || maxDepth < 0 || maxDepth > 24 then
            Error "Invalid collision margin/depth"
        elif
            String.IsNullOrWhiteSpace scene.Id
            || scene.Obstacles |> List.exists (fun o -> not (Model.validShape o.Shape))
        then
            Error "Invalid scene"
        elif
            (scene.Obstacles |> List.map _.Name |> Set.ofList).Count
            <> scene.Obstacles.Length
        then
            Error "Duplicate obstacle names"
        else
            let configuration q =
                try
                    let shapes = Model.shapesAt model q |> List.toArray

                    let contacts =
                        [ for i in 0 .. shapes.Length - 1 do
                              for j in i + 1 .. shapes.Length - 1 do
                                  if not (Model.excluded model shapes[i].Link shapes[j].Link) then
                                      yield
                                          { First = shapes[i].Link
                                            Second = shapes[j].Link
                                            Clearance = distance shapes[i].Shape shapes[j].Shape }

                              for obstacle in scene.Obstacles do
                                  yield
                                      { First = shapes[i].Link
                                        Second = obstacle.Name
                                        Clearance = distance shapes[i].Shape obstacle.Shape } ]

                    let nearest = contacts |> List.sortBy _.Clearance |> List.tryHead

                    Ok
                        { MinimumClearance =
                            nearest |> Option.map _.Clearance |> Option.defaultValue Double.PositiveInfinity
                          Closest = nearest }
                with error ->
                    Error error.Message

            let edge a b (cancellation: CancellationToken) =
                let rec certify depth a b =
                    if cancellation.IsCancellationRequested then
                        Ok InterruptedEdge
                    else
                        match configuration a, configuration b with
                        | Error e, _
                        | _, Error e -> Error e
                        | Ok ra, Ok rb when min ra.MinimumClearance rb.MinimumClearance <= margin ->
                            Ok(
                                BlockedEdge(
                                    if ra.MinimumClearance < rb.MinimumClearance then
                                        ra.Closest
                                    else
                                        rb.Closest
                                )
                            )
                        | Ok ra, Ok rb ->
                            let displacement =
                                a |> Map.toList |> List.sumBy (fun (name, x) -> abs (x - Map.find name b))

                            let bound = 2. * Model.reach model * displacement

                            if min ra.MinimumClearance rb.MinimumClearance - margin > bound then
                                Ok ClearEdge
                            elif depth >= maxDepth then
                                Ok UncertifiedEdge
                            else
                                let mid = a |> Map.map (fun name x -> (x + Map.find name b) / 2.)

                                match certify (depth + 1) a mid with
                                | Ok ClearEdge -> certify (depth + 1) mid b
                                | result -> result

                if not (Model.validConfiguration model a && Model.validConfiguration model b) then
                    Error "Illegal edge endpoints"
                else
                    certify 0 a b

            Ok
                { ModelId = Model.id model
                  SceneId = scene.Id
                  Margin = margin
                  Configuration = configuration
                  Edge = edge }
