namespace FsMotion

open System
open System.Diagnostics
open System.Threading
open FsIK

type PlannerOptions =
    { Seed: int
      MaxNodes: int
      MaxSeconds: float
      ExtensionRadians: float
      GoalBias: float
      Margin: float }

type PlanFailure =
    | InvalidRequest of string
    | StartCollision of Contact option
    | GoalCollision of Contact option
    | CheckerFailure of string
    | SearchExhausted of int
    | PlanningCancelled
    | PlanningTimeout
    | IkFailed of Failure

type PlannedPath =
    { ModelId: string
      SceneId: string
      Margin: float
      ActiveJoints: string list
      Configurations: Map<string, float> list
      NodesExplored: int }

type TimedPlan =
    { Path: PlannedPath
      Rate: int
      DurationSeconds: float
      Samples: Map<string, float> list }

module Planner =
    let defaults =
        { Seed = 0
          MaxNodes = 4000
          MaxSeconds = 10.
          ExtensionRadians = 0.15
          GoalBias = 0.2
          Margin = 0.01 }

    let private length (a: float array) (b: float array) =
        Array.map2 (fun x y -> (x - y) * (x - y)) a b |> Array.sum |> sqrt

    /// Select one kinematic arm chain. All other joints remain exactly fixed.
    /// Search uses ordinary bounded joint interpolation, never wrapped angles.
    let plan
        options
        model
        sceneId
        (checker: CollisionChecker)
        (active: string list)
        (start: Map<string, float>)
        (goal: Map<string, float>)
        (cancellation: CancellationToken)
        : Result<PlannedPath, PlanFailure> =
        let bounds = Model.joints model

        let validOptions =
            options.MaxNodes >= 2
            && options.MaxNodes <= 1000000
            && Double.IsFinite options.MaxSeconds
            && options.MaxSeconds > 0.
            && Double.IsFinite options.ExtensionRadians
            && options.ExtensionRadians > 0.
            && Double.IsFinite options.GoalBias
            && options.GoalBias >= 0.
            && options.GoalBias <= 1.
            && Double.IsFinite options.Margin
            && options.Margin >= 0.

        if
            not validOptions
            || String.IsNullOrWhiteSpace sceneId
            || active.IsEmpty
            || Set.ofList active |> Set.count <> active.Length
            || active |> List.exists (fun name -> not (bounds.ContainsKey name))
        then
            Error(InvalidRequest "Invalid options or active joint group")
        elif
            checker.ModelId <> Model.id model
            || checker.SceneId <> sceneId
            || not (Double.IsFinite checker.Margin)
            || checker.Margin < options.Margin
        then
            Error(InvalidRequest "Collision checker model/scene/margin mismatch")
        elif not (Model.validConfiguration model start && Model.validConfiguration model goal) then
            Error(InvalidRequest "Complete finite, bounded joint configurations required")
        elif
            start
            |> Map.exists (fun name value -> not (List.contains name active) && goal[name] <> value)
        then
            Error(InvalidRequest "Inactive joints must remain fixed")
        else
            use budget = CancellationTokenSource.CreateLinkedTokenSource cancellation
            budget.CancelAfter(TimeSpan.FromSeconds options.MaxSeconds)
            let watch = Stopwatch.StartNew()

            let interrupted () =
                if cancellation.IsCancellationRequested then
                    Some PlanningCancelled
                elif watch.Elapsed.TotalSeconds >= options.MaxSeconds then
                    Some PlanningTimeout
                else
                    None

            let full (q: float array) =
                List.zip active (Array.toList q)
                |> List.fold (fun config (name, value) -> Map.add name value config) start

            let a, b =
                active |> List.map (fun n -> start[n]) |> List.toArray,
                active |> List.map (fun n -> goal[n]) |> List.toArray

            let edge x y =
                checker.Edge (full x) (full y) budget.Token

            let interruptedEdge () =
                if cancellation.IsCancellationRequested then
                    PlanningCancelled
                else
                    PlanningTimeout

            let result points count =
                { ModelId = Model.id model
                  SceneId = sceneId
                  Margin = options.Margin
                  ActiveJoints = active
                  Configurations = points |> List.map full
                  NodesExplored = count }

            match interrupted (), checker.Configuration start, checker.Configuration goal with
            | Some reason, _, _ -> Error reason
            | _, Error error, _
            | _, _, Error error -> Error(CheckerFailure error)
            | _, Ok report, _ when report.MinimumClearance <= options.Margin -> Error(StartCollision report.Closest)
            | _, _, Ok report when report.MinimumClearance <= options.Margin -> Error(GoalCollision report.Closest)
            | _ ->
                match edge a b with
                | Error error -> Error(CheckerFailure error)
                | Ok InterruptedEdge -> Error(interruptedEdge ())
                | Ok ClearEdge ->
                    match interrupted () with
                    | Some error -> Error error
                    | None -> Ok(result [ a; b ] 2)
                | _ ->
                    // Mutable search scratch is owned by this call; output maps/lists are immutable.
                    let random = Random options.Seed
                    let nodes = ResizeArray<float array * int>()
                    nodes.Add(a, -1)
                    let mutable answer = None
                    let mutable failure = None
                    let mutable attempts = 0

                    while answer.IsNone
                          && failure.IsNone
                          && attempts < options.MaxNodes
                          && nodes.Count < options.MaxNodes do
                        attempts <- attempts + 1

                        match interrupted () with
                        | Some reason -> failure <- Some reason
                        | None ->
                            let sample =
                                if random.NextDouble() < options.GoalBias then
                                    b
                                else
                                    active
                                    |> List.map (fun name ->
                                        let bound = bounds[name] in
                                        bound.Lower + random.NextDouble() * (bound.Upper - bound.Lower))
                                    |> List.toArray

                            let nearest =
                                [ 0 .. nodes.Count - 1 ] |> List.minBy (fun i -> length (fst nodes[i]) sample)

                            let parent = fst nodes[nearest]
                            let distance = length parent sample
                            let ratio = min 1. (options.ExtensionRadians / max 1e-15 distance)
                            let candidate = Array.map2 (fun x y -> x + (y - x) * ratio) parent sample

                            match edge parent candidate with
                            | Error error -> failure <- Some(CheckerFailure error)
                            | Ok InterruptedEdge -> failure <- Some(interruptedEdge ())
                            | Ok ClearEdge when distance > 1e-12 ->
                                let index = nodes.Count
                                nodes.Add(candidate, nearest)

                                match edge candidate b with
                                | Error error -> failure <- Some(CheckerFailure error)
                                | Ok InterruptedEdge -> failure <- Some(interruptedEdge ())
                                | Ok ClearEdge ->
                                    let rec trace i tail =
                                        let q, p = nodes[i] in if p < 0 then q :: tail else trace p (q :: tail)

                                    match interrupted () with
                                    | Some error -> failure <- Some error
                                    | None -> answer <- Some(result (trace index [ b ]) nodes.Count)
                                | _ -> ()
                            | _ -> ()

                    match failure, answer with
                    | Some error, _ -> Error error
                    | _, Some path -> Ok path
                    | _ -> Error(SearchExhausted nodes.Count)

    /// IK supplies the endpoint; search and collision checks supply the route.
    let planToPose options model sceneId checker chain start target (cancellation: CancellationToken) =
        let active = Geometry.names chain

        if
            not (Model.ownsChain model chain)
            || active |> List.exists (fun name -> not (Map.containsKey name start))
        then
            Error(InvalidRequest "Chain must belong to model with all starting joints supplied")
        elif cancellation.IsCancellationRequested then
            Error PlanningCancelled
        else
            match Solver.solve Solver.defaults chain target (active |> List.map (fun name -> start[name])) with
            | Failed error -> Error(IkFailed error)
            | Solved solution ->
                let goal =
                    List.zip active solution.Angles
                    |> List.fold (fun q (name, value) -> Map.add name value q) start

                plan options model sceneId checker active start goal cancellation

    /// Conservative stop-at-waypoint retiming, followed by independent swept-edge validation.
    /// The supplied checker must belong to the same immutable scene/model as search.
    let retime
        (limits: MotionLimits)
        model
        chain
        (checker: CollisionChecker)
        (path: PlannedPath)
        (cancellation: CancellationToken)
        : Result<TimedPlan, PlanFailure> =
        if
            Geometry.names chain <> path.ActiveJoints
            || not (Model.ownsChain model chain)
            || path.Configurations.Length < 2
            || checker.ModelId <> path.ModelId
            || Model.id model <> path.ModelId
            || checker.SceneId <> path.SceneId
            || not (Double.IsFinite path.Margin)
            || path.Margin < 0.
            || checker.Margin < path.Margin
        then
            Error(InvalidRequest "Chain/order/scene/margin mismatch or incomplete path")
        elif
            path.Configurations
            |> List.exists (fun q ->
                not (Model.validConfiguration model q)
                || q
                   |> Map.exists (fun name value ->
                       not (List.contains name path.ActiveJoints)
                       && value <> path.Configurations.Head[name]))
        then
            Error(InvalidRequest "Path contains illegal joints or moved inactive joints")
        elif cancellation.IsCancellationRequested then
            Error PlanningCancelled
        else
            let selected =
                path.Configurations
                |> List.map (fun q -> path.ActiveJoints |> List.map (fun name -> q[name]))
            // FsIK currently checks an additional Axol tool offset. Independently
            // enforce the selected URDF tip origin's speed for generic chains too.
            let rec timing attempt workingLimits =
                match Trajectory.retime workingLimits chain selected with
                | Error error -> Error error
                | Ok trajectory ->
                    let speed =
                        trajectory.Samples
                        |> List.map (fun q -> (Geometry.forward chain q).Position)
                        |> List.pairwise
                        |> List.map (fun (a, b) -> Geometry.norm (Geometry.subtract b a) * float limits.Rate)
                        |> List.max

                    if speed <= limits.MaxTipSpeed * 1.000001 then
                        Ok trajectory
                    elif attempt >= 8 then
                        Error(InvalidPath "Tip speed could not be retimed")
                    else
                        let scale = limits.MaxTipSpeed / speed * 0.99

                        timing
                            (attempt + 1)
                            { workingLimits with
                                MaxJointVelocity = workingLimits.MaxJointVelocity * scale
                                MaxJointAcceleration = workingLimits.MaxJointAcceleration * scale * scale
                                MaxTipSpeed = workingLimits.MaxTipSpeed * scale }

            match timing 0 limits with
            | Error error -> Error(InvalidRequest(sprintf "Retiming rejected: %A" error))
            | Ok trajectory ->
                let fixedStart = path.Configurations.Head

                let samples =
                    trajectory.Samples
                    |> List.map (fun angles ->
                        List.zip path.ActiveJoints angles
                        |> List.fold (fun q (name, value) -> Map.add name value q) fixedStart)

                let rec validate pairs =
                    if cancellation.IsCancellationRequested then
                        Error PlanningCancelled
                    else
                        match pairs with
                        | [] ->
                            Ok
                                { Path = path
                                  Rate = limits.Rate
                                  DurationSeconds = trajectory.DurationSeconds
                                  Samples = samples }
                        | (a, b) :: rest ->
                            match checker.Edge a b cancellation with
                            | Ok ClearEdge -> validate rest
                            | Ok InterruptedEdge -> Error PlanningCancelled
                            | Ok _ -> Error(CheckerFailure "Retimed path could not be certified")
                            | Error error -> Error(CheckerFailure error)

                validate (List.pairwise samples)
