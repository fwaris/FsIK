namespace FsIK

open System
open System.Collections.Concurrent
open System.Runtime.CompilerServices
open MathNet.Numerics.LinearAlgebra

type Options =
    { MaxIterations: int
      PositionTolerance: float
      OrientationTolerance: float
      MaxStep: float
      OrientationWeight: float }

type Solution =
    { Angles: float list
      PositionError: float
      OrientationError: float
      Iterations: int }

type Failure =
    | InvalidInput of string
    | NoConvergence of Solution

type SolveResult =
    | Solved of Solution
    | Failed of Failure

module Solver =
    let defaults =
        { MaxIterations = 250
          PositionTolerance = 0.0001
          OrientationTolerance = Math.PI / 1800.
          MaxStep = 0.1
          OrientationWeight = 0.25 }

    type private ReducedBuffers =
        { Jacobian: Matrix<float>
          Scaled: Vector<float>
          Step: Vector<float> }

    type private Workspace =
        { Angles: ResizeArray<float>
          Candidate: ResizeArray<float>
          BestCandidate: ResizeArray<float>
          Axes: struct (ValueVec3 * ValueVec3) array
          Jacobian: Matrix<float>
          Error: Vector<float>
          Coefficients: Vector<float>
          Scaled: Vector<float>
          Delta: Vector<float>
          Free: ResizeArray<int>
          Blocked: ResizeArray<int>
          Reduced: ReducedBuffers option array }

    let private angleBuffer (count: int) =
        let buffer = ResizeArray<float>(count)

        for _ in 1..count do
            buffer.Add 0.

        buffer

    let private workspace count =
        { Angles = angleBuffer count
          Candidate = angleBuffer count
          BestCandidate = angleBuffer count
          Axes = Array.zeroCreate count
          Jacobian = Matrix<float>.Build.Dense(6, count)
          Error = Vector<float>.Build.Dense 6
          Coefficients = Vector<float>.Build.Dense 6
          Scaled = Vector<float>.Build.Dense count
          Delta = Vector<float>.Build.Dense count
          Free = ResizeArray<int>(count)
          Blocked = ResizeArray<int>(count)
          Reduced = Array.create count None }

    // A thin ownership boundary: cached model metadata is immutable; rented
    // numerical storage is exclusive until returned in the solve's finally block.
    // Weak chain keys allow both metadata and scratch to die with the model.
    type private ModelStorage(chain: Chain) =
        let limits = Geometry.limits chain
        let bounds = List.toArray limits
        let available = ConcurrentBag<Workspace>()
        member _.Limits = limits
        member _.Bounds = bounds

        member _.Rent() =
            let mutable rented = Unchecked.defaultof<Workspace>

            if available.TryTake(&rented) then
                rented
            else
                workspace bounds.Length

        member _.Return(rented) = available.Add rented

    let private models = ConditionalWeakTable<Chain, ModelStorage>()

    let private prepare =
        ConditionalWeakTable<Chain, ModelStorage>.CreateValueCallback(fun chain -> ModelStorage(chain))

    let private bounded limits angles =
        List.map2 (fun limit value -> Math.Clamp(value, limit.Lower, limit.Upper)) limits angles

    let private scoreErrors options position orientation =
        position ** 2. + (options.OrientationWeight * orientation) ** 2.

    let private score options (info: Solution) =
        scoreErrors options info.PositionError info.OrientationError

    let private withinErrors options position orientation =
        position <= options.PositionTolerance
        && orientation <= options.OrientationTolerance

    let private within options (info: Solution) =
        withinErrors options info.PositionError info.OrientationError

    let private dampedStep
        (jacobian: Matrix<float>)
        (error: Vector<float>)
        damping
        (coefficients: Vector<float>)
        (scaled: Vector<float>)
        (result: Vector<float>)
        =
        // MathNet still computes the same SVD. Its multiply overloads write into
        // owned vectors rather than allocating new results each iteration.
        let svd = jacobian.Svd(true)
        svd.U.TransposeThisAndMultiply(error, coefficients)

        for i in 0 .. jacobian.ColumnCount - 1 do
            scaled[i] <-
                if i < svd.S.Count then
                    coefficients[i] * svd.S[i] / (svd.S[i] * svd.S[i] + damping * damping)
                else
                    0.

        svd.VT.TransposeThisAndMultiply(scaled, result)

    let private reducedBuffers (storage: Workspace) count =
        match storage.Reduced[count - 1] with
        | Some buffers -> buffers
        | None ->
            let buffers =
                { Jacobian = Matrix<float>.Build.Dense(6, count)
                  Scaled = Vector<float>.Build.Dense count
                  Step = Vector<float>.Build.Dense count }

            storage.Reduced[count - 1] <- Some buffers
            buffers

    let private constrainedStep (storage: Workspace) damping (bounds: Limit array) =
        dampedStep storage.Jacobian storage.Error damping storage.Coefficients storage.Scaled storage.Delta
        storage.Blocked.Clear()

        for joint in 0 .. bounds.Length - 1 do
            let angle, step = storage.Angles[joint], storage.Delta[joint]

            if
                (angle <= bounds[joint].Lower + 1e-10 && step < 0.)
                || (angle >= bounds[joint].Upper - 1e-10 && step > 0.)
            then
                storage.Blocked.Add joint

        if storage.Blocked.Count <> 0 then
            storage.Free.Clear()

            for joint in 0 .. bounds.Length - 1 do
                if not (storage.Blocked.Contains joint) then
                    storage.Free.Add joint

            let rec activeSet () =
                if storage.Free.Count = 0 then
                    storage.Delta.Clear()
                else
                    let reduced = reducedBuffers storage storage.Free.Count

                    for column in 0 .. storage.Free.Count - 1 do
                        for row in 0..5 do
                            reduced.Jacobian[row, column] <- storage.Jacobian[row, storage.Free[column]]

                    dampedStep reduced.Jacobian storage.Error damping storage.Coefficients reduced.Scaled reduced.Step
                    storage.Blocked.Clear()

                    for index in 0 .. storage.Free.Count - 1 do
                        let joint, step = storage.Free[index], reduced.Step[index]
                        let angle = storage.Angles[joint]

                        if
                            (angle <= bounds[joint].Lower + 1e-10 && step < 0.)
                            || (angle >= bounds[joint].Upper - 1e-10 && step > 0.)
                        then
                            storage.Blocked.Add joint

                    if storage.Blocked.Count <> 0 then
                        // Remove backwards so the surviving column order is unchanged.
                        for index in storage.Free.Count - 1 .. -1 .. 0 do
                            if storage.Blocked.Contains storage.Free[index] then
                                storage.Free.RemoveAt index

                        activeSet ()
                    else
                        storage.Delta.Clear()

                        for index in 0 .. storage.Free.Count - 1 do
                            storage.Delta[storage.Free[index]] <- reduced.Step[index]

            activeSet ()

    let private copyAngles (source: ResizeArray<float>) (destination: ResizeArray<float>) =
        for joint in 0 .. source.Count - 1 do
            destination[joint] <- source[joint]

    let private refine options chain (target: ValuePose) (bounds: Limit array) (storage: Workspace) (seed: float list) =
        seed |> List.iteri (fun joint angle -> storage.Angles[joint] <- angle)

        let snapshot count position orientation : Solution =
            { Angles = List.ofSeq storage.Angles
              PositionError = position
              OrientationError = orientation
              Iterations = count }

        let rec iterate count damping =
            let current = Geometry.forwardBuffer chain storage.Angles
            let struct (position, orientation) = ValueGeometry.poseError target current

            if withinErrors options position orientation || count >= options.MaxIterations then
                snapshot count position orientation
            else
                let dp = ValueGeometry.subtract target.Position current.Position

                let dr =
                    ValueGeometry.scale
                        options.OrientationWeight
                        (ValueGeometry.rotationError target.Orientation current.Orientation)

                storage.Error[0] <- dp.X
                storage.Error[1] <- dp.Y
                storage.Error[2] <- dp.Z
                storage.Error[3] <- dr.X
                storage.Error[4] <- dr.Y
                storage.Error[5] <- dr.Z

                Geometry.fillJacobian chain storage.Angles storage.Axes storage.Jacobian

                for column in 0 .. storage.Jacobian.ColumnCount - 1 do
                    for row in 3..5 do
                        storage.Jacobian[row, column] <- storage.Jacobian[row, column] * options.OrientationWeight

                constrainedStep storage damping bounds
                let maxDelta = storage.Delta.AbsoluteMaximum()

                let factor =
                    if maxDelta > options.MaxStep then
                        options.MaxStep / maxDelta
                    else
                        1.

                let mutable bestPosition = 0.
                let mutable bestOrientation = 0.
                let mutable bestScore = 0.
                // The original five line-search candidates, in the original order.
                for candidate in 0..4 do
                    let alpha =
                        match candidate with
                        | 0 -> 1.
                        | 1 -> 0.5
                        | 2 -> 0.25
                        | 3 -> 0.125
                        | _ -> 0.0625

                    for joint in 0 .. bounds.Length - 1 do
                        let step = storage.Delta[joint] * factor

                        storage.Candidate[joint] <-
                            Math.Clamp(storage.Angles[joint] + alpha * step, bounds[joint].Lower, bounds[joint].Upper)

                    let struct (candidatePosition, candidateOrientation) =
                        ValueGeometry.poseError target (Geometry.forwardBuffer chain storage.Candidate)

                    let candidateScore = scoreErrors options candidatePosition candidateOrientation

                    if candidate = 0 || compare candidateScore bestScore < 0 then
                        bestPosition <- candidatePosition
                        bestOrientation <- candidateOrientation
                        bestScore <- candidateScore
                        copyAngles storage.Candidate storage.BestCandidate

                if bestScore < scoreErrors options position orientation - 1e-16 then
                    copyAngles storage.BestCandidate storage.Angles
                    iterate (count + 1) (max 1e-5 (damping * 0.6))
                elif damping < 0.5 then
                    iterate (count + 1) (damping * 3.)
                else
                    snapshot count position orientation

        iterate 0 0.01

    let private solveValid options chain target seed (model: ModelStorage) (storage: Workspace) =
        let limits = model.Limits
        let refineSeed = refine options chain target model.Bounds storage
        let initial = refineSeed seed

        if within options initial then
            Solved initial
        else
            let alternatives =
                [ for i in 0 .. seed.Length - 1 do
                      for direction in [ 1.; -1. ] do
                          yield
                              seed
                              |> List.mapi (fun index angle -> if index = i then angle + direction * 0.08 else angle)
                              |> bounded limits ]

            let attempted = alternatives |> List.map refineSeed
            let valid = attempted |> List.filter (within options)

            match valid with
            | [] ->
                let radicalInverse radix index =
                    let rec digits remaining weight value =
                        if remaining = 0 then
                            value
                        else
                            digits
                                (remaining / radix)
                                (weight / float radix)
                                (value + float (remaining % radix) * weight)

                    digits index (1. / float radix) 0.

                let primes = [| 2; 3; 5; 7; 11; 13; 17; 19; 23; 29; 31; 37 |]

                let rec restart index best =
                    if index > 64 then
                        Failed(NoConvergence best)
                    else
                        let globalSeed =
                            limits
                            |> List.mapi (fun joint limit ->
                                let fraction = radicalInverse primes[joint % primes.Length] index
                                limit.Lower + fraction * (limit.Upper - limit.Lower))

                        let result = refineSeed globalSeed

                        if within options result then
                            Solved result
                        else
                            restart
                                (index + 1)
                                (if score options result < score options best then
                                     result
                                 else
                                     best)

                restart 1 ((initial :: attempted) |> List.minBy (score options))
            | solutions ->
                solutions
                |> List.minBy (fun solution -> List.map2 (fun a b -> (a - b) ** 2.) seed solution.Angles |> List.sum)
                |> Solved

    let solve (options: Options) (chain: Chain) (target: Pose) (seed: float list) : SolveResult =
        let model = models.GetValue(chain, prepare)
        let limits = model.Limits

        let validPose =
            [ target.Position.X
              target.Position.Y
              target.Position.Z
              target.Orientation.W
              target.Orientation.X
              target.Orientation.Y
              target.Orientation.Z ]
            |> List.forall Double.IsFinite

        let quaternionNorm =
            sqrt (
                target.Orientation.W ** 2.
                + target.Orientation.X ** 2.
                + target.Orientation.Y ** 2.
                + target.Orientation.Z ** 2.
            )

        if
            List.isEmpty limits
            || seed.Length <> limits.Length
            || not (List.forall Double.IsFinite seed)
            || not validPose
            || abs (quaternionNorm - 1.) > 1e-6
        then
            Failed(InvalidInput "Finite joints and a unit-quaternion target are required")
        elif not (List.forall2 (fun limit angle -> angle >= limit.Lower && angle <= limit.Upper) limits seed) then
            Failed(InvalidInput "Seed lies outside model joint limits")
        elif
            options.MaxIterations < 1
            || not (
                List.forall
                    (fun value -> Double.IsFinite value && value > 0.)
                    [ options.PositionTolerance
                      options.OrientationTolerance
                      options.MaxStep
                      options.OrientationWeight ]
            )
        then
            Failed(InvalidInput "Invalid solver options")
        else
            let storage = model.Rent()

            try
                solveValid options chain (ValueGeometry.pose target) seed model storage
            finally
                model.Return storage
