namespace FsIK

open System
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

    let private bounded limits angles =
        List.map2 (fun limit value -> Math.Clamp(value, limit.Lower, limit.Upper)) limits angles

    let private summary chain (target: ValuePose) angles iterations =
        let struct (position, orientation) =
            ValueGeometry.poseError target (Geometry.forwardValue chain angles)

        { Angles = angles
          PositionError = position
          OrientationError = orientation
          Iterations = iterations }

    let private score options info =
        info.PositionError ** 2.
        + (options.OrientationWeight * info.OrientationError) ** 2.

    let private within options info =
        info.PositionError <= options.PositionTolerance
        && info.OrientationError <= options.OrientationTolerance

    let private dampedStep (jacobian: Matrix<float>) (error: Vector<float>) damping =
        let svd = jacobian.Svd(true)
        let coefficients = svd.U.TransposeThisAndMultiply(error)

        let scaled =
            Vector<float>.Build
                .Dense(
                    jacobian.ColumnCount,
                    fun i ->
                        if i < svd.S.Count then
                            coefficients[i] * svd.S[i] / (svd.S[i] * svd.S[i] + damping * damping)
                        else
                            0.
                )

        svd.VT.TransposeThisAndMultiply(scaled)

    let private constrainedStep (jacobian: Matrix<float>) error damping (limits: Limit list) (angles: float list) =
        // Resolve with outward-pointing joints at their bounds excluded. Simply
        // clipping an unconstrained step can otherwise stall at a joint limit.
        let rec activeSet free =
            if List.isEmpty free then
                Vector<float>.Build.Dense angles.Length
            else
                let selected =
                    Matrix<float>.Build.Dense(6, free.Length, fun r c -> jacobian[r, free[c]])

                let reduced = dampedStep selected error damping

                let blocked =
                    free
                    |> List.mapi (fun index joint -> joint, reduced[index])
                    |> List.choose (fun (joint, step) ->
                        if
                            (angles[joint] <= limits[joint].Lower + 1e-10 && step < 0.)
                            || (angles[joint] >= limits[joint].Upper - 1e-10 && step > 0.)
                        then
                            Some joint
                        else
                            None)

                if not (List.isEmpty blocked) then
                    activeSet (free |> List.filter (fun joint -> not (List.contains joint blocked)))
                else
                    Vector<float>.Build
                        .Dense(
                            angles.Length,
                            fun joint ->
                                free
                                |> List.tryFindIndex ((=) joint)
                                |> Option.map (fun index -> reduced[index])
                                |> Option.defaultValue 0.
                        )

        // The initial active set contains every column already in the Jacobian.
        // Reuse that matrix and its result when no joint must be excluded.
        let initial = dampedStep jacobian error damping

        let rec blockedJoints joint remainingLimits remainingAngles blocked =
            match remainingLimits, remainingAngles with
            | limit :: limitTail, angle :: angleTail ->
                let step = initial[joint]

                let next =
                    if
                        (angle <= limit.Lower + 1e-10 && step < 0.)
                        || (angle >= limit.Upper - 1e-10 && step > 0.)
                    then
                        joint :: blocked
                    else
                        blocked

                blockedJoints (joint + 1) limitTail angleTail next
            | _ -> blocked

        match blockedJoints 0 limits angles [] with
        | [] -> initial
        | blocked ->
            activeSet (
                [ 0 .. angles.Length - 1 ]
                |> List.filter (fun joint -> not (List.contains joint blocked))
            )

    let private refine options chain (target: ValuePose) seed =
        let limits = Geometry.limits chain

        let rec iterate count damping angles =
            let current = Geometry.forwardValue chain angles
            let struct (position, orientation) = ValueGeometry.poseError target current

            let info =
                { Angles = angles
                  PositionError = position
                  OrientationError = orientation
                  Iterations = count }

            if within options info || count >= options.MaxIterations then
                info
            else
                let dp = ValueGeometry.subtract target.Position current.Position

                let dr =
                    ValueGeometry.scale
                        options.OrientationWeight
                        (ValueGeometry.rotationError target.Orientation current.Orientation)

                let error =
                    Vector<float>.Build.DenseOfArray [| dp.X; dp.Y; dp.Z; dr.X; dr.Y; dr.Z |]

                // The Jacobian is fresh scratch storage owned by this iteration.
                // Weight its orientation rows without allocating a second matrix.
                let j = Geometry.jacobian chain angles

                for column in 0 .. j.ColumnCount - 1 do
                    for row in 3..5 do
                        j[row, column] <- j[row, column] * options.OrientationWeight

                let delta = constrainedStep j error damping limits angles
                let maxDelta = delta.AbsoluteMaximum()

                let factor =
                    if maxDelta > options.MaxStep then
                        options.MaxStep / maxDelta
                    else
                        1.

                let candidateAngles alpha =
                    let rec collect joint remainingLimits remainingAngles =
                        match remainingLimits, remainingAngles with
                        | limit :: limitTail, angle :: angleTail ->
                            let step = delta[joint] * factor

                            Math.Clamp(angle + alpha * step, limit.Lower, limit.Upper)
                            :: collect (joint + 1) limitTail angleTail
                        | [], [] -> []
                        | _ -> invalidArg "angles" "Joint angles and limits must have equal lengths"

                    collect 0 limits angles

                let candidates =
                    [ 1.; 0.5; 0.25; 0.125; 0.0625 ]
                    |> List.map candidateAngles
                    |> List.map (fun candidate -> summary chain target candidate (count + 1))

                let best = candidates |> List.minBy (score options)

                if score options best < score options info - 1e-16 then
                    iterate (count + 1) (max 1e-5 (damping * 0.6)) best.Angles
                elif damping < 0.5 then
                    iterate (count + 1) (damping * 3.) angles
                else
                    info

        iterate 0 0.01 seed

    let solve (options: Options) (chain: Chain) (target: Pose) (seed: float list) : SolveResult =
        let limits = Geometry.limits chain

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
            let target = ValueGeometry.pose target
            let initial = refine options chain target seed

            if within options initial then
                Solved initial
            else
                // Deterministic small perturbations escape straight-arm singularities.
                // These are mathematical seeds only, never intermediate commands.
                let alternatives =
                    [ for i in 0 .. seed.Length - 1 do
                          for direction in [ 1.; -1. ] do
                              yield
                                  seed
                                  |> List.mapi (fun index angle ->
                                      if index = i then angle + direction * 0.08 else angle)
                                  |> bounded limits ]

                let attempted = alternatives |> List.map (refine options chain target)
                let valid = attempted |> List.filter (within options)

                match valid with
                | [] ->
                    // Cover the bounded joint domain when local perturbations remain
                    // trapped in the same basin. Radical inverses provide repeatable,
                    // dispersed seeds without shared random state.
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

                            let result = refine options chain target globalSeed

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
                    |> List.minBy (fun solution ->
                        List.map2 (fun a b -> (a - b) ** 2.) seed solution.Angles |> List.sum)
                    |> Solved
