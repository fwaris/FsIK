namespace FsIK

open System

type MotionLimits =
    { Rate: int
      MaxJointVelocity: float
      MaxJointAcceleration: float
      MaxTipSpeed: float
      MaxDuration: float }

type Trajectory =
    { Samples: float list list
      DurationSeconds: float }

type TrajectoryFailure =
    | InvalidPath of string
    | DurationExceeded of float

module Trajectory =
    let conservative =
        { Rate = 120
          MaxJointVelocity = 0.1
          MaxJointAcceleration = 0.2
          MaxTipSpeed = 0.02
          MaxDuration = 150. }

    let private blend u = u * u * u * (10. + u * (-15. + 6. * u))

    let private interpolate a b u =
        List.map2 (fun x y -> x + (y - x) * u) a b

    let private fillInterpolated (scratch: ResizeArray<float>) a b u =
        scratch.Clear()

        let rec fill remainingA remainingB =
            match remainingA, remainingB with
            | x :: tailA, y :: tailB ->
                scratch.Add(x + (y - x) * u)
                fill tailA tailB
            | [], [] -> ()
            | _ -> invalidArg "waypoints" "Joint count mismatch"

        fill a b

    let private largestJointChange a b =
        let rec largest remainingA remainingB value =
            match remainingA, remainingB with
            | x :: tailA, y :: tailB -> largest tailA tailB (max value (abs (y - x)))
            | [], [] -> value
            | _ -> invalidArg "waypoints" "Joint count mismatch"

        largest a b 0.

    let private tip chain angles =
        let frame = Geometry.forwardBuffer chain angles
        ValueGeometry.add frame.Position (ValueGeometry.rotate frame.Orientation (ValueGeometry.v 0. 0. -0.145))

    /// Conservative offline commissioning path: quintic easing between waypoints,
    /// stationary at every waypoint. This is not a time-optimal trajectory planner.
    let retime (limits: MotionLimits) chain (waypoints: float list list) : Result<Trajectory, TrajectoryFailure> =
        let bounds = Geometry.limits chain

        let validLimits =
            limits.Rate > 0
            && [ limits.MaxJointVelocity
                 limits.MaxJointAcceleration
                 limits.MaxTipSpeed
                 limits.MaxDuration ]
               |> List.forall (fun value -> Double.IsFinite value && value > 0.)

        let validAngles (angles: float list) =
            angles.Length = bounds.Length
            && List.forall Double.IsFinite angles
            && List.forall2 (fun limit value -> value >= limit.Lower && value <= limit.Upper) bounds angles

        if
            not validLimits
            || List.isEmpty bounds
            || waypoints.Length < 2
            || not (List.forall validAngles waypoints)
        then
            Error(InvalidPath "Finite, legal waypoints and positive motion limits required")
        else
            // One reusable angle buffer per retiming call. Only the previous tip
            // is needed for the unchanged 65-point derivative check per segment.
            let scratch = ResizeArray<float>(bounds.Length)

            let segments =
                ResizeArray<struct (float list * float list * float)>(waypoints.Length - 1)

            let mutable totalTicks = 0.

            let rec collect remaining =
                match remaining with
                | a :: (b :: _ as tail) ->
                    let largest = largestJointChange a b
                    fillInterpolated scratch a b 0.
                    let mutable previous = tip chain scratch
                    let mutable tipSlope = 0.

                    for i in 1..64 do
                        fillInterpolated scratch a b (float i / 64.)
                        let current = tip chain scratch
                        tipSlope <- max tipSlope (ValueGeometry.norm (ValueGeometry.subtract current previous) * 64.)
                        previous <- current

                    let duration =
                        max
                            (max
                                (1.875 * largest / limits.MaxJointVelocity)
                                (sqrt ((10. / sqrt 3.) * largest / limits.MaxJointAcceleration)))
                            (1.875 * tipSlope * 1.1 / limits.MaxTipSpeed)

                    let ticks = max 2. (ceil (duration * float limits.Rate))
                    segments.Add(struct (a, b, ticks))
                    totalTicks <- totalTicks + ticks
                    collect tail
                | _ -> ()

            collect waypoints
            let duration = totalTicks / float limits.Rate

            if
                not (Double.IsFinite duration)
                || duration > limits.MaxDuration
                || totalTicks > float Int32.MaxValue
            then
                Error(DurationExceeded duration)
            else
                // Returned frames own separate immutable lists. The scratch
                // buffer is never stored in a trajectory or shared across calls.
                let samples =
                    waypoints.Head
                    :: [ for struct (a, b, ticks) in segments do
                             for i in 1 .. int ticks do
                                 yield interpolate a b (blend (float i / ticks)) ]

                Ok
                    { Samples = samples
                      DurationSeconds = duration }

    /// Shape-preserving C1 joint spline with one smooth clock for the whole path.
    /// Intermediate waypoints are not stops. This is bounded, not time-optimal.
    let retimeContinuous
        (limits: MotionLimits)
        chain
        (waypoints: float list list)
        : Result<Trajectory, TrajectoryFailure> =
        let bounds = Geometry.limits chain |> List.toArray

        let legal angles =
            List.length angles = bounds.Length
            && List.forall Double.IsFinite angles
            && List.forall2
                (fun bound angle -> angle >= bound.Lower && angle <= bound.Upper)
                (Array.toList bounds)
                angles

        let validLimits =
            limits.Rate > 0
            && [ limits.MaxJointVelocity
                 limits.MaxJointAcceleration
                 limits.MaxTipSpeed
                 limits.MaxDuration ]
               |> List.forall (fun value -> Double.IsFinite value && value > 0.)

        if
            not validLimits
            || bounds.Length = 0
            || waypoints.Length < 2
            || not (List.forall legal waypoints)
        then
            Error(InvalidPath "Finite, legal waypoints and positive motion limits required")
        else
            let points = waypoints |> List.map List.toArray |> List.toArray
            let scratch = ResizeArray<float>(bounds.Length)

            let tipOf (angles: float array) =
                scratch.Clear()
                scratch.AddRange angles
                tip chain scratch

            let segments = points.Length - 1
            let slopes = Array.init segments (fun i -> Array.map2 (-) points[i + 1] points[i])
            let tangents = Array.init points.Length (fun _ -> Array.zeroCreate bounds.Length)

            for joint in 0 .. bounds.Length - 1 do
                tangents[0][joint] <- slopes[0][joint]
                tangents[segments][joint] <- slopes[segments - 1][joint]

                for i in 1 .. segments - 1 do
                    let a, b = slopes[i - 1][joint], slopes[i][joint]
                    // Harmonic monotone tangents prevent overshoot of legal endpoints.
                    tangents[i][joint] <- if a * b <= 0. then 0. else 2. * a * b / (a + b)

            let evaluate fraction =
                if fraction <= 0. then
                    Array.copy points[0]
                elif fraction >= 1. then
                    Array.copy points[segments]
                else
                    let position = fraction * float segments
                    let i = min (segments - 1) (int position)
                    let u = position - float i
                    let u2, u3 = u * u, u * u * u

                    Array.init bounds.Length (fun joint ->
                        let value =
                            (2. * u3 - 3. * u2 + 1.) * points[i][joint]
                            + (u3 - 2. * u2 + u) * tangents[i][joint]
                            + (-2. * u3 + 3. * u2) * points[i + 1][joint]
                            + (u3 - u2) * tangents[i + 1][joint]

                        max bounds[joint].Lower (min bounds[joint].Upper value))

            let travel =
                Array.init bounds.Length (fun joint -> slopes |> Array.sumBy (fun step -> abs step[joint]))
                |> Array.max

            let tipTravel =
                points
                |> Array.pairwise
                |> Array.sumBy (fun (a, b) -> ValueGeometry.norm (ValueGeometry.subtract (tipOf b) (tipOf a)))

            let initialDuration =
                max
                    (1.875 * travel / limits.MaxJointVelocity)
                    (max
                        (sqrt ((10. / sqrt 3.) * travel / limits.MaxJointAcceleration))
                        (1.875 * tipTravel * 1.1 / limits.MaxTipSpeed))

            let rate = float limits.Rate

            let rec sample attempt duration =
                let ticks = max 2. (ceil (duration * rate))
                let actualDuration = ticks / rate

                if
                    not (Double.IsFinite ticks)
                    || actualDuration > limits.MaxDuration
                    || ticks >= float Int32.MaxValue
                then
                    Error(DurationExceeded actualDuration)
                elif attempt > 12 then
                    Error(InvalidPath "Continuous retiming did not satisfy sampled motion limits")
                else
                    let frames =
                        Array.init (int ticks + 1) (fun i -> evaluate (blend (float i / ticks)))

                    let mutable peakVelocity, peakAcceleration, peakTipSpeed = 0., 0., 0.
                    let previousVelocity = Array.zeroCreate bounds.Length
                    let mutable previousTip = tipOf frames[0]

                    for i in 1 .. frames.Length - 1 do
                        for joint in 0 .. bounds.Length - 1 do
                            let velocity = (frames[i][joint] - frames[i - 1][joint]) * rate
                            peakVelocity <- max peakVelocity (abs velocity)
                            peakAcceleration <- max peakAcceleration (abs (velocity - previousVelocity[joint]) * rate)
                            previousVelocity[joint] <- velocity

                        let currentTip = tipOf frames[i]

                        peakTipSpeed <-
                            max peakTipSpeed (ValueGeometry.norm (ValueGeometry.subtract currentTip previousTip) * rate)

                        previousTip <- currentTip

                    for velocity in previousVelocity do
                        peakAcceleration <- max peakAcceleration (abs velocity * rate)

                    let stretch =
                        max
                            (peakVelocity / limits.MaxJointVelocity)
                            (max
                                (sqrt (peakAcceleration / limits.MaxJointAcceleration))
                                (peakTipSpeed / limits.MaxTipSpeed))

                    if stretch > 1. then
                        sample (attempt + 1) (actualDuration * stretch * 1.05)
                    else
                        Ok
                            { Samples = frames |> Array.map Array.toList |> Array.toList
                              DurationSeconds = actualDuration }

            sample 0 initialDuration
