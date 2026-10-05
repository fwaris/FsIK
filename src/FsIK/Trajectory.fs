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
