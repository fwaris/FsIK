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

    let private tip chain angles =
        let frame = Geometry.forward chain angles
        Geometry.add frame.Position (Geometry.rotate frame.Orientation (Geometry.v 0. 0. -0.145))

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
            let segments =
                waypoints
                |> List.pairwise
                |> List.map (fun (a, b) ->
                    let largest = List.map2 (fun x y -> abs (y - x)) a b |> List.max
                    // Numerical tip derivative bound receives a margin and is
                    // checked independently on every generated sample afterwards.
                    let tipSlope =
                        [ 0..64 ]
                        |> List.map (fun i -> interpolate a b (float i / 64.) |> tip chain)
                        |> List.pairwise
                        |> List.map (fun (x, y) -> Geometry.norm (Geometry.subtract y x) * 64.)
                        |> List.max

                    let duration =
                        [ 1.875 * largest / limits.MaxJointVelocity
                          sqrt ((10. / sqrt 3.) * largest / limits.MaxJointAcceleration)
                          1.875 * tipSlope * 1.1 / limits.MaxTipSpeed ]
                        |> List.max

                    let ticks = max 2. (ceil (duration * float limits.Rate))
                    a, b, ticks)

            let totalTicks = segments |> List.sumBy (fun (_, _, ticks) -> ticks)
            let duration = totalTicks / float limits.Rate

            if
                not (Double.IsFinite duration)
                || duration > limits.MaxDuration
                || totalTicks > float Int32.MaxValue
            then
                Error(DurationExceeded duration)
            else
                let samples =
                    waypoints.Head
                    :: [ for a, b, ticks in segments do
                             for i in 1 .. int ticks do
                                 yield interpolate a b (blend (float i / ticks)) ]

                Ok
                    { Samples = samples
                      DurationSeconds = duration }
