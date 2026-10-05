namespace FsIK

open System
open System.Globalization
open System.Xml.Linq
open MathNet.Numerics.LinearAlgebra

type Vec3 = { X: float; Y: float; Z: float }

type Quaternion =
    { W: float
      X: float
      Y: float
      Z: float }

type Pose =
    { Position: Vec3
      Orientation: Quaternion }

type Limit = { Lower: float; Upper: float }

type JointKind =
    | Fixed
    | Revolute of Limit

type Joint =
    { Name: string
      Origin: Pose
      Axis: Vec3
      Kind: JointKind }

type Chain = private Chain of Joint list

module Geometry =
    let v x y z : Vec3 = { X = x; Y = y; Z = z }
    let add (a: Vec3) (b: Vec3) = v (a.X + b.X) (a.Y + b.Y) (a.Z + b.Z)
    let subtract (a: Vec3) (b: Vec3) = v (a.X - b.X) (a.Y - b.Y) (a.Z - b.Z)
    let scale s (a: Vec3) = v (s * a.X) (s * a.Y) (s * a.Z)

    let norm (a: Vec3) =
        sqrt (a.X * a.X + a.Y * a.Y + a.Z * a.Z)

    let cross (a: Vec3) (b: Vec3) =
        v (a.Y * b.Z - a.Z * b.Y) (a.Z * b.X - a.X * b.Z) (a.X * b.Y - a.Y * b.X)

    let multiply (a: Quaternion) (b: Quaternion) : Quaternion =
        let w = a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z
        let x = a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y
        let y = a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X
        let z = a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W
        let n = sqrt (w * w + x * x + y * y + z * z)

        { W = w / n
          X = x / n
          Y = y / n
          Z = z / n }

    let conjugate (q: Quaternion) = { q with X = -q.X; Y = -q.Y; Z = -q.Z }

    let rotate (q: Quaternion) (a: Vec3) =
        let crossX = q.Y * a.Z - q.Z * a.Y
        let crossY = q.Z * a.X - q.X * a.Z
        let crossZ = q.X * a.Y - q.Y * a.X
        let twiceW = 2. * q.W

        v
            (a.X + (twiceW * crossX + 2. * (q.Y * crossZ - q.Z * crossY)))
            (a.Y + (twiceW * crossY + 2. * (q.Z * crossX - q.X * crossZ)))
            (a.Z + (twiceW * crossZ + 2. * (q.X * crossY - q.Y * crossX)))

    let private rotation axis angle =
        let factor = sin (angle / 2.) / norm axis

        { W = cos (angle / 2.)
          X = axis.X * factor
          Y = axis.Y * factor
          Z = axis.Z * factor }

    let identity: Pose =
        { Position = v 0. 0. 0.
          Orientation = { W = 1.; X = 0.; Y = 0.; Z = 0. } }

    let compose (a: Pose) (b: Pose) =
        let q = a.Orientation
        let p = b.Position
        let crossX = q.Y * p.Z - q.Z * p.Y
        let crossY = q.Z * p.X - q.X * p.Z
        let crossZ = q.X * p.Y - q.Y * p.X
        let twiceW = 2. * q.W

        { Position =
            v
                (a.Position.X + (p.X + (twiceW * crossX + 2. * (q.Y * crossZ - q.Z * crossY))))
                (a.Position.Y + (p.Y + (twiceW * crossY + 2. * (q.Z * crossX - q.X * crossZ))))
                (a.Position.Z + (p.Z + (twiceW * crossZ + 2. * (q.X * crossY - q.Y * crossX))))
          Orientation = multiply a.Orientation b.Orientation }

    let rotationError (desired: Quaternion) (actual: Quaternion) : Vec3 =
        let q = multiply desired (conjugate actual)

        let q =
            if q.W < 0. then
                { W = -q.W
                  X = -q.X
                  Y = -q.Y
                  Z = -q.Z }
            else
                q

        let vector = v q.X q.Y q.Z
        let length = norm vector

        if length < 1e-12 then
            scale 2. vector
        else
            scale (2. * atan2 length q.W / length) vector

    let poseError (desired: Pose) (actual: Pose) =
        norm (subtract desired.Position actual.Position), norm (rotationError desired.Orientation actual.Orientation)

    let private attr name (node: XElement | null) fallback =
        node
        |> Option.ofObj
        |> Option.bind (fun value -> value.Attribute(XName.Get name) |> Option.ofObj)
        |> Option.map _.Value
        |> Option.defaultValue fallback

    let private number (s: string) =
        Double.Parse(s, CultureInfo.InvariantCulture)

    let private triple (s: string) =
        match s.Split(' ', StringSplitOptions.RemoveEmptyEntries) |> Array.map number with
        | [| x; y; z |] -> v x y z
        | _ -> invalidArg "URDF" "Three coordinates required"

    let load (path: string) (baseLink: string) (tipLink: string) : Chain =
        let doc = XDocument.Load path
        let joints = doc.Descendants(XName.Get "joint") |> Seq.toList

        let rec collect link acc =
            if link = baseLink then
                acc
            else
                let node =
                    joints
                    |> List.find (fun joint -> attr "link" (joint.Element(XName.Get "child")) "" = link)

                let parent = attr "link" (node.Element(XName.Get "parent")) ""
                collect parent (node :: acc)

        collect tipLink []
        |> List.map (fun node ->
            let origin = node.Element(XName.Get "origin")

            let xyz, rpy =
                if isNull origin then
                    v 0. 0. 0., v 0. 0. 0.
                else
                    triple (attr "xyz" origin "0 0 0"), triple (attr "rpy" origin "0 0 0")

            let axis = node.Element(XName.Get "axis")

            let axis =
                if isNull axis then
                    v 1. 0. 0.
                else
                    triple (attr "xyz" axis "1 0 0")

            let axis =
                if norm axis > 1e-12 then
                    scale (1. / norm axis) axis
                else
                    invalidArg "URDF" "Joint axis must be nonzero"

            let kind =
                match attr "type" node "" with
                | "fixed" -> Fixed
                | "revolute" ->
                    let limit = node.Element(XName.Get "limit")

                    Revolute
                        { Lower = number (attr "lower" limit "")
                          Upper = number (attr "upper" limit "") }
                | unsupported -> invalidArg "URDF" ("Unsupported joint type: " + unsupported)

            { Name = attr "name" node ""
              Axis = axis
              Kind = kind
              Origin =
                { Position = xyz
                  Orientation =
                    multiply
                        (rotation (v 0. 0. 1.) rpy.Z)
                        (multiply (rotation (v 0. 1. 0.) rpy.Y) (rotation (v 1. 0. 0.) rpy.X)) } })
        |> Chain

    let limits (Chain joints) : Limit list =
        joints
        |> List.choose (fun joint ->
            match joint.Kind with
            | Fixed -> None
            | Revolute limit -> Some limit)

    let names (Chain joints) : string list =
        joints
        |> List.choose (fun joint ->
            match joint.Kind with
            | Fixed -> None
            | Revolute _ -> Some joint.Name)

    let private evaluate collectAxes (Chain joints) angles =
        let rec loop pending remaining pose axes =
            match pending with
            | [] when remaining = [] -> pose, List.rev axes
            | joint :: rest ->
                let origin = compose pose joint.Origin

                match joint.Kind, remaining with
                | Fixed, _ -> loop rest remaining origin axes
                | Revolute _, angle :: tail ->
                    let next =
                        { Position = origin.Position
                          Orientation = multiply origin.Orientation (rotation joint.Axis angle) }

                    let nextAxes =
                        if collectAxes then
                            let axis = rotate origin.Orientation joint.Axis
                            (origin.Position, axis) :: axes
                        else
                            axes

                    loop rest tail next nextAxes
                | _ -> invalidArg "angles" "Joint count mismatch"
            | _ -> invalidArg "angles" "Joint count mismatch"

        loop joints angles identity []

    let forward (Chain joints) (angles: float list) : Pose =
        // Keep intermediate FK values on the call's stack. Only the final pose
        // needs the immutable records exposed by the public API.
        let product aw ax ay az bw bx by bz =
            let w = aw * bw - ax * bx - ay * by - az * bz
            let x = aw * bx + ax * bw + ay * bz - az * by
            let y = aw * by - ax * bz + ay * bw + az * bx
            let z = aw * bz + ax * by - ay * bx + az * bw
            let n = sqrt (w * w + x * x + y * y + z * z)
            struct (w / n, x / n, y / n, z / n)

        let rec loop pending remaining (struct (px, py, pz, qw, qx, qy, qz)) =
            match pending with
            | [] when remaining = [] ->
                { Position = v px py pz
                  Orientation = { W = qw; X = qx; Y = qy; Z = qz } }
            | joint :: rest ->
                let p = joint.Origin.Position
                let crossX = qy * p.Z - qz * p.Y
                let crossY = qz * p.X - qx * p.Z
                let crossZ = qx * p.Y - qy * p.X
                let twiceW = 2. * qw
                let x = px + (p.X + (twiceW * crossX + 2. * (qy * crossZ - qz * crossY)))
                let y = py + (p.Y + (twiceW * crossY + 2. * (qz * crossX - qx * crossZ)))
                let z = pz + (p.Z + (twiceW * crossZ + 2. * (qx * crossY - qy * crossX)))
                let q = joint.Origin.Orientation
                let struct (w, rx, ry, rz) = product qw qx qy qz q.W q.X q.Y q.Z

                match joint.Kind, remaining with
                | Fixed, _ -> loop rest remaining (struct (x, y, z, w, rx, ry, rz))
                | Revolute _, angle :: tail ->
                    let factor = sin (angle / 2.) / norm joint.Axis

                    let struct (nw, nx, ny, nz) =
                        product
                            w
                            rx
                            ry
                            rz
                            (cos (angle / 2.))
                            (joint.Axis.X * factor)
                            (joint.Axis.Y * factor)
                            (joint.Axis.Z * factor)

                    loop rest tail (struct (x, y, z, nw, nx, ny, nz))
                | _ -> invalidArg "angles" "Joint count mismatch"
            | _ -> invalidArg "angles" "Joint count mismatch"

        loop joints angles (struct (0., 0., 0., 1., 0., 0., 0.))

    let internal jacobian chain angles =
        let pose, axes = evaluate true chain angles

        let columns =
            axes
            |> List.map (fun (origin, axis) ->
                let linear = cross axis (subtract pose.Position origin) in
                [| linear.X; linear.Y; linear.Z; axis.X; axis.Y; axis.Z |])
            |> List.toArray

        Matrix<float>.Build.Dense(6, columns.Length, fun r c -> columns[c][r])

    let jacobianRows (chain: Chain) (angles: float list) : float list list =
        let matrix = jacobian chain angles
        [ for row in 0..5 -> [ for column in 0 .. angles.Length - 1 -> matrix[row, column] ] ]
