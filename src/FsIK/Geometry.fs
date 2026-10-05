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

[<Struct>]
type internal ValueVec3 = { X: float; Y: float; Z: float }

[<Struct>]
type internal ValueQuaternion =
    { W: float
      X: float
      Y: float
      Z: float }

[<Struct>]
type internal ValuePose =
    { Position: ValueVec3
      Orientation: ValueQuaternion }

// Internal double-precision geometry avoids boxing and temporary public records.
// Conversions belong at API boundaries; each numerical call owns its scratch.
module internal ValueGeometry =
    let v x y z : ValueVec3 = { X = x; Y = y; Z = z }
    let vector (a: Vec3) = v a.X a.Y a.Z
    let publicVector (a: ValueVec3) : Vec3 = { X = a.X; Y = a.Y; Z = a.Z }

    let quaternion (q: Quaternion) : ValueQuaternion = { W = q.W; X = q.X; Y = q.Y; Z = q.Z }

    let publicQuaternion (q: ValueQuaternion) : Quaternion = { W = q.W; X = q.X; Y = q.Y; Z = q.Z }

    let pose (p: Pose) : ValuePose =
        { Position = vector p.Position
          Orientation = quaternion p.Orientation }

    let publicPose (p: ValuePose) : Pose =
        { Position = publicVector p.Position
          Orientation = publicQuaternion p.Orientation }

    let add (a: ValueVec3) (b: ValueVec3) = v (a.X + b.X) (a.Y + b.Y) (a.Z + b.Z)
    let subtract (a: ValueVec3) (b: ValueVec3) = v (a.X - b.X) (a.Y - b.Y) (a.Z - b.Z)
    let scale s (a: ValueVec3) = v (s * a.X) (s * a.Y) (s * a.Z)

    let norm (a: ValueVec3) =
        sqrt (a.X * a.X + a.Y * a.Y + a.Z * a.Z)

    let cross (a: ValueVec3) (b: ValueVec3) =
        v (a.Y * b.Z - a.Z * b.Y) (a.Z * b.X - a.X * b.Z) (a.X * b.Y - a.Y * b.X)

    let multiply (a: ValueQuaternion) (b: ValueQuaternion) : ValueQuaternion =
        let w = a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z
        let x = a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y
        let y = a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X
        let z = a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W
        let n = sqrt (w * w + x * x + y * y + z * z)

        { W = w / n
          X = x / n
          Y = y / n
          Z = z / n }

    let conjugate (q: ValueQuaternion) = { q with X = -q.X; Y = -q.Y; Z = -q.Z }

    let rotate (q: ValueQuaternion) (a: ValueVec3) =
        let crossX = q.Y * a.Z - q.Z * a.Y
        let crossY = q.Z * a.X - q.X * a.Z
        let crossZ = q.X * a.Y - q.Y * a.X
        let twiceW = 2. * q.W

        v
            (a.X + (twiceW * crossX + 2. * (q.Y * crossZ - q.Z * crossY)))
            (a.Y + (twiceW * crossY + 2. * (q.Z * crossX - q.X * crossZ)))
            (a.Z + (twiceW * crossZ + 2. * (q.X * crossY - q.Y * crossX)))

    let rotation (axis: ValueVec3) angle : ValueQuaternion =
        let factor = sin (angle / 2.) / norm axis

        { W = cos (angle / 2.)
          X = axis.X * factor
          Y = axis.Y * factor
          Z = axis.Z * factor }

    let identity: ValuePose =
        { Position = v 0. 0. 0.
          Orientation = { W = 1.; X = 0.; Y = 0.; Z = 0. } }

    let compose (a: ValuePose) (b: ValuePose) : ValuePose =
        { Position = add a.Position (rotate a.Orientation b.Position)
          Orientation = multiply a.Orientation b.Orientation }

    let rotationError (desired: ValueQuaternion) (actual: ValueQuaternion) =
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

    let poseError (desired: ValuePose) (actual: ValuePose) =
        struct (norm (subtract desired.Position actual.Position),
                norm (rotationError desired.Orientation actual.Orientation))


module Geometry =
    let v x y z : Vec3 = { X = x; Y = y; Z = z }

    let add (a: Vec3) (b: Vec3) =
        ValueGeometry.add (ValueGeometry.vector a) (ValueGeometry.vector b)
        |> ValueGeometry.publicVector

    let subtract (a: Vec3) (b: Vec3) =
        ValueGeometry.subtract (ValueGeometry.vector a) (ValueGeometry.vector b)
        |> ValueGeometry.publicVector

    let scale s (a: Vec3) =
        ValueGeometry.scale s (ValueGeometry.vector a) |> ValueGeometry.publicVector

    let norm (a: Vec3) =
        ValueGeometry.norm (ValueGeometry.vector a)

    let cross (a: Vec3) (b: Vec3) =
        ValueGeometry.cross (ValueGeometry.vector a) (ValueGeometry.vector b)
        |> ValueGeometry.publicVector

    let multiply (a: Quaternion) (b: Quaternion) : Quaternion =
        ValueGeometry.multiply (ValueGeometry.quaternion a) (ValueGeometry.quaternion b)
        |> ValueGeometry.publicQuaternion

    let conjugate (q: Quaternion) =
        ValueGeometry.conjugate (ValueGeometry.quaternion q)
        |> ValueGeometry.publicQuaternion

    let rotate (q: Quaternion) (a: Vec3) =
        ValueGeometry.rotate (ValueGeometry.quaternion q) (ValueGeometry.vector a)
        |> ValueGeometry.publicVector

    let private rotation axis angle =
        ValueGeometry.rotation (ValueGeometry.vector axis) angle
        |> ValueGeometry.publicQuaternion

    let identity: Pose = ValueGeometry.publicPose ValueGeometry.identity

    let compose (a: Pose) (b: Pose) =
        ValueGeometry.compose (ValueGeometry.pose a) (ValueGeometry.pose b)
        |> ValueGeometry.publicPose

    let rotationError (desired: Quaternion) (actual: Quaternion) : Vec3 =
        ValueGeometry.rotationError (ValueGeometry.quaternion desired) (ValueGeometry.quaternion actual)
        |> ValueGeometry.publicVector

    let poseError (desired: Pose) (actual: Pose) =
        let struct (position, orientation) =
            ValueGeometry.poseError (ValueGeometry.pose desired) (ValueGeometry.pose actual)

        position, orientation

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

    let internal forwardValue (Chain joints) (angles: float list) : ValuePose =
        let rec loop pending remaining (pose: ValuePose) =
            match pending with
            | [] when remaining = [] -> pose
            | joint :: rest ->
                let origin = ValueGeometry.compose pose (ValueGeometry.pose joint.Origin)

                match joint.Kind, remaining with
                | Fixed, _ -> loop rest remaining origin
                | Revolute _, angle :: tail ->
                    let next =
                        { origin with
                            Orientation =
                                ValueGeometry.multiply
                                    origin.Orientation
                                    (ValueGeometry.rotation (ValueGeometry.vector joint.Axis) angle) }

                    loop rest tail next
                | _ -> invalidArg "angles" "Joint count mismatch"
            | _ -> invalidArg "angles" "Joint count mismatch"

        loop joints angles ValueGeometry.identity

    // Consumes caller-owned scratch synchronously; no buffer reference escapes.
    let internal forwardBuffer (Chain joints) (angles: ResizeArray<float>) : ValuePose =
        let rec loop pending index (pose: ValuePose) =
            match pending with
            | [] when index = angles.Count -> pose
            | joint :: rest ->
                let origin = ValueGeometry.compose pose (ValueGeometry.pose joint.Origin)

                match joint.Kind with
                | Fixed -> loop rest index origin
                | Revolute _ when index < angles.Count ->
                    let next =
                        { origin with
                            Orientation =
                                ValueGeometry.multiply
                                    origin.Orientation
                                    (ValueGeometry.rotation (ValueGeometry.vector joint.Axis) angles[index]) }

                    loop rest (index + 1) next
                | _ -> invalidArg "angles" "Joint count mismatch"
            | _ -> invalidArg "angles" "Joint count mismatch"

        loop joints 0 ValueGeometry.identity

    let forward (chain: Chain) (angles: float list) : Pose =
        forwardValue chain angles |> ValueGeometry.publicPose

    let internal jacobian (Chain joints) (angles: float list) =
        // Origins and axes are value pairs in per-call scratch, in joint order.
        let axes = Array.zeroCreate<struct (ValueVec3 * ValueVec3)> angles.Length

        let rec loop pending remaining (pose: ValuePose) column =
            match pending with
            | [] when remaining = [] -> pose
            | joint :: rest ->
                let origin = ValueGeometry.compose pose (ValueGeometry.pose joint.Origin)

                match joint.Kind, remaining with
                | Fixed, _ -> loop rest remaining origin column
                | Revolute _, angle :: tail ->
                    let axis = ValueGeometry.vector joint.Axis
                    axes[column] <- struct (origin.Position, ValueGeometry.rotate origin.Orientation axis)

                    let next =
                        { origin with
                            Orientation = ValueGeometry.multiply origin.Orientation (ValueGeometry.rotation axis angle) }

                    loop rest tail next (column + 1)
                | _ -> invalidArg "angles" "Joint count mismatch"
            | _ -> invalidArg "angles" "Joint count mismatch"

        let tip = loop joints angles ValueGeometry.identity 0
        let matrix = Matrix<float>.Build.Dense(6, axes.Length)

        for column in 0 .. axes.Length - 1 do
            let struct (origin, axis) = axes[column]
            let linear = ValueGeometry.cross axis (ValueGeometry.subtract tip.Position origin)
            matrix[0, column] <- linear.X
            matrix[1, column] <- linear.Y
            matrix[2, column] <- linear.Z
            matrix[3, column] <- axis.X
            matrix[4, column] <- axis.Y
            matrix[5, column] <- axis.Z

        matrix

    // Every entry is overwritten before use; caller owns both buffers exclusively.
    let internal fillJacobian
        (Chain joints)
        (angles: ResizeArray<float>)
        (axes: struct (ValueVec3 * ValueVec3) array)
        (matrix: Matrix<float>)
        =
        let rec loop pending (pose: ValuePose) column =
            match pending with
            | [] when column = angles.Count -> pose
            | joint :: rest ->
                let origin = ValueGeometry.compose pose (ValueGeometry.pose joint.Origin)

                match joint.Kind with
                | Fixed -> loop rest origin column
                | Revolute _ when column < angles.Count ->
                    let axis = ValueGeometry.vector joint.Axis
                    axes[column] <- struct (origin.Position, ValueGeometry.rotate origin.Orientation axis)

                    let next =
                        { origin with
                            Orientation =
                                ValueGeometry.multiply origin.Orientation (ValueGeometry.rotation axis angles[column]) }

                    loop rest next (column + 1)
                | _ -> invalidArg "angles" "Joint count mismatch"
            | _ -> invalidArg "angles" "Joint count mismatch"

        let tip = loop joints ValueGeometry.identity 0

        for column in 0 .. angles.Count - 1 do
            let struct (origin, axis) = axes[column]
            let linear = ValueGeometry.cross axis (ValueGeometry.subtract tip.Position origin)
            matrix[0, column] <- linear.X
            matrix[1, column] <- linear.Y
            matrix[2, column] <- linear.Z
            matrix[3, column] <- axis.X
            matrix[4, column] <- axis.Y
            matrix[5, column] <- axis.Z

    let jacobianRows (chain: Chain) (angles: float list) : float list list =
        let matrix = jacobian chain angles
        [ for row in 0..5 -> [ for column in 0 .. angles.Length - 1 -> matrix[row, column] ] ]
