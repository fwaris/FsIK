namespace FsMotion

open System
open System.IO
open System.Globalization
open System.Security.Cryptography
open System.Xml.Linq
open FsIK

/// A conservative, solid enclosing shape in its link's coordinates.
type Capsule = { A: Vec3; B: Vec3; Radius: float }
type LinkShape = { Link: string; Shape: Capsule }

type RobotJoint =
    { Name: string
      Parent: string
      Child: string
      Origin: Pose
      Axis: Vec3
      Bounds: Limit option }

type RobotModel =
    private
        { Id: string
          Root: string
          Joints: RobotJoint list
          Shapes: LinkShape list
          Excluded: Set<string * string>
          Reach: float
          Chains: Map<string, Chain> }

type Obstacle = { Name: string; Shape: Capsule }

type Scene =
    { Id: string; Obstacles: Obstacle list }

module Model =
    let pair a b = if a < b then a, b else b, a

    let private number (text: string) =
        Double.Parse(text, CultureInfo.InvariantCulture)

    let private attribute name fallback (node: XElement | null) =
        node
        |> Option.ofObj
        |> Option.bind (fun element -> element.Attribute(XName.Get name) |> Option.ofObj)
        |> Option.map _.Value
        |> Option.defaultValue fallback

    let private vector (text: string) =
        match
            text.Split([| ' '; '\t'; '\r'; '\n' |], StringSplitOptions.RemoveEmptyEntries)
            |> Array.map number
        with
        | [| x; y; z |] when [ x; y; z ] |> List.forall Double.IsFinite -> Geometry.v x y z
        | _ -> invalidArg "URDF" "Finite three-coordinate vector required"

    let private rotation axis angle : Quaternion =
        let s = sin (angle / 2.) / Geometry.norm axis

        { W = cos (angle / 2.)
          X = axis.X * s
          Y = axis.Y * s
          Z = axis.Z * s }

    let private origin (node: XElement | null) =
        let xyz = vector (attribute "xyz" "0 0 0" node)
        let rpy = vector (attribute "rpy" "0 0 0" node)

        { Position = xyz
          Orientation =
            Geometry.multiply
                (rotation (Geometry.v 0. 0. 1.) rpy.Z)
                (Geometry.multiply (rotation (Geometry.v 0. 1. 0.) rpy.Y) (rotation (Geometry.v 1. 0. 0.) rpy.X)) }

    let transform (pose: Pose) p =
        Geometry.add pose.Position (Geometry.rotate pose.Orientation p)

    let transformShape pose shape =
        { shape with
            A = transform pose shape.A
            B = transform pose shape.B }

    let validShape shape =
        [ shape.A.X
          shape.A.Y
          shape.A.Z
          shape.B.X
          shape.B.Y
          shape.B.Z
          shape.Radius ]
        |> List.forall Double.IsFinite
        && shape.Radius > 0.

    /// Fits a capsule around all vertices. Convexity encloses triangle interiors too.
    let private enclosing (vertices: Vec3 list) =
        if vertices.IsEmpty then
            invalidArg "mesh" "Empty mesh"

        let lo =
            Geometry.v (vertices |> List.minBy _.X).X (vertices |> List.minBy _.Y).Y (vertices |> List.minBy _.Z).Z

        let hi =
            Geometry.v (vertices |> List.maxBy _.X).X (vertices |> List.maxBy _.Y).Y (vertices |> List.maxBy _.Z).Z

        let center = Geometry.scale 0.5 (Geometry.add lo hi)
        let spans = [| hi.X - lo.X; hi.Y - lo.Y; hi.Z - lo.Z |]
        let axis = [ 0; 1; 2 ] |> List.maxBy (fun i -> spans[i])

        let along (p: Vec3) =
            match axis with
            | 0 -> Geometry.v p.X center.Y center.Z
            | 1 -> Geometry.v center.X p.Y center.Z
            | _ -> Geometry.v center.X center.Y p.Z

        let radius =
            vertices
            |> List.map (fun p -> Geometry.norm (Geometry.subtract p (along p)))
            |> List.max

        { A = along lo
          B = along hi
          Radius = max 1e-9 radius }

    let private stl (path: string) (scale: Vec3) =
        let bytes = File.ReadAllBytes path

        if bytes.Length < 84 then
            invalidArg "mesh" "Binary STL required"

        let count = BitConverter.ToUInt32(bytes, 80)

        if int64 bytes.Length <> 84L + 50L * int64 count || count > 1000000u then
            invalidArg "mesh" "Invalid or oversized binary STL"

        [ for triangle in 0 .. int count - 1 do
              for vertex in 0..2 do
                  let offset = 84 + 50 * triangle + 12 + 12 * vertex

                  yield
                      Geometry.v
                          (float (BitConverter.ToSingle(bytes, offset)) * scale.X)
                          (float (BitConverter.ToSingle(bytes, offset + 4)) * scale.Y)
                          (float (BitConverter.ToSingle(bytes, offset + 8)) * scale.Z) ]
        |> fun points ->
            if
                points
                |> List.exists (fun p -> not ([ p.X; p.Y; p.Z ] |> List.forall Double.IsFinite))
            then
                invalidArg "mesh" "Nonfinite vertices"

            enclosing points

    /// Caller supplies package URI resolution and explicit allowed collision pairs.
    /// Only directly joint-connected links are excluded automatically.
    let load
        (path: string)
        (resolveMesh: string -> string)
        (allowedPairs: (string * string) list)
        : Result<RobotModel, string> =
        try
            let xml = XDocument.Load path

            let rootElement =
                xml.Root
                |> Option.ofObj
                |> Option.defaultWith (fun () -> invalidArg "URDF" "Missing root")

            let nodes = rootElement.Elements(XName.Get "joint") |> Seq.toList

            let links =
                rootElement.Elements(XName.Get "link")
                |> Seq.map (attribute "name" "")
                |> Set.ofSeq

            let joints =
                nodes
                |> List.map (fun node ->
                    let kind = attribute "type" "" node

                    let bounds =
                        match kind with
                        | "fixed" -> None
                        | "revolute" ->
                            let limit = node.Element(XName.Get "limit")

                            let lower, upper =
                                number (attribute "lower" "" limit), number (attribute "upper" "" limit)

                            if not (Double.IsFinite lower && Double.IsFinite upper) || lower >= upper then
                                invalidArg "URDF" "Invalid joint limits"

                            Some { Lower = lower; Upper = upper }
                        | _ -> invalidArg "URDF" ("Unsupported joint type: " + kind)

                    if not (isNull (node.Element(XName.Get "mimic"))) then
                        invalidArg "URDF" "Mimic joints require an explicit model"

                    let axis = vector (attribute "xyz" "1 0 0" (node.Element(XName.Get "axis")))

                    if Geometry.norm axis < 1e-12 then
                        invalidArg "URDF" "Zero joint axis"

                    { Name = attribute "name" "" node
                      Parent = attribute "link" "" (node.Element(XName.Get "parent"))
                      Child = attribute "link" "" (node.Element(XName.Get "child"))
                      Origin = origin (node.Element(XName.Get "origin"))
                      Axis = axis
                      Bounds = bounds })

            if
                joints
                |> List.exists (fun j -> not (links.Contains j.Parent && links.Contains j.Child))
            then
                invalidArg "URDF" "Unknown link"

            if
                (joints |> List.map _.Name |> Set.ofList).Count <> joints.Length
                || (joints |> List.map _.Child |> Set.ofList).Count <> joints.Length
            then
                invalidArg "URDF" "Duplicate joint or multiple parents"

            let root =
                Set.difference links (joints |> List.map _.Child |> Set.ofList)
                |> Set.toList
                |> List.exactlyOne

            let rec order reached pending result =
                match pending with
                | [] -> List.rev result
                | _ ->
                    match pending |> List.tryFind (fun j -> Set.contains j.Parent reached) with
                    | None -> invalidArg "URDF" "Disconnected or cyclic joint tree"
                    | Some joint ->
                        order
                            (Set.add joint.Child reached)
                            (pending |> List.filter (fun j -> j.Name <> joint.Name))
                            (joint :: result)

            let ordered = order (Set.singleton root) joints []
            let assetHashes = ResizeArray<string>()

            let shapes =
                rootElement.Elements(XName.Get "link")
                |> Seq.collect (fun link ->
                    link.Elements(XName.Get "collision")
                    |> Seq.map (fun collision ->
                        let geometry =
                            collision.Element(XName.Get "geometry")
                            |> Option.ofObj
                            |> Option.defaultWith (fun () -> invalidArg "URDF" "Missing geometry")

                        let shapeNode = geometry.Elements() |> Seq.exactlyOne

                        let shape =
                            match shapeNode.Name.LocalName with
                            | "mesh" ->
                                let filename = resolveMesh (attribute "filename" "" shapeNode)
                                assetHashes.Add(Convert.ToHexString(SHA256.HashData(File.ReadAllBytes filename)))
                                stl filename (vector (attribute "scale" "1 1 1" shapeNode))
                            | "sphere" ->
                                { A = Geometry.v 0. 0. 0.
                                  B = Geometry.v 0. 0. 0.
                                  Radius = number (attribute "radius" "" shapeNode) }
                            | "cylinder" ->
                                let half = number (attribute "length" "" shapeNode) / 2.

                                if half <= 0. then
                                    invalidArg "URDF" "Invalid cylinder"

                                { A = Geometry.v 0. 0. -half
                                  B = Geometry.v 0. 0. half
                                  Radius = number (attribute "radius" "" shapeNode) }
                            | "box" ->
                                let size = vector (attribute "size" "" shapeNode)

                                if min size.X (min size.Y size.Z) <= 0. then
                                    invalidArg "URDF" "Invalid box"

                                [ for x in [ -0.5; 0.5 ] do
                                      for y in [ -0.5; 0.5 ] do
                                          for z in [ -0.5; 0.5 ] do
                                              yield Geometry.v (x * size.X) (y * size.Y) (z * size.Z) ]
                                |> enclosing
                            | _ -> invalidArg "URDF" "Unsupported collision geometry"

                        if not (validShape shape) then
                            invalidArg "URDF" "Invalid collision shape"

                        { Link = attribute "name" "" link
                          Shape = transformShape (origin (collision.Element(XName.Get "origin"))) shape }))
                |> Seq.toList

            if shapes.IsEmpty then
                invalidArg "URDF" "Collision geometry required"

            if
                allowedPairs
                |> List.exists (fun (a, b) -> a = b || not (links.Contains a && links.Contains b))
            then
                invalidArg "allowedPairs" "Unknown or identical links"

            let excluded =
                (allowedPairs @ (joints |> List.map (fun j -> j.Parent, j.Child)))
                |> List.map (fun (a, b) -> pair a b)
                |> Set.ofList

            let shapeReach =
                shapes
                |> List.map (fun s -> max (Geometry.norm s.Shape.A) (Geometry.norm s.Shape.B) + s.Shape.Radius)
                |> List.max

            let reach =
                (joints |> List.sumBy (fun j -> Geometry.norm j.Origin.Position)) + shapeReach

            let identity =
                File.ReadAllText path + String.concat "" assetHashes + sprintf "%A" excluded

            let chains =
                links
                |> Set.toList
                |> List.map (fun tip -> tip, Geometry.load path root tip)
                |> Map.ofList

            Ok
                { Id = Convert.ToHexString(SHA256.HashData(Text.Encoding.UTF8.GetBytes identity))
                  Root = root
                  Joints = ordered
                  Shapes = shapes
                  Excluded = excluded
                  Reach = reach
                  Chains = chains }
        with error ->
            Error error.Message

    let id (model: RobotModel) = model.Id

    let joints (model: RobotModel) =
        model.Joints
        |> List.choose (fun j -> j.Bounds |> Option.map (fun b -> j.Name, b))
        |> Map.ofList

    let shapeCount (model: RobotModel) = model.Shapes.Length
    let reach (model: RobotModel) = model.Reach

    let excluded (model: RobotModel) a b =
        a = b || model.Excluded.Contains(pair a b)

    let chain (model: RobotModel) tip =
        match model.Chains.TryFind tip with
        | Some chain -> Ok chain
        | None -> Error("Unknown tip link: " + tip)

    let ownsChain (model: RobotModel) chain =
        model.Chains |> Map.exists (fun _ loaded -> loaded = chain)

    let validConfiguration model (q: Map<string, float>) =
        let bounds = joints model

        q.Count = bounds.Count
        && bounds
           |> Map.forall (fun name b ->
               q.TryFind name
               |> Option.exists (fun x -> Double.IsFinite x && x >= b.Lower && x <= b.Upper))

    let shapesAt model q =
        if not (validConfiguration model q) then
            invalidArg "q" "Full legal joint configuration required"

        let poses =
            model.Joints
            |> List.fold
                (fun poses joint ->
                    let mounted = Geometry.compose (Map.find joint.Parent poses) joint.Origin

                    let pose =
                        match joint.Bounds with
                        | None -> mounted
                        | Some _ ->
                            Geometry.compose
                                mounted
                                { Geometry.identity with
                                    Orientation = rotation joint.Axis (Map.find joint.Name q) }

                    Map.add joint.Child pose poses)
                (Map.ofList [ model.Root, Geometry.identity ])

        model.Shapes
        |> List.map (fun shape ->
            { shape with
                Shape = transformShape (Map.find shape.Link poses) shape.Shape })
