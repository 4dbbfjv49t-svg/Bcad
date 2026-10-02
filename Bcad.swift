import SwiftUI
import AppKit
import simd
import UniformTypeIdentifiers

@main
enum Entry {
    static func main() {
        let args = CommandLine.arguments
        if let i = args.firstIndex(of: "--render-icon"), i + 1 < args.count {
            MainActor.assumeIsolated { Art.render(IconArt(), size: CGSize(width: 1024, height: 1024), scale: 1, to: args[i + 1]) }
            return
        }
        #if SELFTEST
        if let i = args.firstIndex(of: "--selftest"), i + 1 < args.count {
            let ok = MainActor.assumeIsolated { SelfTest.run(URL(fileURLWithPath: args[i + 1])) }
            exit(ok ? 0 : 1)
        }
        #endif
        BcadApp.main()
    }
}

// MARK: - Model

enum PrimKind: Int, Codable, CaseIterable, Sendable {
    case box, cylinder, cone, sphere, prism, torus, wedge, pyramid, hemisphere, bowl, ring, glass, oval, ovalTorus
}

struct Primitive: Codable, Equatable, Sendable {
    var kind: PrimKind
    // Corners of a prism or pyramid; a torus's tube: 0 round, 3 a triangle with its point up, 6 a hexagon lying flat.
    var sides = 0
    var size: [Double]

    static func make(_ kind: PrimKind, sides: Int = 0) -> Primitive {
        switch kind {
        case .box, .wedge: Primitive(kind: kind, size: [20, 20, 20])
        case .cylinder: Primitive(kind: kind, size: [20, 20])
        case .cone: Primitive(kind: kind, size: [20, 0, 20])
        case .sphere, .hemisphere: Primitive(kind: kind, size: [20])
        case .prism, .pyramid: Primitive(kind: kind, sides: sides, size: [20, 20])
        case .torus: Primitive(kind: kind, sides: sides, size: [30, 8])
        case .bowl: Primitive(kind: kind, size: [40, 2])
        case .ring: Primitive(kind: kind, size: [30, 20, 5])
        case .glass: Primitive(kind: kind, size: [30, 40, 2, 3])
        case .oval: Primitive(kind: kind, size: [20, 14, 90, 20])
        case .ovalTorus: Primitive(kind: kind, sides: sides, size: [30, 20, 90, 6])
        }
    }

    var sided: Bool { [.prism, .pyramid, .torus, .ovalTorus].contains(kind) }
    var params: [Double] { sided ? [Double(sides)] + size : size }

    var fields: [String] {
        let tube = sides == 0 ? "Tube diameter" : "Tube width"
        switch kind {
        case .box, .wedge: return ["Width", "Depth", "Height"]
        case .cylinder, .prism, .pyramid: return ["Diameter", "Height"]
        case .cone: return ["Bottom diameter", "Top diameter", "Height"]
        case .sphere, .hemisphere: return ["Diameter"]
        case .torus: return ["Outer diameter", tube]
        case .bowl: return ["Diameter", "Wall thickness"]
        case .ring: return ["Outer diameter", "Inner diameter", "Height"]
        case .glass: return ["Diameter", "Height", "Wall thickness", "Bottom thickness"]
        case .oval: return ["Diameter A", "Diameter B", "Angle between diameters", "Height"]
        case .ovalTorus: return ["Diameter A", "Diameter B", "Angle between diameters", tube]
        }
    }

    // The axis (x 0, y 1, z 2) each size runs along, when it runs along just one.
    var axes: [Int?] {
        switch kind {
        case .box, .wedge: [0, 1, 2]
        case .cylinder, .prism, .pyramid: [nil, 2]
        case .cone, .ring: [nil, nil, 2]
        case .sphere, .hemisphere: [nil]
        case .torus, .bowl: [nil, nil]
        case .glass: [nil, 2, nil, nil]
        case .oval: [0, 1, nil, 2]
        case .ovalTorus: [0, 1, nil, nil]
        }
    }

    // Sizes given in degrees rather than millimetres (they never follow a resize).
    var degrees: Set<Int> { kind == .oval || kind == .ovalTorus ? [2] : [] }

    // How much of a polygon tube's width its height is.
    private var tall: Double { sides == 0 ? 1 : sqrt(3) / 2 }

    // The shape's bounding size; the kernel centres every primitive on its own origin.
    var extent: SIMD3<Double> {
        switch kind {
        case .box, .wedge: return SIMD3(size[0], size[1], size[2])
        case .cylinder, .glass: return SIMD3(size[0], size[0], size[1])
        case .cone: return SIMD3(max(size[0], size[1]), max(size[0], size[1]), size[2])
        case .sphere: return SIMD3(repeating: size[0])
        case .hemisphere, .bowl: return SIMD3(size[0], size[0], size[0] / 2)
        case .prism, .pyramid:
            // Corners on a circle, one side facing -y (as the kernel lays them out).
            let n = max(3, sides), r = size[0] / 2
            let angles = (0..<n).map { -Double.pi / 2 + .pi / Double(n) + 2 * .pi * Double($0) / Double(n) }
            let xs = angles.map { cos($0) }, ys = angles.map { sin($0) }
            return SIMD3((xs.max()! - xs.min()!) * r, (ys.max()! - ys.min()!) * r, size[1])
        case .torus: return SIMD3(size[0], size[0], size[1] * tall)
        case .ring: return SIMD3(size[0], size[0], size[2])
        case .oval:
            let half = Primitive.span(max(size[0], 0.01) / 2, max(size[1], 0.01) / 2, size[2])
            return SIMD3(half.x * 2, half.y * 2, size[3])
        case .ovalTorus:
            // The outline is the oval through the tube's middle, pushed out by half the tube.
            let w = size[3] / 2, half = Primitive.span(size[0] / 2 - w, size[1] / 2 - w, size[2])
            return SIMD3((half.x + w) * 2, (half.y + w) * 2, size[3] * tall)
        }
    }

    // How far the oval with semi-diameters a (along x) and b (at `degrees` from it) reaches along x and y.
    static func span(_ a: Double, _ b: Double, _ degrees: Double) -> SIMD2<Double> {
        let t = min(175, max(5, degrees)) * .pi / 180
        return SIMD2(sqrt(a * a + b * b * cos(t) * cos(t)), b * sin(t))
    }

    // An oval torus's tube must stay thinner than its tightest bend (radius minor²/major) by `margin`, or the kernel refuses it.
    func bends(_ margin: Double = 1.05) -> Bool {
        guard kind == .ovalTorus else { return true }
        let w = size[3] / 2, a = size[0] / 2 - w, b = size[1] / 2 - w, t = min(175, max(5, size[2])) * .pi / 180
        guard a > 0, b > 0 else { return false }
        let sxx = a * a + b * b * cos(t) * cos(t), sxy = b * b * cos(t) * sin(t), syy = b * b * sin(t) * sin(t)
        let mean = (sxx + syy) / 2, spread = hypot((sxx - syy) / 2, sxy)
        let major = sqrt(mean + spread), minor = sqrt(max(mean - spread, 0))
        return w * margin < minor * minor / major
    }

    // The sizes around the current one that still bend (they form one stretch: the bend is tightest at the extremes). A
    // tube that doesn't fit counts from the thinnest one, which always fits better.
    private func bending(_ i: Int, _ low: Double, _ high: Double) -> ClosedRange<Double> {
        func ok(_ v: Double) -> Bool { var p = self; p.size[i] = v; return p.bends(1.06) }
        let v = min(high, max(low, size[i]))
        guard let good = ok(v) ? v : (i == 3 && ok(low) ? low : nil) else { return low...max(low, high) }
        func edge(_ bad: Double) -> Double {
            guard !ok(bad) else { return bad }
            var (g, b) = (good, bad)
            for _ in 0..<48 { let m = (g + b) / 2; if ok(m) { g = m } else { b = m } }
            return g
        }
        let lo = (edge(low) * 100).rounded(.up) / 100, hi = (edge(high) * 100).rounded(.down) / 100
        return min(lo, good)...max(hi, good)
    }

    // What size i may be, given the others: walls fit inside, a tube inside its torus, one cone end stays wider than zero,
    // nothing larger than `limit` (the print bed).
    func range(_ i: Int, limit: Double) -> ClosedRange<Double> {
        if degrees.contains(i) { return 5...175 }
        var low = mayBeZero.contains(i) ? 0 : 0.1, high = limit
        // A polygon tube keeps a hole in the middle; a round one may close it.
        let hole = sides == 0 ? 1.06 : 2.1
        switch (kind, i) {
        case (.cone, 0), (.cone, 1): if size[1 - i] == 0 { low = 0.1 }
        case (.torus, 0): low = max(low, size[1] * hole)
        case (.torus, 1): high = min(high, size[0] / hole)
        case (.ovalTorus, 0), (.ovalTorus, 1): return bending(i, max(low, size[3] * 2.1), high)
        case (.ovalTorus, 3): return bending(i, low, min(high, min(size[0], size[1]) / 2.1))
        case (.ring, 0): low = max(low, size[1] + 0.1)
        case (.ring, 1): high = min(high, size[0] - 0.1)
        case (.bowl, 0): low = max(low, size[1] * 2 / 0.95)
        case (.bowl, 1): high = min(high, size[0] / 2 * 0.95)
        case (.glass, 0): low = max(low, size[2] * 2 / 0.95)
        case (.glass, 1): low = max(low, size[3] / 0.95)
        case (.glass, 2): high = min(high, size[0] / 2 * 0.95)
        case (.glass, 3): high = min(high, size[1] * 0.95)
        default: break
        }
        return low...max(low, high)
    }

    // Sizes that may be zero (a pointed cone, a ring without a hole).
    var mayBeZero: Set<Int> {
        switch kind {
        case .cone: [0, 1]
        case .ring: [1]
        default: []
        }
    }

    var name: String {
        let n = ["3": "Triangle", "4": "Square", "5": "Pentagon", "6": "Hexagon", "8": "Octagon"][String(sides)] ?? "\(sides)-sided"
        switch kind {
        case .box: return "Cube"
        case .cylinder: return "Cylinder"
        case .cone: return "Cone"
        case .sphere: return "Sphere"
        case .torus: return ["3": "Triangular torus", "6": "Hexagonal torus"][String(sides)] ?? "Torus"
        case .ovalTorus: return ["3": "Oval triangular torus", "6": "Oval hexagonal torus"][String(sides)] ?? "Oval torus"
        case .wedge: return "Wedge"
        case .prism: return n + " prism"
        case .pyramid: return n + " pyramid"
        case .hemisphere: return "Half-sphere"
        case .bowl: return "Bowl"
        case .ring: return "Ring"
        case .glass: return "Glass"
        case .oval: return "Oval cylinder"
        }
    }

    // Applies a scale to the sizes, keeping round shapes round (the axis changed most wins) and walls as they are; a torus's
    // tube is its height, and stays thin enough for its ring.
    mutating func scale(by s: SIMD3<Double>) {
        func most(_ v: [Double]) -> Double { v.max { abs($0 - 1) < abs($1 - 1) } ?? 1 }
        let k = most([s.x, s.y])
        var next = size
        switch kind {
        case .box, .wedge: next = [size[0] * s.x, size[1] * s.y, size[2] * s.z]
        case .cylinder, .prism, .pyramid: next = [size[0] * k, size[1] * s.z]
        case .cone: next = [size[0] * k, size[1] * k, size[2] * s.z]
        case .sphere, .hemisphere: next = [size[0] * most([s.x, s.y, s.z])]
        case .torus: next = [size[0] * k, min(size[1] * s.z, size[0] * k / (sides == 0 ? 1.06 : 2.1))]
        case .bowl: next = [size[0] * most([s.x, s.y, s.z]), size[1]]
        case .ring: next = [size[0] * k, size[1] * k, size[2] * s.z]
        case .glass: next = [size[0] * k, size[1] * s.z, size[2], size[3]]
        case .oval: next = [size[0] * s.x, size[1] * s.y, size[2], size[3] * s.z]
        case .ovalTorus: next = [size[0] * s.x, size[1] * s.y, size[2], size[3] * s.z]
        }
        size = next.enumerated().map { i, v in degrees.contains(i) ? v : max(mayBeZero.contains(i) ? 0 : 0.1, (v * 100).rounded() / 100) }
        if !bends(1.06) { size[3] = range(3, limit: .infinity).upperBound }
    }
}

struct Fastener: Codable, Equatable, Sendable {
    var nut: Bool
    var size: Int
    var length: Double
    var threadOnly: Bool

    @MainActor var name: String {
        let m = String(cString: bk_thread_name(Int32(size)))
        if nut { return threadOnly ? L("{m} threaded sleeve", ["m": m]) : L("{m} nut", ["m": m]) }
        return threadOnly ? L("{m} threaded rod", ["m": m]) : L("{m} bolt", ["m": m])
    }

    // Its bounding size; the kernel centres it on its own origin like a primitive.
    func extent(clearance: Double) -> SIMD3<Double> {
        var out = [Double](repeating: 0, count: 3)
        bk_fastener_extent(Int32(size), length, nut ? 1 : 0, threadOnly ? 1 : 0, clearance, &out)
        return SIMD3(out[0], out[1], out[2])
    }
}

struct Placement: Codable, Equatable, Sendable {
    var move = SIMD3<Double>(0, 0, 0)
    var turn = SIMD3<Double>(0, 0, 0)
    var scale = SIMD3<Double>(1, 1, 1)

    var rotation: simd_double3x3 {
        let r = turn * .pi / 180
        let (cx, sx, cy, sy, cz, sz) = (cos(r.x), sin(r.x), cos(r.y), sin(r.y), cos(r.z), sin(r.z))
        let rx = simd_double3x3(rows: [SIMD3(1, 0, 0), SIMD3(0, cx, -sx), SIMD3(0, sx, cx)])
        let ry = simd_double3x3(rows: [SIMD3(cy, 0, sy), SIMD3(0, 1, 0), SIMD3(-sy, 0, cy)])
        let rz = simd_double3x3(rows: [SIMD3(cz, -sz, 0), SIMD3(sz, cz, 0), SIMD3(0, 0, 1)])
        return rz * ry * rx
    }

    var matrix: simd_double4x4 {
        let l = rotation * simd_double3x3(diagonal: scale)
        return simd_double4x4(columns: (SIMD4(l.columns.0, 0), SIMD4(l.columns.1, 0), SIMD4(l.columns.2, 0), SIMD4(move, 1)))
    }

    // Row-major 3x4 for the kernel.
    var kernel: [Double] {
        let m = matrix
        return (0..<3).flatMap { r in (0..<4).map { c in m[c][r] } }
    }

    static func from(_ m: simd_double4x4) -> Placement {
        var p = Placement()
        p.move = SIMD3(m.columns.3.x, m.columns.3.y, m.columns.3.z)
        var c0 = SIMD3(m.columns.0.x, m.columns.0.y, m.columns.0.z)
        var c1 = SIMD3(m.columns.1.x, m.columns.1.y, m.columns.1.z)
        var c2 = SIMD3(m.columns.2.x, m.columns.2.y, m.columns.2.z)
        p.scale = SIMD3(length(c0), length(c1), length(c2))
        c0 /= max(1e-12, p.scale.x); c1 /= max(1e-12, p.scale.y); c2 /= max(1e-12, p.scale.z)
        p.turn = euler(simd_double3x3(columns: (c0, c1, c2)))
        return p
    }

    static func euler(_ r: simd_double3x3) -> SIMD3<Double> {
        // r = Rz·Ry·Rx; r[col][row]
        let r20 = r[0][2]
        let y = asin(max(-1, min(1, -r20)))
        let x: Double, z: Double
        if abs(r20) < 0.99999 {
            x = atan2(r[1][2], r[2][2])
            z = atan2(r[0][1], r[0][0])
        } else {
            x = atan2(-r[2][1], r[1][1])
            z = 0
        }
        return SIMD3(x, y, z) * 180 / .pi
    }
}

extension SIMD4 {
    var xyz: SIMD3<Scalar> { SIMD3(x, y, z) }
}

struct Plane: Codable, Equatable, Sendable {
    var point: SIMD3<Double>
    var normal: SIMD3<Double>
}

struct Pick: Codable, Equatable, Sendable {
    var kind: Int32
    var a: SIMD3<Double>
    var b: SIMD3<Double>

    // The same pick on its shape stretched by k along the shape's own axes: points stretch with it, normals the other way.
    func stretched(_ k: SIMD3<Double>) -> Pick {
        switch kind {
        case Int32(BK_PICK_EDGE): Pick(kind: kind, a: a * k, b: unit(b * k))
        case Int32(BK_PICK_CORNER), Int32(BK_PICK_FACE): Pick(kind: kind, a: unit(a / k), b: b * k)
        default: self
        }
    }
}

func unit(_ v: SIMD3<Double>) -> SIMD3<Double> {
    let l = simd_length(v)
    return l > 1e-12 ? v / l : v
}

struct Part: Codable, Equatable, Sendable {
    var node: Node
    var place: Placement
    var name: String?
    var color: SIMD3<UInt8>?
}

// A face of a hollowed shape with its own wall thickness.
struct Wall: Codable, Equatable, Sendable {
    var face: Pick
    var thickness: Double
}

indirect enum Node: Codable, Equatable, Sendable {
    case primitive(Primitive)
    case fastener(Fastener)
    case group(op: Int32, parts: [Part])
    case split(of: Node, plane: Plane, side: Int32)
    case round(of: Node, picks: [Pick], radius: Double)
    case hollow(of: Node, open: [Pick], walls: [Wall], thickness: Double)
    // A bevel: legs along each edge's face A and face B; corner > 0 rounds where the bevel meets the faces.
    case bevel(of: Node, picks: [Pick], legs: SIMD2<Double>, corner: Double)
    // Inward rounding: a concave quarter-round cut along the edges.
    case cove(of: Node, picks: [Pick], radius: Double)

    var inner: Node? {
        switch self {
        case .split(let n, _, _), .round(let n, _, _), .hollow(let n, _, _, _), .bevel(let n, _, _, _), .cove(let n, _, _): n
        default: nil
        }
    }

    // The same wrapper (split, rounding, hollow) around a different inner node.
    func wrapping(_ n: Node) -> Node {
        switch self {
        case .split(_, let p, let s): .split(of: n, plane: p, side: s)
        case .round(_, let p, let r): .round(of: n, picks: p, radius: r)
        case .hollow(_, let o, let w, let t): .hollow(of: n, open: o, walls: w, thickness: t)
        case .bevel(_, let p, let l, let c): .bevel(of: n, picks: p, legs: l, corner: c)
        case .cove(_, let p, let r): .cove(of: n, picks: p, radius: r)
        default: n
        }
    }

    func replacingBase(_ f: (Node) -> Node) -> Node {
        if let n = inner { return wrapping(n.replacingBase(f)) }
        return f(self)
    }

    var base: Node { inner?.base ?? self }

    // The roundings, bevels, splits and hollows moved along with a base shape stretched by k (the base itself stays).
    func following(_ k: SIMD3<Double>) -> Node {
        switch self {
        case .split(let n, let p, let s): .split(of: n.following(k), plane: Plane(point: p.point * k, normal: unit(p.normal / k)), side: s)
        case .round(let n, let p, let r): .round(of: n.following(k), picks: p.map { $0.stretched(k) }, radius: r)
        case .hollow(let n, let o, let w, let t):
            .hollow(of: n.following(k), open: o.map { $0.stretched(k) }, walls: w.map { Wall(face: $0.face.stretched(k), thickness: $0.thickness) }, thickness: t)
        case .bevel(let n, let p, let l, let c): .bevel(of: n.following(k), picks: p.map { $0.stretched(k) }, legs: l, corner: c)
        case .cove(let n, let p, let r): .cove(of: n.following(k), picks: p.map { $0.stretched(k) }, radius: r)
        default: self
        }
    }

    // The bounding size of a primitive or a fastener, which sit centred on their own origin; none for a merged shape.
    func extent(clearance: Double) -> SIMD3<Double>? {
        switch self {
        case .primitive(let p): p.extent
        case .fastener(let f): f.extent(clearance: clearance)
        default: nil
        }
    }
}

struct Solid: Codable, Equatable, Identifiable, Sendable {
    var id = UUID()
    var name: String
    var color: SIMD3<UInt8>
    var hidden = false
    var node: Node
    var place = Placement()
}

struct Document: Codable, Equatable, Sendable {
    var bodies: [Solid] = []
}

// Colours are red, green and blue from 0 to 255.
enum Palette {
    static let colors: [SIMD3<UInt8>] = [
        SIMD3(51, 199, 242), SIMD3(245, 89, 140), SIMD3(255, 184, 51), SIMD3(115, 230, 140),
        SIMD3(179, 140, 255), SIMD3(235, 235, 242), SIMD3(255, 128, 77), SIMD3(140, 153, 179)
    ]
    static func color(_ c: SIMD3<UInt8>) -> Color { Color(red: Double(c.x) / 255, green: Double(c.y) / 255, blue: Double(c.z) / 255) }
    // The palette colour after c (the first one after a mixed colour).
    static func after(_ c: SIMD3<UInt8>) -> SIMD3<UInt8> { colors[((colors.firstIndex(of: c) ?? -1) + 1) % colors.count] }

    // A saved colour: red, green and blue, or (in files from earlier versions) its number in the palette.
    static func decode<K: CodingKey>(_ c: KeyedDecodingContainer<K>, _ key: K) throws -> SIMD3<UInt8>? {
        if let rgb = try? c.decodeIfPresent(SIMD3<UInt8>.self, forKey: key) { return rgb }
        return try c.decodeIfPresent(Int.self, forKey: key).map { colors[($0 % colors.count + colors.count) % colors.count] }
    }
}

extension Solid {
    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        id = try c.decode(UUID.self, forKey: .id)
        name = try c.decode(String.self, forKey: .name)
        color = try Palette.decode(c, .color) ?? Palette.colors[0]
        hidden = try c.decode(Bool.self, forKey: .hidden)
        node = try c.decode(Node.self, forKey: .node)
        place = try c.decode(Placement.self, forKey: .place)
    }
}

extension Part {
    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        node = try c.decode(Node.self, forKey: .node)
        place = try c.decode(Placement.self, forKey: .place)
        name = try c.decodeIfPresent(String.self, forKey: .name)
        color = try Palette.decode(c, .color)
    }
}

// MARK: - Kernel bridge

final class ShapeRef: @unchecked Sendable {
    private let ptr: OpaquePointer
    init(_ p: OpaquePointer) { ptr = p }
    deinit { bk_free(ptr) }
    // The pointer is valid only inside the closure: ARC may free a shape right after its last use, not at the end of scope.
    func with<T>(_ body: (OpaquePointer) -> T) -> T { withExtendedLifetime(self) { body(ptr) } }
}

// A circle on a shape: a hole's or a peg's rim, the round edge of a cylinder or a ring.
struct Ring {
    var center: SIMD3<Double>
    var axis: SIMD3<Double>
    var radius: Double
}

struct Mesh {
    var vertices: [SIMD4<Float>] = []   // xyz + face id
    var normals: [SIMD4<Float>] = []
    var indices: [UInt32] = []
    var faceInfo: [(normal: SIMD3<Double>, centroid: SIMD3<Double>)] = []
    var edges: [[SIMD3<Float>]] = []
    // The faces either side of each edge (-1 when there is none).
    var edgeFaces: [SIMD2<Int32>] = []
    var circles: [Ring] = []
    var corners: [SIMD3<Float>] = []
    var low = SIMD3<Double>(0, 0, 0)
    var high = SIMD3<Double>(0, 0, 0)
    var volume = 0.0
    var valid = true
    var stamp = Int.random(in: 1...Int.max)

    init() {}

    init(_ m: UnsafeMutablePointer<BKMesh>) {
        let b = m.pointee
        let vc = Int(b.vertexCount), tc = Int(b.triangleCount)
        var faceOf = [UInt32](repeating: 0, count: vc)
        for t in 0..<tc {
            let f = b.triangleFace[t]
            for k in 0..<3 { faceOf[Int(b.indices[t * 3 + k])] = f }
        }
        vertices.reserveCapacity(vc)
        normals.reserveCapacity(vc)
        for i in 0..<vc {
            vertices.append(SIMD4(b.positions[i * 3], b.positions[i * 3 + 1], b.positions[i * 3 + 2], Float(faceOf[i])))
            normals.append(SIMD4(b.normals[i * 3], b.normals[i * 3 + 1], b.normals[i * 3 + 2], 0))
        }
        indices = Array(UnsafeBufferPointer(start: b.indices, count: tc * 3))
        for f in 0..<Int(b.faceCount) {
            let p = b.faceInfo + f * 6
            faceInfo.append((SIMD3(p[0], p[1], p[2]), SIMD3(p[3], p[4], p[5])))
        }
        for e in 0..<Int(b.edgeCount) {
            let s = Int(b.edgeStart[e]), t = Int(b.edgeStart[e + 1])
            edges.append((s..<t).map { SIMD3(b.edgePoints[$0 * 3], b.edgePoints[$0 * 3 + 1], b.edgePoints[$0 * 3 + 2]) })
            edgeFaces.append(SIMD2(b.edgeFaces[e * 2], b.edgeFaces[e * 2 + 1]))
        }
        circles = (0..<Int(b.circleCount)).map { i in
            let c = b.circles + i * 7
            return Ring(center: SIMD3(c[0], c[1], c[2]), axis: SIMD3(c[3], c[4], c[5]), radius: c[6])
        }
        corners = (0..<Int(b.cornerCount)).map { SIMD3(b.corners[$0 * 3], b.corners[$0 * 3 + 1], b.corners[$0 * 3 + 2]) }
        low = SIMD3(b.bbox.0, b.bbox.1, b.bbox.2)
        high = SIMD3(b.bbox.3, b.bbox.4, b.bbox.5)
        volume = b.volume
        valid = b.valid != 0
    }

    // A saved shape back in its body's own coordinates: flat-shaded triangles, shown until the kernel's exact mesh (with
    // its edges and faces) takes their place.
    init?(saved s: SavedMesh, place: Placement) {
        let m = place.matrix
        guard abs(m.determinant) > 1e-12 else { return nil }
        let back = simd_float4x4(m.inverse)
        let local = s.points.map { (back * SIMD4($0, 1)).xyz }
        var lo = SIMD3<Float>(repeating: .infinity), hi = SIMD3<Float>(repeating: -.infinity)
        var signed = 0.0
        for t in s.triangles {
            let a = local[Int(t.x)], b = local[Int(t.y)], c = local[Int(t.z)]
            let n = cross(b - a, c - a)
            let normal = SIMD4(length(n) > 0 ? normalize(n) : SIMD3<Float>(0, 0, 1), 0)
            for p in [a, b, c] {
                indices.append(UInt32(vertices.count))
                vertices.append(SIMD4(p, 0))
                normals.append(normal)
                lo = simd_min(lo, p)
                hi = simd_max(hi, p)
            }
            signed += Double(dot(a, cross(b, c))) / 6
        }
        low = SIMD3<Double>(lo)
        high = SIMD3<Double>(hi)
        volume = abs(signed)
    }

    var size: SIMD3<Double> { high - low }
}

// One serial worker thread with a large stack: OpenCascade booleans and fillets recurse deeply (GCD threads get 512 KB).
final class Worker: @unchecked Sendable {
    private let lock = NSCondition()
    private var jobs: [() -> Void] = []

    init() {
        let t = Thread { [unowned self] in
            while true {
                lock.lock()
                while jobs.isEmpty { lock.wait() }
                let job = jobs.removeFirst()
                lock.unlock()
                job()
            }
        }
        t.stackSize = 64 << 20
        t.qualityOfService = .userInitiated
        t.start()
    }

    func async(_ job: @escaping () -> Void) {
        lock.lock()
        jobs.append(job)
        lock.signal()
        lock.unlock()
    }

    func sync<T>(_ job: () -> T) -> T {
        withoutActuallyEscaping(job) { job in
            var out: T?
            let done = DispatchSemaphore(value: 0)
            async {
                out = job()
                done.signal()
            }
            done.wait()
            return out!
        }
    }
}

final class Kernel: @unchecked Sendable {
    static let shared = Kernel()
    let queue = Worker()
    private var cache: [String: ShapeRef] = [:]
    private(set) var problems: [String] = []
    var clearance = 0.2

    private func key(_ node: Node) -> String {
        let e = JSONEncoder()
        e.outputFormatting = .sortedKeys
        let data = (try? e.encode(node)) ?? Data()
        return String(format: "%.3f|", clearance) + data.base64EncodedString()
    }

    func takeProblems() -> [String] { defer { problems = [] }; return problems }

    func shape(_ node: Node) -> ShapeRef? {
        let k = key(node)
        if let s = cache[k] { return s }
        guard let p = build(node) else { return nil }
        // Nothing solid left (all of it cut away, shapes that don't overlap, a split beside the shape) is a failure too.
        if bk_piece_count(p) == 0 {
            bk_free(p)
            problems.append("empty")
            return nil
        }
        if cache.count > 400 { cache.removeAll() }
        let ref = ShapeRef(p)
        cache[k] = ref
        return ref
    }

    // A kernel result; when there is none, the kernel's reason is recorded.
    private func made(_ p: OpaquePointer?) -> OpaquePointer? {
        if p == nil { problems.append(String(cString: bk_last_error())) }
        return p
    }

    private func build(_ node: Node) -> OpaquePointer? {
        switch node {
        case .primitive(let p):
            // An oval torus whose tube is too thick for its tightest bend is said so plainly, not as a failure.
            guard p.bends() else { problems.append("bend"); return nil }
            return made(p.params.withUnsafeBufferPointer { bk_primitive(Int32(p.kind.rawValue), $0.baseAddress) })
        case .fastener(let f):
            return made(f.nut ? bk_nut(Int32(f.size), f.length, f.threadOnly ? 1 : 0, clearance) : bk_bolt(Int32(f.size), f.length, f.threadOnly ? 1 : 0, clearance))
        case .group(let op, let parts):
            // Every part has to build: leaving one out would silently change what the others are merged with or cut from.
            var result: OpaquePointer?
            for part in parts {
                let m = part.place.kernel
                guard let s = shape(part.node), let placed = made(s.with({ sp in m.withUnsafeBufferPointer { bk_transform(sp, $0.baseAddress) } })) else {
                    if let result { bk_free(result) }
                    return nil
                }
                if let r = result {
                    let next = made(bk_boolean(op, r, placed))
                    bk_free(r)
                    bk_free(placed)
                    guard let next else { return nil }
                    result = next
                } else {
                    result = placed
                }
            }
            if op == BK_UNION, parts.count > 1, let r = result, bk_piece_count(r) > 1 { problems.append("pieces") }
            return result
        case .split(let of, let plane, let side):
            guard let s = shape(of) else { return nil }
            let p = [plane.point.x, plane.point.y, plane.point.z], n = [plane.normal.x, plane.normal.y, plane.normal.z]
            return made(s.with { bk_split($0, p, n, side) })
        case .round(let of, let picks, let radius):
            guard let s = shape(of) else { return nil }
            let (kinds, data) = Self.flat(picks)
            var maxR = 0.0
            var missing: Int32 = 0
            let out = s.with { bk_fillet($0, kinds, data, Int32(picks.count), radius, &maxR, &missing) }
            if missing > 0 { problems.append("missing") }
            if let out = out ?? beneath(of, node) { return out }
            problems.append(maxR > 0 ? "max:\(maxR)" : "round")
            return s.with { bk_copy($0) }
        case .bevel(let of, let picks, let legs, let corner):
            guard let s = shape(of) else { return nil }
            let (kinds, data) = Self.flat(picks)
            var missing: Int32 = 0
            let out = s.with { bk_chamfer($0, kinds, data, Int32(picks.count), legs.x, legs.y, corner, &missing) }
            if missing > 0 { problems.append("missing") }
            if let out = out ?? beneath(of, node) { return out }
            problems.append("bevel")
            return s.with { bk_copy($0) }
        case .cove(let of, let picks, let radius):
            guard let s = shape(of) else { return nil }
            let (kinds, data) = Self.flat(picks)
            var maxR = 0.0
            var missing: Int32 = 0
            let out = s.with { bk_cove($0, kinds, data, Int32(picks.count), radius, &maxR, &missing) }
            if missing > 0 { problems.append("missing") }
            if let out = out ?? beneath(of, node) { return out }
            problems.append(maxR > 0 ? "max:\(maxR)" : "cove")
            return s.with { bk_copy($0) }
        case .hollow(let of, let open, let walls, let thickness):
            guard let s = shape(of) else { return nil }
            let faces = open.flatMap { [$0.a.x, $0.a.y, $0.a.z, $0.b.x, $0.b.y, $0.b.z] }
            let own = walls.flatMap { [$0.face.a.x, $0.face.a.y, $0.face.a.z, $0.face.b.x, $0.face.b.y, $0.face.b.z] }
            let values = walls.map(\.thickness)
            var missing: Int32 = 0
            let out = s.with { bk_hollow($0, faces, Int32(open.count), own, values, Int32(walls.count), thickness, &missing) }
            if missing > 0 { problems.append("missing") }
            if let out { return out }
            problems.append("hollow")
            return s.with { bk_copy($0) }
        }
    }

    // A rounding or bevel that doesn't fit on top of an earlier rounding or bevel often does beneath it: these edges
    // treated first, the earlier treatment after. Tried one level deep; nil when that doesn't work either.
    private var swapping = false
    private func beneath(_ of: Node, _ node: Node) -> OpaquePointer? {
        guard !swapping, let inner = of.inner else { return nil }
        switch of {
        case .round, .bevel, .cove: break
        default: return nil
        }
        swapping = true
        defer { swapping = false }
        let mark = problems.count
        if let s = shape(of.wrapping(node.wrapping(inner))), !problems[mark...].contains(where: Self.failure) { return s.with { bk_copy($0) } }
        problems.removeSubrange(mark...)
        return nil
    }

    // Problems that mean the shape didn't come out as asked (skipped picks and a merge in pieces still did).
    static func failure(_ p: String) -> Bool { p != "missing" && p != "pieces" }

    // A new treatment of `node`'s inner shape, built apart from anything left over from earlier work: its mesh, and what
    // went wrong with it.
    func attempt(_ node: Node) -> (mesh: Mesh?, problems: [String]) {
        if let inner = node.inner { _ = shape(inner) }
        if case .group(_, let parts) = node { for p in parts { _ = shape(p.node) } }
        // Built afresh: a failed try keeps the unchanged shape in the cache, which mustn't pass for a success next time.
        cache[key(node)] = nil
        problems = []
        let m = mesh(node)
        return (m, takeProblems())
    }

    private static func flat(_ picks: [Pick]) -> ([Int32], [Double]) {
        (picks.map(\.kind), picks.flatMap { [$0.a.x, $0.a.y, $0.a.z, $0.b.x, $0.b.y, $0.b.z] })
    }

    // The shape cut across a picked edge, for the 2D angle editor.
    func section(_ node: Node, _ pick: Pick) -> Section? {
        probe(String(format: "section cached %d", cache[key(node)] != nil ? 1 : 0)) // probe
        guard let s = shape(node) else { return nil }
        let data = [pick.a.x, pick.a.y, pick.a.z, pick.b.x, pick.b.y, pick.b.z]
        guard let c = s.with({ bk_section($0, pick.kind, data, 20) }) else { return nil }
        defer { bk_section_free(c) }
        let sec = c.pointee
        var loops: [[SIMD2<Double>]] = []
        for l in 0..<Int(sec.loopCount) {
            let from = Int(sec.loopStart[l]), to = Int(sec.loopStart[l + 1])
            loops.append((from..<to).map { SIMD2(sec.points[2 * $0], sec.points[2 * $0 + 1]) })
        }
        let point = SIMD3(sec.point.0, sec.point.1, sec.point.2), direction = SIMD3(sec.direction.0, sec.direction.1, sec.direction.2)
        return Section(loops: loops, angle: sec.angle, point: point, direction: direction)
    }

    func mesh(_ node: Node, deflection: Double = 0.05) -> Mesh? {
        guard let s = shape(node), let m = s.with({ bk_mesh($0, deflection) }) else { return nil }
        defer { bk_mesh_free(m) }
        let mesh = Mesh(m)
        // A shape reaching past 100 m (or nowhere) is broken geometry; shown, it would throw the view out.
        guard mesh.vertices.isEmpty || [mesh.low, mesh.high].allSatisfy({ $0.finite && simd_reduce_max(simd_abs($0)) < 1e5 }) else {
            problems.append("bounds")
            return nil
        }
        return mesh
    }

    // World-space shape of a body (for export).
    func placed(_ b: Solid) -> ShapeRef? {
        guard let s = shape(b.node) else { return nil }
        return s.with { sp in b.place.kernel.withUnsafeBufferPointer { bk_transform(sp, $0.baseAddress) } }.map(ShapeRef.init)
    }

    // A body's fine mesh in world coordinates, for files.
    func worldMesh(_ b: Solid, deflection: Double = 0.01) -> Mesh? {
        guard let s = placed(b), let m = s.with({ bk_mesh($0, deflection) }) else { return nil }
        defer { bk_mesh_free(m) }
        return Mesh(m)
    }

    func exportStep(_ bodies: [Solid], to path: String) -> Bool {
        let shapes = bodies.compactMap { placed($0) }
        guard shapes.count == bodies.count else { return false }
        return withExtendedLifetime(shapes) {
            let ptrs: [OpaquePointer?] = shapes.map { $0.with { $0 } }
            return ptrs.withUnsafeBufferPointer { bk_export_step($0.baseAddress, Int32(ptrs.count), path) } != 0
        }
    }
}

// MARK: - Settings

enum Action: String, CaseIterable, Codable {
    case move, rotate, scale, round, split, hollow, drop, frame, hide, showAll

    var name: String {
        switch self {
        case .move: "Move"
        case .rotate: "Rotate"
        case .scale: "Scale"
        case .round: "Round edges"
        case .split: "Split"
        case .hollow: "Hollow"
        case .drop: "Drop onto the bed"
        case .frame: "Zoom to fit"
        case .hide: "Hide selection"
        case .showAll: "Show all"
        }
    }
    @MainActor var label: String { L(name) }
}

struct Settings: Codable, Equatable {
    static let defaultKeys: [String: String] = [
        "move": "KeyG", "rotate": "KeyT", "scale": "KeyY", "round": "KeyR", "split": "KeyS", "hollow": "KeyO", "drop": "KeyB", "frame": "KeyF", "hide": "KeyH", "showAll": "KeyU"
    ]
    var keys = Settings.defaultKeys
    var snap = 1.0
    var turnStep = 15.0
    var dropToBed = true
    var bed = SIMD3<Double>(256, 256, 256)
    var clearance = 0.2
    var autoLink = true
    var uniform = false
    // Resizing moves both sides of a size (the middle stays) instead of one (the left, front or bottom stays).
    var symmetric = false
    // Shortcuts switched off: they don't fire and their keys are free for others.
    var off: Set<String> = []

    init() {}

    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        var k = Settings.defaultKeys
        if let saved = try? c.decode([String: String].self, forKey: .keys) { k.merge(saved) { _, s in s } }
        keys = k
        snap = min(100, max(0.01, (try? c.decode(Double.self, forKey: .snap)) ?? 1))
        turnStep = min(90, max(0.1, (try? c.decode(Double.self, forKey: .turnStep)) ?? 15))
        dropToBed = (try? c.decode(Bool.self, forKey: .dropToBed)) ?? true
        bed = (try? c.decode(SIMD3<Double>.self, forKey: .bed)) ?? SIMD3(256, 256, 256)
        clearance = min(2, max(0, (try? c.decode(Double.self, forKey: .clearance)) ?? 0.2))
        autoLink = (try? c.decode(Bool.self, forKey: .autoLink)) ?? true
        uniform = (try? c.decode(Bool.self, forKey: .uniform)) ?? false
        symmetric = (try? c.decode(Bool.self, forKey: .symmetric)) ?? false
        off = Set(((try? c.decode([String].self, forKey: .off)) ?? []).filter { Action(rawValue: $0) != nil })
    }

    // The print bed's longest side: no size or thread is made longer.
    var longest: Double { max(bed.x, bed.y, bed.z) }

    func key(_ a: Action) -> String { keys[a.rawValue] ?? Settings.defaultKeys[a.rawValue] ?? "" }
    func isOn(_ a: Action) -> Bool { !off.contains(a.rawValue) }
    // The switched-on action a key belongs to, other than `except`.
    func owner(of name: String, except: Action? = nil) -> Action? {
        Action.allCases.first { $0 != except && isOn($0) && key($0) == name }
    }
}

struct Store: Codable {
    var style: Style?
    var language: String?
    var brightness: Double?
    var settings: Settings?
    var look: SkinSettings?
    var shapes: [String: String]?
}

enum Paths {
    static let dir = FileManager.default.urls(for: .applicationSupportDirectory, in: .userDomainMask)[0].appendingPathComponent("Bcad", isDirectory: true)
    static let state = dir.appendingPathComponent("state.json")
}

enum Mode: Equatable { case select, round, split, hollow, angles, thread }

// The inspector's screens: its segments and the gizmo they bring.
enum Screen: Int, CaseIterable {
    case move, resize, rotate, angles, thread

    var title: String {
        switch self {
        case .move: "Move"
        case .resize: "Resize"
        case .rotate: "Rotate"
        case .angles: "Angles"
        case .thread: "Thread"
        }
    }

    // The thread's own picture is drawn (there's no symbol for a thread).
    var icon: String? {
        switch self {
        case .move: "arrow.up.and.down.and.arrow.left.and.right"
        case .resize: "arrow.up.left.and.arrow.down.right"
        case .rotate: "arrow.triangle.2.circlepath"
        case .angles: "angle"
        case .thread: nil
        }
    }
}

// The shape cut across an edge, seen end-on: the edge at the origin, face A leaving along −x, face B at 180° + angle,
// material between them (the top-right corner of a block for a right angle).
struct Section: Equatable {
    var loops: [[SIMD2<Double>]]
    var angle: Double
    var point: SIMD3<Double>
    var direction: SIMD3<Double>

    var phi: Double { angle * .pi / 180 }
    var dirA: SIMD2<Double> { SIMD2(-1, 0) }
    var dirB: SIMD2<Double> { SIMD2(-cos(phi), -sin(phi)) }

    // How far each face runs straight from the edge (until the outline turns more than 30°).
    var runs: SIMD2<Double> {
        guard let loop = loops.min(by: { closest($0).1 < closest($1).1 }), loop.count > 2 else { return SIMD2(10, 10) }
        let (i, _) = closest(loop)
        func run(_ step: Int) -> (SIMD2<Double>, Double) {
            var length = 0.0, turned = 0.0, k = i
            var heading: SIMD2<Double>?
            for _ in 0..<loop.count {
                let n = (k + step + loop.count) % loop.count
                let d = loop[n] - loop[k]
                let l = simd_length(d)
                if l > 1e-9 {
                    let h = d / l
                    if let heading { turned += acos(max(-1, min(1, simd_dot(heading, h)))) }
                    if turned > .pi / 6 { break }
                    heading = h
                    length += l
                }
                k = n
            }
            return (heading.map { _ in simd_normalize(loop[(i + step + loop.count) % loop.count] - loop[i]) } ?? .zero, length)
        }
        let (d1, l1) = run(1), (_, l2) = run(-1)
        return simd_dot(d1, dirA) > simd_dot(d1, dirB) ? SIMD2(l1, l2) : SIMD2(l2, l1)
    }

    private func closest(_ loop: [SIMD2<Double>]) -> (Int, Double) {
        var best = (0, Double.infinity)
        for (i, p) in loop.enumerated() where simd_length(p) < best.1 { best = (i, simd_length(p)) }
        return best
    }
}

// Working on the corner along picked edges in the 2D section view: rounding it (outward or inward) or bevelling it.
struct AngleEdit: Equatable {
    enum Treatment: Equatable { case rounded, angled }
    enum Rounding: Equatable { case outbound, inbound }

    var body: UUID
    var picks: [Pick]
    var section: Section
    var runs: SIMD2<Double>
    var treatment = Treatment.rounded
    var rounding = Rounding.outbound
    var radius: Double
    var legs: SIMD2<Double>
    var roundedCorners = false

    init(body: UUID, picks: [Pick], section: Section) {
        self.body = body
        self.picks = picks
        self.section = section
        runs = section.runs
        let room = max(0.2, min(runs.x, runs.y))
        radius = (min(2, room * 0.4) * 10).rounded() / 10
        legs = SIMD2(repeating: (min(2, room * 0.4) * 10).rounded() / 10)
    }

    var phi: Double { section.phi }
    // The cut's slanted side.
    var hypotenuse: Double { sqrt(legs.x * legs.x + legs.y * legs.y - 2 * legs.x * legs.y * cos(phi)) }
    // Angle between the bevel and face A, degrees.
    var angleA: Double { atan2(legs.y * sin(phi), legs.x - legs.y * cos(phi)) * 180 / .pi }
    // How deep the bevel reaches into the corner, measured square to it.
    var depth: Double { legs.x * legs.y * sin(phi) / max(1e-9, hypotenuse) }
    // Bevel angles the corner allows (the two angles at the bevel add up to 180° − corner angle).
    var angleRange: ClosedRange<Double> { 1...max(1, 179 - section.angle) }
    var radiusRange: ClosedRange<Double> {
        let room = min(runs.x, runs.y)
        let top = rounding == .outbound ? room * tan(phi / 2) : room
        return 0.1...max(0.1, (top * 100).rounded(.down) / 100)
    }
    var cornerRadius: Double { roundedCorners ? min(legs.x, legs.y) * 0.3 : 0 }

    mutating func setAngle(_ degrees: Double) { setShape(angle: degrees, depth: depth) }
    mutating func setDepth(_ d: Double) { setShape(angle: angleA, depth: d) }
    mutating func setHypotenuse(_ h: Double) { legs *= h / max(1e-9, hypotenuse); tidy() }
    mutating func setLeg(_ i: Int, _ v: Double) { legs[i] = v; tidy() }

    private mutating func setShape(angle: Double, depth d: Double) {
        let a = min(angleRange.upperBound, max(angleRange.lowerBound, angle)) * .pi / 180
        let b = .pi - phi - a
        legs = SIMD2(d / max(1e-9, sin(a)), d / max(1e-9, sin(b)))
        tidy()
    }

    // Legs keep full precision, so a chosen angle stays exact; the fields show two decimals.
    private mutating func tidy() {
        legs = SIMD2(max(0.01, legs.x), max(0.01, legs.y))
    }

    // The treatment around a node, for the picked edges or for the whole shape.
    func wrapping(_ node: Node, whole: Bool) -> Node {
        let p = whole ? [Pick(kind: Int32(BK_PICK_BODY), a: .zero, b: .zero)] : picks
        switch (treatment, rounding) {
        case (.rounded, .outbound): return .round(of: node, picks: p, radius: radius)
        case (.rounded, .inbound): return .cove(of: node, picks: p, radius: radius)
        case (.angled, _): return .bevel(of: node, picks: p, legs: legs, corner: cornerRadius)
        }
    }
}

// Shapes of the bottom bar, grouped by kind; each group's first member is the program's own default.
enum ShapeKind: String, CaseIterable {
    case cube, wedge, cylinder, hexagon, glass, cone, pyramid, sphere, halfSphere, bowl, torus, triangleTorus, hexagonTorus, ring

    var primitive: Primitive {
        switch self {
        case .cube: .make(.box)
        case .wedge: .make(.wedge)
        case .cylinder: .make(.cylinder)
        case .hexagon: .make(.prism, sides: 6)
        case .glass: .make(.glass)
        case .cone: .make(.cone)
        case .pyramid: .make(.pyramid, sides: 4)
        case .sphere: .make(.sphere)
        case .halfSphere: .make(.hemisphere)
        case .bowl: .make(.bowl)
        case .torus: .make(.torus)
        case .triangleTorus: .make(.torus, sides: 3)
        case .hexagonTorus: .make(.torus, sides: 6)
        case .ring: .make(.ring)
        }
    }
}

enum ShapeGroup: String, CaseIterable {
    case blocks, cylinders, cones, spheres, rings

    var members: [ShapeKind] {
        switch self {
        case .blocks: [.cube, .wedge]
        case .cylinders: [.cylinder, .hexagon, .glass]
        case .cones: [.cone, .pyramid]
        case .spheres: [.sphere, .halfSphere, .bowl]
        case .rings: [.torus, .triangleTorus, .hexagonTorus, .ring]
        }
    }
}
enum Gizmo: Equatable { case move, rotate, scale }

struct Hover: Equatable {
    var body: UUID?
    var face: Int = -1
    var edge: Int = -1
    var corner: Int = -1
    var point = SIMD3<Double>(0, 0, 0)
}

// MARK: - Workbench (app state)

@Observable @MainActor
final class Workbench: DesignHost {
    static let shared = Workbench()

    var style: Style = .classic
    var brightness = 0.8
    var palette: Style { style }
    var settings = Settings()
    var doc = Document()
    var fileURL: URL?
    // The document as last opened or saved; it has changes while it differs from that.
    private var saved = Document()
    var dirty: Bool { doc != saved }
    var selection: [UUID] = []
    var meshes: [UUID: Mesh] = [:]
    var sceneVersion = 0
    var building = false
    var mode: Mode = .select
    var gizmo: Gizmo = .move
    var hover = Hover()
    var editBody: UUID?
    var edgePicks: [Pick] = []
    var roundRadius = 2.0
    var hollowOpen: [Pick] = []
    var hollowWalls: [Wall] = []
    var hollowThickness = 2.0
    var focusWall: Int?
    var thread = Fastener(nut: false, size: 4, length: 30, threadOnly: false)
    var splitAxis = 2
    var splitOffset = 0.0
    var splitTilt = SIMD2<Double>(0, 0)
    var showSettings = false
    var drawerOpen = true
    // Work under way (with a spinner), and a short message that goes by itself.
    var busy: String?
    var note: String?
    var capturing: Action?
    var captureFail: Action?
    var angleEdit: AngleEdit?
    var angleOpening = false
    // Which way the inspector last switched screens (+1 to the right), so the screens slide the right way.
    var screenStep = 1
    // Each shape group's quick shape when it isn't the program's default (chosen by holding it in the group's row).
    var shapes: [String: String] = [:]

    @ObservationIgnored private var undoStack: [Document] = []
    @ObservationIgnored private var redoStack: [Document] = []
    @ObservationIgnored private var built: [UUID: Node] = [:]
    @ObservationIgnored private var builtClearance = -1.0
    @ObservationIgnored private var pendingBuild = false
    @ObservationIgnored private var previewBusy = false
    @ObservationIgnored private var previewAgain = false
    // New shapes being tried before they go into the document.
    @ObservationIgnored private(set) var trying = false
    @ObservationIgnored private var flashTask: Task<Void, Never>?
    @ObservationIgnored private var saveTask: Task<Void, Never>?
    @ObservationIgnored var camera = Camera()
    @ObservationIgnored var requestFit = false
    // Where the inspector and its row of names sit in the window (for two-finger swipes between its screens).
    @ObservationIgnored var inspectorFrame = CGRect.zero
    @ObservationIgnored var namesFrame = CGRect.zero
    @ObservationIgnored private var swipe = 0.0
    @ObservationIgnored private var swiped = false
    @ObservationIgnored private var cameraBeforeAngles: Camera?
    @ObservationIgnored private var flight: Task<Void, Never>?

    var accent: Color { Skin.shared.accent }
    var accent2: Color { Skin.shared.accent2 }
    var accent3: Color { Skin.shared.accent3 }

    init() {
        Skin.shared.host = self
        load()
        NSEvent.addLocalMonitorForEvents(matching: .keyDown) { [weak self] e in
            guard let self else { return e }
            return self.key(e) ? nil : e
        }
        NSEvent.addLocalMonitorForEvents(matching: .scrollWheel) { [weak self] e in
            guard let self else { return e }
            return self.swipeInspector(e) ? nil : e
        }
    }

    // MARK: persistence

    private func load() {
        let s = (try? Data(contentsOf: Paths.state)).flatMap { try? JSONDecoder().decode(Store.self, from: $0) }
        style = s?.style ?? .classic
        brightness = min(1, max(0, s?.brightness ?? 0.8))
        settings = s?.settings ?? Settings()
        L10n.shared.id = L10n.valid(s?.language)
        Skin.shared.apply(s?.look ?? SkinSettings())
        shapes = (s?.shapes ?? [:]).filter { g, k in ShapeGroup(rawValue: g).map { $0.members.dropFirst().contains { $0.rawValue == k } } ?? false }
    }

    func save() {
        let s = Store(style: style, language: L10n.shared.id, brightness: brightness, settings: settings, look: Skin.shared.values, shapes: shapes)
        try? FileManager.default.createDirectory(at: Paths.dir, withIntermediateDirectories: true)
        if let data = try? JSONEncoder().encode(s) { try? data.write(to: Paths.state, options: .atomic) }
    }

    private func scheduleSave() {
        saveTask?.cancel()
        saveTask = Task { [weak self] in
            try? await Task.sleep(for: .milliseconds(600))
            guard !Task.isCancelled else { return }
            self?.save()
        }
    }

    func setStyle(_ s: Style) {
        withAnimation(Neon.glide) { style = s }
        scheduleSave()
    }

    func setLanguage(_ id: String) {
        withAnimation(.smooth(duration: 0.55)) { L10n.shared.id = L10n.valid(id) }
        MenuText.apply()
        scheduleSave()
    }

    func setBrightness(_ v: Double) {
        let b = (min(1, max(0, v)) * 100).rounded() / 100
        guard b != brightness else { return }
        brightness = b
        scheduleSave()
    }

    func setLook(_ change: (inout SkinSettings) -> Void) {
        var v = Skin.shared.values
        change(&v)
        guard v != Skin.shared.values else { return }
        let appearance = v.dark != Skin.shared.values.dark || v.simplified != Skin.shared.values.simplified
        if appearance { withAnimation(Neon.glide) { Skin.shared.apply(v) } } else { Skin.shared.apply(v) }
        sceneVersion += 1
        scheduleSave()
    }

    func updateSettings(_ change: (inout Settings) -> Void) {
        var s = settings
        change(&s)
        guard s != settings else { return }
        let rebuild = s.clearance != settings.clearance
        settings = s
        scheduleSave()
        if rebuild { rebuildScene() }
    }

    func restoreDefaults() {
        updateSettings { $0 = Settings() }
    }

    // MARK: capture

    func beginCapture(_ a: Action) { withAnimation(Neon.spring) { capturing = a; captureFail = nil } }

    func endCapture() {
        guard capturing != nil else { return }
        withAnimation(Neon.spring) { capturing = nil }
    }

    // Switching a shortcut back on keeps its key only while nobody else took it; otherwise the field empties and listens.
    func toggleShortcut(_ a: Action) {
        if settings.isOn(a) {
            if capturing == a { endCapture() }
            updateSettings { $0.off.insert(a.rawValue) }
            return
        }
        let k = settings.key(a)
        let free = !k.isEmpty && !Keys.reserved.contains(k) && settings.owner(of: k, except: a) == nil
        updateSettings {
            $0.off.remove(a.rawValue)
            if !free { $0.keys[a.rawValue] = "" }
        }
        if !free { beginCapture(a) }
    }

    private func capture(_ name: String) {
        guard let a = capturing else { return }
        withAnimation(Neon.spring) { capturing = nil }
        guard !name.isEmpty else { return }
        if Keys.reserved.contains(name) || settings.owner(of: name, except: a) != nil {
            withAnimation(Neon.spring) { captureFail = a }
            Task {
                try? await Task.sleep(for: .seconds(1.2))
                if captureFail == a { withAnimation(Neon.spring) { captureFail = nil } }
            }
            return
        }
        updateSettings { $0.keys[a.rawValue] = name }
    }

    func flash(_ text: String) {
        probe("flash " + text) // probe
        flashTask?.cancel()
        withAnimation(Neon.spring) { note = text }
        flashTask = Task { [weak self] in
            try? await Task.sleep(for: .seconds(3.5))
            guard !Task.isCancelled, let self, self.note == text else { return }
            withAnimation(Neon.spring) { self.note = nil }
        }
    }

    // Takes down the work note `text`, unless another one has taken its place by now.
    func ended(_ text: String) {
        if busy == text { withAnimation(Neon.spring) { busy = nil } }
    }

    func toggleSettings() {
        endCapture()
        withAnimation(Neon.glide) {
            showSettings.toggle()
            drawerOpen = true
        }
    }

    // MARK: document edits

    func body(_ id: UUID?) -> Solid? { doc.bodies.first { $0.id == id } }
    var primary: Solid? { body(selection.last) }
    var selected: [Solid] { doc.bodies.filter { selection.contains($0.id) } }

    // Records an undo step before a change (drags call begin once, then edit freely).
    func begin() {
        undoStack.append(doc)
        if undoStack.count > 200 { undoStack.removeFirst() }
        redoStack.removeAll()
    }

    func commit(_ change: (inout Document) -> Void) {
        begin()
        change(&doc)
        rebuildScene()
    }

    // Builds the new shapes first and runs `apply` (which commits them) only when all of them came out, so a rounding,
    // hollow, bevel, merge or split that doesn't work never enters the document: it is said once, and nothing changes.
    func tryThen(_ nodes: [Node], apply: @escaping () -> Void) {
        guard !trying else { return }
        trying = true
        let clearance = settings.clearance, note = L("Building…")
        // Only a slow try shows that it's working.
        let shown = Task { [weak self] in
            try? await Task.sleep(for: .milliseconds(300))
            guard !Task.isCancelled, let self, self.trying else { return }
            withAnimation(Neon.spring) { self.busy = note }
        }
        probe("try start") // probe
        Kernel.shared.queue.async {
            probe("try queue start") // probe
            Kernel.shared.clearance = clearance
            var problems: [String] = [], ok = true
            for n in nodes {
                let (m, p) = Kernel.shared.attempt(n)
                problems += p
                if m?.vertices.isEmpty ?? true || p.contains(where: Kernel.failure) { ok = false }
            }
            if !ok && !problems.contains(where: Kernel.failure) { problems.append("failed") }
            DispatchQueue.main.async {
                MainActor.assumeIsolated {
                    probe("try main") // probe
                    shown.cancel()
                    self.trying = false
                    self.ended(note)
                    if ok { apply() }
                    self.report(problems)
                }
            }
        }
    }

    func undo() {
        guard let d = undoStack.popLast() else { return }
        redoStack.append(doc)
        restore(d)
    }

    func redo() {
        guard let d = redoStack.popLast() else { return }
        undoStack.append(doc)
        restore(d)
    }

    // Puts back an earlier or later document: an open tool closes (its picks belong to the shapes as they were), and the
    // selection keeps the shapes that are still there.
    private func restore(_ d: Document) {
        doc = d
        selection = selection.filter { id in d.bodies.contains { $0.id == id } }
        if angleEdit != nil { closeAngles() }
        if mode != .select { cancelMode() }
        rebuildScene()
    }

    func mutate(_ id: UUID, _ change: (inout Solid) -> Void) {
        guard let i = doc.bodies.firstIndex(where: { $0.id == id }) else { return }
        change(&doc.bodies[i])
    }

    // A colour shown at once without an undo step of its own (the colour mixer keeps one for all it changes).
    func paint(_ id: UUID, _ c: SIMD3<UInt8>) {
        guard body(id)?.color != c else { return }
        mutate(id) { $0.color = c }
    }

    private func nextColor() -> SIMD3<UInt8> { Palette.colors[doc.bodies.count % Palette.colors.count] }

    private func spawnPoint() -> SIMD3<Double> {
        let t = camera.target
        let s = settings.snap
        return SIMD3((Double(t.x) / s).rounded() * s, (Double(t.y) / s).rounded() * s, 0)
    }

    func add(_ node: Node, name: String) {
        let body = Solid(name: name, color: nextColor(), node: node, place: Placement(move: spawnPoint()))
        commit { $0.bodies.append(body) }
        selection = [body.id]
        dropSoon([body.id])
    }

    func addShape(_ k: ShapeKind) {
        let p = k.primitive
        add(.primitive(p), name: L(p.name))
    }

    // MARK: shape groups

    func quick(_ g: ShapeGroup) -> ShapeKind { shapes[g.rawValue].flatMap(ShapeKind.init) ?? g.members[0] }

    func setQuick(_ g: ShapeGroup, _ k: ShapeKind) {
        withAnimation(Neon.spring) { shapes[g.rawValue] = k == g.members[0] ? nil : k.rawValue }
        scheduleSave()
    }

    func rename(_ id: UUID, _ name: String) {
        let t = name.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !t.isEmpty, body(id)?.name != t else { return }
        begin()
        mutate(id) { $0.name = t }
    }

    func addFastener(_ f: Fastener) {
        add(.fastener(f), name: f.name)
    }

    // ⌘B: the bolt or nut set up last, added at once, with the Thread tab open to change it.
    func addThread() {
        addFastener(thread)
        choose(.thread)
    }

    @ObservationIgnored private var dropQueue: Set<UUID> = []
    private func dropSoon(_ ids: [UUID]) { if settings.dropToBed { dropQueue.formUnion(ids) } }

    // World bounding box of a body: exact from the shape's box when unrotated, from the mesh otherwise.
    func worldBounds(_ b: Solid) -> (SIMD3<Double>, SIMD3<Double>)? {
        guard let m = meshes[b.id], !m.vertices.isEmpty else { return nil }
        if b.place.turn == SIMD3(0, 0, 0) {
            let a = m.low * b.place.scale + b.place.move, c = m.high * b.place.scale + b.place.move
            return (simd_min(a, c), simd_max(a, c))
        }
        let mat = b.place.matrix
        var lo = SIMD3<Double>(repeating: .infinity), hi = SIMD3<Double>(repeating: -.infinity)
        let step = max(1, m.vertices.count / 4000)
        for i in stride(from: 0, to: m.vertices.count, by: step) {
            let v = m.vertices[i]
            let w = mat * SIMD4<Double>(Double(v.x), Double(v.y), Double(v.z), 1)
            lo = simd_min(lo, SIMD3(w.x, w.y, w.z))
            hi = simd_max(hi, SIMD3(w.x, w.y, w.z))
        }
        return (lo, hi)
    }

    private func applyDrops() {
        guard !dropQueue.isEmpty else { return }
        for id in dropQueue {
            guard let b = body(id), let (lo, _) = worldBounds(b) else { continue }
            mutate(id) { $0.place.move.z -= lo.z }
        }
        dropQueue.removeAll()
    }

    func deleteSelection() {
        guard !selection.isEmpty else { return }
        let ids = Set(selection)
        commit { $0.bodies.removeAll { ids.contains($0.id) } }
        selection = []
    }

    func duplicate() {
        let copies = selected.map { b -> Solid in
            var c = b
            c.id = UUID()
            c.name = b.name
            c.place.move.x += max(10, settings.snap * 10)
            return c
        }
        guard !copies.isEmpty else { return }
        commit { $0.bodies.append(contentsOf: copies) }
        selection = copies.map(\.id)
    }

    func selectAll() { selection = doc.bodies.filter { !$0.hidden }.map(\.id) }

    func hideSelection() {
        let ids = Set(selection)
        guard !ids.isEmpty else { return }
        commit { d in for i in d.bodies.indices where ids.contains(d.bodies[i].id) { d.bodies[i].hidden = true } }
        selection = []
    }

    func showAll() {
        guard doc.bodies.contains(where: \.hidden) else { return }
        commit { d in for i in d.bodies.indices { d.bodies[i].hidden = false } }
    }

    func combine(_ op: Int32) {
        let parts = selection.compactMap { body($0) }
        guard parts.count >= 2 else { flash(L("Select two or more shapes")); return }
        let first = parts[0]
        let name = op == BK_UNION ? L("Merge") : op == BK_SUBTRACT ? L("Subtract") : L("Intersect")
        let group = Solid(name: name, color: first.color, node: .group(op: op, parts: parts.map { Part(node: $0.node, place: $0.place, name: $0.name, color: $0.color) }))
        let ids = Set(parts.map(\.id))
        tryThen([group.node]) { [weak self] in
            guard let self, parts.allSatisfy({ self.body($0.id) == $0 }) else { return }
            self.commit { d in
                let at = d.bodies.firstIndex { ids.contains($0.id) } ?? d.bodies.count
                d.bodies.removeAll { ids.contains($0.id) }
                d.bodies.insert(group, at: min(at, d.bodies.count))
            }
            self.selection = [group.id]
        }
    }

    func ungroup() {
        guard let b = primary, case .group(_, let parts) = b.node else { flash(L("Select a merged shape")); return }
        let bodies = parts.enumerated().map { i, p in
            Solid(name: p.name ?? L("Part {n}", ["n": i + 1]), color: p.color ?? b.color, node: p.node,
                  place: Placement.from(b.place.matrix * p.place.matrix))
        }
        commit { d in
            let at = d.bodies.firstIndex { $0.id == b.id } ?? d.bodies.count
            d.bodies.removeAll { $0.id == b.id }
            d.bodies.insert(contentsOf: bodies, at: min(at, d.bodies.count))
        }
        selection = bodies.map(\.id)
    }

    // MARK: split

    var splitPlane: Plane? {
        let bodies = selected
        guard !bodies.isEmpty else { return nil }
        var lo = SIMD3<Double>(repeating: .infinity), hi = SIMD3<Double>(repeating: -.infinity)
        for b in bodies {
            if let (l, h) = worldBounds(b) { lo = simd_min(lo, l); hi = simd_max(hi, h) }
        }
        guard lo.x.isFinite else { return nil }
        var n = SIMD3<Double>(0, 0, 0)
        n[splitAxis] = 1
        let tx = splitTilt.x * .pi / 180, ty = splitTilt.y * .pi / 180
        let a = (splitAxis + 1) % 3, c = (splitAxis + 2) % 3
        var t = n
        t[a] += tan(tx)
        t[c] += tan(ty)
        var p = (lo + hi) / 2
        p[splitAxis] += splitOffset
        return Plane(point: p, normal: normalize(t))
    }

    func split() {
        guard let plane = splitPlane else { return }
        let targets = selected
        var halves: [UUID: [Solid]] = [:]
        for b in targets {
            let inv = b.place.matrix.inverse
            let lp = inv * SIMD4(plane.point, 1)
            let ln = simd_transpose(b.place.matrix) * SIMD4(plane.normal, 0)
            let local = Plane(point: SIMD3(lp.x, lp.y, lp.z), normal: normalize(SIMD3(ln.x, ln.y, ln.z)))
            var one = b, two = b
            one.id = UUID(); two.id = UUID()
            one.node = .split(of: b.node, plane: local, side: 0)
            two.node = .split(of: b.node, plane: local, side: 1)
            one.name = b.name + " ▲"
            two.name = b.name + " ▼"
            two.color = Palette.after(b.color)
            halves[b.id] = [one, two]
        }
        // A plane beside a shape leaves one half empty: that split isn't made.
        tryThen(halves.values.flatMap { $0.map(\.node) }) { [weak self] in
            guard let self, targets.allSatisfy({ self.body($0.id) == $0 }) else { return }
            self.commit { d in
                for (i, b) in d.bodies.enumerated().reversed() {
                    if let pair = halves[b.id] { d.bodies.replaceSubrange(i...i, with: pair) }
                }
            }
            self.selection = targets.flatMap { halves[$0.id]?.map(\.id) ?? [] }
            withAnimation(Neon.spring) { self.mode = .select }
        }
    }

    // MARK: rounding

    func commitRound() {
        guard let id = editBody, let b = body(id), !edgePicks.isEmpty, roundRadius >= 0.01 else { return }
        let node = Node.round(of: b.node, picks: edgePicks, radius: roundRadius)
        tryThen([node]) { [weak self] in
            guard let self, self.body(id)?.node == b.node else { return }
            self.commit { d in
                if let i = d.bodies.firstIndex(where: { $0.id == id }) { d.bodies[i].node = node }
            }
            self.edgePicks = []
        }
    }

    // Live rounding while dragging: builds the rounded shape in the background, newest radius wins.
    func previewRound() {
        guard let id = editBody, let b = body(id), !edgePicks.isEmpty else { return }
        if previewBusy { previewAgain = true; return }
        previewBusy = true
        let node = Node.round(of: b.node, picks: edgePicks, radius: roundRadius)
        let clearance = settings.clearance
        Kernel.shared.queue.async {
            Kernel.shared.clearance = clearance
            let (mesh, problems) = Kernel.shared.attempt(node)
            DispatchQueue.main.async {
                MainActor.assumeIsolated {
                    self.previewBusy = false
                    guard self.mode == .round, self.editBody == id else { return }
                    if let mesh {
                        self.meshes[id] = mesh
                        self.built[id] = node
                    }
                    self.report(problems)
                    if self.previewAgain { self.previewAgain = false; self.previewRound() }
                }
            }
        }
    }

    // Puts back the body's own mesh after a preview that wasn't applied.
    func clearPreview() {
        guard let id = editBody, let b = body(id), built[id] != b.node else { return }
        built[id] = nil
        rebuildScene()
    }

    // After a drag: rotated parts settle back onto the bed.
    func finishTransform() {
        if settings.dropToBed && gizmo == .rotate { dropSoon(selection) }
        applyDrops()
        sceneVersion += 1
    }

    // A resize drag: the shapes in `starts` stretched by f along axis i (all axes when uniform), a lone shape along its own
    // axis, several along the world's, spreading from their shared box as one. The left, front and bottom sides stay put,
    // or the middle when symmetric.
    func stretch(_ starts: [UUID: Placement], axis i: Int, by f: Double, uniform: Bool, symmetric: Bool) {
        let axes = uniform ? [0, 1, 2] : [i]
        if starts.count == 1, let (id, s) = starts.first {
            var keep = SIMD3<Double>(0, 0, 0)
            if let m = meshes[id] {
                keep = (m.low + m.high) / 2
                if !symmetric { for k in axes { keep[k] = m.low[k] } }
            }
            var scale = s.scale
            for k in axes { scale[k] *= f }
            mutate(id) { $0.place.scale = scale; $0.place.move = s.move + s.rotation * ((s.scale - scale) * keep) }
        } else {
            var lo = SIMD3<Double>(repeating: .infinity), hi = SIMD3<Double>(repeating: -.infinity)
            for (id, s) in starts {
                guard var b = body(id) else { continue }
                b.place = s
                if let (l, h) = worldBounds(b) { lo = simd_min(lo, l); hi = simd_max(hi, h) }
            }
            guard lo.x.isFinite else { return }
            let pivot = symmetric ? (lo + hi) / 2 : lo
            var d = SIMD3<Double>(1, 1, 1)
            for k in axes { d[k] = f }
            for (id, s) in starts {
                var scale = s.scale
                if uniform {
                    scale *= f
                } else {
                    // The shape's own axis lying closest to the world's axis i.
                    let r = s.rotation
                    scale[(0..<3).max { abs(r[$0][i]) < abs(r[$1][i]) } ?? i] *= f
                }
                mutate(id) { $0.place.scale = scale; $0.place.move = pivot + d * (s.move - pivot) }
            }
        }
        sceneVersion += 1
    }

    // After a resize: a primitive or a fastener (rounded, split or hollowed too) takes the stretch into its sizes, so roundings
    // keep their radius and threads never distort. Nothing drops to the bed: shapes stay where the resize left them.
    func finishScale() {
        for id in selection {
            guard let b = body(id), b.place.scale != SIMD3(1, 1, 1) else { continue }
            let s = b.place.scale
            let next: Node
            switch b.node.base {
            case .primitive(var p):
                p.scale(by: s)
                next = .primitive(p)
            case .fastener(var f):
                f.length = max(1, (f.length * s.z * 100).rounded() / 100)
                next = .fastener(f)
            default:
                continue
            }
            mutate(id) { $0.node = followed($0.node, to: next); $0.place.scale = SIMD3(1, 1, 1) }
        }
        rebuildScene()
    }

    // New sizes for a body's primitive or fastener (typed in, or switched from round to oval): the left, front and bottom
    // sides stay where they were (the middle, when resizing is symmetric), and roundings, bevels, splits and hollows go along.
    func reshape(_ id: UUID, _ f: (Node) -> Node) {
        guard let b = body(id) else { return }
        let next = f(b.node.base)
        guard next != b.node.base else { return }
        begin()
        let c = settings.clearance
        var move = b.place.move
        if !settings.symmetric, let e0 = b.node.base.extent(clearance: c), let e1 = next.extent(clearance: c) {
            move += b.place.rotation * (b.place.scale * (e1 - e0) / 2)
        }
        mutate(id) { $0.node = followed($0.node, to: next); $0.place.move = move }
        rebuildScene()
    }

    // A body's node on a new base, its picks, planes and faces moved by as much as the base grew along each axis.
    private func followed(_ node: Node, to next: Node) -> Node {
        let c = settings.clearance
        let out = node.replacingBase { _ in next }
        guard let e0 = node.base.extent(clearance: c), let e1 = next.extent(clearance: c), e0.min() > 0 else { return out }
        return out.following(e1 / e0)
    }

    // A click without dragging leaves no undo step behind.
    func undoLastIfUnchanged() {
        if undoStack.last == doc { undoStack.removeLast() }
    }

    // MARK: hollow

    // A click on a face: opens it, or closes it again; ⌥ gives it its own wall instead.
    func pickHollowFace(_ face: Pick, ownWall: Bool) {
        if ownWall {
            hollowOpen.removeAll { $0 == face }
            if let i = hollowWalls.firstIndex(where: { $0.face == face }) {
                focusWall = i
            } else {
                hollowWalls.append(Wall(face: face, thickness: hollowThickness))
                focusWall = hollowWalls.count - 1
            }
        } else {
            hollowWalls.removeAll { $0.face == face }
            focusWall = nil
            if let i = hollowOpen.firstIndex(of: face) { hollowOpen.remove(at: i) } else { hollowOpen.append(face) }
        }
    }

    func removeWall(_ i: Int) {
        guard hollowWalls.indices.contains(i) else { return }
        hollowWalls.remove(at: i)
        focusWall = nil
    }

    func commitHollow() {
        guard let id = editBody, let b = body(id) else { flash(L("Select a shape to hollow")); return }
        let node = Node.hollow(of: b.node, open: hollowOpen, walls: hollowWalls, thickness: hollowThickness)
        tryThen([node]) { [weak self] in
            guard let self, self.body(id)?.node == b.node else { return }
            self.commit { d in
                if let i = d.bodies.firstIndex(where: { $0.id == id }) { d.bodies[i].node = node }
            }
            withAnimation(Neon.spring) {
                self.hollowOpen = []
                self.hollowWalls = []
                self.focusWall = nil
                self.mode = .select
            }
        }
    }

    // MARK: bed

    // Puts the selection (or every visible shape) down on the bed.
    func dropToBed() {
        let ids = selection.isEmpty ? doc.bodies.filter { !$0.hidden }.map(\.id) : selection
        let moves = ids.compactMap { id -> (UUID, Double)? in
            guard let b = body(id), let (lo, _) = worldBounds(b), abs(lo.z) > 0.000_1 else { return nil }
            return (id, lo.z)
        }
        guard !moves.isEmpty else { return }
        begin()
        for (id, z) in moves { mutate(id) { $0.place.move.z -= z } }
        sceneVersion += 1
    }

    // Wrapper nodes (roundings, splits, hollows) of a body, outermost first.
    func stack(_ b: Solid) -> [Node] {
        var out: [Node] = []
        var n: Node? = b.node
        while let c = n {
            out.append(c)
            n = c.inner
        }
        return out
    }

    private func rewrite(_ node: Node, level: Int, _ f: (Node) -> Node?) -> Node {
        if level == 0 { return f(node) ?? node.inner ?? node }
        guard let n = node.inner else { return node }
        return node.wrapping(rewrite(n, level: level - 1, f))
    }

    func removeLayer(_ id: UUID, level: Int) {
        commit { d in
            if let i = d.bodies.firstIndex(where: { $0.id == id }) { d.bodies[i].node = rewrite(d.bodies[i].node, level: level) { _ in nil } }
        }
    }

    // Edits one wrapper layer (a rounding, split or hollow) of a body.
    func editLayer(_ id: UUID, level: Int, _ f: (Node) -> Node) {
        begin()
        mutate(id) { b in b.node = rewrite(b.node, level: level) { f($0) } }
        rebuildScene()
    }

    func setPlace(_ id: UUID, record: Bool = true, _ f: (inout Placement) -> Void) {
        if record { begin() }
        mutate(id) { f(&$0.place) }
        sceneVersion += 1
    }

    // A scale typed in for one shape: kept like a resize drag (which side stays, sizes taken in).
    func rescale(_ id: UUID, axis i: Int, by f: Double) {
        guard let b = body(id), f.isFinite, f > 0, abs(f - 1) > 1e-9 else { return }
        begin()
        stretch([id: b.place], axis: i, by: f, uniform: settings.uniform, symmetric: settings.symmetric)
        finishScale()
    }

    // MARK: inspector screens

    var screen: Screen {
        switch mode {
        case .angles: return .angles
        case .thread: return .thread
        default: break
        }
        switch gizmo {
        case .move: return .move
        case .scale: return .resize
        case .rotate: return .rotate
        }
    }

    func choose(_ s: Screen) {
        // The current screen again does nothing, unless a tool (split, round, hollow) has the inspector hidden.
        guard s != screen || [.round, .split, .hollow].contains(mode) else { return }
        if mode == .round { clearPreview() }
        screenStep = s.rawValue >= screen.rawValue ? 1 : -1
        withAnimation(.spring(response: 0.42, dampingFraction: 0.84)) {
            switch s {
            case .move: gizmo = .move
            case .resize: gizmo = .scale
            case .rotate: gizmo = .rotate
            case .angles, .thread: break
            }
            mode = s == .angles ? .angles : s == .thread ? .thread : .select
            edgePicks = []
            editBody = s == .angles ? selection.last : nil
        }
    }

    // A two-finger sideways swipe over the inspector moves to the neighbouring screen, one per swipe.
    private func swipeInspector(_ e: NSEvent) -> Bool {
        guard e.hasPreciseScrollingDeltas, e.momentumPhase == [], !selection.isEmpty, angleEdit == nil,
              let height = e.window?.contentView?.bounds.height else { return false }
        let p = CGPoint(x: e.locationInWindow.x, y: height - e.locationInWindow.y)
        if e.phase == .began { swipe = 0; swiped = false }
        guard inspectorFrame.contains(p), !namesFrame.contains(p), abs(e.scrollingDeltaX) > abs(e.scrollingDeltaY) else { return false }
        swipe += e.scrollingDeltaX
        if !swiped, abs(swipe) > 50 {
            swiped = true
            let step = (swipe < 0 ? 1 : -1) * (Skin.shared.rtl ? -1 : 1)
            if let next = Screen(rawValue: screen.rawValue + step) { choose(next) }
        }
        return true
    }

    // MARK: angles

    // Opens the 2D angle editor for the picked edges: the camera flies to the edge and turns to look along it,
    // then the cut through the shape takes the whole view.
    func workWithAngles() {
        guard let id = editBody, let b = body(id), let first = edgePicks.first, !angleOpening else { return }
        angleOpening = true
        probe("angles pressed") // probe
        Task { for _ in 0..<400 { let t0 = CFAbsoluteTimeGetCurrent(); try? await Task.sleep(for: .milliseconds(50)); let gap = CFAbsoluteTimeGetCurrent() - t0; if gap > 0.2 { probe(String(format: "main stalled %.0f ms", gap * 1000)) } } } // probe
        let node = b.node, picks = edgePicks, clearance = settings.clearance
        Kernel.shared.queue.async {
            probe("angles queue start") // probe
            Kernel.shared.clearance = clearance
            let section = Kernel.shared.section(node, first)
            probe("angles section done") // probe
            DispatchQueue.main.async {
                MainActor.assumeIsolated {
                    probe("angles main") // probe
                    guard let section, self.mode == .angles, let b = self.body(id) else {
                        self.angleOpening = false
                        if section == nil { self.flash(L("This edge can't be shown in a cut")) }
                        return
                    }
                    let m = b.place.matrix
                    let at = m * SIMD4(section.point, 1)
                    let along = simd_normalize((m * SIMD4(section.direction, 0)).xyz)
                    self.cameraBeforeAngles = self.camera
                    self.fly(to: SIMD3<Float>(at.xyz), looking: SIMD3<Float>(along), distance: 60) {
                        probe("angles flown") // probe
                        withAnimation(.spring(response: 0.55, dampingFraction: 0.86)) {
                            self.angleEdit = AngleEdit(body: id, picks: picks, section: section)
                        }
                        self.angleOpening = false
                    }
                }
            }
        }
    }

    func closeAngles() {
        guard angleEdit != nil else { return }
        withAnimation(.spring(response: 0.5, dampingFraction: 0.88)) { angleEdit = nil }
        if let c = cameraBeforeAngles { fly(to: c.target, yaw: c.yaw, pitch: c.pitch, distance: c.distance) }
        cameraBeforeAngles = nil
    }

    // Applies the editor's rounding or bevel to the picked edges, to the whole shape, or to every selected shape.
    func applyAngles(whole: Bool = false, everyShape: Bool = false) {
        guard let e = angleEdit else { return }
        let made = (everyShape ? selected : body(e.body).map { [$0] } ?? []).map { (id: $0.id, was: $0.node, node: e.wrapping($0.node, whole: whole || everyShape)) }
        tryThen(made.map { $0.node }) { [weak self] in
            guard let self, made.allSatisfy({ self.body($0.id)?.node == $0.was }) else { return }
            self.commit { d in
                for m in made {
                    if let i = d.bodies.firstIndex(where: { $0.id == m.id }) { d.bodies[i].node = m.node }
                }
            }
            self.edgePicks = []
            self.closeAngles()
        }
    }

    func updateAngles(_ change: (inout AngleEdit) -> Void) {
        guard var e = angleEdit else { return }
        change(&e)
        withAnimation(.spring(response: 0.36, dampingFraction: 0.84)) { angleEdit = e }
    }

    // Glides the camera to a new view over half a second (eased), redrawing each frame.
    private func fly(to target: SIMD3<Float>, looking dir: SIMD3<Float>, distance: Float, done: @escaping () -> Void) {
        let v = simd_length(dir) > 0 ? -simd_normalize(dir) : SIMD3<Float>(0, -1, 0)
        let facing = simd_dot(v, camera.eye - camera.target) < 0 ? -v : v
        let pitch = asin(max(-0.999, min(0.999, facing.z)))
        let yaw = atan2(facing.x, -facing.y)
        fly(to: target, yaw: yaw, pitch: max(-1.55, min(1.55, pitch)), distance: distance, done: done)
    }

    private func fly(to target: SIMD3<Float>, yaw: Float, pitch: Float, distance: Float, done: (() -> Void)? = nil) {
        flight?.cancel()
        let from = camera
        var turn = yaw - from.yaw
        while turn > .pi { turn -= 2 * .pi }
        while turn < -.pi { turn += 2 * .pi }
        let steps = Neon.calm ? 8 : 30
        flight = Task { [weak self] in
            for i in 1...steps {
                try? await Task.sleep(for: .milliseconds(16))
                guard let self, !Task.isCancelled else { return }
                probe(String(format: "fly step %d", i)) // probe
                let t = Float(i) / Float(steps), k = t * t * (3 - 2 * t)
                self.camera.target = from.target + (target - from.target) * k
                self.camera.yaw = from.yaw + turn * k
                self.camera.pitch = from.pitch + (pitch - from.pitch) * k
                self.camera.distance = exp(log(from.distance) + (log(distance) - log(from.distance)) * k)
                self.sceneVersion += 1
            }
            done?()
        }
    }

    // MARK: modes

    func enter(_ m: Mode) {
        if m == .split && selection.isEmpty { flash(L("Select a shape to split")); return }
        if mode == .round { clearPreview() }
        withAnimation(Neon.spring) {
            mode = mode == m ? .select : m
            edgePicks = []
            hollowOpen = []
            hollowWalls = []
            focusWall = nil
            editBody = mode == .hollow ? selection.last : nil
            splitOffset = 0
            splitTilt = .zero
        }
    }

    func cancelMode() {
        if angleEdit != nil { closeAngles(); return }
        if mode == .round { clearPreview() }
        withAnimation(Neon.spring) {
            if mode != .select { mode = .select } else { selection = [] }
            edgePicks = []
            hollowOpen = []
            hollowWalls = []
            focusWall = nil
        }
    }

    // MARK: rebuild

    func rebuildScene() {
        if building { pendingBuild = true; return }
        let bodies = doc.bodies
        let clearance = settings.clearance
        var todo: [(id: UUID, node: Node, name: String)] = []
        let rebuildAll = clearance != builtClearance
        for b in bodies where rebuildAll || built[b.id] != b.node || meshes[b.id] == nil { todo.append((b.id, b.node, b.name)) }
        let alive = Set(bodies.map(\.id))
        meshes = meshes.filter { alive.contains($0.key) }
        built = built.filter { alive.contains($0.key) }
        sceneVersion += 1
        guard !todo.isEmpty else { applyDrops(); return }
        building = true
        builtClearance = clearance
        let slow = todo.count > 1 || todo.contains { if case .fastener = $0.node.base { true } else { false } }
        let note = L("Building…")
        if slow { busy = note }
        probe(String(format: "rebuild %d", todo.count)) // probe
        Kernel.shared.queue.async {
            probe("rebuild queue start") // probe
            Kernel.shared.clearance = clearance
            // Nothing left over from other work (a save, a cut for the angle editor) is said as if it happened here.
            _ = Kernel.shared.takeProblems()
            var trouble: (name: String, problems: [String])?
            // Each shape shows as soon as it's built, rather than all of them at the end.
            for (id, node, name) in todo {
                let mesh = Kernel.shared.mesh(node)
                let problems = Kernel.shared.takeProblems()
                if trouble == nil, !problems.isEmpty { trouble = (name, problems) }
                DispatchQueue.main.async {
                    MainActor.assumeIsolated {
                        self.built[id] = node
                        // A shape the kernel can't build keeps what it showed (its saved look after opening a file).
                        self.meshes[id] = mesh ?? self.meshes[id] ?? Mesh()
                        self.sceneVersion += 1
                    }
                }
            }
            let found = trouble
            probe("rebuild queue done") // probe
            DispatchQueue.main.async {
                MainActor.assumeIsolated {
                    self.building = false
                    if slow { self.ended(note) }
                    self.sceneVersion += 1
                    self.applyDrops()
                    // With several shapes built (a file opened), the message says which one.
                    if let found { self.report(found.problems, name: todo.count > 1 ? found.name : nil) }
                    if self.pendingBuild { self.pendingBuild = false; self.rebuildScene() }
                }
            }
        }
    }

    private func report(_ problems: [String], name: String? = nil) {
        guard let text = Self.message(problems) else { return }
        flash(name.map { "\($0): \(text)" } ?? text)
    }

    private static func message(_ problems: [String]) -> String? {
        if let m = problems.first(where: { $0.hasPrefix("max:") }) {
            return L("Rounding too large — the most this edge takes is {r} mm", ["r": String(format: "%.2f", Double(m.dropFirst(4)) ?? 0)])
        }
        if problems.contains("round") { return L("These edges can't be rounded") }
        if problems.contains("cove") { return L("These edges can't be rounded inward") }
        if problems.contains("bevel") { return L("This bevel doesn't fit these edges — try smaller sizes") }
        if problems.contains("bend") { return L("The tube is too thick for this torus's tightest bend") }
        if problems.contains("empty") { return L("Nothing is left of this shape") }
        if problems.contains("pieces") { return L("These shapes don't touch, so the merge stays in separate pieces") }
        if problems.contains("hollow") { return L("These walls don't fit this shape — try thinner walls") }
        if problems.contains("missing") { return L("Some picked edges or faces no longer exist and were skipped") }
        return problems.isEmpty ? nil : L("The shape operation failed")
    }

    // MARK: keys

    private func key(_ e: NSEvent) -> Bool {
        if NSApp.modalWindow != nil { return false }
        let name = Keys.codes[e.keyCode] ?? ""
        if capturing != nil { capture(e.keyCode == 53 ? "" : name); return true }
        if let r = NSApp.keyWindow?.firstResponder, r is NSText || r is NSTextView { return false }
        let mods = e.modifierFlags.intersection([.command, .control, .option])
        if !mods.isEmpty { return false }
        let shift = e.modifierFlags.contains(.shift)
        switch name {
        case "Enter":
            if angleEdit != nil { applyAngles(); return true }
            if mode == .split { split(); return true }
            if mode == .round { commitRound(); return true }
            if mode == .hollow { commitHollow(); return true }
            return false
        case "Backspace":
            deleteSelection(); return true
        case "ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown", "PageUp", "PageDown":
            nudge(name, big: shift); return true
        case "KeyX" where mode == .split, "KeyZ" where mode == .split:
            withAnimation(Neon.spring) { splitAxis = name == "KeyX" ? 0 : 2 }; return true
        case "KeyA" where mode == .round || mode == .angles && angleEdit == nil:
            if let b = editBody ?? selection.last { editBody = b; edgePicks = [Pick(kind: Int32(BK_PICK_BODY), a: .zero, b: .zero)] }
            return true
        default: break
        }
        if e.keyCode == 53 { cancelMode(); return true }
        if name.hasPrefix("Digit"), let n = Int(name.dropFirst(5)), n <= 6 { camera.preset(n); sceneVersion += 1; return true }
        if mode == .split, name == settings.key(.split) { return false }
        if mode == .split, name == "KeyY" { withAnimation(Neon.spring) { splitAxis = 1 }; return true }
        guard !name.isEmpty, let a = settings.owner(of: name) else { return false }
        perform(a)
        return true
    }

    func perform(_ a: Action) {
        switch a {
        case .move: choose(.move)
        case .rotate: choose(.rotate)
        case .scale: choose(.resize)
        case .round: enter(.round)
        case .split: enter(.split)
        case .hollow: enter(.hollow)
        case .drop: dropToBed()
        case .frame: requestFit = true; sceneVersion += 1
        case .hide: hideSelection()
        case .showAll: showAll()
        }
    }

    private func nudge(_ key: String, big: Bool) {
        guard !selection.isEmpty else { return }
        let step = settings.snap * (big ? 10 : 1)
        var d = SIMD3<Double>(0, 0, 0)
        switch key {
        case "ArrowLeft": d.x = -step
        case "ArrowRight": d.x = step
        case "ArrowUp": d.y = step
        case "ArrowDown": d.y = -step
        case "PageUp": d.z = step
        default: d.z = -step
        }
        begin()
        for id in selection { mutate(id) { $0.place.move += d } }
        sceneVersion += 1
    }

    // MARK: files

    var title: String { fileURL?.deletingPathExtension().lastPathComponent ?? L("Untitled") }

    // Asks about unsaved changes before they'd be lost; `done` learns whether to go ahead (once saved, if that was chosen).
    func confirmDiscard(_ done: @escaping (Bool) -> Void) {
        guard dirty else { done(true); return }
        let a = NSAlert()
        a.messageText = L("Save changes to “{name}”?", ["name": title])
        a.informativeText = L("Your changes are lost if you don't save them.")
        a.addButton(withTitle: L("Save"))
        a.addButton(withTitle: L("Cancel")).keyEquivalent = "\u{1b}"
        a.addButton(withTitle: L("Don't Save"))
        switch a.runModal() {
        case .alertFirstButtonReturn: saveDocument(done: done)
        case .alertThirdButtonReturn: done(true)
        default: done(false)
        }
    }

    func newDocument() {
        confirmDiscard { go in
            guard go else { return }
            self.resetEditing()
            self.doc = Document()
            self.saved = self.doc
            self.fileURL = nil
            self.rebuildScene()
        }
    }

    // Leaves every tool, editor and pending step of the current document behind, before another one comes in.
    private func resetEditing() {
        flight?.cancel()
        angleEdit = nil
        angleOpening = false
        cameraBeforeAngles = nil
        mode = .select
        editBody = nil
        edgePicks = []
        hollowOpen = []
        hollowWalls = []
        focusWall = nil
        hover = Hover()
        selection = []
        dropQueue = []
        capturing = nil
        undoStack = []
        redoStack = []
    }

    func openDocument() {
        confirmDiscard { go in
            guard go else { return }
            let panel = NSOpenPanel()
            panel.allowedContentTypes = [UTType(filenameExtension: "3mf") ?? .data]
            guard panel.runModal() == .OK, let url = panel.url else { return }
            self.open(url)
        }
    }

    func open(_ url: URL) {
        do {
            let (d, shapes) = try ThreeMF.read(url)
            resetEditing()
            doc = d
            saved = d
            fileURL = url
            // The shapes show at once as they were saved; the kernel then rebuilds each exactly and replaces it.
            meshes = [:]
            for b in d.bodies { meshes[b.id] = shapes[b.id].flatMap { Mesh(saved: $0, place: b.place) } }
            built = [:]
            requestFit = true
            rebuildScene()
        } catch FileError.notBcad {
            flash(L("This 3MF wasn't made by Bcad and can't be edited"))
        } catch {
            flash(L("This file is damaged and can't be opened"))
        }
    }

    // The file is written on the kernel's thread, so the window stays live; `done` learns whether it worked.
    func saveDocument(as: Bool = false, done: @escaping (Bool) -> Void = { _ in }) {
        var url = fileURL
        if url == nil || `as` {
            let panel = NSSavePanel()
            panel.allowedContentTypes = [UTType(filenameExtension: "3mf") ?? .data]
            panel.nameFieldStringValue = title + ".3mf"
            guard panel.runModal() == .OK, let u = panel.url else { done(false); return }
            url = u
        }
        guard let url else { done(false); return }
        let doc = self.doc, clearance = settings.clearance, note = L("Saving…")
        withAnimation(Neon.spring) { busy = note }
        Kernel.shared.queue.async {
            Kernel.shared.clearance = clearance
            let meshes = doc.bodies.filter { !$0.hidden }.compactMap { b in Kernel.shared.worldMesh(b).map { (b, $0) } }
            let ok = (try? ThreeMF.write(url, meshes: meshes, doc: doc)) != nil
            DispatchQueue.main.async {
                MainActor.assumeIsolated {
                    self.ended(note)
                    if ok {
                        self.fileURL = url
                        self.saved = doc
                        self.flash(L("Saved {name}", ["name": url.lastPathComponent]))
                    } else {
                        self.flash(L("Couldn't save the file"))
                    }
                    done(ok)
                }
            }
        }
    }

    func export(step: Bool) {
        let bodies = (selection.isEmpty ? doc.bodies : selected).filter { !$0.hidden }
        guard !bodies.isEmpty else { flash(L("Nothing to export")); return }
        let panel = NSSavePanel()
        panel.allowedContentTypes = [UTType(filenameExtension: step ? "step" : "stl") ?? .data]
        panel.nameFieldStringValue = title + (step ? ".step" : ".stl")
        guard panel.runModal() == .OK, let url = panel.url else { return }
        let clearance = settings.clearance, note = L("Exporting…")
        withAnimation(Neon.spring) { busy = note }
        Kernel.shared.queue.async {
            Kernel.shared.clearance = clearance
            let ok: Bool
            if step {
                ok = Kernel.shared.exportStep(bodies, to: url.path)
            } else {
                let meshes = bodies.compactMap { Kernel.shared.worldMesh($0) }
                ok = meshes.count == bodies.count && (try? STL.write(url, meshes: meshes)) != nil
            }
            DispatchQueue.main.async {
                MainActor.assumeIsolated {
                    self.ended(note)
                    self.flash(ok ? L("Exported {name}", ["name": url.lastPathComponent]) : L("Export failed"))
                }
            }
        }
    }
}

// MARK: - App

final class AppDelegate: NSObject, NSApplicationDelegate {
    func applicationDidFinishLaunching(_ n: Notification) {
        MainActor.assumeIsolated {
            MenuText.install()
            Workbench.shared.rebuildScene()
        }
    }

    func applicationShouldTerminateAfterLastWindowClosed(_ app: NSApplication) -> Bool { true }

    func applicationShouldTerminate(_ sender: NSApplication) -> NSApplication.TerminateReply {
        MainActor.assumeIsolated {
            let lib = Workbench.shared
            lib.save()
            guard lib.dirty else { return .terminateNow }
            lib.confirmDiscard { go in DispatchQueue.main.async { MainActor.assumeIsolated { NSApp.reply(toApplicationShouldTerminate: go) } } }
            return .terminateLater
        }
    }

    // OpenCascade tears itself down as the process exits and crashes if a shape is still being built; everything worth
    // keeping is saved by now, so the app leaves without that teardown.
    func applicationWillTerminate(_ n: Notification) { _exit(0) }

    func application(_ app: NSApplication, open urls: [URL]) {
        MainActor.assumeIsolated {
            guard let u = urls.first else { return }
            Workbench.shared.confirmDiscard { go in if go { Workbench.shared.open(u) } }
        }
    }
}

struct BcadApp: App {
    @NSApplicationDelegateAdaptor(AppDelegate.self) private var delegate
    @State private var lib = Workbench.shared

    var body: some Scene {
        Window("Bcad", id: "main") {
            RootView()
                .environment(lib)
                .frame(minWidth: 1000, minHeight: 600)
                .skinEnvironment()
        }
        .windowStyle(.hiddenTitleBar)
        .defaultSize(width: 1320, height: 820)
        .commands {
            CommandGroup(replacing: .newItem) {
                Button(L("New")) { lib.newDocument() }.keyboardShortcut("n")
                Button(L("Open…")) { lib.openDocument() }.keyboardShortcut("o")
            }
            CommandGroup(replacing: .saveItem) {
                Button(L("Save")) { lib.saveDocument() }.keyboardShortcut("s")
                Button(L("Save As…")) { lib.saveDocument(as: true) }.keyboardShortcut("s", modifiers: [.command, .shift])
                Divider()
                Button(L("Export STL…")) { lib.export(step: false) }.keyboardShortcut("e", modifiers: [.command, .shift])
                Button(L("Export STEP…")) { lib.export(step: true) }.keyboardShortcut("e", modifiers: [.command, .option])
            }
            CommandGroup(replacing: .appSettings) {
                Button(L("Settings…")) { lib.toggleSettings() }.keyboardShortcut(",")
            }
            CommandGroup(replacing: .undoRedo) {
                Button(L("Undo")) { Edits.send(#selector(UndoManager.undo)) ? () : lib.undo() }.keyboardShortcut("z")
                Button(L("Redo")) { Edits.send(#selector(UndoManager.redo)) ? () : lib.redo() }.keyboardShortcut("z", modifiers: [.command, .shift])
            }
            CommandGroup(replacing: .pasteboard) {
                Button(L("Cut")) { _ = Edits.send(#selector(NSText.cut(_:))) }.keyboardShortcut("x")
                Button(L("Copy")) { _ = Edits.send(#selector(NSText.copy(_:))) }.keyboardShortcut("c")
                Button(L("Paste")) { _ = Edits.send(#selector(NSText.paste(_:))) }.keyboardShortcut("v")
                Button(L("Select All")) { Edits.send(#selector(NSText.selectAll(_:))) ? () : lib.selectAll() }.keyboardShortcut("a")
                Button(L("Duplicate")) { lib.duplicate() }.keyboardShortcut("d")
                Button(L("Delete")) { lib.deleteSelection() }
            }
            CommandMenu(L("Shape")) {
                Button(L("Merge")) { lib.combine(Int32(BK_UNION)) }.keyboardShortcut("u")
                Button(L("Subtract")) { lib.combine(Int32(BK_SUBTRACT)) }.keyboardShortcut(.delete, modifiers: .command)
                Button(L("Intersect")) { lib.combine(Int32(BK_INTERSECT)) }.keyboardShortcut("i")
                Button(L("Ungroup")) { lib.ungroup() }.keyboardShortcut("g", modifiers: [.command, .shift])
                Divider()
                Button(L("Split")) { lib.enter(.split) }
                Button(L("Round edges")) { lib.enter(.round) }
                Divider()
                Button(L("Hollow")) { lib.enter(.hollow) }
                Button(L("Drop onto the bed")) { lib.dropToBed() }
                Divider()
                Button(L("Add thread")) { lib.addThread() }.keyboardShortcut("b")
            }
        }
    }
}

enum Edits {
    // Sends an editing command to a focused text field; false when no text field has focus.
    @MainActor static func send(_ action: Selector) -> Bool {
        guard let r = NSApp.keyWindow?.firstResponder, r is NSText else { return false }
        return NSApp.sendAction(action, to: nil, from: nil)
    }
}

func probe(_ s: String) { // probe
    let line = ISO8601DateFormatter.string(from: Date(), timeZone: .gmt, formatOptions: [.withInternetDateTime, .withFractionalSeconds]) + " " + s + "\n" // probe
    if let h = FileHandle(forWritingAtPath: "/tmp/bcad-probe.txt") { h.seekToEndOfFile(); h.write(Data(line.utf8)); h.closeFile() } else { FileManager.default.createFile(atPath: "/tmp/bcad-probe.txt", contents: Data(line.utf8)) } // probe
} // probe
