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

struct Primitive: Codable, Hashable, Sendable {
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

    // Applies a scale to the sizes. Stretched unevenly across, a cylinder becomes an oval cylinder and a torus an oval
    // torus, just as the stretch shows; other round shapes stay round (the axis changed most wins). Walls stay as they are;
    // a torus's tube is its height, and stays thin enough for its ring.
    mutating func scale(by s: SIMD3<Double>) {
        func most(_ v: [Double]) -> Double { v.max { abs($0 - 1) < abs($1 - 1) } ?? 1 }
        if abs(s.x - s.y) > 1e-9 * max(abs(s.x), abs(s.y)) {
            switch kind {
            case .cylinder: kind = .oval; size = [size[0], size[0], 90, size[1]]
            case .torus: kind = .ovalTorus; size = [size[0], size[0], 90, size[1]]
            default: break
            }
        }
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

// A bolt or nut on an ISO metric coarse thread; the kernel knows every kind's sizes and what they may be (BcadKernel.h).
struct Fastener: Codable, Hashable, Sendable {
    // In the kernel's order (BK_ROD … BK_CONE_NUT).
    enum Kind: String, Codable, CaseIterable, Sendable {
        case rod, hex, hexCone, socket, socketCone, twelve, twelveCone, torx, torxCone, phHex, phHexCone, phCone
        case sleeve, squareNut, hexNut, coneNut

        var code: Int32 { Int32(Self.allCases.firstIndex(of: self)!) }
        var nut: Bool { code >= Int32(BK_SLEEVE) }
        var fields: [Field] { Field.allCases.filter { bk_fastener_fields(code) & (1 << $0.rawValue) != 0 } }
        var countersunk: Bool { [.socketCone, .torxCone, .phCone].contains(self) }
        var coneBelow: Bool { [.hexCone, .twelveCone, .phHexCone].contains(self) }
        var keyDrive: Bool { self == .socket || self == .socketCone }
        var torxDrive: Bool { self == .torx || self == .torxCone }
        var phillips: Bool { [.phHex, .phHexCone, .phCone].contains(self) }
        static let bolts = allCases.filter { !$0.nut }
        static let nuts = allCases.filter(\.nut)

        var label: String {
            switch self {
            case .rod: "Threaded rod"
            case .hex, .hexNut: "Hex"
            case .hexCone: "Hex · cone below"
            case .socket: "Hex socket"
            case .socketCone: "Hex socket · countersunk"
            case .twelve: "12-point"
            case .twelveCone: "12-point · cone below"
            case .torx: "Torx"
            case .torxCone: "Torx · countersunk"
            case .phHex: "PH · hex"
            case .phHexCone: "PH · hex · cone below"
            case .phCone: "PH · countersunk"
            case .sleeve: "Sleeve"
            case .squareNut: "Square"
            case .coneNut: "Cone"
            }
        }
    }

    // In the kernel's order (BK_LENGTH … BK_DEPTH).
    enum Field: Int32, CaseIterable, Sendable { case length, width, height, angle, seat, drive, recess, depth }

    var kind: Kind
    var size: Int
    var length: Double
    var width = 0.0, height = 0.0, angle = 0.0, seat = 0.0, drive = 0.0, recess = 0.0, depth = 0.0

    var nut: Bool { kind.nut }

    // A new one: its kind's sizes for this thread, and the usual length.
    init(kind: Kind, size: Int) {
        self.kind = kind
        self.size = size
        length = 0
        self = through { bk_fastener_defaults(&$0, 1) }
    }

    var c: BKFastener {
        BKFastener(kind: kind.code, size: Int32(size), length: length, width: width, height: height, angle: angle, seat: seat,
                   drive: drive, recess: recess, depth: depth)
    }

    // The same with the kernel's change made to it.
    private func through(_ change: (inout BKFastener) -> Void) -> Fastener {
        var b = c
        change(&b)
        var n = self
        (n.length, n.width, n.height, n.angle, n.seat, n.drive, n.recess, n.depth) = (b.length, b.width, b.height, b.angle, b.seat, b.drive, b.recess, b.depth)
        return n
    }

    // Another kind on the same thread, with that kind's sizes; a nut's height is one of them, and so is a bolt's length
    // when it was a nut.
    func becoming(_ k: Kind) -> Fastener {
        var n = self
        n.kind = k
        let length = k.nut || nut ? 1 : 0
        return n.through { bk_fastener_defaults(&$0, Int32(length)) }
    }

    // Another thread: its sizes and usual length.
    func threaded(_ size: Int) -> Fastener { Fastener(kind: kind, size: size) }

    subscript(_ field: Field) -> Double {
        get { [length, width, height, angle, seat, drive, recess, depth][Int(field.rawValue)] }
        set {
            switch field {
            case .length: length = newValue
            case .width: width = newValue
            case .height: height = newValue
            case .angle: angle = newValue
            case .seat: seat = newValue
            case .drive: drive = newValue
            case .recess: recess = newValue
            case .depth: depth = newValue
            }
        }
    }

    // One size changed; the ones depending on it follow into what they may be (a Phillips size brings its recess).
    func setting(_ field: Field, _ v: Double) -> Fastener {
        if field == .drive { return through { bk_fastener_drive(&$0, v) } }
        var n = self
        n[field] = v
        return n.through { bk_fastener_fit(&$0) }
    }

    // What one size may be with the others as they are, or (loose) with the sizes depending on it following (nil when
    // nothing fits).
    func range(_ field: Field, loose: Bool = false) -> ClosedRange<Double>? {
        var b = c, out = [0.0, 0.0]
        bk_fastener_range(&b, field.rawValue, loose ? 1 : 0, &out)
        return out[0] <= out[1] + 1e-6 ? out[0]...max(out[0], out[1]) : nil
    }

    @MainActor var name: String {
        let m = String(cString: bk_thread_name(Int32(size)))
        return switch kind {
        case .rod: L("{m} threaded rod", ["m": m])
        case .sleeve: L("{m} threaded sleeve", ["m": m])
        case .squareNut: L("{m} square nut", ["m": m])
        case .coneNut: L("{m} cone nut", ["m": m])
        case .hexNut: L("{m} nut", ["m": m])
        default: L("{m} bolt", ["m": m])
        }
    }

    // Its bounding size; the kernel centres it on its own origin like a primitive.
    func extent(clearance: Double) -> SIMD3<Double> {
        var b = c, out = [0.0, 0.0, 0.0]
        bk_fastener_extent(&b, clearance, &out)
        return SIMD3(out[0], out[1], out[2])
    }
}

extension Fastener {
    private enum Saved: String, CodingKey { case kind, size, length, width, height, angle, seat, drive, recess, depth, nut, threadOnly }

    // Files from earlier versions keep a bolt or nut as hex or plain (threadOnly), without head sizes: it gets the standard ones.
    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: Saved.self)
        let size = try c.decode(Int.self, forKey: .size), length = try c.decode(Double.self, forKey: .length)
        if let kind = try c.decodeIfPresent(Kind.self, forKey: .kind) {
            self.kind = kind
            self.size = size
            self.length = length
            width = try c.decode(Double.self, forKey: .width)
            height = try c.decode(Double.self, forKey: .height)
            angle = try c.decode(Double.self, forKey: .angle)
            seat = try c.decode(Double.self, forKey: .seat)
            drive = try c.decode(Double.self, forKey: .drive)
            recess = try c.decode(Double.self, forKey: .recess)
            depth = try c.decode(Double.self, forKey: .depth)
        } else {
            let nut = try c.decode(Bool.self, forKey: .nut), plain = try c.decode(Bool.self, forKey: .threadOnly)
            self = Fastener(kind: nut ? (plain ? .sleeve : .hexNut) : (plain ? .rod : .hex), size: size)
            self.length = length
        }
    }
}

struct Placement: Codable, Hashable, Sendable {
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

struct Plane: Codable, Hashable, Sendable {
    var point: SIMD3<Double>
    var normal: SIMD3<Double>
}

struct Pick: Codable, Hashable, Sendable {
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

struct Part: Codable, Hashable, Sendable {
    var node: Node
    var place: Placement
    var name: String?
    var color: SIMD3<UInt8>?
}

// A face of a hollowed shape with its own wall thickness.
struct Wall: Codable, Hashable, Sendable {
    var face: Pick
    var thickness: Double
}

indirect enum Node: Codable, Hashable, Sendable {
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

    // A treatment's picks.
    var picks: [Pick]? {
        switch self {
        case .round(_, let p, _), .cove(_, let p, _), .bevel(_, let p, _, _): p
        default: nil
        }
    }

    // The same treatment of other edges, around a node.
    func withPicks(_ p: [Pick], of n: Node) -> Node {
        switch self {
        case .round(_, _, let r): .round(of: n, picks: p, radius: r)
        case .cove(_, _, let r): .cove(of: n, picks: p, radius: r)
        case .bevel(_, _, let l, let c): .bevel(of: n, picks: p, legs: l, corner: c)
        default: self
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

    // The same shape with its roundings up to radius r left out, inside merged parts too.
    func sharp(upTo r: Double) -> Node {
        switch self {
        case .round(let n, _, let radius) where radius <= r: n.sharp(upTo: r)
        case .group(let op, let parts): .group(op: op, parts: parts.map { var p = $0; p.node = p.node.sharp(upTo: r); return p })
        default: inner.map { wrapping($0.sharp(upTo: r)) } ?? self
        }
    }

    // The same shape with its roundings, bevels and inward roundings left out, inside merged parts too (hollows and cuts
    // kept).
    var untreated: Node {
        switch self {
        case .round(let n, _, _), .cove(let n, _, _), .bevel(let n, _, _, _): n.untreated
        case .group(let op, let parts): .group(op: op, parts: parts.map { var p = $0; p.node = p.node.untreated; return p })
        default: inner.map { wrapping($0.untreated) } ?? self
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
    // A merge this shape was taken out of to be edited, to be made again (every shape carrying the same link is in it).
    var link: MergeLink?
}

// A merge switched off: what it was made with (its operation, its placement, its name and colour, and the layers on it —
// roundings, hollows — around a merge of its parts), and where this shape stood among its parts.
struct MergeLink: Codable, Hashable, Sendable {
    var id: UUID
    var op: Int32
    var order: Int
    var shell: Node
    var place: Placement
    var name: String
    var color: SIMD3<UInt8>
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
        link = try c.decodeIfPresent(MergeLink.self, forKey: .link)
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
    // Several shapes' pointers at once, as a C array.
    static func with<T>(_ refs: [ShapeRef], _ body: (UnsafePointer<OpaquePointer?>?) -> T) -> T {
        withExtendedLifetime(refs) { refs.map { Optional($0.ptr) }.withUnsafeBufferPointer { body($0.baseAddress) } }
    }
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

    // Its triangles where the placement puts them (for files; faces and normals left as they are).
    func placed(_ place: Placement) -> Mesh {
        let m = simd_float4x4(place.matrix)
        var out = self
        out.vertices = vertices.map { v in SIMD4((m * SIMD4(v.xyz, 1)).xyz, v.w) }
        return out
    }
}

// A job waited on for a while: begun, or dropped by the one waiting.
private final class Pending<T>: @unchecked Sendable {
    private let lock = NSLock()
    private var dropped = false, begun = false
    var out: T?

    func begin() -> Bool {
        lock.lock()
        defer { lock.unlock() }
        begun = !dropped
        return begun
    }

    func drop() -> T? {
        lock.lock()
        defer { lock.unlock() }
        dropped = !begun
        return nil
    }
}

// One serial worker thread with a large stack (GCD threads get 512 KB): a deep tree of shapes is built recursively.
final class Worker: @unchecked Sendable {
    private let lock = NSCondition()
    private var jobs: [() -> Void] = []
    private var running = false

    // Nothing waiting and nothing under way: a job asked for now starts at once.
    var idle: Bool {
        lock.lock()
        defer { lock.unlock() }
        return jobs.isEmpty && !running
    }

    init() {
        let t = Thread { [unowned self] in
            while true {
                lock.lock()
                while jobs.isEmpty { lock.wait() }
                let job = jobs.removeFirst()
                running = true
                lock.unlock()
                job()
                lock.lock()
                running = false
                lock.unlock()
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

    // As sync, waiting at most `limit` seconds: nil when earlier work held the worker that long, and then the job is
    // dropped if it hasn't begun.
    func sync<T>(within limit: TimeInterval, _ job: @escaping () -> T) -> T? {
        let p = Pending<T>(), done = DispatchSemaphore(value: 0)
        async {
            guard p.begin() else { return }
            p.out = job()
            done.signal()
        }
        return done.wait(timeout: .now() + limit) == .success ? p.out : p.drop()
    }

    // Escaping: the worker may still hold the job a moment after it has signalled that it's done.
    func sync<T>(_ job: @escaping () -> T) -> T {
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

final class Kernel: @unchecked Sendable {
    static let shared = Kernel()
    let queue = Worker()
    private var cache: [Key: ShapeRef] = [:]
    private(set) var problems: [String] = []
    var clearance = 0.2

    // A shape as built for a clearance (to the thousandth of a millimetre): hashed as it is, nothing encoded.
    private struct Key: Hashable {
        let node: Node
        let clearance: Double
    }

    private func key(_ node: Node) -> Key { Key(node: node, clearance: (clearance * 1000).rounded() / 1000) }

    func takeProblems() -> [String] { defer { problems = [] }; return problems }

    // keep: false for a shape shown only for a moment (a step of a live resize), so it doesn't crowd out the others.
    func shape(_ node: Node, keep: Bool = true) -> ShapeRef? {
        let k = key(node)
        if let s = cache[k] { return s }
        guard let p = build(node) else { return nil }
        // Nothing solid left (all of it cut away, shapes that don't overlap, a split beside the shape) is a failure too.
        if bk_piece_count(p) == 0 {
            bk_free(p)
            problems.append("empty")
            return nil
        }
        let ref = ShapeRef(p)
        guard keep else { return ref }
        if cache.count > 400 { cache.removeAll() }
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
            var b = f.c
            return made(bk_fastener(&b, clearance))
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
            let numeric = out == nil && Self.numeric()
            if missing > 0 { problems.append("missing") }
            if let out = out ?? beneath(of, node) { return out }
            problems.append(numeric ? "failed" : maxR > 0 ? "max:\(maxR)" : "round")
            return s.with { bk_copy($0) }
        case .bevel(let of, let picks, let legs, let corner):
            guard let s = shape(of) else { return nil }
            let (kinds, data) = Self.flat(picks)
            var missing: Int32 = 0
            let out = s.with { bk_chamfer($0, kinds, data, Int32(picks.count), legs.x, legs.y, corner, &missing) }
            let numeric = out == nil && Self.numeric()
            if missing > 0 { problems.append("missing") }
            if let out = out ?? beneath(of, node) { return out }
            problems.append(numeric ? "failed" : "bevel")
            return s.with { bk_copy($0) }
        case .cove(let of, let picks, let radius):
            guard let s = shape(of) else { return nil }
            let (kinds, data) = Self.flat(picks)
            var maxR = 0.0
            var missing: Int32 = 0
            let out = s.with { bk_cove($0, kinds, data, Int32(picks.count), radius, &maxR, &missing) }
            let numeric = out == nil && Self.numeric()
            if missing > 0 { problems.append("missing") }
            if let out = out ?? beneath(of, node) { return out }
            problems.append(numeric ? "failed" : maxR > 0 ? "max:\(maxR)" : "cove")
            return s.with { bk_copy($0) }
        case .hollow(let of, let open, let walls, let thickness):
            guard let s = shape(of) else { return nil }
            let faces = open.flatMap { [$0.a.x, $0.a.y, $0.a.z, $0.b.x, $0.b.y, $0.b.z] }
            let own = walls.flatMap { [$0.face.a.x, $0.face.a.y, $0.face.a.z, $0.face.b.x, $0.face.b.y, $0.face.b.z] }
            let values = walls.map(\.thickness)
            // Kept for when the shape itself doesn't offset: the shape without the roundings no thicker than the walls,
            // then without any (each only when it builds as asked). What building them says isn't said of this hollow.
            var sharp: [ShapeRef] = []
            for n in [of.sharp(upTo: values.reduce(thickness, max)), of.sharp(upTo: .infinity)] where n != of {
                let mark = problems.count
                if let r = shape(n), !problems[mark...].contains(where: Self.failure), !sharp.contains(where: { $0 === r }) { sharp.append(r) }
                problems.removeSubrange(mark...)
            }
            var missing: Int32 = 0
            let out = s.with { sp in
                ShapeRef.with(sharp) { bk_hollow(sp, $0, Int32(sharp.count), faces, Int32(open.count), own, values, Int32(walls.count), thickness, &missing) }
            }
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

    // Whether the engine's last refusal wasn't for size but because it couldn't work the shape out (said as a failure,
    // not as "too large").
    static func numeric() -> Bool { String(cString: bk_last_error()).hasPrefix("numeric") }

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

    // MARK: treated faces

    // Where a click lands on a face a rounding, bevel or inward rounding made: that layer, the edge it made the face along
    // (as it was before, sharp) and its other edges. It is the first layer, outermost first, before which the clicked point
    // wasn't on the shape's surface (nor facing the same way); a merge's parts are looked into, the one whose surface the
    // point is on. Nil on a face the shape had before any treatment (or one a hollow or a cut made).
    // With `edge`, p is a clicked edge's middle, and the layer is the first before which the shape had no edge there (a
    // rounding's seams, a bevel's borders).
    func treatedAt(_ node: Node, _ p: SIMD3<Double>, edge: Bool = false) -> Spot? {
        // On the finely meshed surface (a click lands on the coarser one shown), so faces kept as they were match closely.
        guard let top = mesh(node, deflection: 0.01) else { return nil }
        if edge {
            guard let e = Self.nearestEdgeAt(top, p) else { return nil }
            return treatedAt(node, e.point, nil, path: [])
        }
        guard let hit = Self.nearest(top, p) else { return nil }
        return treatedAt(node, hit.point, hit.normal, path: [])
    }

    // `facing` nil: p is on an edge, looked for as an edge.
    private func treatedAt(_ node: Node, _ p: SIMD3<Double>, _ facing: SIMD3<Double>?, path: [Int]) -> Spot? {
        var level = 0
        var n: Node? = node
        while let c = n {
            if let inner = c.inner {
                guard let m = mesh(inner, deflection: 0.01) else { return nil }
                let gone: Bool
                if let facing {
                    guard let hit = Self.nearest(m, p) else { return nil }
                    gone = hit.distance > 0.04 || simd_dot(hit.normal, facing) < cos(5 * Double.pi / 180)
                } else {
                    gone = (Self.nearestEdgeAt(m, p)?.distance ?? .infinity) > 0.04
                }
                if gone {
                    switch c {
                    case .round, .cove, .bevel:
                        guard let edge = madeOn(c, m, p) else { return nil }
                        return Spot(path: path, level: level, edges: [edge], rest: [], layer: c)
                    default:
                        return nil
                    }
                }
            } else if case .group(_, let parts) = c {
                for (i, part) in parts.enumerated() {
                    // In the part's own frame (normals carried by the transpose).
                    let q = (part.place.matrix.inverse * SIMD4(p, 1)).xyz
                    let f = facing.map { simd_normalize((simd_transpose(part.place.matrix) * SIMD4($0, 0)).xyz) }
                    guard let m = mesh(part.node, deflection: 0.01) else { continue }
                    let near = f == nil ? Self.nearestEdgeAt(m, q)?.distance : Self.nearest(m, q)?.distance
                    guard (near ?? .infinity) < 0.06 else { continue }
                    if let s = treatedAt(part.node, q, f, path: path + [level, i]) { return s }
                }
                return nil
            }
            n = c.inner
            level += 1
        }
        return nil
    }

    // The edge a layer worked on nearest p, as it was before (sharp): among the edges the layer's picks name, so another
    // edge of a narrow face beside it isn't taken for it.
    private func madeOn(_ layer: Node, _ m: Mesh, _ p: SIMD3<Double>) -> Pick? {
        let own = rest(of: layer, without: []).map(\.a)
        return (Self.nearestEdgeAt(m, p, through: own) ?? Self.nearestEdgeAt(m, p))?.pick
    }

    // Whether the faces either side of a picked edge meet smoothly there: no corner to work on.
    func smooth(_ node: Node, _ pick: Pick) -> Bool {
        guard let m = mesh(node, deflection: 0.01), let e = Self.nearestEdgeAt(m, pick.a), e.distance < 0.1 else { return false }
        let faces = m.edgeFaces[e.index]
        // Each side's nearest point of the mesh on the edge, and which way the surface faces there.
        var best = [Double.infinity, .infinity], normal = [SIMD3<Double>.zero, .zero]
        for (i, v) in m.vertices.enumerated() {
            let f = Int32(v.w), side = f == faces.x ? 0 : f == faces.y ? 1 : -1
            let q = SIMD3<Double>(v.xyz)
            guard side >= 0, simd_length(Self.onEdge(m.edges[e.index], q) - q) < 1e-3 else { continue }
            let d = simd_length(q - e.point)
            if d < best[side] { best[side] = d; normal[side] = SIMD3<Double>(m.normals[i].xyz) }
        }
        guard best.allSatisfy(\.isFinite), simd_length(normal[0]) > 0, simd_length(normal[1]) > 0 else { return false }
        return simd_dot(simd_normalize(normal[0]), simd_normalize(normal[1])) > cos(2 * Double.pi / 180)
    }

    // A layer's edges (one edge pick each, on the shape before it) but the ones given.
    func rest(of layer: Node, without edited: [Pick]) -> [Pick] {
        guard let inner = layer.inner, let s = shape(inner), let picks = layer.picks else { return [] }
        let (kinds, data) = Self.flat(picks)
        let n = Int(s.with { bk_pick_edges($0, kinds, data, Int32(picks.count), nil, 0) })
        guard n > 0 else { return [] }
        var out = [Double](repeating: 0, count: 6 * n)
        _ = s.with { bk_pick_edges($0, kinds, data, Int32(picks.count), &out, Int32(n)) }
        var edges = (0..<n).map { i in Pick(kind: Int32(BK_PICK_EDGE), a: SIMD3(out[6 * i], out[6 * i + 1], out[6 * i + 2]), b: SIMD3(out[6 * i + 3], out[6 * i + 4], out[6 * i + 5])) }
        for e in edited {
            if let i = edges.indices.min(by: { simd_length(edges[$0].a - e.a) < simd_length(edges[$1].a - e.a) }) { edges.remove(at: i) }
        }
        return edges
    }

    // The nearest triangle of a mesh to p: how far, which way it faces, and the point on it.
    static func nearest(_ m: Mesh, _ p: SIMD3<Double>) -> (distance: Double, normal: SIMD3<Double>, point: SIMD3<Double>)? {
        var best = Double.infinity, normal = SIMD3<Double>(0, 0, 1), point = p
        var t = 0
        while t + 2 < m.indices.count {
            let a = SIMD3<Double>(m.vertices[Int(m.indices[t])].xyz), b = SIMD3<Double>(m.vertices[Int(m.indices[t + 1])].xyz),
                c = SIMD3<Double>(m.vertices[Int(m.indices[t + 2])].xyz)
            let q = Self.closest(p, a, b, c)
            let d = simd_length(p - q)
            if d < best {
                best = d
                point = q
                let n = simd_cross(b - a, c - a)
                if simd_length(n) > 0 { normal = simd_normalize(n) }
            }
            t += 3
        }
        return best.isFinite ? (best, normal, point) : nil
    }

    // The point of triangle abc nearest p.
    static func closest(_ p: SIMD3<Double>, _ a: SIMD3<Double>, _ b: SIMD3<Double>, _ c: SIMD3<Double>) -> SIMD3<Double> {
        let ab = b - a, ac = c - a, ap = p - a
        let d1 = simd_dot(ab, ap), d2 = simd_dot(ac, ap)
        if d1 <= 0 && d2 <= 0 { return a }
        let bp = p - b, d3 = simd_dot(ab, bp), d4 = simd_dot(ac, bp)
        if d3 >= 0 && d4 <= d3 { return b }
        let vc = d1 * d4 - d3 * d2
        if vc <= 0 && d1 >= 0 && d3 <= 0 { return a + ab * (d1 / (d1 - d3)) }
        let cp = p - c, d5 = simd_dot(ab, cp), d6 = simd_dot(ac, cp)
        if d6 >= 0 && d5 <= d6 { return c }
        let vb = d5 * d2 - d1 * d6
        if vb <= 0 && d2 >= 0 && d6 <= 0 { return a + ac * (d2 / (d2 - d6)) }
        let va = d3 * d6 - d5 * d4
        if va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0 { return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6))) }
        let den = 1 / (va + vb + vc)
        return a + ab * (vb * den) + ac * (vc * den)
    }

    // The point of an edge's line nearest p.
    static func onEdge(_ e: [SIMD3<Float>], _ p: SIMD3<Double>) -> SIMD3<Double> {
        var best = Double.infinity, point = p
        for k in 1..<e.count {
            let a = SIMD3<Double>(e[k - 1]), b = SIMD3<Double>(e[k]), ab = b - a
            let t = simd_dot(ab, ab) > 0 ? max(0, min(1, simd_dot(p - a, ab) / simd_dot(ab, ab))) : 0
            let q = a + ab * t
            if simd_length(p - q) < best { best = simd_length(p - q); point = q }
        }
        return point
    }

    // The mesh's edge between two faces nearest p, as an edge pick.
    static func nearestEdge(_ m: Mesh, _ p: SIMD3<Double>) -> Pick? { nearestEdgeAt(m, p)?.pick }

    // The same, with which edge it is, how far and its point nearest p; with `through`, only edges passing within 0.04 of
    // one of those points.
    static func nearestEdgeAt(_ m: Mesh, _ p: SIMD3<Double>, through: [SIMD3<Double>]? = nil)
        -> (index: Int, distance: Double, point: SIMD3<Double>, pick: Pick)? {
        var best = Double.infinity, hit = -1, point = p
        for (i, e) in m.edges.enumerated() where e.count > 1 && i < m.edgeFaces.count {
            let f = m.edgeFaces[i]
            guard f.x >= 0, f.y >= 0, f.x != f.y else { continue }
            if let through, !through.contains(where: { simd_length(onEdge(e, $0) - $0) < 0.04 }) { continue }
            let q = onEdge(e, p), d = simd_length(p - q)
            if d < best { best = d; hit = i; point = q }
        }
        guard hit >= 0 else { return nil }
        let (mid, dir) = Picking.midpoint(m.edges[hit])
        return (hit, best, point, Pick(kind: Int32(BK_PICK_EDGE), a: SIMD3<Double>(mid), b: SIMD3<Double>(dir)))
    }

    // The shape cut across a picked edge, for the 2D angle editor.
    func section(_ node: Node, _ pick: Pick) -> Section? {
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

    func mesh(_ node: Node, deflection: Double = 0.05, keep: Bool = true) -> Mesh? {
        guard let s = shape(node, keep: keep), let m = s.with({ bk_mesh($0, deflection) }) else { return nil }
        defer { bk_mesh_free(m) }
        let mesh = Mesh(m)
        // A shape reaching past 100 m (or nowhere) is broken geometry; shown, it would throw the view out.
        guard mesh.vertices.isEmpty || [mesh.low, mesh.high].allSatisfy({ $0.finite && simd_reduce_max(simd_abs($0)) < 1e5 }) else {
            problems.append("bounds")
            return nil
        }
        return mesh
    }

    // The box a shape fills placed as given (turned or stretched any way); `exact` false where it's only as close as its
    // mesh.
    func bounds(_ node: Node, _ place: Placement) -> (low: SIMD3<Double>, high: SIMD3<Double>, exact: Bool)? {
        guard let s = shape(node) else { return nil }
        var out = [Double](repeating: 0, count: 6)
        let m = place.kernel
        let r = s.with { sp in m.withUnsafeBufferPointer { bk_bounds(sp, $0.baseAddress, &out) } }
        guard r >= 0, out.allSatisfy(\.isFinite) else { return nil }
        return (SIMD3(out[0], out[1], out[2]), SIMD3(out[3], out[4], out[5]), r == 1)
    }

    // World-space shape of a body (for export).
    func placed(_ b: Solid) -> ShapeRef? {
        guard let s = shape(b.node) else { return nil }
        return s.with { sp in b.place.kernel.withUnsafeBufferPointer { bk_transform(sp, $0.baseAddress) } }.map(ShapeRef.init)
    }

    // The shortest distance between two measurement ends.
    func distance(_ a: GapEnd, _ b: GapEnd) -> Gap? {
        // An end on a shape that can't be built has nothing to measure from.
        let ra = a.node.map { shape($0) }, rb = b.node.map { shape($0) }
        if case .some(.none) = ra { return nil }
        if case .some(.none) = rb { return nil }
        func with<T>(_ r: ShapeRef??, _ f: (OpaquePointer?) -> T) -> T { if let r = r ?? nil { r.with { f($0) } } else { f(nil) } }
        var out = [Double](repeating: 0, count: 6)
        let d = with(ra) { pa in
            with(rb) { pb in
                bk_distance(pa, a.place, a.kind, a.index, a.point, pb, b.place, b.kind, b.index, b.point, &out)
            }
        }
        guard d >= 0, d.isFinite else { return nil }
        return Gap(distance: d, a: SIMD3(out[0], out[1], out[2]), b: SIMD3(out[3], out[4], out[5]))
    }

    // A body's fine mesh in world coordinates, for files.
    func worldMesh(_ b: Solid, deflection: Double = 0.01) -> Mesh? {
        guard let s = placed(b), let m = s.with({ bk_mesh($0, deflection) }) else { return nil }
        defer { bk_mesh_free(m) }
        return Mesh(m)
    }

    // A STEP file of the bodies, each a solid under its own name.
    func exportStep(_ bodies: [Solid], to path: String) -> Bool {
        let shapes = bodies.compactMap { placed($0) }
        guard shapes.count == bodies.count else { return false }
        let names = bodies.map { strdup($0.name) }
        defer { names.forEach { free($0) } }
        return withExtendedLifetime(shapes) {
            let ptrs: [OpaquePointer?] = shapes.map { $0.with { $0 } }
            let named: [UnsafePointer<CChar>?] = names.map { $0.map { UnsafePointer($0) } }
            return ptrs.withUnsafeBufferPointer { p in
                named.withUnsafeBufferPointer { n in bk_export_step(p.baseAddress, n.baseAddress, Int32(ptrs.count), path) }
            } != 0
        }
    }
}

// MARK: - Settings

enum Action: String, CaseIterable, Codable {
    // Angles was once a separate rounding tool's key: settings saved then know it by that name.
    case move, rotate, scale, angles = "round", split, hollow, measure, drop, frame, hide, showAll

    var name: String {
        switch self {
        case .move: "Move"
        case .rotate: "Rotate"
        case .scale: "Scale"
        case .angles: "Angles"
        case .split: "Split"
        case .hollow: "Hollow"
        case .measure: "Measure"
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
        "move": "KeyG", "rotate": "KeyT", "scale": "KeyY", "round": "KeyR", "split": "KeyS", "hollow": "KeyO", "measure": "KeyM", "drop": "KeyB", "frame": "KeyF", "hide": "KeyH", "showAll": "KeyU"
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

enum Mode: Equatable {
    case select, split, hollow, angles, thread, measure

    // A tool with its own bar at the bottom in place of the inspector.
    var isTool: Bool { [.split, .hollow, .measure].contains(self) }
}

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

// An earlier treatment's edges clicked to work on again: the layer (through merges, by each merge's level and the part's
// place in it, then its level among the layers there), the edges clicked (as they were before it, sharp) and its others.
struct Spot: Equatable {
    var path: [Int]
    var level: Int
    var edges: [Pick]
    var rest: [Pick]
    var layer: Node
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
    // Earlier treatments' edges clicked: each made again from its sharp edge (whatever kind it was, it becomes this one).
    var edits: [Spot] = []
    // The picks of edges still sharp: treated anew.
    var fresh: [Pick]

    init(body: UUID, picks: [Pick], section: Section) {
        self.body = body
        self.picks = picks
        self.fresh = picks
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

    // Starting from an earlier treatment's own values.
    mutating func adopt(_ layer: Node) {
        switch layer {
        case .round(_, _, let r): treatment = .rounded; rounding = .outbound; radius = r
        case .cove(_, _, let r): treatment = .rounded; rounding = .inbound; radius = r
        case .bevel(_, _, let l, let c): treatment = .angled; legs = l; roundedCorners = c > 0
        default: break
        }
    }

    // The treatment around a node, for the picked edges or for the whole shape.
    func wrapping(_ node: Node, whole: Bool) -> Node {
        wrapping(node, picks: whole ? [Pick(kind: Int32(BK_PICK_BODY), a: .zero, b: .zero)] : picks)
    }

    func wrapping(_ node: Node, picks p: [Pick]) -> Node {
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

// One end of a measurement: where it is (world, mm) and what it stands for. A corner, a circle's centre or an edge's middle
// is a point; an edge or a face is also measured from as a whole, for the shortest distance between surfaces.
struct MeasureEnd: Equatable {
    enum Snap { case corner, centre, midpoint, edge, face }
    var snap: Snap
    var point: SIMD3<Double>
    var body: UUID?
    var index = -1   // the edge's or face's number in the body's mesh

    @MainActor var name: String {
        switch snap {
        case .corner: L("Corner")
        case .centre: L("Centre")
        case .midpoint: L("Midpoint")
        case .edge: L("Edge")
        case .face: L("Face")
        }
    }
}

// The ruler's numbers: millimetres to two places.
enum Ruler {
    @MainActor static func mm(_ v: Double) -> String { String(format: "%.2f ", v) + L("mm") }
}

// The shortest distance between the edges or faces at a measurement's ends, and the two points it runs between.
struct Gap: Equatable, Sendable {
    var distance: Double
    var a, b: SIMD3<Double>
}

// A measurement's end as the kernel takes it: a point, or an edge or face of a built shape in place.
struct GapEnd: Sendable {
    var kind: Int32
    var index: Int32 = 0
    var node: Node?
    var place = [Double](repeating: 0, count: 12)
    var point: [Double]
}

// Shapes on the clipboard, and the box they filled where they were copied.
struct Clip: Codable {
    static let type = NSPasteboard.PasteboardType("local.bohdan.bcad.shapes")
    var bodies: [Solid]
    var low, high: SIMD3<Double>?
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
    // The name given to a document that isn't saved yet; the Save panel offers it.
    var docName: String?
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
    // Where each pick was clicked on its shape (to tell a face an earlier treatment made).
    var pickPoints: [Pick: SIMD3<Double>] = [:]
    var hollowOpen: [Pick] = []
    var hollowWalls: [Wall] = []
    var hollowThickness = 2.0
    var focusWall: Int?
    var thread = Fastener(kind: .hex, size: 4)
    var splitAxis = 2
    // The ruler: its ends, the end the pointer is on, and the shortest distance between surfaces when an end is one.
    var measureA: MeasureEnd?
    var measureB: MeasureEnd?
    var measureHover: MeasureEnd?
    var gap: Gap?
    var splitOffset = 0.0
    var splitTilt = SIMD2<Double>(0, 0)
    var showSettings = false
    var drawerOpen = true { didSet { glideInset() } }
    // How far the 3D view sits in from the side panel's edge. It follows the panel as it slides (the 3D view itself can't
    // be animated), rather than jumping while the panel is still on its way.
    private(set) var drawerInset: CGFloat = 320
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
    // New shapes being tried before they go into the document.
    @ObservationIgnored private(set) var trying = false
    @ObservationIgnored private var flashTask: Task<Void, Never>?
    @ObservationIgnored private var saveTask: Task<Void, Never>?
    @ObservationIgnored var camera = Camera()
    @ObservationIgnored var requestFit = false
    // Where the inspector and its row of names sit in the window (for two-finger swipes between its screens).
    @ObservationIgnored var inspectorFrame = CGRect.zero
    @ObservationIgnored var namesFrame = CGRect.zero
    // The other panels over the 3D view, which the gizmo's handles keep clear of (window points, from the top left).
    @ObservationIgnored var railFrame = CGRect.zero
    @ObservationIgnored var shapeBarFrame = CGRect.zero
    @ObservationIgnored private var swipe = 0.0
    @ObservationIgnored private var swiped = false
    @ObservationIgnored private var cameraBeforeAngles: Camera?
    // Counts the documents opened or begun here: work begun for one (a save, a build) changes nothing of the next.
    @ObservationIgnored private var generation = 0
    @ObservationIgnored private var flight: Task<Void, Never>?
    @ObservationIgnored private var measureRun = 0
    @ObservationIgnored private var insetGlide: Task<Void, Never>?

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
        Kernel.shared.queue.async {
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
        var place = Placement(move: spawnPoint())
        // A shape whose size is known before it's built (a primitive, a bolt, centred on its own origin) goes straight down
        // onto the bed: nothing waits for its build, and a copy made at once sits where it does.
        let known = settings.dropToBed ? node.extent(clearance: settings.clearance) : nil
        if let e = known { place.move.z = e.z / 2 }
        let body = Solid(name: name, color: nextColor(), node: node, place: place)
        commit { $0.bodies.append(body) }
        selection = [body.id]
        if known == nil { dropSoon([body.id]) }
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

    // Built first, so a bolt or nut that can't be made never enters the document; `then` follows its adding.
    func addFastener(_ f: Fastener, then: (() -> Void)? = nil) {
        tryThen([.fastener(f)]) { [weak self] in
            self?.add(.fastener(f), name: f.name)
            then?()
        }
    }

    // ⌘B: the bolt or nut set up last, added at once, with the Thread tab open to change it.
    func addThread() {
        addFastener(thread) { [weak self] in self?.choose(.thread) }
    }

    @ObservationIgnored private var dropQueue: Set<UUID> = []
    private func dropSoon(_ ids: [UUID]) { if settings.dropToBed { dropQueue.formUnion(ids) } }

    // Where the camera frames these shapes from: their middle, and how far away they fill the view. A shape reaching
    // nowhere sensible (a broken file) would throw the camera out with it; it's left out. Nil when nothing is left.
    func framing(_ bodies: [Solid]) -> (target: SIMD3<Float>, distance: Float)? {
        var lo = SIMD3<Double>(repeating: .infinity), hi = SIMD3<Double>(repeating: -.infinity)
        func sane(_ v: SIMD3<Double>) -> Bool { v.finite && simd_reduce_max(simd_abs(v)) < 1e5 }
        for b in bodies {
            if let (l, h) = worldBounds(b), sane(l), sane(h) { lo = simd_min(lo, l); hi = simd_max(hi, h) }
        }
        guard lo.x.isFinite else { return nil }
        let r = max(10, simd_length(hi - lo) / 2)
        return (SIMD3<Float>((lo + hi) / 2), Float(r / tan(Double(camera.fov) / 2) * 1.25))
    }

    // The whole bed in view, for an empty scene.
    var bedFraming: (target: SIMD3<Float>, distance: Float) { (.zero, Float(max(settings.bed.x, settings.bed.y)) * 2.4) }

    // A turned shape's box as it stands before it's moved: per mesh, turn and stretch (a move only shifts it, so a drag
    // costs nothing). First from every point of its mesh (never past the shape, short of a curved one by the chord error
    // at most), then exactly, from the kernel, asked in the background so a turn never waits for it.
    private struct Stance: Equatable {
        var stamp: Int
        var turn: SIMD3<Double>
        var scale: SIMD3<Double>
    }
    @ObservationIgnored private var turnedBoxes: [UUID: (stance: Stance, lo: SIMD3<Double>, hi: SIMD3<Double>, exact: Bool)] = [:]
    // Boxes asked of the kernel and not yet answered, and the stance wanted next for each (the latest turn).
    @ObservationIgnored private var boxesAsked: Set<UUID> = []
    @ObservationIgnored private var boxesWanted: [UUID: Stance] = [:]

    // World bounding box of a body: the shape's own box when unturned (exact, from the kernel), the turned box otherwise.
    func worldBounds(_ b: Solid) -> (SIMD3<Double>, SIMD3<Double>)? {
        guard let m = meshes[b.id], !m.vertices.isEmpty else { return nil }
        if b.place.turn == SIMD3(0, 0, 0) {
            let a = m.low * b.place.scale + b.place.move, c = m.high * b.place.scale + b.place.move
            return (simd_min(a, c), simd_max(a, c))
        }
        let stance = Stance(stamp: m.stamp, turn: b.place.turn, scale: b.place.scale)
        if let k = turnedBoxes[b.id], k.stance == stance { return (k.lo + b.place.move, k.hi + b.place.move) }
        var still = b.place
        still.move = .zero
        let mat = still.matrix
        var lo = SIMD3<Double>(repeating: .infinity), hi = SIMD3<Double>(repeating: -.infinity)
        for v in m.vertices {
            let w = mat * SIMD4<Double>(Double(v.x), Double(v.y), Double(v.z), 1)
            lo = simd_min(lo, w.xyz)
            hi = simd_max(hi, w.xyz)
        }
        turnedBoxes[b.id] = (stance, lo, hi, false)
        askBox(b.id, stance)
        return (lo + b.place.move, hi + b.place.move)
    }

    // The exact box of a turned shape, from the kernel in the background, one at a time per shape (the latest turn next).
    private func askBox(_ id: UUID, _ stance: Stance) {
        guard !boxesAsked.contains(id) else { boxesWanted[id] = stance; return }
        guard let b = body(id) else { return }
        boxesAsked.insert(id)
        let node = b.node, clearance = settings.clearance
        var still = b.place
        still.move = .zero
        Kernel.shared.queue.async {
            Kernel.shared.clearance = clearance
            let box = Kernel.shared.bounds(node, still)
            DispatchQueue.main.async {
                MainActor.assumeIsolated {
                    self.boxesAsked.remove(id)
                    if let box, let k = self.turnedBoxes[id], k.stance == stance {
                        self.turnedBoxes[id] = (stance, box.low, box.high, true)
                        self.sceneVersion += 1
                    }
                    if let next = self.boxesWanted.removeValue(forKey: id), next != stance, self.turnedBoxes[id]?.stance == next {
                        self.askBox(id, next)
                    }
                }
            }
        }
    }

    // A shape's box exactly, waiting for the kernel when it hasn't said yet: for putting shapes down on the bed.
    func exactBounds(_ b: Solid) -> (SIMD3<Double>, SIMD3<Double>)? {
        guard let box = worldBounds(b) else { return nil }
        if b.place.turn == SIMD3(0, 0, 0) { return box }
        if let k = turnedBoxes[b.id], k.exact, k.stance.turn == b.place.turn, k.stance.scale == b.place.scale { return box }
        let node = b.node, clearance = settings.clearance
        var still = b.place
        still.move = .zero
        let exact = Kernel.shared.queue.sync { () -> (low: SIMD3<Double>, high: SIMD3<Double>, exact: Bool)? in
            Kernel.shared.clearance = clearance
            return Kernel.shared.bounds(node, still)
        }
        guard let exact else { return box }
        return (exact.low + b.place.move, exact.high + b.place.move)
    }

    private func applyDrops() {
        guard !dropQueue.isEmpty else { return }
        for id in dropQueue {
            guard let b = body(id), let (lo, _) = exactBounds(b) else { continue }
            mutate(id) { $0.place.move.z -= lo.z }
        }
        dropQueue.removeAll()
    }

    // MARK: middles

    // A shape's middle: the middle of its own box (as made, before it's turned), where it's placed. It stays put as the
    // shape turns, and it's the shape's Position (for a primitive, simply where it's placed; for a merged or split shape,
    // the middle of what there is).
    func middle(_ b: Solid) -> SIMD3<Double> {
        guard let m = meshes[b.id], !m.vertices.isEmpty else { return b.place.move }
        return (b.place.matrix * SIMD4((m.low + m.high) / 2, 1)).xyz
    }

    // Where the selection turns about: the middle of its shapes' middles, which a turn about it leaves where it is (a box
    // round them would move as uneven shapes turn).
    var turnPivot: SIMD3<Double> {
        let ms = selected.map(middle)
        return ms.isEmpty ? .zero : ms.reduce(.zero, +) / Double(ms.count)
    }

    // A Position typed in: the shape moves so its middle is there.
    func placeMiddle(_ id: UUID, axis i: Int, at v: Double) {
        guard let b = body(id), v.isFinite else { return }
        let d = v - middle(b)[i]
        setPlace(id) { $0.move[i] += d }
    }

    // A turn typed in: the shape turns about its middle, which stays where it is.
    func turn(_ id: UUID, to t: SIMD3<Double>) {
        guard let b = body(id), t.finite else { return }
        let c = middle(b), m = meshes[id].map { ($0.low + $0.high) / 2 } ?? .zero
        setPlace(id) { p in
            p.turn = t
            p.move = c - p.rotation * (p.scale * m)
        }
    }

    func deleteSelection() {
        guard !selection.isEmpty else { return }
        let ids = Set(selection)
        commit { $0.bodies.removeAll { ids.contains($0.id) } }
        selection = []
    }

    func duplicate() {
        let originals = selected
        let copies = originals.map { b -> Solid in
            var c = b
            c.id = UUID()
            c.name = b.name
            c.link = nil
            c.place.move.x += max(10, settings.snap * 10)
            return c
        }
        guard !copies.isEmpty else { return }
        // A copy is the same shape: shown at once from its original's mesh (nothing to build), and still to go down onto
        // the bed if its original is.
        for (b, c) in zip(originals, copies) {
            if let m = meshes[b.id], built[b.id] == b.node { meshes[c.id] = m; built[c.id] = c.node }
            if dropQueue.contains(b.id) { dropQueue.insert(c.id) }
        }
        commit { $0.bodies.append(contentsOf: copies) }
        selection = copies.map(\.id)
    }

    func selectAll() { selection = doc.bodies.filter { !$0.hidden }.map(\.id) }

    // MARK: between files

    // The selected shapes on the clipboard, with the box they fill, for pasting here or into another file.
    @discardableResult func copySelection() -> Bool {
        let picked = selected
        guard !picked.isEmpty else { return false }
        var lo = SIMD3<Double>(repeating: .infinity), hi = -lo
        for b in picked { if let (l, h) = worldBounds(b) { lo = simd_min(lo, l); hi = simd_max(hi, h) } }
        let clip = Clip(bodies: picked, low: lo.x.isFinite ? lo : nil, high: lo.x.isFinite ? hi : nil)
        guard let data = try? JSONEncoder().encode(clip) else { return false }
        let board = NSPasteboard.general
        board.clearContents()
        board.setData(data, forType: Clip.type)
        return true
    }

    func cutSelection() { if copySelection() { deleteSelection() } }

    // Shapes coming in (pasted, added from a file): new ids, and the merges switched off among them their own (a merge's
    // shapes coming in together can be merged again together, never with the ones they were copied from).
    static func afresh(_ bodies: [Solid]) -> [Solid] {
        var links: [UUID: UUID] = [:]
        return bodies.map { b in
            var c = b
            c.id = UUID()
            if let l = b.link {
                let id = links[l.id] ?? UUID()
                links[l.id] = id
                c.link?.id = id
            }
            return c
        }
    }

    func paste() {
        guard let data = NSPasteboard.general.data(forType: Clip.type), let clip = try? JSONDecoder().decode(Clip.self, from: data),
              !clip.bodies.isEmpty, Document(bodies: clip.bodies).valid else { return }
        insert(Self.afresh(clip.bodies), low: clip.low, high: clip.high)
    }

    // Shapes from Bcad files dropped on the window join this document (which stays the one open), each file's beside
    // what is there when they would overlap it. False when nothing could be added.
    @discardableResult func addFiles(_ urls: [URL]) -> Bool {
        var problem: String?
        var added = false
        for url in urls {
            guard url.pathExtension.lowercased() == "3mf" else { problem = L("Only 3MF files made by Bcad can be added"); continue }
            do {
                let (d, shapes) = try ThreeMF.read(url)
                var lo = SIMD3<Double>(repeating: .infinity), hi = -lo
                var looks: [UUID: Mesh] = [:]
                let fresh = Self.afresh(d.bodies)
                let incoming = zip(d.bodies, fresh).map { b, c -> Solid in
                    if let s = shapes[b.id] {
                        for p in s.points { lo = simd_min(lo, SIMD3<Double>(p)); hi = simd_max(hi, SIMD3<Double>(p)) }
                        looks[c.id] = Mesh(saved: s, place: b.place)
                    }
                    return c
                }
                guard !incoming.isEmpty else { continue }
                insert(incoming, low: lo.x.isFinite ? lo : nil, high: lo.x.isFinite ? hi : nil, looks: looks)
                added = true
            } catch FileError.notBcad {
                problem = L("This 3MF wasn't made by Bcad and can't be edited")
            } catch {
                problem = L("This file is damaged and can't be opened")
            }
        }
        if let problem { flash(problem) }
        return added
    }

    // New shapes joining the document where they were, or moved along x beside everything already there when their box
    // would overlap a shape's. They show at once as `looks` (their saved meshes) until they are built, and are selected.
    private func insert(_ shapes: [Solid], low: SIMD3<Double>?, high: SIMD3<Double>?, looks: [UUID: Mesh] = [:]) {
        var shapes = shapes
        if let low, let high {
            var right = -Double.infinity, overlaps = false
            for b in doc.bodies {
                guard let (l, h) = worldBounds(b) else { continue }
                right = max(right, h.x)
                if simd_reduce_max(l - high) < -0.01 && simd_reduce_max(low - h) < -0.01 { overlaps = true }
            }
            if overlaps {
                let dx = right + max(10, settings.snap * 10) - low.x
                for i in shapes.indices { shapes[i].place.move.x += dx }
            }
        }
        if mode != .select { cancelMode() }
        for (id, m) in looks { meshes[id] = m }
        commit { $0.bodies.append(contentsOf: shapes) }
        selection = shapes.filter { !$0.hidden }.map(\.id)
    }

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

    // Any number of shapes at once, touching or apart (apart, they stay apart within it; touching anywhere, they're one).
    // A merge merged again with more shapes takes them in as parts of its own (one merge, every part to edit).
    func combine(_ op: Int32) {
        let parts = selection.compactMap { body($0) }
        guard parts.count >= 2 else { flash(L("Select two or more shapes")); return }
        let first = parts[0]
        let name = op == BK_UNION ? L("Merge") : op == BK_SUBTRACT ? L("Subtract") : L("Intersect")
        let members = parts.flatMap { b -> [Part] in
            if op == BK_UNION, case .group(let inOp, let inner) = b.node, inOp == op {
                return inner.map { p in Part(node: p.node, place: Placement.from(b.place.matrix * p.place.matrix), name: p.name, color: p.color ?? b.color) }
            }
            return [Part(node: b.node, place: b.place, name: b.name, color: b.color)]
        }
        let group = Solid(name: name, color: first.color, node: .group(op: op, parts: members))
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
        guard let b = primary else { flash(L("Select a merged shape")); return }
        unmerge(b.id)
    }

    // A merge switched off: its parts shapes of their own again, to move, resize or treat, each remembering the merge (its
    // layers too: a rounding of the merged shape comes back with it) so it can be switched on again, saved or not.
    func unmerge(_ id: UUID) {
        guard let b = body(id), case .group(let op, let parts) = b.node.base else { flash(L("Select a merged shape")); return }
        let mergeID = UUID()
        let bodies = parts.enumerated().map { i, p in
            Solid(name: p.name ?? L("Part {n}", ["n": i + 1]), color: p.color ?? b.color, node: p.node,
                  place: Placement.from(b.place.matrix * p.place.matrix),
                  link: MergeLink(id: mergeID, op: op, order: i, shell: b.node, place: b.place, name: b.name, color: b.color))
        }
        commit { d in
            let at = d.bodies.firstIndex { $0.id == b.id } ?? d.bodies.count
            d.bodies.removeAll { $0.id == b.id }
            d.bodies.insert(contentsOf: bodies, at: min(at, d.bodies.count))
        }
        selection = bodies.map(\.id)
    }

    // The shapes of a merge switched off (as they are now, edited or not), merged again with the layers it had.
    func remerge(_ id: UUID) {
        guard let link = body(id)?.link else { return }
        let members = doc.bodies.filter { $0.link?.id == link.id }.sorted { ($0.link?.order ?? 0) < ($1.link?.order ?? 0) }
        guard !members.isEmpty else { return }
        // In the merge's own frame, where its layers' picks are.
        let back = link.place.matrix.inverse
        let parts = members.map { m in Part(node: m.node, place: Placement.from(back * m.place.matrix), name: m.name, color: m.color) }
        let node = link.shell.replacingBase { _ in .group(op: link.op, parts: parts) }
        let merged = Solid(name: link.name, color: link.color, node: node, place: link.place)
        let ids = Set(members.map(\.id))
        tryThen([node]) { [weak self] in
            guard let self, members.allSatisfy({ self.body($0.id) == $0 }) else { return }
            self.commit { d in
                let at = d.bodies.firstIndex { ids.contains($0.id) } ?? d.bodies.count
                d.bodies.removeAll { ids.contains($0.id) }
                d.bodies.insert(merged, at: min(at, d.bodies.count))
            }
            self.selection = [merged.id]
        }
    }

    // The shapes of a merge switched off (as many as are left).
    func mergeMembers(_ link: MergeLink) -> Int { doc.bodies.filter { $0.link?.id == link.id }.count }

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

    // The square drawn for the split plane: around the selection's middle, and wide enough that every cut through the
    // selection lies inside it at any tilt and offset (a cut lies within the sphere around the selection's box).
    func splitPatch(_ plane: Plane) -> (centre: SIMD3<Double>, half: Double) {
        var lo = SIMD3<Double>(repeating: .infinity), hi = SIMD3<Double>(repeating: -.infinity)
        for b in selected {
            if let (l, h) = worldBounds(b) { lo = simd_min(lo, l); hi = simd_max(hi, h) }
        }
        guard lo.x.isFinite else { return (plane.point, 60) }
        let m = (lo + hi) / 2
        return (m - plane.normal * simd_dot(m - plane.point, plane.normal), max(60, simd_length(hi - lo) * 0.75))
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

    // After a move or turn: shapes stay where the drag left them, as after a resize.
    func finishTransform() {
        sceneVersion += 1
    }

    // A resize under way: each shape as it began (its node and its own box, which a live resize changes as it goes) and the
    // point of it that stays put (in those coordinates; for several shapes, each one's own origin), and the box round all
    // of them as they began.
    private struct Resize {
        var node: Node
        var low: SIMD3<Double>
        var high: SIMD3<Double>
        var keep = SIMD3<Double>(0, 0, 0)
    }
    @ObservationIgnored private var resizing: [UUID: Resize] = [:]
    // A live resize step found the kernel busy with other work: the next steps wait for it to be free.
    @ObservationIgnored private var resizeStalled = false
    @ObservationIgnored private var resizeBox: (lo: SIMD3<Double>, hi: SIMD3<Double>)?

    // A new resize begins (a handle pressed, a size typed in).
    func beginResize() {
        resizing = [:]
        resizeStalled = false
        resizeBox = nil
    }

    // A resize drag: the shapes in `starts` stretched by f along axis i (all axes when uniform), a lone shape along its own
    // axis, several along the world's, spreading from their shared box as one. The left, front and bottom sides stay put,
    // or the middle when symmetric.
    // low: the low side along axis i moves and the high one stays (a handle on the low side).
    func stretch(_ starts: [UUID: Placement], axis i: Int, by f: Double, uniform: Bool, symmetric: Bool, low: Bool = false) {
        let axes = uniform ? [0, 1, 2] : [i]
        if Set(starts.keys) != Set(resizing.keys) {
            // As it begins: each shape's node and box before the resize changes them, and the box round all of them.
            beginResize()
            var lo = SIMD3<Double>(repeating: .infinity), hi = SIMD3<Double>(repeating: -.infinity)
            for (id, s) in starts {
                guard var b = body(id) else { continue }
                let m = meshes[id]
                resizing[id] = Resize(node: b.node, low: m?.low ?? .zero, high: m?.high ?? .zero)
                b.place = s
                if let (l, h) = worldBounds(b) { lo = simd_min(lo, l); hi = simd_max(hi, h) }
            }
            resizeBox = lo.x.isFinite ? (lo, hi) : nil
        }
        // Each shape's new stretch, and where in the world the point of it that stays put is.
        var plan: [(id: UUID, start: Placement, scale: SIMD3<Double>, at: SIMD3<Double>)] = []
        if starts.count == 1, let (id, s) = starts.first, var r = resizing[id] {
            var keep = (r.low + r.high) / 2
            if !symmetric { for k in axes { keep[k] = low && k == i ? r.high[k] : r.low[k] } }
            r.keep = keep
            resizing[id] = r
            var scale = s.scale
            for k in axes { scale[k] *= f }
            plan.append((id, s, scale, s.move + s.rotation * (s.scale * keep)))
        } else if let box = resizeBox {
            var pivot = symmetric ? (box.lo + box.hi) / 2 : box.lo
            if low, !symmetric { pivot[i] = box.hi[i] }
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
                // Each one's own origin goes where the shared stretch takes it.
                plan.append((id, s, scale, pivot + d * (s.move - pivot)))
            }
        }
        var live: [(UUID, Node)] = []
        for p in plan {
            guard let r = resizing[p.id] else { continue }
            // A primitive builds in far less than a frame: a resize shows the very shape it makes as it goes (a cylinder grown
            // oval, a sphere kept round), not a stretched picture of the old one.
            if case .primitive = r.node, let next = baked(r.node, by: p.scale) {
                // Its sizes taken in at once; the kept point where it was, on the base as it really grew.
                let node = followed(r.node, to: next), k = grown(r.node, to: next)
                mutate(p.id) { $0.node = node; $0.place.scale = SIMD3(1, 1, 1); $0.place.move = p.at - p.start.rotation * (r.keep * k) }
                live.append((p.id, node))
            } else {
                mutate(p.id) { $0.node = r.node; $0.place.scale = p.scale; $0.place.move = p.at - p.start.rotation * (p.scale * r.keep) }
            }
        }
        // While the kernel is busy with other work, the shape shown catches up at a later step or as the resize ends, rather
        // than the window waiting on it.
        if !live.isEmpty, !resizeStalled || Kernel.shared.queue.idle { resizeStalled = !buildNow(live) }
        sceneVersion += 1
    }

    // Shapes built at once and shown, waiting for the kernel: a resize, where that takes far less than a frame. What
    // building them says is said when the resize ends, if it still holds.
    // False when the kernel was busy with other work too long to wait for.
    @discardableResult private func buildNow(_ shapes: [(UUID, Node)]) -> Bool {
        let clearance = settings.clearance
        let made = Kernel.shared.queue.sync(within: 0.1) { () -> [(UUID, Node, Mesh?)] in
            Kernel.shared.clearance = clearance
            let out = shapes.map { ($0.0, $0.1, Kernel.shared.mesh($0.1, keep: false)) }
            _ = Kernel.shared.takeProblems()
            return out
        }
        guard let made else { return false }
        for (id, node, mesh) in made {
            guard let mesh else { continue }
            meshes[id] = mesh
            built[id] = node
        }
        return true
    }

    // A body's base with a stretch taken into its sizes: a primitive's (kept round or made oval, see Primitive.scale) or a
    // bolt's length; nil for a shape that keeps its stretch (a merged one).
    private func baked(_ node: Node, by s: SIMD3<Double>) -> Node? {
        switch node.base {
        case .primitive(var p):
            p.scale(by: s)
            return .primitive(p)
        case .fastener(let f):
            return .fastener(f.setting(.length, max(1, (f.length * s.z * 100).rounded() / 100)))
        default:
            return nil
        }
    }

    // How much a base grew along each of its own axes taking new sizes (1 where that can't be told).
    private func grown(_ node: Node, to next: Node) -> SIMD3<Double> {
        let c = settings.clearance
        guard let e0 = node.base.extent(clearance: c), let e1 = next.extent(clearance: c), e0.min() > 0 else { return SIMD3(1, 1, 1) }
        return e1 / e0
    }

    // After a resize: a primitive or a fastener (rounded, split or hollowed too) takes the stretch into its sizes, so roundings
    // keep their radius and threads never distort. The point that stayed put in the resize stays put however the sizes came
    // out (rounded to 0.01 mm, a sphere kept round): where the stretch left it, on the base as it really grew. Nothing
    // drops to the bed.
    func finishScale() {
        for id in selection {
            guard let b = body(id), b.place.scale != SIMD3(1, 1, 1), let next = baked(b.node, by: b.place.scale) else { continue }
            let keep = resizing[id]?.keep ?? (meshes[id].map { ($0.low + $0.high) / 2 } ?? .zero)
            let at = b.place.move + b.place.rotation * (b.place.scale * keep), k = grown(b.node, to: next)
            mutate(id) { $0.node = followed($0.node, to: next); $0.place.scale = SIMD3(1, 1, 1); $0.place.move = at - $0.place.rotation * (keep * k) }
        }
        beginResize()
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

    // The hollow a body already has, beneath later roundings too: the Hollow tool changes it rather than hollowing the
    // hollow shape again (which never fits).
    func existingHollow(_ b: Solid) -> (level: Int, open: [Pick], walls: [Wall], thickness: Double)? {
        for (level, n) in stack(b).enumerated() {
            if case .hollow(_, let open, let walls, let thickness) = n { return (level, open, walls, thickness) }
        }
        return nil
    }

    // The body the Hollow tool works on, starting from the openings, walls and thickness of the hollow it already has.
    func loadHollow(_ id: UUID?) {
        editBody = id
        focusWall = nil
        if let id, let b = body(id), let h = existingHollow(b) {
            hollowOpen = h.open
            hollowWalls = h.walls
            hollowThickness = h.thickness
        } else {
            hollowOpen = []
            hollowWalls = []
        }
    }

    // Picks of one face taken from different meshes of it (before and after hollowing) differ by a hair.
    private func sameFace(_ p: Pick, _ q: Pick) -> Bool { simd_dot(p.a, q.a) > 0.99 && simd_length(p.b - q.b) < 0.3 }

    // A click on a face: opens it, or closes it again; ⌥ gives it its own wall instead.
    func pickHollowFace(_ face: Pick, ownWall: Bool) {
        if ownWall {
            hollowOpen.removeAll { sameFace($0, face) }
            if let i = hollowWalls.firstIndex(where: { sameFace($0.face, face) }) {
                focusWall = i
            } else {
                hollowWalls.append(Wall(face: face, thickness: hollowThickness))
                focusWall = hollowWalls.count - 1
            }
        } else {
            hollowWalls.removeAll { sameFace($0.face, face) }
            focusWall = nil
            if let i = hollowOpen.firstIndex(where: { sameFace($0, face) }) { hollowOpen.remove(at: i) } else { hollowOpen.append(face) }
        }
    }

    func removeWall(_ i: Int) {
        guard hollowWalls.indices.contains(i) else { return }
        hollowWalls.remove(at: i)
        focusWall = nil
    }

    func commitHollow() {
        guard let id = editBody, let b = body(id) else { flash(L("Select a shape to hollow")); return }
        let open = hollowOpen, walls = hollowWalls, thickness = hollowThickness
        let node = existingHollow(b).map { h in
            rewrite(b.node, level: h.level) { n in
                guard case .hollow(let of, _, _, _) = n else { return n }
                return .hollow(of: of, open: open, walls: walls, thickness: thickness)
            }
        } ?? .hollow(of: b.node, open: open, walls: walls, thickness: thickness)
        // The hollow left as it was: the tool just closes.
        guard node != b.node else { leaveHollow(); return }
        tryThen([node]) { [weak self] in
            guard let self, self.body(id)?.node == b.node else { return }
            self.commit { d in
                if let i = d.bodies.firstIndex(where: { $0.id == id }) { d.bodies[i].node = node }
            }
            self.leaveHollow()
        }
    }

    private func leaveHollow() {
        withAnimation(Neon.spring) {
            hollowOpen = []
            hollowWalls = []
            focusWall = nil
            mode = .select
        }
    }

    // MARK: bed

    // Puts the selection (or every visible shape) down on the bed.
    func dropToBed() {
        let ids = selection.isEmpty ? doc.bodies.filter { !$0.hidden }.map(\.id) : selection
        let moves = ids.compactMap { id -> (UUID, Double)? in
            guard let b = body(id), let (lo, _) = exactBounds(b), abs(lo.z) > 0.000_1 else { return nil }
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
        beginResize()
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
        // The current screen again does nothing, unless a tool (split, hollow, the ruler) has the inspector hidden.
        guard s != screen || mode.isTool else { return }
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
    // A pick on a face an earlier rounding, bevel or inward rounding made works on that treatment again, from the sharp edge
    // it was made on: the cut is taken there, the editor starts from its values, and applying replaces it.
    func workWithAngles() {
        guard let id = editBody, let b = body(id), let first = edgePicks.first, !angleOpening else { return }
        angleOpening = true
        let node = b.node, picks = edgePicks, points = pickPoints, clearance = settings.clearance
        Kernel.shared.queue.async {
            let k = Kernel.shared
            k.clearance = clearance
            // Clicks on treated faces: the layers they lead to, each layer's clicked edges together.
            var spots: [Spot] = [], fresh: [Pick] = []
            for pk in picks {
                // An edge clicked by its line, a face by the point clicked.
                let edge = pk.kind == Int32(BK_PICK_EDGE)
                guard let at = edge ? pk.a : points[pk], let s = k.treatedAt(node, at, edge: edge) else { fresh.append(pk); continue }
                if let i = spots.firstIndex(where: { $0.path == s.path && $0.level == s.level }) { spots[i].edges += s.edges } else { spots.append(s) }
            }
            for i in spots.indices { spots[i].rest = k.rest(of: spots[i].layer, without: spots[i].edges) }
            // The cut: across the first clicked edge, as it was (sharp) when it's an earlier treatment's.
            let sharp = fresh.first != first ? spots.first : nil
            // A smooth seam no treatment made has no corner to cut across.
            let smooth = sharp == nil && first.kind == Int32(BK_PICK_EDGE) && k.smooth(node, first)
            let section = smooth ? nil : sharp.flatMap { s in s.layer.inner.flatMap { k.section($0, s.edges[0]) } } ?? (sharp == nil ? k.section(node, first) : nil)
            let frame = sharp.map { Self.frame(node, $0.path) } ?? matrix_identity_double4x4
            let edits = spots, sharpPicks = fresh
            DispatchQueue.main.async {
                MainActor.assumeIsolated {
                    guard self.editBody == id, self.edgePicks == picks, self.body(id)?.node == node else {
                        self.angleOpening = false
                        return
                    }
                    guard let section, self.mode == .angles, let b = self.body(id) else {
                        self.angleOpening = false
                        if section == nil { self.flash(L(smooth ? "This edge is smooth: there's no corner to work on" : "This edge can't be shown in a cut")) }
                        return
                    }
                    let m = b.place.matrix * frame
                    let at = m * SIMD4(section.point, 1)
                    let along = simd_normalize((m * SIMD4(section.direction, 0)).xyz)
                    self.cameraBeforeAngles = self.camera
                    self.fly(to: SIMD3<Float>(at.xyz), looking: SIMD3<Float>(along), distance: 60, cancelled: {
                        // Another flight took over (a view picked meanwhile): the editor doesn't open, and can be opened again.
                        self.angleOpening = false
                        self.cameraBeforeAngles = nil
                    }) {
                        var e = AngleEdit(body: id, picks: picks, section: section)
                        e.edits = edits
                        e.fresh = sharpPicks
                        if let l = edits.first?.layer { e.adopt(l) }
                        withAnimation(.spring(response: 0.55, dampingFraction: 0.86)) { self.angleEdit = e }
                        self.angleOpening = false
                    }
                }
            }
        }
    }

    // The placement from a part reached through merges (each merge's level, then the part's place in it) to its shape.
    nonisolated static func frame(_ node: Node, _ path: [Int]) -> simd_double4x4 {
        var m = matrix_identity_double4x4, n = node, i = 0
        while i + 1 < path.count {
            var g: Node? = n
            for _ in 0..<path[i] { g = g?.inner }
            guard case .group(_, let parts) = g, parts.indices.contains(path[i + 1]) else { break }
            m = m * parts[path[i + 1]].place.matrix
            n = parts[path[i + 1]].node
            i += 2
        }
        return m
    }

    private func glideInset() {
        insetGlide?.cancel()
        let from = drawerInset, to: CGFloat = drawerOpen ? 320 : 0
        guard from != to else { return }
        guard !Neon.calm else { drawerInset = to; return }
        insetGlide = Task { [weak self] in
            for i in 1...32 {
                try? await Task.sleep(for: .milliseconds(16))
                guard let self, !Task.isCancelled else { return }
                let t = CGFloat(i) / 32
                self.drawerInset = from + (to - from) * (1 - pow(1 - t, 3))
            }
        }
    }

    // Glides round to look straight along an axis at the selection, or at every shape when nothing is selected.
    func look(from side: Side) {
        let f = framing(selection.isEmpty ? doc.bodies.filter { !$0.hidden } : selected) ?? bedFraming
        fly(to: f.target, yaw: side.yaw, pitch: side.pitch, distance: f.distance)
    }

    func closeAngles() {
        guard angleEdit != nil else { return }
        withAnimation(.spring(response: 0.5, dampingFraction: 0.88)) { angleEdit = nil }
        if let c = cameraBeforeAngles { fly(to: c.target, yaw: c.yaw, pitch: c.pitch, distance: c.distance) }
        cameraBeforeAngles = nil
    }

    // Applies the editor's rounding or bevel to the picked edges, to the whole shape, or to every selected shape. Edges an
    // earlier treatment made faces along are treated anew from their sharp edges, in its place; "all edges" means every
    // edge, earlier treatments' too.
    func applyAngles(whole: Bool = false, everyShape: Bool = false) {
        guard let e = angleEdit else { return }
        let all = whole || everyShape
        let made = (everyShape ? selected : body(e.body).map { [$0] } ?? []).map { b in
            (id: b.id, was: b.node, node: all ? everyEdge(b.node, e) : treated(b.node, e))
        }
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

    // The editor's treatment of the picked edges: each earlier layer clicked made again (its other edges kept as they were,
    // beneath), the edges still sharp treated on top.
    func treated(_ node: Node, _ e: AngleEdit) -> Node {
        var n = node
        // Deepest first: a layer made two leaves the levels above it as they were.
        for s in e.edits.sorted(by: { ($0.path.count, $0.level) > ($1.path.count, $1.level) }) {
            n = rewriting(n, path: s.path, level: s.level) { layer in
                guard let of = layer.inner else { return layer }
                return e.wrapping(s.rest.isEmpty ? of : layer.withPicks(s.rest, of: of), picks: s.edges)
            }
        }
        if !e.fresh.isEmpty { n = e.wrapping(n, picks: e.fresh) }
        return n
    }

    // Every edge treated: each run of treatments one over another (between hollows and cuts) made this one, of every
    // edge, in their place; with none, on top. Merged parts' own treatments are left out (this one takes their edges).
    func everyEdge(_ node: Node, _ e: AngleEdit) -> Node {
        var layers: [Node] = []
        var n: Node? = node
        while let c = n { layers.append(c); n = c.inner }
        guard var result = layers.last?.untreated else { return node }
        var pending = false, any = false
        for layer in layers.dropLast().reversed() {
            switch layer {
            case .round, .cove, .bevel:
                pending = true
                any = true
            default:
                if pending { result = e.wrapping(result, whole: true); pending = false }
                result = layer.wrapping(result)
            }
        }
        if pending || !any { result = e.wrapping(result, whole: true) }
        return result
    }

    // A layer rewritten, inside the merges along `path` (each merge's level and the part's place in it).
    private func rewriting(_ node: Node, path: [Int], level: Int, _ f: (Node) -> Node) -> Node {
        guard path.count >= 2 else { return rewrite(node, level: level) { f($0) } }
        return rewrite(node, level: path[0]) { g in
            guard case .group(let op, var parts) = g, parts.indices.contains(path[1]) else { return g }
            parts[path[1]].node = rewriting(parts[path[1]].node, path: Array(path.dropFirst(2)), level: level, f)
            return .group(op: op, parts: parts)
        }
    }

    func updateAngles(_ change: (inout AngleEdit) -> Void) {
        guard var e = angleEdit else { return }
        change(&e)
        withAnimation(.spring(response: 0.36, dampingFraction: 0.84)) { angleEdit = e }
    }

    // Glides the camera to a new view over half a second (eased), redrawing each frame.
    private func fly(to target: SIMD3<Float>, looking dir: SIMD3<Float>, distance: Float, cancelled: (() -> Void)? = nil, done: @escaping () -> Void) {
        let v = simd_length(dir) > 0 ? -simd_normalize(dir) : SIMD3<Float>(0, -1, 0)
        let facing = simd_dot(v, camera.eye - camera.target) < 0 ? -v : v
        let pitch = asin(max(-0.999, min(0.999, facing.z)))
        let yaw = atan2(facing.x, -facing.y)
        fly(to: target, yaw: yaw, pitch: max(-1.55, min(1.55, pitch)), distance: distance, cancelled: cancelled, done: done)
    }

    private func fly(to target: SIMD3<Float>, yaw: Float, pitch: Float, distance: Float, cancelled: (() -> Void)? = nil, done: (() -> Void)? = nil) {
        flight?.cancel()
        let from = camera
        var turn = yaw - from.yaw
        while turn > .pi { turn -= 2 * .pi }
        while turn < -.pi { turn += 2 * .pi }
        // Timed by the clock, not by frames: a busy moment shortens the flight rather than holding it up.
        let length = Neon.calm ? 0.13 : 0.5, start = Date()
        flight = Task { [weak self] in
            var t: Float = 0
            while t < 1 {
                try? await Task.sleep(for: .milliseconds(16))
                guard let self else { return }
                guard !Task.isCancelled else { cancelled?(); return }
                t = Float(min(1, Date().timeIntervalSince(start) / length))
                let k = t * t * (3 - 2 * t)
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
        withAnimation(Neon.spring) {
            mode = mode == m ? .select : m
            edgePicks = []
            clearMeasure()
            loadHollow(mode == .hollow ? selection.last : nil)
            splitOffset = 0
            splitTilt = .zero
        }
    }

    func cancelMode() {
        if angleEdit != nil { closeAngles(); return }
        if mode == .measure, measureA != nil { clearMeasure(); return }
        withAnimation(Neon.spring) {
            if mode != .select { mode = .select } else { selection = [] }
            edgePicks = []
            clearMeasure()
            hollowOpen = []
            hollowWalls = []
            focusWall = nil
        }
    }

    // MARK: ruler

    // Sets the first end, then the second; a further click starts again.
    func measure(_ end: MeasureEnd) {
        measureRun += 1
        gap = nil
        guard let a = measureA, measureB == nil else { measureA = end; measureB = nil; return }
        measureB = end
        let ga = gapEnd(a), gb = gapEnd(end)
        // Two points need no more than their distance, and an edge or face measured to itself has none.
        guard ga.kind != Int32(BK_END_POINT) || gb.kind != Int32(BK_END_POINT),
              !(ga.kind == gb.kind && a.body == end.body && a.index == end.index) else { return }
        let run = measureRun
        Kernel.shared.queue.async {
            let g = Kernel.shared.distance(ga, gb)
            DispatchQueue.main.async {
                MainActor.assumeIsolated {
                    guard run == self.measureRun, let g else { return }
                    self.gap = g
                }
            }
        }
    }

    // The gap, when it says more than the distance between the ends themselves.
    var shownGap: Gap? {
        guard let g = gap, let a = measureA, let b = measureB, abs(g.distance - simd_length(b.point - a.point)) > 0.005 else { return nil }
        return g
    }

    func clearMeasure() {
        measureRun += 1
        if measureA != nil { measureA = nil }
        if measureB != nil { measureB = nil }
        if gap != nil { gap = nil }
    }

    // What the kernel measures from: the edge or face itself while its body's exact mesh is shown, otherwise the point.
    private func gapEnd(_ e: MeasureEnd) -> GapEnd {
        let point = [e.point.x, e.point.y, e.point.z]
        guard let id = e.body, let b = body(id), let m = meshes[id], e.index >= 0 else { return GapEnd(kind: Int32(BK_END_POINT), point: point) }
        switch e.snap {
        case .edge where e.index < m.edges.count:
            return GapEnd(kind: Int32(BK_END_EDGE), index: Int32(e.index), node: b.node, place: b.place.kernel, point: point)
        case .face where e.index < m.faceInfo.count:
            return GapEnd(kind: Int32(BK_END_FACE), index: Int32(e.index), node: b.node, place: b.place.kernel, point: point)
        default:
            return GapEnd(kind: Int32(BK_END_POINT), point: point)
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
        turnedBoxes = turnedBoxes.filter { alive.contains($0.key) }
        sceneVersion += 1
        guard !todo.isEmpty else { applyDrops(); return }
        building = true
        builtClearance = clearance
        let slow = todo.count > 1 || todo.contains { if case .fastener = $0.node.base { true } else { false } }
        let note = L("Building…"), generation = self.generation
        if slow { busy = note }
        Kernel.shared.queue.async {
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
                        // A shape of a document since closed isn't shown in the next (it may hold the same shape ids).
                        guard self.generation == generation else { return }
                        self.built[id] = node
                        // A shape the kernel can't build keeps what it showed (its saved look after opening a file).
                        self.meshes[id] = mesh ?? self.meshes[id] ?? Mesh()
                        self.sceneVersion += 1
                    }
                }
            }
            let found = trouble
            DispatchQueue.main.async {
                MainActor.assumeIsolated {
                    self.building = false
                    if slow { self.ended(note) }
                    self.sceneVersion += 1
                    self.applyDrops()
                    // With several shapes built (a file opened), the message says which one.
                    if let found, self.generation == generation { self.report(found.problems, name: todo.count > 1 ? found.name : nil) }
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
        if problems.contains(where: { $0.hasPrefix("bolt") || $0.hasPrefix("nut") }) { return L("These sizes don't fit this bolt or nut") }
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
            if mode == .hollow { commitHollow(); return true }
            return false
        case "Backspace":
            deleteSelection(); return true
        case "ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown", "PageUp", "PageDown":
            nudge(name, big: shift); return true
        case "KeyX" where mode == .split, "KeyZ" where mode == .split:
            withAnimation(Neon.spring) { splitAxis = name == "KeyX" ? 0 : 2 }; return true
        case "KeyA" where mode == .angles && angleEdit == nil:
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
        case .angles: choose(.angles)
        case .split: enter(.split)
        case .hollow: enter(.hollow)
        case .measure: enter(.measure)
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

    var title: String { fileURL?.deletingPathExtension().lastPathComponent ?? docName ?? L("Untitled") }

    // A name typed for the document: a saved one's file is renamed where it is, an unsaved one keeps it for saving.
    func renameDocument(_ name: String) {
        let t = name.trimmingCharacters(in: .whitespacesAndNewlines).replacingOccurrences(of: "/", with: "-").replacingOccurrences(of: ":", with: "-")
        guard !t.isEmpty, t != title else { return }
        guard let url = fileURL else { docName = t; return }
        let to = url.deletingLastPathComponent().appendingPathComponent(t).appendingPathExtension(url.pathExtension)
        // Another file by that name is never replaced; the same file may change only the case of its letters.
        if FileManager.default.fileExists(atPath: to.path), !Self.sameFile(url, to) {
            flash(L("A file named “{name}” already exists", ["name": to.lastPathComponent]))
            return
        }
        guard Darwin.rename(url.path, to.path) == 0 else { flash(L("Couldn't rename the file")); return }
        fileURL = to
    }

    private static func sameFile(_ a: URL, _ b: URL) -> Bool {
        let id = { (u: URL) in (try? u.resourceValues(forKeys: [.fileResourceIdentifierKey]))?.fileResourceIdentifier as? NSObject }
        guard let x = id(a), let y = id(b) else { return false }
        return x.isEqual(y)
    }

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
            self.docName = nil
            self.rebuildScene()
        }
    }

    // Leaves every tool, editor and pending step of the current document behind, before another one comes in.
    private func resetEditing() {
        generation += 1
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
            docName = nil
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
        let doc = self.doc, clearance = settings.clearance, note = L("Saving…"), generation = self.generation
        // What the engine can't build is saved as it's shown.
        let looks = meshes
        withAnimation(Neon.spring) { busy = note }
        Kernel.shared.queue.async {
            Kernel.shared.clearance = clearance
            let shown = { (b: Solid) in looks[b.id].flatMap { $0.vertices.isEmpty ? nil : $0.placed(b.place) } }
            let meshes = doc.bodies.filter { !$0.hidden }.compactMap { b in (Kernel.shared.worldMesh(b) ?? shown(b)).map { (b, $0) } }
            let ok = (try? ThreeMF.write(url, meshes: meshes, doc: doc)) != nil
            DispatchQueue.main.async {
                MainActor.assumeIsolated {
                    self.ended(note)
                    // Another document opened or begun meanwhile stays as it is: the file saved was the one before it.
                    if ok {
                        if self.generation == generation {
                            self.fileURL = url
                            self.docName = nil
                            self.saved = doc
                        }
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

    // A shape may still be being built on the worker as the app quits; everything worth keeping is saved by now, so the app
    // leaves at once rather than tearing down around it.
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
                // In a text field these edit its text; otherwise they work on shapes, between files too.
                Button(L("Cut")) { if !Edits.send(#selector(NSText.cut(_:))) { lib.cutSelection() } }.keyboardShortcut("x")
                Button(L("Copy")) { if !Edits.send(#selector(NSText.copy(_:))) { lib.copySelection() } }.keyboardShortcut("c")
                Button(L("Paste")) { if !Edits.send(#selector(NSText.paste(_:))) { lib.paste() } }.keyboardShortcut("v")
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
                Button(L("Angles")) { lib.choose(.angles) }
                Divider()
                Button(L("Hollow")) { lib.enter(.hollow) }
                Button(L("Measure")) { lib.enter(.measure) }
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
