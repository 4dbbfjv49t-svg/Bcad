// Sculpting a body by hand: the brushes, and the engine's live mesh of the body being shaped (Engine/Sculpt.hpp).
import Foundation
import simd

// The brushes, numbered as the engine knows them (BK_BRUSH_…).
enum SculptBrush: Int, CaseIterable {
    case grab, draw, inflate, smooth, flatten, pinch, crease, detail, clay, layer, blob, scrape, smudge, snakeHook, twist

    // As settings keep it.
    var key: String {
        switch self {
        case .grab: "grab"
        case .draw: "draw"
        case .inflate: "inflate"
        case .smooth: "smooth"
        case .flatten: "flatten"
        case .pinch: "pinch"
        case .crease: "crease"
        case .detail: "detail"
        case .clay: "clay"
        case .layer: "layer"
        case .blob: "blob"
        case .scrape: "scrape"
        case .smudge: "smudge"
        case .snakeHook: "snakeHook"
        case .twist: "twist"
        }
    }

    init?(key: String) {
        guard let b = SculptBrush.allCases.first(where: { $0.key == key }) else { return nil }
        self = b
    }

    @MainActor var label: String {
        switch self {
        case .grab: L("Grab")
        case .draw: L("Draw")
        case .inflate: L("Inflate")
        case .smooth: L("Smooth")
        case .flatten: L("Flatten")
        case .pinch: L("Pinch")
        case .crease: L("Crease")
        case .detail: L("Detail")
        case .clay: L("Clay")
        case .layer: L("Layer")
        case .blob: L("Blob")
        case .scrape: L("Scrape")
        case .smudge: L("Smudge")
        case .snakeHook: L("Snake hook")
        case .twist: L("Twist")
        }
    }

    @MainActor var hint: String {
        switch self {
        case .grab: L("Pulls the surface along with the pointer")
        case .draw: L("Raises the surface · ⌥ carves it")
        case .inflate: L("Swells the surface outwards · ⌥ shrinks it")
        case .smooth: L("Evens out bumps")
        case .flatten: L("Levels the surface")
        case .pinch: L("Draws the surface together · ⌥ spreads it")
        case .crease: L("Cuts a sharp groove · ⌥ makes a ridge")
        case .detail: L("Makes the triangles under it the Detail size · the surface stays where it is")
        case .clay: L("Builds the surface up flat, like adding clay · ⌥ takes it away")
        case .layer: L("Raises the surface to one height however often you go over it · ⌥ lowers it")
        case .blob: L("Pushes out round blobs · ⌥ pulls them in")
        case .scrape: L("Cuts down what stands out · ⌥ fills hollows")
        case .smudge: L("Smears the surface along the stroke")
        case .snakeHook: L("Pulls the surface out after the pointer: horns, tentacles")
        case .twist: L("Turns the surface round the brush's middle · ⌥ the other way")
        }
    }

    // Dragged in a plane facing the view, off the surface (the rest follow the surface under the pointer).
    var drags: Bool { self == .grab || self == .snakeHook }

    // The picker's rows: brushes that add, that shape, that move.
    static let rows: [[SculptBrush]] = [[.draw, .clay, .layer, .inflate, .blob], [.smooth, .flatten, .scrape, .pinch, .crease],
                                        [.grab, .snakeHook, .smudge, .twist, .detail]]
}

// A brush's own settings: its strength (0…1) and its tip: hardness, the part of its radius at full strength, and
// rigidity, how crisp its fade from there (0…1 each); oval, how wide it is across as along (0.05…1), turned by angle from
// the stroke's way; tilt, how far its push leans toward that way (degrees).
struct SculptTip: Codable, Equatable {
    var strength = 0.5
    var hardness = 0.0
    var rigidity = 0.0
    var oval = 1.0
    var angle = 0.0
    var tilt = 0.0

    init() {}

    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        func read(_ k: CodingKeys, _ lo: Double, _ hi: Double, _ otherwise: Double) -> Double {
            guard let v = try? c.decode(Double.self, forKey: k), v.isFinite else { return otherwise }
            return min(hi, max(lo, v))
        }
        strength = read(.strength, 0.01, 1, 0.5)
        hardness = read(.hardness, 0, 1, 0)
        rigidity = read(.rigidity, 0, 1, 0)
        oval = read(.oval, 0.05, 1, 1)
        angle = read(.angle, 0, 180, 0)
        tilt = read(.tilt, -80, 80, 0)
    }
}

// Sculpting as it was left: the brush, its size (1…200: tenths of a millimetre of its radius; none until it's been set,
// then each body gets one to suit it) and each brush's own strength and tip.
struct SculptSettings: Codable, Equatable {
    var brush = SculptBrush.draw.key
    var size: Double?
    var tips: [String: SculptTip] = [:]

    init() {}

    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        brush = (try? c.decode(String.self, forKey: .brush)).flatMap { SculptBrush(key: $0)?.key } ?? SculptBrush.draw.key
        size = (try? c.decode(Double.self, forKey: .size)).flatMap { $0.isFinite ? min(200, max(1, $0.rounded())) : nil }
        tips = ((try? c.decode([String: SculptTip].self, forKey: .tips)) ?? [:]).filter { SculptBrush(key: $0.key) != nil }
    }
}

// The brush where the pointer is (the body's own coordinates): its rim, an oval turned from the stroke's way; its core,
// where it works at full strength, fainter; a tick along its push; each again across every mirror in use (planes
// through `middle`). Each line its points and how strongly it's drawn (0…1).
enum SculptCursor {
    static func lines(at c: SIMD3<Double>, normal: SIMD3<Double>, way: SIMD3<Double>, radius r: Double, tip: SculptTip,
                      mirror: Int, middle: SIMD3<Double> = .zero) -> [(points: [SIMD3<Double>], alpha: Double)] {
        let up = length(normal) > 0.5 ? normalize(normal) : SIMD3<Double>(0, 0, 1)
        var t = way - up * dot(way, up)
        if length(t) < 1e-9 { t = abs(up.z) < 0.9 ? cross(up, SIMD3(0, 0, 1)) : cross(up, SIMD3(1, 0, 0)) }
        t = normalize(t)
        let b = cross(up, t), a = tip.angle * .pi / 180
        let e1 = t * cos(a) + b * sin(a), e2 = b * cos(a) - t * sin(a)
        func outline(_ k: Double) -> [SIMD3<Double>] {
            (0...64).map { i in
                let u = Double(i) / 64 * 2 * .pi
                return c + (e1 * cos(u) + e2 * (sin(u) * tip.oval)) * (r * k)
            }
        }
        let lean = tip.tilt * .pi / 180, push = up * cos(lean) + t * sin(lean)
        var one: [(points: [SIMD3<Double>], alpha: Double)] = [(points: outline(1), alpha: 0.95), (points: [c, c + push * (r * 0.3)], alpha: 0.95)]
        if tip.hardness > 0.02 { one.append((points: outline(tip.hardness), alpha: 0.4)) }
        var all = one
        for k in 1..<8 where (k & ~mirror) == 0 {
            let s = SIMD3<Double>(k & 1 != 0 ? -1 : 1, k & 2 != 0 ? -1 : 1, k & 4 != 0 ? -1 : 1)
            for line in one { all.append((points: line.points.map { middle + ($0 - middle) * s }, alpha: line.alpha * 0.45)) }
        }
        return all
    }
}

// The brush's circle where the pointer is on the surface (in the body's own coordinates).
struct SculptRing: Equatable {
    var at: SIMD3<Double>
    var normal: SIMD3<Double>
}

// A body being sculpted: the engine's mesh of it as it is now (its points and normals change with every stroke; with a
// detail set under the brush its triangles too, points and triangles coming and going in slots), the strokes made on it
// to undo, and which points and triangles changed, in order, for the view to catch up with.
final class SculptSession {
    private let ptr: OpaquePointer
    // The detail it was made at (mm), and its largest side.
    let detail: Double
    let size: Double
    // Where the mirrors' planes meet: the body's own origin where that's within it (a shape, a sculpt), else the middle
    // of its box (a merge, placed away from the origin); a figure's spine (set when it's opened).
    var middle = SIMD3<Double>(0, 0, 0)
    // Changes, in the order they happened: a point's slot, or a triangle's with `triangleMark` added; how many changes
    // came before the first of them.
    static let triangleMark: UInt32 = 1 << 31
    private var log: [UInt32] = []
    private var logStart = 0
    var logEnd: Int { logStart + log.count }

    init?(_ d: SculptData, detail: Double) {
        let made = d.positions.withUnsafeBufferPointer { p in
            d.indices.withUnsafeBufferPointer { i in bk_sculpt_new(p.baseAddress, Int32(d.pointCount), i.baseAddress, Int32(d.triangleCount)) }
        }
        guard let made else { return nil }
        ptr = made
        self.detail = detail
        var lo = SIMD3<Float>(repeating: .infinity)
        var hi = -lo
        var i = 0
        while i + 2 < d.positions.count {
            let q = SIMD3(d.positions[i], d.positions[i + 1], d.positions[i + 2])
            lo = simd_min(lo, q)
            hi = simd_max(hi, q)
            i += 3
        }
        size = d.positions.isEmpty ? 0 : Double(simd_reduce_max(hi - lo))
        if !d.positions.isEmpty && !(all(lo .<= SIMD3<Float>(repeating: 0)) && all(hi .>= SIMD3<Float>(repeating: 0))) {
            middle = SIMD3<Double>((lo + hi) / 2)
        }
    }

    deinit { bk_sculpt_free(ptr) }

    // Slots for points and for triangles (they only grow; a free triangle's corners are all 0), and the triangles in use.
    var vertexCount: Int { Int(bk_sculpt_vertex_count(ptr)) }
    var triangleCount: Int { Int(bk_sculpt_triangle_count(ptr)) }
    var liveTriangles: Int { Int(bk_sculpt_live_triangle_count(ptr)) }

    // 3 floats per point each; 3 point numbers per triangle (read again after every change: they move as they grow).
    var positions: UnsafePointer<Float> { bk_sculpt_positions(ptr) }
    var normals: UnsafePointer<Float> { bk_sculpt_normals(ptr) }
    var indices: UnsafePointer<UInt32> { bk_sculpt_indices(ptr) }

    // Where a ray (the body's own coordinates) first meets the surface, and the surface's normal there.
    func ray(_ o: SIMD3<Double>, _ d: SIMD3<Double>) -> SculptRing? {
        let oo = [o.x, o.y, o.z], dd = [d.x, d.y, d.z]
        var at = [0.0, 0, 0], n = [0.0, 0, 0]
        guard bk_sculpt_ray(ptr, oo, dd, &at, &n) == 1 else { return nil }
        return SculptRing(at: SIMD3(at[0], at[1], at[2]), normal: SIMD3(n[0], n[1], n[2]))
    }

    // The size the strokes begun after make the triangles under them (mm; 0: leaves them as they are).
    func setDetail(_ d: Double) { bk_sculpt_set_detail(ptr, d) }

    // mirror: bits for x, y and z (across those planes through `middle`); across: the way the stroke is taken to go until
    // it moves (for an oval and a tilt).
    func begin(_ b: SculptBrush, at p: SIMD3<Double>, radius: Double, strength: Double, mirror: Int, invert: Bool, tip: SculptTip = SculptTip(),
               across: SIMD3<Double> = .zero) {
        var br = BKBrush()
        br.brush = Int32(b.rawValue)
        br.radius = radius
        br.strength = strength
        br.mirror = Int32(mirror)
        br.invert = invert ? 1 : 0
        br.hardness = tip.hardness
        br.rigidity = tip.rigidity
        br.oval = tip.oval
        br.angle = tip.angle
        br.tilt = tip.tilt
        br.across = (across.x, across.y, across.z)
        br.middle = (middle.x, middle.y, middle.z)
        let a = [p.x, p.y, p.z]
        bk_sculpt_begin_brush(ptr, &br, a)
    }

    // Pressure scales its strength, size its radius (0…1; both 1 for a mouse); a pen held at a slant leans it by tilt
    // (degrees) instead of its tip's.
    func dab(_ p: SIMD3<Double>, pressure: Double, size: Double = 1, tilt: Double? = nil) {
        let a = [p.x, p.y, p.z]
        bk_sculpt_dab_tilted(ptr, a, pressure, size, tilt ?? .nan)
        sync()
    }

    func end() {
        bk_sculpt_end(ptr)
        sync()
    }

    // A stroke undone or done again: false when there was none.
    func undo() -> Bool {
        let done = bk_sculpt_undo(ptr) == 1
        sync()
        return done
    }

    func redo() -> Bool {
        let done = bk_sculpt_redo(ptr) == 1
        sync()
        return done
    }

    private func sync() {
        let n = Int(bk_sculpt_sync(ptr)), m = Int(bk_sculpt_changed_triangle_count(ptr))
        guard n + m > 0 else { return }
        // (Long after, the view takes it all again rather than every change one by one.)
        if log.count + n + m > 2 * (vertexCount + triangleCount) {
            logStart += log.count
            log.removeAll(keepingCapacity: true)
        }
        if n > 0, let c = bk_sculpt_changed(ptr) { log.append(contentsOf: UnsafeBufferPointer(start: c, count: n)) }
        if m > 0, let t = bk_sculpt_changed_triangles(ptr) {
            log.append(contentsOf: UnsafeBufferPointer(start: t, count: m).lazy.map { $0 | Self.triangleMark })
        }
    }

    // What changed since a reader's place among the changes; nil when those are forgotten (it reads all again).
    func changes(since k: Int) -> ArraySlice<UInt32>? {
        guard k >= logStart, k <= logEnd else { return nil }
        return log[(k - logStart)...]
    }

    // Its mesh as the document keeps it: the points in use and the triangles, renumbered.
    func data() -> SculptData {
        guard let c = bk_sculpt_mesh(ptr) else { return SculptData(positions: [], indices: []) }
        defer { bk_sculpt_mesh_free(c) }
        return SculptData(positions: Array(UnsafeBufferPointer(start: c.pointee.positions, count: 3 * Int(c.pointee.vertexCount))),
                          indices: Array(UnsafeBufferPointer(start: c.pointee.indices, count: 3 * Int(c.pointee.triangleCount))))
    }

    // Shown as its body's mesh (one smooth face) until the kernel's own takes its place: from its data, when that's been
    // read already.
    func mesh(from given: SculptData? = nil) -> Mesh { Mesh(sculpt: given ?? data()) }
}
