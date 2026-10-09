import Foundation
import simd

// Sketches: points in a plane, lines, arcs and circles through them, and the rules on them (constraints and dimensions);
// the regions the curves bound, and solids made of chosen regions — stood up (extruded) or turned (revolved). The
// engine solves the rules, finds the regions and makes the solids (BcadKernel.h). (Foundation and simd only: the same on
// the iPhone.)

struct SketchCurve: Codable, Hashable, Sendable {
    enum Kind: Int32, Codable, Sendable {
        case line = 0, arc = 1, circle = 2
    }
    var kind: Kind
    // A line's two ends · an arc's centre, start and end (counter-clockwise) · a circle's centre.
    var points: [Int32]
    // A circle's (an arc's is the distance from its centre to its start).
    var radius = 0.0
    // A construction curve bounds no region; a reference one (a face's edge it was drawn on) stays where it is.
    var construction = false
    var reference = false

    var count: Int { kind == .line ? 2 : kind == .arc ? 3 : 1 }
    var round: Bool { kind != .line }
}

struct SketchRule: Codable, Hashable, Sendable {
    var kind: Int32
    var points: [Int32] = []
    var curves: [Int32] = []
    // A dimension's size (mm, or degrees for an angle).
    var value = 0.0
    // Which of two ways it holds (as found when it was made).
    var side: Int32 = 1
    // Where a dimension's value is shown (sketch coordinates).
    var label = SIMD2<Double>(0, 0)

    var dimension: Bool { kind >= Int32(BK_DIM_DISTANCE) }
}

struct Sketch: Codable, Hashable, Sendable {
    var points: [SIMD2<Double>] = []
    var fixed: [Bool] = []
    var curves: [SketchCurve] = []
    var rules: [SketchRule] = []

    // A new sketch: its origin and the ends of its two axes, held where they are, and the axes themselves (lines to
    // dimension from and to turn round; they bound no region).
    static func start() -> Sketch {
        var s = Sketch()
        s.points = [SIMD2(0, 0), SIMD2(-1, 0), SIMD2(1, 0), SIMD2(0, -1), SIMD2(0, 1)]
        s.fixed = [true, true, true, true, true]
        s.curves = [SketchCurve(kind: .line, points: [1, 2], construction: true, reference: true),
                    SketchCurve(kind: .line, points: [3, 4], construction: true, reference: true)]
        return s
    }
    static let xAxis: Int32 = 0, yAxis: Int32 = 1

    // A point added.
    mutating func point(_ p: SIMD2<Double>) -> Int32 {
        points.append(p)
        fixed.append(false)
        return Int32(points.count - 1)
    }

    // What the user drew: the curves after the two axes.
    var drawn: Int { curves.indices.filter { !curves[$0].reference }.count }
}

// A region chosen to make a solid of: by its sides, and a point that was inside it.
struct RegionRef: Codable, Hashable, Sendable {
    var sides: [Int32]
    var seed: SIMD2<Double>
}

// Stood up from `low` to `high` mm along the sketch's normal, or turned about line `axis` from `low` to `high` degrees.
struct Form: Codable, Hashable, Sendable {
    var kind: Int32 = Int32(BK_FORM_EXTRUDE)
    var low = 0.0
    var high = 10.0
    var axis: Int32 = -1

    var revolve: Bool { kind == Int32(BK_FORM_REVOLVE) }
}

// A sketch and what's made of it: the solid in the sketch's own coordinates, placed by `frame` (row-major 3×4: the
// sketch's x, y and normal, and its origin, in the coordinates of the shape it's on — the world's for a body of its own).
struct Profile: Codable, Hashable, Sendable {
    var sketch: Sketch
    var regions: [RegionRef]
    var form: Form
    var frame: [Double] = Profile.identity

    static let identity: [Double] = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]

    var matrix: simd_double4x4 { Profile.matrix(frame) }

    static func matrix(_ f: [Double]) -> simd_double4x4 {
        guard f.count == 12 else { return matrix_identity_double4x4 }
        return simd_double4x4(columns: (SIMD4(f[0], f[4], f[8], 0), SIMD4(f[1], f[5], f[9], 0), SIMD4(f[2], f[6], f[10], 0), SIMD4(f[3], f[7], f[11], 1)))
    }

    static func frame(_ m: simd_double4x4) -> [Double] {
        (0..<3).flatMap { r in (0..<4).map { c in m[c][r] } }
    }

    // The same, its shape stretched by k along the shape's own axes: the plane moves and turns with it, the sketch
    // keeps its sizes.
    func following(_ k: SIMD3<Double>) -> Profile {
        let m = matrix
        let x = SIMD3(m.columns.0.x, m.columns.0.y, m.columns.0.z), n = SIMD3(m.columns.2.x, m.columns.2.y, m.columns.2.z)
        let o = SIMD3(m.columns.3.x, m.columns.3.y, m.columns.3.z) * k
        let n2 = unit(n / k)
        var x2 = x * k
        x2 = unit(x2 - n2 * dot(n2, x2))
        let y2 = cross(n2, x2)
        var p = self
        p.frame = Profile.frame(simd_double4x4(columns: (SIMD4(x2, 0), SIMD4(y2, 0), SIMD4(n2, 0), SIMD4(o, 1))))
        return p
    }
}

// MARK: - The engine's sketch

// A sketch as the engine takes it (its arrays held while it's called).
struct EngineSketch {
    var points: [Double]
    var fixed: [UInt8]
    var curves: [BKCurve]
    var rules: [BKRule]

    init(_ s: Sketch) {
        points = s.points.flatMap { [$0.x, $0.y] }
        fixed = s.points.indices.map { i in i < s.fixed.count && s.fixed[i] ? 1 : 0 }
        curves = s.curves.map { c in
            let p = c.points + [-1, -1, -1]
            return BKCurve(kind: c.kind.rawValue, p: (p[0], p[1], p[2]), radius: c.radius,
                           flags: (c.construction ? Int32(BK_CURVE_CONSTRUCTION) : 0) | (c.reference ? Int32(BK_CURVE_REFERENCE) : 0))
        }
        rules = s.rules.map { r in
            let p = r.points + [-1, -1, -1], c = r.curves + [-1, -1]
            return BKRule(kind: r.kind, p: (p[0], p[1], p[2]), c: (c[0], c[1]), value: r.value, side: r.side)
        }
    }

    // Calls f with the sketch; the points (and circles' radii) as the engine left them are kept.
    mutating func call<T>(_ f: (UnsafeMutablePointer<BKSketch>) -> T) -> T {
        let np = Int32(points.count / 2), nc = Int32(curves.count), nr = Int32(rules.count)
        var pts = points, cs = curves
        let fx = fixed, rs = rules
        let out = pts.withUnsafeMutableBufferPointer { p in
            fx.withUnsafeBufferPointer { x in
                cs.withUnsafeMutableBufferPointer { c in
                    rs.withUnsafeBufferPointer { r in
                        var s = BKSketch(pointCount: np, curveCount: nc, ruleCount: nr, points: p.baseAddress, fixed: x.baseAddress, curves: c.baseAddress,
                                         rules: r.baseAddress)
                        return f(&s)
                    }
                }
            }
        }
        points = pts
        curves = cs
        return out
    }
}

// What solving a sketch found.
struct Solved: Equatable, Sendable {
    var ok = true
    // How many ways it can still move (-1: not worked out, as while dragging).
    var freedom = 0
    // The first rule that holds nothing the others don't (-1: none); whether it can't hold with them.
    var dependent = -1
    var conflicting = false
    var pointFixed: [Bool] = []
    var curveFixed: [Bool] = []
}

// A region the curves bound, as the engine found it.
struct SketchRegion: Equatable, Sendable {
    var area: Double
    var seed: SIMD2<Double>
    var sides: [Int32]
    var loops: [[SIMD2<Double>]]
    var triangles: [SIMD2<Double>]

    var ref: RegionRef { RegionRef(sides: sides, seed: seed) }
}

extension Sketch {
    // The rules made to hold, the points moved as little as they can (those in `drag` towards `to` first). Unchanged
    // when they can't all hold.
    mutating func solve(drag: [Int32] = [], to: [SIMD2<Double>] = []) -> Solved {
        var e = EngineSketch(self)
        var pf = [UInt8](repeating: 0, count: points.count), cf = [UInt8](repeating: 0, count: curves.count)
        let targets = to.flatMap { [$0.x, $0.y] }
        var report = BKSolveReport()
        let status = pf.withUnsafeMutableBufferPointer { p in
            cf.withUnsafeMutableBufferPointer { c in
                drag.withUnsafeBufferPointer { d in
                    targets.withUnsafeBufferPointer { t in
                        e.call { s -> Int32 in
                            report.pointFixed = p.baseAddress
                            report.curveFixed = c.baseAddress
                            return bk_sketch_solve(s, Int32(drag.count), d.baseAddress, t.baseAddress, &report)
                        }
                    }
                }
            }
        }
        let ok = status == Int32(BK_SOLVE_OK)
        if ok {
            for i in points.indices { points[i] = SIMD2(e.points[2 * i], e.points[2 * i + 1]) }
            for c in curves.indices where curves[c].kind == .circle { curves[c].radius = e.curves[c].radius }
        }
        return Solved(ok: ok, freedom: Int(report.freedom), dependent: Int(report.dependent), conflicting: report.conflicting != 0,
                      pointFixed: pf.map { $0 != 0 }, curveFixed: cf.map { $0 != 0 })
    }

    // The regions the curves bound (construction curves left out), their outlines' arcs as chords within `deflection`.
    func regions(deflection: Double = 0.05) -> [SketchRegion] {
        var e = EngineSketch(self)
        guard let r = e.call({ bk_sketch_regions($0, deflection) }) else { return [] }
        defer { bk_sketch_regions_free(r) }
        let g = r.pointee
        var out: [SketchRegion] = []
        for k in 0..<Int(g.regionCount) {
            let sides = (Int(g.sideStart[k])..<Int(g.sideStart[k + 1])).map { g.sides[$0] }
            var loops: [[SIMD2<Double>]] = []
            for l in Int(g.loopStart[k])..<Int(g.loopStart[k + 1]) {
                loops.append((Int(g.pointStart[l])..<Int(g.pointStart[l + 1])).map { SIMD2(g.points[2 * $0], g.points[2 * $0 + 1]) })
            }
            var tris: [SIMD2<Double>] = []
            for t in Int(g.triangleStart[k])..<Int(g.triangleStart[k + 1]) {
                for c in 0..<3 { tris.append(SIMD2(g.triangles[6 * t + 2 * c], g.triangles[6 * t + 2 * c + 1])) }
            }
            out.append(SketchRegion(area: g.area[k], seed: SIMD2(g.seed[2 * k], g.seed[2 * k + 1]), sides: sides, loops: loops, triangles: tris))
        }
        return out
    }

    // Regions chosen earlier, as the regions now (-1 where one is gone).
    func match(_ refs: [RegionRef]) -> [Int] {
        guard !refs.isEmpty else { return [] }
        var e = EngineSketch(self)
        let (start, sides, seeds) = Sketch.flat(refs)
        var out = [Int32](repeating: -1, count: refs.count)
        let ok = start.withUnsafeBufferPointer { st in
            sides.withUnsafeBufferPointer { sd in
                seeds.withUnsafeBufferPointer { se in
                    out.withUnsafeMutableBufferPointer { o in e.call { bk_sketch_match($0, st.baseAddress, sd.baseAddress, se.baseAddress, Int32(refs.count), o.baseAddress) } }
                }
            }
        }
        return ok != 0 ? out.map(Int.init) : [Int](repeating: -1, count: refs.count)
    }

    static func flat(_ refs: [RegionRef]) -> (start: [Int32], sides: [Int32], seeds: [Double]) {
        var start: [Int32] = [0], sides: [Int32] = [], seeds: [Double] = []
        for r in refs {
            sides += r.sides
            start.append(Int32(sides.count))
            seeds += [r.seed.x, r.seed.y]
        }
        return (start, sides, seeds)
    }

    // The sketch without these points, curves and rules, and whatever uses them (points left unused go too); the rest
    // numbered anew, with where each curve went.
    func removing(points gone: Set<Int32>, curves goneCurves: Set<Int32>, rules goneRules: Set<Int>) -> (Sketch, [Int32: Int32]) {
        var curvesGone = goneCurves
        for (i, c) in curves.enumerated() where c.points.contains(where: gone.contains) { curvesGone.insert(Int32(i)) }
        var s = Sketch()
        var curveMap: [Int32: Int32] = [:]
        for (i, c) in curves.enumerated() where !curvesGone.contains(Int32(i)) {
            curveMap[Int32(i)] = Int32(s.curves.count)
            s.curves.append(c)
        }
        // Points no curve or rule left uses go too (the axes' own stay).
        var used = Set<Int32>()
        for c in s.curves { used.formUnion(c.points) }
        var rulesKept: [SketchRule] = []
        for (i, r) in rules.enumerated() where !goneRules.contains(i) {
            if r.points.contains(where: gone.contains) || r.curves.contains(where: { curvesGone.contains($0) }) { continue }
            rulesKept.append(r)
            used.formUnion(r.points)
        }
        var pointMap: [Int32: Int32] = [:]
        for i in points.indices where used.contains(Int32(i)) || (i < fixed.count && fixed[i]) {
            pointMap[Int32(i)] = Int32(s.points.count)
            s.points.append(points[i])
            s.fixed.append(i < fixed.count ? fixed[i] : false)
        }
        for i in s.curves.indices { s.curves[i].points = s.curves[i].points.map { pointMap[$0] ?? 0 } }
        s.rules = rulesKept.map { r in
            var r = r
            r.points = r.points.map { pointMap[$0] ?? 0 }
            r.curves = r.curves.map { curveMap[$0] ?? 0 }
            return r
        }
        return (s, curveMap)
    }
}

extension RegionRef {
    // The same region's sides with the curves numbered anew (a side on a curve no longer there left out).
    func renumbered(_ curveMap: [Int32: Int32]) -> RegionRef {
        RegionRef(sides: sides.compactMap { s in curveMap[s / 2].map { 2 * $0 + s % 2 } }.sorted(), seed: seed)
    }
}

extension Profile {
    // The solid, placed by its frame (in the coordinates of the shape it's on). Nil: bk_last_error says why.
    func solid() -> OpaquePointer? {
        var e = EngineSketch(sketch)
        let (start, sides, seeds) = Sketch.flat(regions)
        var f = BKForm(kind: form.kind, low: form.low, high: form.high, axis: form.axis)
        guard let made = start.withUnsafeBufferPointer({ st in
            sides.withUnsafeBufferPointer { sd in
                seeds.withUnsafeBufferPointer { se in e.call { bk_sketch_solid($0, st.baseAddress, sd.baseAddress, se.baseAddress, Int32(regions.count), &f) } }
            }
        }) else { return nil }
        if frame == Profile.identity { return made }
        defer { bk_free(made) }
        return frame.withUnsafeBufferPointer { bk_transform(made, $0.baseAddress) }
    }

    // Numbers a file may hold for it: counts, indices, sizes within 10 m, a frame that only turns and moves.
    var valid: Bool {
        let s = sketch, np = s.points.count, nc = s.curves.count
        guard np <= 4000, nc <= 4000, s.rules.count <= 8000, s.fixed.count == np, regions.count <= 10_000 else { return false }
        guard s.points.allSatisfy({ $0.x.isFinite && $0.y.isFinite && abs($0.x) <= 10_000 && abs($0.y) <= 10_000 }) else { return false }
        for c in s.curves {
            guard c.points.count == c.count, c.points.allSatisfy({ (0..<Int32(np)).contains($0) }), c.radius.isFinite, c.radius >= 0, c.radius <= 10_000 else { return false }
        }
        for r in s.rules {
            guard (Int32(BK_RULE_COINCIDENT)...Int32(BK_DIM_ANGLE)).contains(r.kind), r.points.count <= 3, r.curves.count <= 2, r.value.isFinite, abs(r.value) <= 10_000,
                  r.points.allSatisfy({ (-1..<Int32(np)).contains($0) }), r.curves.allSatisfy({ (-1..<Int32(nc)).contains($0) }), r.label.x.isFinite, r.label.y.isFinite
            else { return false }
        }
        guard regions.allSatisfy({ $0.seed.x.isFinite && $0.seed.y.isFinite && $0.sides.count <= 100_000 }) else { return false }
        guard (Int32(BK_FORM_EXTRUDE)...Int32(BK_FORM_REVOLVE)).contains(form.kind), form.low.isFinite, form.high.isFinite, abs(form.low) <= 10_000,
              abs(form.high) <= 10_000, (-1..<Int32(nc)).contains(form.axis) else { return false }
        guard frame.count == 12, frame.allSatisfy(\.isFinite) else { return false }
        let m = matrix
        let c0 = SIMD3(m.columns.0.x, m.columns.0.y, m.columns.0.z), c1 = SIMD3(m.columns.1.x, m.columns.1.y, m.columns.1.z), c2 = SIMD3(m.columns.2.x, m.columns.2.y, m.columns.2.z)
        let l = simd_length(c0)
        return l > 1e-9 && abs(simd_length(c1) - l) <= 1e-6 * l && abs(simd_length(c2) - l) <= 1e-6 * l && abs(dot(c0, c1)) <= 1e-6 * l * l &&
            abs(dot(c0, c2)) <= 1e-6 * l * l && abs(dot(c1, c2)) <= 1e-6 * l * l
    }
}
