import AppKit
import SwiftUI
import simd

// Sketching in the app: a plane picked (the bed, an upright plane through the middle, or a flat face), lines,
// rectangles, circles and arcs drawn on it (snapping to points, curves and the grid, and keeping themselves level or
// upright), constraints and dimensions put on them, then their regions stood up or turned into a body of their own, or
// into the body the face is on (joined, cut or kept where they overlap). Opened again later from the body's layer.

enum SketchTool: Equatable {
    case select, line, rectangle, circle, arc, dimension
}

// Drawing, or choosing regions to stand up (extrude) or turn (revolve).
enum SketchStage: Equatable {
    case draw, extrude, revolve
}

enum SketchItem: Hashable {
    case point(Int32)
    case curve(Int32)
    case rule(Int)
}

// What the pointer is on in the sketch (sketch coordinates).
struct SketchSnap: Equatable {
    enum Kind: Equatable { case point, midpoint, curve, grid, free }
    var kind: Kind
    var at: SIMD2<Double>
    var point: Int32 = -1
    var curve: Int32 = -1
    // Lined up with the start of the line being drawn: 1 level, 2 upright.
    var aligned = 0
}

struct SketchSession {
    var sketch: Sketch
    // The plane: the sketch's x, y and normal, and its origin, in the world.
    var world: simd_double4x4
    // The body whose face it's drawn on (nil: none), the plane in that body's own coordinates, and whether that body
    // takes a sketch's solid inside it (its placement turns and moves it, and scales it the same every way).
    var onBody: UUID?
    var local: simd_double4x4 = matrix_identity_double4x4
    var joinable = false
    // Opened again from a body's layer (which one, how deep): put back there.
    var editing: (id: UUID, level: Int)?
    var tool: SketchTool = .line
    var stage: SketchStage = .draw
    var selection: [SketchItem] = []
    // The clicks so far of what's being drawn, and the point the next line starts from.
    var clicks: [SIMD2<Double>] = []
    var startSnap: SketchSnap?
    var chain: Int32 = -1
    var hover: SketchSnap?
    var regions: [SketchRegion] = []
    var chosen: [Int] = []
    var distance = 10.0
    var symmetric = false
    var angle = 360.0
    var axis: Int32 = Sketch.yAxis
    // -1: a body of its own; otherwise joined (BK_UNION), cut (BK_SUBTRACT) or kept where they overlap (BK_INTERSECT).
    var op: Int32 = -1
    var solved = Solved()
    var past: [Sketch] = []
    var ahead: [Sketch] = []
    // The dimension tool's picks so far, and the rule whose value is being typed.
    var picks: [SketchItem] = []
    var editingRule: Int?

    func world(_ p: SIMD2<Double>) -> SIMD3<Double> { (world * SIMD4(p.x, p.y, 0, 1)).xyz }
    var normal: SIMD3<Double> { simd_normalize(world.columns.2.xyz) }
}

// The solid an extrude or revolve would make, shown as it's shaped (in the sketch's own coordinates, placed by `place`).
struct SketchPreview {
    var mesh: Mesh
    var place: simd_double4x4
    var cut: Bool
}

extension SIMD2 where Scalar == Double {
    var length: Double { simd_length(self) }
}

extension Workbench {
    // MARK: entering and leaving

    func enterSketch() {
        if mode == .sculpt || sculptBusy { finishSculpt() }
        if angleEdit != nil { closeAngles() }
        withAnimation(Neon.spring) {
            mode = .sketch
            sketch = nil
            sketchPreview = nil
            edgePicks = []
            clearMeasure()
        }
    }

    // Leaves the sketch: one that made nothing and has curves asks first (and stays when not to discard it).
    func leaveSketch() {
        guard mode == .sketch else { return }
        if let s = sketch, s.editing == nil, s.sketch.drawn > 0 {
            let a = NSAlert()
            a.messageText = L("Discard this sketch?")
            a.informativeText = L("Nothing has been made of it yet.")
            a.addButton(withTitle: L("Discard"))
            a.addButton(withTitle: L("Keep sketching"))
            guard answer(a) == .alertFirstButtonReturn else { return }
        }
        closeSketch()
    }

    // Leaves at once, the view put back as it was.
    func closeSketch() {
        sketchToken += 1
        withAnimation(Neon.spring) {
            sketch = nil
            sketchPreview = nil
            if mode == .sketch { mode = .select }
        }
        if let c = cameraBeforeSketch { fly(to: c.target, yaw: c.yaw, pitch: c.pitch, distance: c.distance) }
        cameraBeforeSketch = nil
        sceneVersion += 1
    }

    // Esc: one step back — what's being drawn, the selection, the extrude or revolve, the tool, then the sketch.
    func sketchBack() {
        guard var s = sketch else { closeSketch(); return }
        if s.editingRule != nil {
            s.editingRule = nil
        } else if !s.clicks.isEmpty || s.chain >= 0 || !s.picks.isEmpty {
            s.clicks = []
            s.startSnap = nil
            s.chain = -1
            s.picks = []
        } else if s.stage != .draw {
            s.stage = .draw
            sketchPreview = nil
        } else if !s.selection.isEmpty {
            s.selection = []
        } else if s.tool != .select {
            s.tool = .select
        } else {
            leaveSketch()
            return
        }
        withAnimation(Neon.spring) { sketch = s }
    }

    // The keys a sketch takes: its tools, Enter, Delete (the arrows do nothing to the shapes behind it).
    func sketchKey(_ name: String, shift: Bool) -> Bool {
        switch name {
        case "Enter":
            sketchEnter()
        case "Backspace", "Delete":
            sketchDelete()
        case "ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown", "PageUp", "PageDown":
            break
        case "KeyL": setSketchTool(.line)
        case "KeyR": setSketchTool(.rectangle)
        case "KeyC": setSketchTool(.circle)
        case "KeyA": setSketchTool(.arc)
        case "KeyD": setSketchTool(.dimension)
        case "KeyX": toggleConstruction()
        case "KeyE": sketchExtrude()
        case "KeyV": sketchRevolve()
        default: return false
        }
        return true
    }

    func sketchEnter() {
        guard let s = sketch else { return }
        switch s.stage {
        case .draw:
            // Enter ends a chain of lines; with nothing under way, it stands the regions up.
            if s.chain >= 0 || !s.clicks.isEmpty { updateSketch { $0.chain = -1; $0.clicks = []; $0.startSnap = nil } } else { sketchExtrude() }
        case .extrude, .revolve: sketchCommit()
        }
    }

    func setSketchTool(_ t: SketchTool) {
        guard sketch != nil else { return }
        updateSketch {
            $0.tool = t
            $0.stage = .draw
            $0.clicks = []
            $0.startSnap = nil
            $0.chain = -1
            $0.picks = []
        }
        sketchPreview = nil
    }

    func updateSketch(_ change: (inout SketchSession) -> Void) {
        guard var s = sketch else { return }
        change(&s)
        sketch = s
    }

    // MARK: the plane

    // Upright planes through the bed's middle: the bed itself (0), facing the front (1) or the side (2).
    static func sketchPlane(_ k: Int) -> simd_double4x4 {
        switch k {
        case 1: simd_double4x4(columns: (SIMD4(1, 0, 0, 0), SIMD4(0, 0, 1, 0), SIMD4(0, -1, 0, 0), SIMD4(0, 0, 0, 1)))
        case 2: simd_double4x4(columns: (SIMD4(0, 1, 0, 0), SIMD4(0, 0, 1, 0), SIMD4(1, 0, 0, 0), SIMD4(0, 0, 0, 1)))
        default: matrix_identity_double4x4
        }
    }

    func sketchOnPlane(_ k: Int) {
        startSketch(world: Self.sketchPlane(k), onBody: nil, references: [])
    }

    // Whether a face is flat: every point of it facing the way the face does.
    static func flatFace(_ m: Mesh, _ face: Int) -> Bool {
        guard m.faceInfo.indices.contains(face) else { return false }
        let n = SIMD3<Float>(m.faceInfo[face].normal)
        var any = false
        for (i, v) in m.vertices.enumerated() where Int(v.w) == face {
            any = true
            if simd_dot(m.normals[i].xyz, n) < 0.99999 { return false }
        }
        return any
    }

    // A sketch on a body's flat face: its plane through the face, x level in the world where it can be, the origin
    // where the body's middle meets the plane; the face's edges drawn on it to snap and size to.
    func sketchOnFace(_ id: UUID, face: Int) {
        guard let b = body(id), let m = meshes[id], Self.flatFace(m, face) else { flash(L("Pick the bed or a flat face to sketch on")); return }
        let place = b.place.matrix
        let info = m.faceInfo[face]
        let nWorld = simd_normalize((place.inverse.transpose * SIMD4(info.normal, 0)).xyz)
        let onFace = (place * SIMD4(info.centroid, 1)).xyz
        func tidy(_ v: SIMD3<Double>) -> SIMD3<Double> {
            var w = v
            for k in 0..<3 where abs(w[k]) < 1e-12 { w[k] = 0 }
            for k in 0..<3 where abs(abs(w[k]) - 1) < 1e-12 { w[k] = w[k] > 0 ? 1 : -1 }
            return w
        }
        let n = tidy(nWorld)
        var u = abs(n.z) > 0.999999 ? SIMD3<Double>(1, 0, 0) : simd_normalize(SIMD3(-n.y, n.x, 0))
        if abs(n.z) > 0.999999 && n.z < 0 { u = SIMD3(-1, 0, 0) }
        u = tidy(u)
        let v = tidy(simd_cross(n, u))
        // The body's middle put onto the plane.
        let middle = place.columns.3.xyz
        let origin = middle + n * simd_dot(onFace - middle, n)
        let world = simd_double4x4(columns: (SIMD4(u, 0), SIMD4(v, 0), SIMD4(n, 0), SIMD4(origin, 1)))
        let local = place.inverse * world
        // (Inside a body whose placement stretches it unevenly a sketch's solid can't be kept exact: a body of its own.)
        let c0 = local.columns.0.xyz, c1 = local.columns.1.xyz, c2 = local.columns.2.xyz
        let l = simd_length(c0)
        let even = abs(simd_length(c1) - l) <= 1e-9 * l && abs(simd_length(c2) - l) <= 1e-9 * l
        var refs: [[SIMD3<Double>]] = []
        for (e, faces) in zip(m.edges, m.edgeFaces) where faces.x == Int32(face) || faces.y == Int32(face) {
            refs.append(e.map { (place * SIMD4(SIMD3<Double>($0), 1)).xyz })
        }
        startSketch(world: world, onBody: id, local: local, joinable: even, references: refs)
    }

    func startSketch(world: simd_double4x4, onBody: UUID?, local: simd_double4x4 = matrix_identity_double4x4, joinable: Bool = false, references: [[SIMD3<Double>]]) {
        var sk = Sketch.start()
        // The face's edges, as fixed lines (points a hair apart made one).
        let back = world.inverse
        var made: [SIMD2<Double>: Int32] = [:]
        func point(_ w: SIMD3<Double>) -> Int32 {
            let q = (back * SIMD4(w, 1)).xyz
            let p = SIMD2((q.x * 1e6).rounded() / 1e6, (q.y * 1e6).rounded() / 1e6)
            if let i = made[p] { return i }
            let i = sk.point(p)
            sk.fixed[Int(i)] = true
            made[p] = i
            return i
        }
        var count = 0
        for e in references where e.count > 1 && count < 2000 {
            var prev = point(e[0])
            for w in e.dropFirst() {
                let next = point(w)
                if next != prev {
                    sk.curves.append(SketchCurve(kind: .line, points: [prev, next], reference: true))
                    count += 1
                }
                prev = next
            }
        }
        var s = SketchSession(sketch: sk, world: world)
        s.onBody = onBody
        s.local = local
        s.joinable = joinable && onBody != nil
        s.op = s.joinable ? Int32(BK_UNION) : -1
        s.solved = s.sketch.solve()
        s.regions = s.sketch.regions()
        cameraBeforeSketch = cameraBeforeSketch ?? camera
        withAnimation(Neon.spring) { sketch = s }
        faceSketch(world)
    }

    // Glides round to look straight at the sketch's plane (its x to the right, its y up).
    func faceSketch(_ world: simd_double4x4) {
        let n = simd_normalize(world.columns.2.xyz), u = simd_normalize(world.columns.0.xyz)
        let pitch = Float(asin(max(-1, min(1, n.z))))
        // (Looking straight down or up, the yaw is the sketch's x.)
        let yaw = Float(atan2(u.y, u.x))
        let target = SIMD3<Float>(world.columns.3.xyz)
        fly(to: target, yaw: yaw, pitch: pitch, distance: camera.distance)
    }

    // MARK: drawing

    // What the pointer is on: a point, a line's middle, a curve, the grid (⌘: free); level or upright with the start of
    // the line being drawn when near it.
    func sketchSnap(_ p: SIMD2<Double>, perPoint: Double, free: Bool) -> SketchSnap {
        guard let s = sketch else { return SketchSnap(kind: .free, at: p) }
        let sk = s.sketch
        if !free {
            var best = 9 * perPoint, hit: SketchSnap?
            for i in sk.points.indices where shownPoint(sk, Int32(i)) {
                let d = (sk.points[i] - p).length
                if d < best { best = d; hit = SketchSnap(kind: .point, at: sk.points[i], point: Int32(i)) }
            }
            if let hit { return hit }
            for (i, c) in sk.curves.enumerated() where c.kind == .line && !c.reference {
                let m = (sk.points[Int(c.points[0])] + sk.points[Int(c.points[1])]) / 2
                let d = (m - p).length
                if d < best { best = d; hit = SketchSnap(kind: .midpoint, at: m, curve: Int32(i)) }
            }
            if let hit { return hit }
            best = 7 * perPoint
            for i in sk.curves.indices {
                guard let q = nearest(sk, Int32(i), p) else { continue }
                let d = (q - p).length
                if d < best { best = d; hit = SketchSnap(kind: .curve, at: q, curve: Int32(i)) }
            }
            if let hit { return hit }
        }
        let g = settings.snap
        var at = free ? p : SIMD2((p.x / g).rounded() * g, (p.y / g).rounded() * g)
        var snap = SketchSnap(kind: free ? .free : .grid, at: at)
        if let a = lineStart(s) {
            switch aligned(a, at, perPoint) {
            case 1: at.y = a.y; snap.aligned = 1
            case 2: at.x = a.x; snap.aligned = 2
            default: break
            }
            snap.at = at
        }
        return snap
    }

    // Where a line being drawn starts (its first click, or the end of the last line in the chain).
    private func lineStart(_ s: SketchSession) -> SIMD2<Double>? {
        guard s.stage == .draw else { return nil }
        if s.tool == .line {
            if s.chain >= 0 { return s.sketch.points[Int(s.chain)] }
            return s.clicks.first
        }
        return nil
    }

    // Level (1) or upright (2) when the way from a to b is within a couple of degrees of it (and not a click on a).
    private func aligned(_ a: SIMD2<Double>, _ b: SIMD2<Double>, _ perPoint: Double) -> Int {
        let d = b - a
        guard d.length > 4 * perPoint else { return 0 }
        if abs(d.y) <= abs(d.x) * 0.035 { return 1 }
        if abs(d.x) <= abs(d.y) * 0.035 { return 2 }
        return 0
    }

    // Points to snap to: the origin and every curve's points but the axes' ends.
    func shownPoint(_ sk: Sketch, _ i: Int32) -> Bool {
        if i == 0 { return true }
        return sk.curves.contains { c in !(c.reference && c.construction) && c.points.contains(i) }
    }

    // The nearest point to p on curve c (an axis as the whole line it lies on).
    func nearest(_ sk: Sketch, _ c: Int32, _ p: SIMD2<Double>) -> SIMD2<Double>? {
        let cv = sk.curves[Int(c)]
        switch cv.kind {
        case .line:
            let a = sk.points[Int(cv.points[0])], b = sk.points[Int(cv.points[1])], d = b - a
            let l2 = simd_dot(d, d)
            guard l2 > 0 else { return nil }
            var t = simd_dot(p - a, d) / l2
            if !(cv.reference && cv.construction) { t = max(0, min(1, t)) }
            return a + d * t
        case .circle:
            let c0 = sk.points[Int(cv.points[0])], d = p - c0
            return d.length > 0 ? c0 + d / d.length * cv.radius : nil
        case .arc:
            let c0 = sk.points[Int(cv.points[0])], s = sk.points[Int(cv.points[1])], e = sk.points[Int(cv.points[2])]
            let r = (s - c0).length, d = p - c0
            guard d.length > 0, r > 0 else { return nil }
            let a0 = atan2(s.y - c0.y, s.x - c0.x), a1 = atan2(e.y - c0.y, e.x - c0.x), a = atan2(d.y, d.x)
            func ccw(_ x: Double) -> Double { var y = x - a0; while y < 0 { y += 2 * .pi }; while y >= 2 * .pi { y -= 2 * .pi }; return y }
            let span = ccw(a1) == 0 ? 2 * .pi : ccw(a1)
            if ccw(a) <= span { return c0 + d / d.length * r }
            return (s - p).length < (e - p).length ? s : e
        }
    }

    func sketchHover(_ p: SIMD2<Double>, perPoint: Double, free: Bool) {
        guard sketch != nil else { return }
        let snap = sketchSnap(p, perPoint: perPoint, free: free)
        if sketch?.hover != snap { sketch?.hover = snap }
    }

    // A click: what the tool does with it.
    func sketchClick(_ p: SIMD2<Double>, perPoint: Double, free: Bool = false, shift: Bool = false) {
        guard let s = sketch else { return }
        let snap = sketchSnap(p, perPoint: perPoint, free: free)
        switch s.stage {
        case .extrude, .revolve:
            // A line clicked while turning is the axis; anything else, a region chosen or let go.
            if s.stage == .revolve, let c = curveAt(p, perPoint: perPoint), s.sketch.curves[Int(c)].kind == .line {
                updateSketch { $0.axis = c }
                refreshPreview()
                return
            }
            if let r = regionAt(p) {
                updateSketch { if let i = $0.chosen.firstIndex(of: r) { $0.chosen.remove(at: i) } else { $0.chosen.append(r) } }
                refreshPreview()
            }
            return
        case .draw: break
        }
        switch s.tool {
        case .select: selectAt(p, perPoint: perPoint, add: shift)
        case .line: lineClick(snap)
        case .rectangle: rectangleClick(snap)
        case .circle: circleClick(snap)
        case .arc: arcClick(snap)
        case .dimension: dimensionClick(p, perPoint: perPoint)
        }
    }

    // The item under the pointer: a point (9 points away at most), else a curve (7), else a dimension's label.
    func itemAt(_ p: SIMD2<Double>, perPoint: Double) -> SketchItem? {
        guard let sk = sketch?.sketch else { return nil }
        var best = 9 * perPoint, hit: SketchItem?
        for i in sk.points.indices where shownPoint(sk, Int32(i)) {
            let d = (sk.points[i] - p).length
            if d < best { best = d; hit = .point(Int32(i)) }
        }
        if let hit { return hit }
        if let c = curveAt(p, perPoint: perPoint) { return .curve(c) }
        for (i, r) in sk.rules.enumerated() where r.dimension && (r.label - p).length < 14 * perPoint { return .rule(i) }
        return nil
    }

    func curveAt(_ p: SIMD2<Double>, perPoint: Double) -> Int32? {
        guard let sk = sketch?.sketch else { return nil }
        var best = 7 * perPoint, hit: Int32?
        for i in sk.curves.indices {
            guard let q = nearest(sk, Int32(i), p) else { continue }
            let d = (q - p).length
            if d < best { best = d; hit = Int32(i) }
        }
        return hit
    }

    // The region p lies in (the smallest, where several do).
    func regionAt(_ p: SIMD2<Double>) -> Int? {
        guard let s = sketch else { return nil }
        var best: Int?
        for (k, r) in s.regions.enumerated() {
            var inside = false
            var t = 0
            while t + 2 < r.triangles.count {
                let a = r.triangles[t], b = r.triangles[t + 1], c = r.triangles[t + 2]
                func side(_ u: SIMD2<Double>, _ v: SIMD2<Double>) -> Double { (v.x - u.x) * (p.y - u.y) - (v.y - u.y) * (p.x - u.x) }
                let d0 = side(a, b), d1 = side(b, c), d2 = side(c, a)
                if (d0 >= 0 && d1 >= 0 && d2 >= 0) || (d0 <= 0 && d1 <= 0 && d2 <= 0) { inside = true; break }
                t += 3
            }
            if inside && (best == nil || r.area < s.regions[best!].area) { best = k }
        }
        return best
    }

    private func selectAt(_ p: SIMD2<Double>, perPoint: Double, add: Bool) {
        let item = itemAt(p, perPoint: perPoint)
        updateSketch { s in
            guard let item else { if !add { s.selection = [] }; return }
            if add {
                if let i = s.selection.firstIndex(of: item) { s.selection.remove(at: i) } else { s.selection.append(item) }
            } else {
                s.selection = [item]
            }
        }
    }

    // A point where a click snapped: the point it's on, or a new one (on a curve, with the rule keeping it there; at a
    // line's middle, with that one). The rules made are returned to add once the curve using the point is there.
    private func pointFor(_ snap: SketchSnap, _ sk: inout Sketch, _ rules: inout [SketchRule]) -> Int32 {
        switch snap.kind {
        case .point: return snap.point
        case .midpoint:
            let i = sk.point(snap.at)
            rules.append(SketchRule(kind: Int32(BK_RULE_MIDPOINT), points: [i], curves: [snap.curve]))
            return i
        case .curve:
            let i = sk.point(snap.at)
            rules.append(SketchRule(kind: Int32(BK_RULE_ON), points: [i], curves: [snap.curve]))
            return i
        default:
            return sk.point(snap.at)
        }
    }

    private func lineClick(_ snap: SketchSnap) {
        guard var s = sketch else { return }
        // The first click only marks where the line starts (its point made with the line).
        guard s.chain >= 0 || s.startSnap != nil else {
            s.startSnap = snap
            s.clicks = [snap.at]
            sketch = s
            return
        }
        guard (snap.at - (s.chain >= 0 ? s.sketch.points[Int(s.chain)] : s.clicks[0])).length > 1e-9 else { return }
        var sk = s.sketch, rules: [SketchRule] = []
        let start = s.chain >= 0 ? s.chain : pointFor(s.startSnap!, &sk, &rules)
        let end = pointFor(snap, &sk, &rules)
        guard end != start else { return }
        sk.curves.append(SketchCurve(kind: .line, points: [start, end]))
        let line = Int32(sk.curves.count - 1)
        if snap.aligned == 1 { rules.append(SketchRule(kind: Int32(BK_RULE_HORIZONTAL), curves: [line])) }
        if snap.aligned == 2 { rules.append(SketchRule(kind: Int32(BK_RULE_VERTICAL), curves: [line])) }
        record()
        s = sketch ?? s
        s.sketch = sk
        // Ending on a point already there (closing a shape) ends the chain.
        s.chain = snap.kind == .point ? -1 : end
        s.startSnap = nil
        s.clicks = []
        sketch = s
        addRules(rules, auto: true)
    }

    private func rectangleClick(_ snap: SketchSnap) {
        guard var s = sketch else { return }
        guard let first = s.clicks.first else { s.clicks = [snap.at]; sketch = s; return }
        let a = first, b = snap.at
        guard abs(b.x - a.x) > 1e-9, abs(b.y - a.y) > 1e-9 else { return }
        record()
        s = sketch ?? s
        var sk = s.sketch
        let p = [sk.point(a), sk.point(SIMD2(b.x, a.y)), sk.point(b), sk.point(SIMD2(a.x, b.y))]
        let base = Int32(sk.curves.count)
        for k in 0..<4 { sk.curves.append(SketchCurve(kind: .line, points: [p[k], p[(k + 1) % 4]])) }
        s.sketch = sk
        s.clicks = []
        sketch = s
        addRules([SketchRule(kind: Int32(BK_RULE_HORIZONTAL), curves: [base]), SketchRule(kind: Int32(BK_RULE_VERTICAL), curves: [base + 1]),
                  SketchRule(kind: Int32(BK_RULE_HORIZONTAL), curves: [base + 2]), SketchRule(kind: Int32(BK_RULE_VERTICAL), curves: [base + 3])], auto: true)
    }

    private func circleClick(_ snap: SketchSnap) {
        guard var s = sketch else { return }
        guard let centre = s.clicks.first else { s.clicks = [snap.at]; sketch = s; return }
        let r = (snap.at - centre).length
        guard r > 1e-9 else { return }
        record()
        s = sketch ?? s
        var sk = s.sketch
        let c = sk.point(centre)
        sk.curves.append(SketchCurve(kind: .circle, points: [c], radius: r))
        s.sketch = sk
        s.clicks = []
        sketch = s
        sketchChanged()
    }

    // An arc through three clicks: its start, its end, and a point it passes through.
    private func arcClick(_ snap: SketchSnap) {
        guard var s = sketch else { return }
        if s.clicks.count < 2 { s.clicks.append(snap.at); sketch = s; return }
        let a = s.clicks[0], b = s.clicks[1], m = snap.at
        guard let c = Self.centre(a, b, m) else { return }
        record()
        s = sketch ?? s
        var sk = s.sketch
        // Counter-clockwise from start to end the way through m, else the other way round.
        let ccw = (b.x - a.x) * (m.y - a.y) - (b.y - a.y) * (m.x - a.x) < 0
        let pc = sk.point(c), pa = sk.point(a), pb = sk.point(b)
        sk.curves.append(SketchCurve(kind: .arc, points: ccw ? [pc, pa, pb] : [pc, pb, pa]))
        s.sketch = sk
        s.clicks = []
        sketch = s
        sketchChanged()
    }

    // The centre of the circle through three points (nil when they're in a line).
    static func centre(_ a: SIMD2<Double>, _ b: SIMD2<Double>, _ c: SIMD2<Double>) -> SIMD2<Double>? {
        let d = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y))
        guard abs(d) > 1e-12 * max(1, simd_length_squared(b - a)) else { return nil }
        let a2 = simd_length_squared(a), b2 = simd_length_squared(b), c2 = simd_length_squared(c)
        return SIMD2((a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d, (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d)
    }

    // MARK: rules

    // An undo step for the sketch (its own while it's open).
    func record() {
        guard var s = sketch else { return }
        s.past.append(s.sketch)
        if s.past.count > 200 { s.past.removeFirst() }
        s.ahead = []
        sketch = s
    }

    func sketchUndo() {
        guard var s = sketch, let last = s.past.popLast() else { return }
        s.ahead.append(s.sketch)
        s.sketch = last
        s.selection = []
        sketch = s
        sketchChanged()
    }

    func sketchRedo() {
        guard var s = sketch, let next = s.ahead.popLast() else { return }
        s.past.append(s.sketch)
        s.sketch = next
        s.selection = []
        sketch = s
        sketchChanged()
    }

    // Solved again and its regions found again, after any change.
    func sketchChanged() {
        guard var s = sketch else { return }
        let solved = s.sketch.solve()
        s.solved = solved
        s.regions = s.sketch.regions()
        if s.stage != .draw {
            // (Regions chosen before the change, found again.)
            s.chosen = s.chosen.filter { s.regions.indices.contains($0) }
        }
        sketch = s
    }

    // Rules added and the sketch solved with them. Rules added by themselves (drawing) that the sketch already holds or
    // can't hold are left out quietly; one asked for that way is refused, said why, and nothing changes.
    @discardableResult
    func addRules(_ rules: [SketchRule], auto: Bool) -> Bool {
        guard var s = sketch, !rules.isEmpty else { sketchChanged(); return true }
        let before = s.sketch
        for r in rules {
            var trial = s.sketch
            trial.rules.append(r)
            let solved = trial.solve()
            let mine = solved.dependent == trial.rules.count - 1
            if solved.ok && !mine {
                s.sketch = trial
                continue
            }
            if auto { continue }
            s.sketch = before
            sketch = s
            sketchChanged()
            flash(solved.ok ? L("This is already fixed by other constraints") : L("This conflicts with the sketch's other constraints"))
            return false
        }
        if !auto {
            s.past.append(before)
            s.ahead = []
        }
        sketch = s
        sketchChanged()
        return true
    }

    // The constraints that suit what's chosen, each with its name.
    func sketchConstraints() -> [(kind: Int32, name: String)] {
        guard let s = sketch else { return [] }
        let sk = s.sketch
        var points: [Int32] = [], lines: [Int32] = [], rounds: [Int32] = []
        for item in s.selection {
            switch item {
            case .point(let p): points.append(p)
            case .curve(let c): if sk.curves[Int(c)].kind == .line { lines.append(c) } else { rounds.append(c) }
            case .rule: break
            }
        }
        var out: [(kind: Int32, name: String)] = []
        func add(_ k: Int) { out.append((Int32(k), Self.ruleName(Int32(k)))) }
        switch (points.count, lines.count, rounds.count) {
        case (0, 1, 0): add(BK_RULE_HORIZONTAL); add(BK_RULE_VERTICAL)
        case (0, 2, 0): add(BK_RULE_PARALLEL); add(BK_RULE_PERPENDICULAR); add(BK_RULE_EQUAL); add(BK_RULE_COLLINEAR)
        case (0, 1, 1): add(BK_RULE_TANGENT)
        case (0, 0, 2): add(BK_RULE_EQUAL); add(BK_RULE_CONCENTRIC); add(BK_RULE_TANGENT)
        case (1, 1, 0): add(BK_RULE_ON); add(BK_RULE_MIDPOINT)
        case (1, 0, 1): add(BK_RULE_ON)
        case (2, 0, 0): add(BK_RULE_COINCIDENT); add(BK_RULE_HORIZONTAL); add(BK_RULE_VERTICAL)
        case (2, 1, 0): add(BK_RULE_SYMMETRIC)
        case (1, 0, 0): add(BK_RULE_FIX)
        default: break
        }
        return out
    }

    // The constraint asked for on what's chosen, held the way the sketch is now.
    func applyConstraint(_ kind: Int32) {
        guard let s = sketch else { return }
        let sk = s.sketch
        var points: [Int32] = [], curves: [Int32] = []
        for item in s.selection {
            if case .point(let p) = item { points.append(p) }
            if case .curve(let c) = item { curves.append(c) }
        }
        // Lines first, then arcs and circles (as the rules take them).
        curves.sort { (sk.curves[Int($0)].kind == .line ? 0 : 1, $0) < (sk.curves[Int($1)].kind == .line ? 0 : 1, $1) }
        var r = SketchRule(kind: kind, points: points, curves: curves)
        func way(_ c: Int32) -> SIMD2<Double> {
            let cv = sk.curves[Int(c)]
            return sk.points[Int(cv.points[1])] - sk.points[Int(cv.points[0])]
        }
        func centre(_ c: Int32) -> SIMD2<Double> { sk.points[Int(sk.curves[Int(c)].points[0])] }
        func radius(_ c: Int32) -> Double {
            let cv = sk.curves[Int(c)]
            return cv.kind == .circle ? cv.radius : (sk.points[Int(cv.points[1])] - centre(c)).length
        }
        switch Int(kind) {
        case BK_RULE_PARALLEL where curves.count == 2:
            r.side = simd_dot(way(curves[0]), way(curves[1])) < 0 ? -1 : 1
        case BK_RULE_PERPENDICULAR where curves.count == 2:
            let a = way(curves[0]), b = way(curves[1])
            r.side = a.x * b.y - a.y * b.x < 0 ? -1 : 1
        case BK_RULE_TANGENT where curves.count == 2:
            if sk.curves[Int(curves[0])].kind == .line {
                let a = sk.points[Int(sk.curves[Int(curves[0])].points[0])], d = way(curves[0]), c = centre(curves[1])
                r.side = d.x * (c.y - a.y) - d.y * (c.x - a.x) < 0 ? -1 : 1
            } else {
                let d = (centre(curves[1]) - centre(curves[0])).length, r0 = radius(curves[0]), r1 = radius(curves[1])
                // Apart (radii added), or one inside the other.
                r.side = abs(d - (r0 + r1)) <= abs(d - abs(r0 - r1)) ? 1 : r0 >= r1 ? -1 : -2
            }
        default: break
        }
        addRules([r], auto: false)
    }

    // MARK: dimensions

    // The dimension tool: picks (a line, a circle or arc, two points, a point and a line, two lines), then a click
    // where its value goes; then the value is typed.
    private func dimensionClick(_ p: SIMD2<Double>, perPoint: Double) {
        guard var s = sketch else { return }
        let item = itemAt(p, perPoint: perPoint)
        var onRule = false
        if case .rule = item { onRule = true }
        // Something to measure: picked (two at most).
        if let item, !onRule, s.picks.count < 2, !s.picks.contains(item) {
            s.picks.append(item)
            sketch = s
            return
        }
        // Elsewhere: what's picked measured, its value put where clicked.
        let picks = s.picks
        s.picks = []
        sketch = s
        guard !picks.isEmpty, var rule = Self.dimension(picks, s.sketch, at: p) else { return }
        rule.label = p
        var trial = s.sketch
        trial.rules.append(rule)
        let solved = trial.solve()
        guard solved.ok, solved.dependent != trial.rules.count - 1 else {
            flash(L("This is already fixed by other constraints"))
            return
        }
        record()
        updateSketch {
            $0.sketch = trial
            $0.editingRule = trial.rules.count - 1
        }
        sketchChanged()
    }

    // The dimension picks stand for, at the size it has now (nil: none).
    static func dimension(_ picks: [SketchItem], _ sk: Sketch, at label: SIMD2<Double>) -> SketchRule? {
        var points: [Int32] = [], lines: [Int32] = [], rounds: [Int32] = []
        for p in picks {
            switch p {
            case .point(let i): points.append(i)
            case .curve(let c): if sk.curves[Int(c)].kind == .line { lines.append(c) } else { rounds.append(c) }
            case .rule: break
            }
        }
        func pt(_ i: Int32) -> SIMD2<Double> { sk.points[Int(i)] }
        func ends(_ c: Int32) -> (SIMD2<Double>, SIMD2<Double>) { (pt(sk.curves[Int(c)].points[0]), pt(sk.curves[Int(c)].points[1])) }
        func cross(_ a: SIMD2<Double>, _ b: SIMD2<Double>) -> Double { a.x * b.y - a.y * b.x }
        switch (points.count, lines.count, rounds.count) {
        case (0, 1, 0):
            let (a, b) = ends(lines[0])
            return SketchRule(kind: Int32(BK_DIM_LENGTH), curves: lines, value: (b - a).length)
        case (0, 0, 1):
            let c = sk.curves[Int(rounds[0])]
            let r = c.kind == .circle ? c.radius : (pt(c.points[1]) - pt(c.points[0])).length
            return c.kind == .circle ? SketchRule(kind: Int32(BK_DIM_DIAMETER), curves: rounds, value: 2 * r) : SketchRule(kind: Int32(BK_DIM_RADIUS), curves: rounds, value: r)
        case (2, 0, 0):
            return SketchRule(kind: Int32(BK_DIM_DISTANCE), points: points, value: (pt(points[1]) - pt(points[0])).length)
        case (1, 1, 0):
            let (a, b) = ends(lines[0]), d = b - a
            let off = cross(d, pt(points[0]) - a) / d.length
            return SketchRule(kind: Int32(BK_DIM_POINT_LINE), points: points, curves: lines, value: abs(off), side: off < 0 ? -1 : 1)
        case (0, 2, 0):
            let (a0, b0) = ends(lines[0]), (a1, b1) = ends(lines[1])
            let d0 = b0 - a0, d1 = b1 - a1
            if abs(cross(d0, d1)) <= 1e-9 * d0.length * d1.length {
                let m = (a1 + b1) / 2, off = cross(d0, m - a0) / d0.length
                return SketchRule(kind: Int32(BK_DIM_LINES), curves: lines, value: abs(off), side: off < 0 ? -1 : 1)
            }
            // The angle between them, measured between the ways that open towards the label.
            var side: Int32 = 0
            var u = d0, v = d1
            let corner = Self.meet(a0, d0, a1, d1) ?? a0
            if simd_dot(u, label - corner) < 0 { u = -u; side |= 1 }
            if simd_dot(v, label - corner) < 0 { v = -v; side |= 2 }
            var deg = atan2(cross(u, v), simd_dot(u, v)) * 180 / .pi
            if deg < 0 {
                // Measured the other way round: the lines given in the other order.
                deg = -deg
                return SketchRule(kind: Int32(BK_DIM_ANGLE), curves: [lines[1], lines[0]], value: deg, side: ((side & 1) << 1) | ((side & 2) >> 1))
            }
            return SketchRule(kind: Int32(BK_DIM_ANGLE), curves: lines, value: deg, side: side)
        default:
            return nil
        }
    }

    // Where two lines (point and way) meet.
    static func meet(_ a: SIMD2<Double>, _ u: SIMD2<Double>, _ b: SIMD2<Double>, _ v: SIMD2<Double>) -> SIMD2<Double>? {
        let d = u.x * v.y - u.y * v.x
        guard abs(d) > 1e-12 else { return nil }
        let t = ((b.x - a.x) * v.y - (b.y - a.y) * v.x) / d
        return a + u * t
    }

    // A dimension's new value (from its field): the sketch made to hold it, or it's said why not and stays as it was.
    func setDimension(_ rule: Int, _ value: Double) {
        guard var s = sketch, s.sketch.rules.indices.contains(rule) else { return }
        s.editingRule = nil
        guard value.isFinite, value > 0 || s.sketch.rules[rule].kind == Int32(BK_DIM_ANGLE) && value >= 0 else {
            sketch = s
            flash(L("These sizes don't make a shape"))
            return
        }
        var trial = s.sketch
        trial.rules[rule].value = value
        let solved = trial.solve()
        guard solved.ok else {
            sketch = s
            flash(L("These dimensions can't all hold — try another value"))
            return
        }
        s.past.append(s.sketch)
        s.ahead = []
        s.sketch = trial
        sketch = s
        sketchChanged()
        refreshPreview()
    }

    func editDimension(_ rule: Int) {
        updateSketch { $0.editingRule = rule }
    }

    // MARK: dragging points

    func sketchDragBegin() { record() }

    // A free point dragged: the sketch follows it as its rules let it.
    func sketchDrag(_ point: Int32, to p: SIMD2<Double>) {
        guard var s = sketch else { return }
        var sk = s.sketch
        let solved = sk.solve(drag: [point], to: [p])
        guard solved.ok else { return }
        s.sketch = sk
        sketch = s
    }

    func sketchDragEnd() {
        sketchChanged()
    }

    // MARK: editing

    func sketchDelete() {
        guard var s = sketch else { return }
        if s.stage != .draw { return }
        var points = Set<Int32>(), curves = Set<Int32>(), rules = Set<Int>()
        for item in s.selection {
            switch item {
            case .point(let p) where !(p < Int32(s.sketch.fixed.count) && s.sketch.fixed[Int(p)]): points.insert(p)
            case .curve(let c) where !s.sketch.curves[Int(c)].reference: curves.insert(c)
            case .rule(let r): rules.insert(r)
            default: break
            }
        }
        guard !points.isEmpty || !curves.isEmpty || !rules.isEmpty else { return }
        record()
        s = sketch ?? s
        s.sketch = s.sketch.removing(points: points, curves: curves, rules: rules).0
        s.selection = []
        s.chain = -1
        s.clicks = []
        sketch = s
        sketchChanged()
    }

    // The chosen curves made construction lines (bounding no region), or back.
    func toggleConstruction() {
        guard let s = sketch else { return }
        let curves = s.selection.compactMap { item -> Int32? in if case .curve(let c) = item, !s.sketch.curves[Int(c)].reference { return c }; return nil }
        guard !curves.isEmpty else { return }
        record()
        updateSketch { t in for c in curves { t.sketch.curves[Int(c)].construction.toggle() } }
        sketchChanged()
    }

    func sketchSelectAll() {
        updateSketch { s in
            s.selection = s.sketch.curves.indices.filter { !s.sketch.curves[$0].reference }.map { .curve(Int32($0)) }
        }
    }

    // The rules on the chosen curves and points, to take off.
    func rulesOnSelection() -> [Int] {
        guard let s = sketch else { return [] }
        var points = Set<Int32>(), curves = Set<Int32>()
        for item in s.selection {
            if case .point(let p) = item { points.insert(p) }
            if case .curve(let c) = item { curves.insert(c) }
        }
        return s.sketch.rules.indices.filter { i in
            let r = s.sketch.rules[i]
            return r.points.contains(where: points.contains) || r.curves.contains(where: curves.contains)
        }
    }

    func removeRule(_ i: Int) {
        guard let s = sketch, s.sketch.rules.indices.contains(i) else { return }
        record()
        updateSketch { $0.sketch.rules.remove(at: i); $0.selection = $0.selection.filter { if case .rule = $0 { return false }; return true } }
        sketchChanged()
    }

    static func ruleName(_ k: Int32) -> String {
        switch Int(k) {
        case BK_RULE_COINCIDENT: L("Coincident")
        case BK_RULE_ON: L("On the curve")
        case BK_RULE_HORIZONTAL: L("Horizontal")
        case BK_RULE_VERTICAL: L("Vertical")
        case BK_RULE_PARALLEL: L("Parallel")
        case BK_RULE_PERPENDICULAR: L("Perpendicular")
        case BK_RULE_TANGENT: L("Tangent")
        case BK_RULE_EQUAL: L("Equal")
        case BK_RULE_CONCENTRIC: L("Concentric")
        case BK_RULE_MIDPOINT: L("Midpoint")
        case BK_RULE_COLLINEAR: L("Collinear")
        case BK_RULE_SYMMETRIC: L("Symmetric")
        case BK_RULE_FIX: L("Fix")
        case BK_DIM_ANGLE: L("Angle")
        case BK_DIM_RADIUS: L("Radius")
        case BK_DIM_DIAMETER: L("Diameter")
        default: L("Dimension")
        }
    }

    // A dimension's value as shown.
    static func dimensionText(_ r: SketchRule) -> String {
        switch Int(r.kind) {
        case BK_DIM_ANGLE: MMField.format(r.value) + "°"
        case BK_DIM_RADIUS: "R " + MMField.format(r.value)
        case BK_DIM_DIAMETER: "⌀ " + MMField.format(r.value)
        default: MMField.format(r.value)
        }
    }

    // MARK: extrude and revolve

    func sketchExtrude() { formStage(.extrude) }
    func sketchRevolve() { formStage(.revolve) }

    private func formStage(_ stage: SketchStage) {
        guard var s = sketch else { return }
        if s.regions.isEmpty { flash(L("This sketch has no closed regions")); return }
        s.stage = stage
        s.clicks = []
        s.chain = -1
        s.picks = []
        // A lone region chosen at once.
        if s.chosen.isEmpty && s.regions.count == 1 { s.chosen = [0] }
        if stage == .revolve {
            // About the last construction line drawn, else the sketch's y axis.
            s.axis = Int32(s.sketch.curves.indices.last { s.sketch.curves[$0].kind == .line && s.sketch.curves[$0].construction && !s.sketch.curves[$0].reference }
                ?? Int(Sketch.yAxis))
        }
        withAnimation(Neon.spring) { sketch = s }
        refreshPreview()
    }

    // What's being made, as a profile in the sketch's own coordinates.
    func sketchForm() -> Form? {
        guard let s = sketch else { return nil }
        if s.stage == .revolve { return Form(kind: Int32(BK_FORM_REVOLVE), low: 0, high: max(0.1, min(360, s.angle)), axis: s.axis) }
        let d = s.distance
        let (lo, hi) = s.symmetric ? (-abs(d) / 2, abs(d) / 2) : (min(0, d), max(0, d))
        return Form(kind: Int32(BK_FORM_EXTRUDE), low: lo, high: hi, axis: -1)
    }

    func setSketchForm(distance: Double? = nil, symmetric: Bool? = nil, angle: Double? = nil, op: Int32? = nil) {
        updateSketch { s in
            if let distance { s.distance = distance }
            if let symmetric { s.symmetric = symmetric }
            if let angle { s.angle = angle }
            if let op { s.op = op }
        }
        refreshPreview()
    }

    // The solid made again to show (on the kernel's queue), shown once it's there unless something changed meanwhile.
    func refreshPreview() {
        sketchToken += 1
        guard let s = sketch, s.stage != .draw, !s.chosen.isEmpty, let form = sketchForm() else { sketchPreview = nil; return }
        let p = Profile(sketch: s.sketch, regions: s.chosen.map { s.regions[$0].ref }, form: form)
        let token = sketchToken, place = s.world, cut = s.op == Int32(BK_SUBTRACT)
        Kernel.shared.queue.async {
            let mesh = Kernel.shared.mesh(.profile(p), keep: false)
            _ = Kernel.shared.takeProblems()
            DispatchQueue.main.async {
                MainActor.assumeIsolated {
                    guard self.sketchToken == token, self.mode == .sketch else { return }
                    self.sketchPreview = mesh.map { SketchPreview(mesh: $0, place: place, cut: cut) }
                    self.sceneVersion += 1
                }
            }
        }
    }

    // The chosen regions made: a body of their own, or into the body the sketch is on; one opened again from a layer
    // goes back there. Built first: what can't be made is said, and nothing changes.
    func sketchCommit() {
        guard let s = sketch, let form = sketchForm() else { return }
        guard !s.chosen.isEmpty else { flash(L("Click the regions to make")); return }
        let refs = s.chosen.map { s.regions[$0].ref }
        if let e = s.editing, let b = body(e.id) {
            let node = rewrite(b.node, level: e.level) { n -> Node? in
                switch n {
                case .profile(let p): return Node.profile(Profile(sketch: s.sketch, regions: refs, form: form, frame: p.frame))
                case .feature(let of, let p, let op):
                    return Node.feature(of: of, profile: Profile(sketch: s.sketch, regions: refs, form: form, frame: p.frame), op: s.op >= 0 ? s.op : op)
                default: return n
                }
            }
            tryThen([node]) { [weak self] in
                guard let self, self.body(e.id)?.node == b.node else { return }
                self.commit { d in if let i = d.bodies.firstIndex(where: { $0.id == e.id }) { d.bodies[i].node = node } }
                self.closeSketch()
            }
            return
        }
        if s.op >= 0, s.joinable, let id = s.onBody, let b = body(id) {
            let p = Profile(sketch: s.sketch, regions: refs, form: form, frame: Profile.frame(s.local))
            let node = Node.feature(of: b.node, profile: p, op: s.op)
            tryThen([node]) { [weak self] in
                guard let self, self.body(id)?.node == b.node else { return }
                self.commit { d in if let i = d.bodies.firstIndex(where: { $0.id == id }) { d.bodies[i].node = node } }
                self.selection = [id]
                self.closeSketch()
            }
            return
        }
        let node = Node.profile(Profile(sketch: s.sketch, regions: refs, form: form))
        let made = Solid(name: form.revolve ? L("Revolve") : L("Extrude"), color: nextColor(), node: node, place: Placement.from(s.world))
        tryThen([node]) { [weak self] in
            guard let self else { return }
            self.commit { $0.bodies.append(made) }
            self.selection = [made.id]
            self.closeSketch()
        }
    }

    // A body's sketch opened again from its layer: the plane where it was, its regions and sizes as they were.
    func editSketch(_ id: UUID, level: Int) {
        guard let b = body(id) else { return }
        let layers = stack(b)
        guard layers.indices.contains(level) else { return }
        let p: Profile, op: Int32
        switch layers[level] {
        case .profile(let q): p = q; op = -1
        case .feature(_, let q, let o): p = q; op = o
        default: return
        }
        if mode == .sculpt || sculptBusy { finishSculpt() }
        let world = b.place.matrix * p.matrix
        // (The sketch's plane in the world, its scale taken out: sketches are in millimetres.)
        let scale = simd_length(world.columns.0.xyz)
        let unitWorld = scale > 0 ? simd_double4x4(columns: (world.columns.0 / scale, world.columns.1 / scale, world.columns.2 / scale, world.columns.3)) : world
        var s = SketchSession(sketch: p.sketch, world: unitWorld)
        s.onBody = id
        s.local = p.matrix
        s.joinable = op >= 0
        s.op = op
        s.editing = (id, level)
        s.tool = .select
        s.solved = s.sketch.solve()
        s.regions = s.sketch.regions()
        let found = s.sketch.match(p.regions)
        s.chosen = found.filter { $0 >= 0 }
        if p.form.revolve {
            s.angle = p.form.high - p.form.low
            s.axis = p.form.axis
        } else {
            s.distance = p.form.low < 0 && p.form.high > 0 && abs(p.form.low + p.form.high) < 1e-9 ? p.form.high - p.form.low : (p.form.low < 0 ? p.form.low : p.form.high)
            s.symmetric = p.form.low < 0 && p.form.high > 0 && abs(p.form.low + p.form.high) < 1e-9
        }
        cameraBeforeSketch = camera
        withAnimation(Neon.spring) {
            mode = .sketch
            sketch = s
        }
        faceSketch(unitWorld)
    }

    // What the engine's "sketch: …" refusals say.
    static func sketchMessage(_ m: String) -> String {
        if m.contains("crosses the axis") { return L("The profile crosses the axis") }
        if m.contains("region is gone") { return L("The chosen region is gone from the sketch — choose it again") }
        if m.contains("too close") { return L("Curves too close together to build") }
        if m.contains("no depth") { return L("These sizes don't make a shape") }
        if m.contains("too large") || m.contains("100 m") { return L("This sketch is too large") }
        return L("This sketch can't be made at these sizes")
    }
}

extension Node {
    // The same sketch's layer with its form changed (anything else as it is).
    func withForm(_ change: (inout Form) -> Void) -> Node {
        switch self {
        case .profile(var p):
            change(&p.form)
            return .profile(p)
        case .feature(let of, var p, let op):
            change(&p.form)
            return .feature(of: of, profile: p, op: op)
        default:
            return self
        }
    }
}

extension Workbench {
    // What the bar says to do next.
    func sketchHint() -> String {
        guard let s = sketch else { return L("Click the bed or a flat face to sketch on") }
        switch s.stage {
        case .extrude: return s.chosen.isEmpty ? L("Click the regions to extrude") : L("Set the distance · Enter makes it")
        case .revolve: return L("Click the regions to turn · click a line for the axis")
        case .draw: break
        }
        switch s.tool {
        case .select: return L("Click curves and points to constrain them · drag a point to move it")
        case .line: return s.chain >= 0 || !s.clicks.isEmpty ? L("Click the next point · Enter or Esc ends the line") : L("Click where the line starts")
        case .rectangle: return s.clicks.isEmpty ? L("Click a corner") : L("Click the opposite corner")
        case .circle: return s.clicks.isEmpty ? L("Click the centre") : L("Click to set the radius")
        case .arc: return s.clicks.count == 0 ? L("Click where the arc starts") : s.clicks.count == 1 ? L("Click where it ends") : L("Click a point it passes through")
        case .dimension: return s.picks.isEmpty ? L("Click a line, a circle or two points to dimension") : L("Click another to measure between, or where the value goes")
        }
    }
}

// The sketch's bar: the planes to start on; then the tools, the constraints that suit what's chosen (and those on it, to
// take off), how free it still is; then extrude's or revolve's sizes and what they do.
struct SketchBar: View {
    @Environment(Workbench.self) private var lib

    var body: some View {
        if let s = lib.sketch {
            switch s.stage {
            case .draw: drawing(s)
            case .extrude: forming(s, revolve: false)
            case .revolve: forming(s, revolve: true)
            }
        } else {
            HStack(spacing: 4) {
                Chip(text: L("Bed"), chosen: false, tint: lib.accent2) { lib.sketchOnPlane(0) }
                Chip(text: L("Front"), chosen: false, tint: lib.accent2) { lib.sketchOnPlane(1) }
                Chip(text: L("Side"), chosen: false, tint: lib.accent2) { lib.sketchOnPlane(2) }
            }
        }
    }

    @ViewBuilder private func drawing(_ s: SketchSession) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack(spacing: 4) {
                tool(.select, L("Select"), "")
                tool(.line, L("Line"), "L")
                tool(.rectangle, L("Rectangle"), "R")
                tool(.circle, L("Circle"), "C")
                tool(.arc, L("Arc"), "A")
                tool(.dimension, L("Dimension"), "D")
                if s.selection.contains(where: { if case .curve = $0 { true } else { false } }) {
                    Chip(text: L("Construction"), chosen: false, tint: lib.accent3) { lib.toggleConstruction() }
                        .help(L("Construction lines bound no region (X)"))
                }
                Spacer(minLength: 12)
                Text(freedom(s)).foregroundStyle(s.solved.freedom == 0 ? lib.accent : Ink.text.opacity(0.55)).monospacedDigit()
            }
            HStack(spacing: 4) {
                ForEach(lib.sketchConstraints(), id: \.kind) { c in
                    Chip(text: c.name, chosen: false, tint: lib.accent2) { lib.applyConstraint(c.kind) }
                }
                ForEach(lib.rulesOnSelection(), id: \.self) { i in
                    Chip(text: Workbench.ruleName(s.sketch.rules[i].kind) + "  ×", chosen: true, tint: lib.accent3) { lib.removeRule(i) }
                        .help(L("Take this constraint off"))
                }
                Spacer(minLength: 12)
                Button(L("Extrude")) { lib.sketchExtrude() }
                    .buttonStyle(PillStyle(tint: lib.accent2))
                    .frame(width: 96)
                    .help(L("Stand regions up (E)"))
                Button(L("Revolve")) { lib.sketchRevolve() }
                    .buttonStyle(PillStyle(tint: lib.accent2))
                    .frame(width: 96)
                    .help(L("Turn regions about a line (V)"))
            }
        }
    }

    private func freedom(_ s: SketchSession) -> String {
        if !s.solved.ok { return L("The constraints can't all hold") }
        return s.solved.freedom == 0 ? L("Fully constrained") : L("{n} degrees of freedom", ["n": s.solved.freedom])
    }

    private func tool(_ t: SketchTool, _ name: String, _ key: String) -> some View {
        Chip(text: name, chosen: lib.sketch?.tool == t, tint: lib.accent2) { lib.setSketchTool(t) }
            .help(key.isEmpty ? name : "\(name) (\(key))")
    }

    @ViewBuilder private func forming(_ s: SketchSession, revolve: Bool) -> some View {
        HStack(spacing: 10) {
            if revolve {
                Text(L("Angle")).foregroundStyle(Ink.text.opacity(0.55))
                MMField(value: s.angle, unit: "°", range: 0.1...360, width: 58) { lib.setSketchForm(angle: $0) }
            } else {
                Text(L("Distance")).foregroundStyle(Ink.text.opacity(0.55))
                MMField(value: s.distance, unit: L("mm"), range: -10_000...10_000, width: 64) { v in if v != 0 { lib.setSketchForm(distance: v) } }
                Chip(text: L("Symmetric"), chosen: s.symmetric, tint: lib.accent2) { lib.setSketchForm(symmetric: !s.symmetric) }
                    .help(L("As far each way from the sketch"))
                Chip(text: L("Flip"), chosen: false, tint: lib.accent2) { lib.setSketchForm(distance: -s.distance) }
                    .help(L("The other way from the sketch"))
            }
            HStack(spacing: 4) {
                Chip(text: L("New body"), chosen: s.op < 0, tint: lib.accent) { lib.setSketchForm(op: -1) }
                if s.joinable {
                    Chip(text: L("Merge"), chosen: s.op == Int32(BK_UNION), tint: lib.accent) { lib.setSketchForm(op: Int32(BK_UNION)) }
                    Chip(text: L("Subtract"), chosen: s.op == Int32(BK_SUBTRACT), tint: Neon.red) { lib.setSketchForm(op: Int32(BK_SUBTRACT)) }
                    Chip(text: L("Intersect"), chosen: s.op == Int32(BK_INTERSECT), tint: lib.accent) { lib.setSketchForm(op: Int32(BK_INTERSECT)) }
                }
            }
            Button(L("Apply")) { lib.sketchCommit() }
                .buttonStyle(PillStyle(tint: lib.accent))
                .frame(width: 90)
                .disabled(s.chosen.isEmpty)
        }
    }
}
