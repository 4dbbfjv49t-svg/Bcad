// Sculpting a body by hand: the brushes, and the engine's live mesh of the body being shaped (Engine/Sculpt.hpp).
import Foundation
import simd

// The brushes, numbered as the engine knows them (BK_BRUSH_…).
enum SculptBrush: Int, CaseIterable {
    case grab, draw, inflate, smooth, flatten, pinch, crease, detail

    var name: String {
        switch self {
        case .grab: "Grab"
        case .draw: "Draw"
        case .inflate: "Inflate"
        case .smooth: "Smooth"
        case .flatten: "Flatten"
        case .pinch: "Pinch"
        case .crease: "Crease"
        case .detail: "Detail"
        }
    }

    var hint: String {
        switch self {
        case .grab: "Pulls the surface along with the pointer"
        case .draw: "Raises the surface · ⌥ carves it"
        case .inflate: "Swells the surface outwards · ⌥ shrinks it"
        case .smooth: "Evens out bumps"
        case .flatten: "Levels the surface"
        case .pinch: "Draws the surface together · ⌥ spreads it"
        case .crease: "Cuts a sharp groove · ⌥ makes a ridge"
        case .detail: "Makes the triangles under it the Detail size · the surface stays where it is"
        }
    }

    @MainActor var label: String { L(name) }
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

    func begin(_ b: SculptBrush, at p: SIMD3<Double>, radius: Double, strength: Double, mirror: Bool, invert: Bool) {
        let a = [p.x, p.y, p.z]
        bk_sculpt_begin(ptr, Int32(b.rawValue), a, radius, strength, mirror ? 1 : 0, invert ? 1 : 0)
    }

    // Pressure scales its strength, size its radius (0…1; both 1 for a mouse).
    func dab(_ p: SIMD3<Double>, pressure: Double, size: Double = 1) {
        let a = [p.x, p.y, p.z]
        bk_sculpt_dab(ptr, a, pressure, size)
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

    // Shown as its body's mesh (one smooth face) until the kernel's own takes its place.
    func mesh() -> Mesh {
        var m = Mesh()
        let d = data(), p = d.positions
        let count = p.count / 3
        m.vertices = (0..<count).map { SIMD4(p[3 * $0], p[3 * $0 + 1], p[3 * $0 + 2], 0) }
        m.indices = d.indices
        // Each point's normal (its triangles' own, by their areas), and the volume.
        var sums = [SIMD3<Float>](repeating: .zero, count: count)
        var signed = 0.0
        var t = 0
        while t + 2 < m.indices.count {
            let i = Int(m.indices[t]), j = Int(m.indices[t + 1]), k = Int(m.indices[t + 2])
            let a = m.vertices[i].xyz, b = m.vertices[j].xyz, c = m.vertices[k].xyz
            let f = cross(b - a, c - a)
            sums[i] += f
            sums[j] += f
            sums[k] += f
            signed += Double(dot(a, cross(b, c))) / 6
            t += 3
        }
        m.normals = sums.map { f -> SIMD4<Float> in length(f) > 0 ? SIMD4(normalize(f), 0) : .zero }
        var lo = SIMD3<Float>(repeating: .infinity)
        var hi = -lo
        for v in m.vertices {
            lo = simd_min(lo, v.xyz)
            hi = simd_max(hi, v.xyz)
        }
        if count > 0 {
            m.low = SIMD3<Double>(lo)
            m.high = SIMD3<Double>(hi)
        }
        m.volume = abs(signed)
        m.faceInfo = [(SIMD3(0, 0, 1), (m.low + m.high) / 2)]
        return m
    }
}
