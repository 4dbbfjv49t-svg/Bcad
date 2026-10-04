// Sculpting a body by hand: the brushes, and the engine's live mesh of the body being shaped (Engine/Sculpt.hpp).
import Foundation
import simd

// The brushes, numbered as the engine knows them (BK_BRUSH_…).
enum SculptBrush: Int, CaseIterable {
    case grab, draw, inflate, smooth, flatten, pinch, crease

    var name: String {
        switch self {
        case .grab: "Grab"
        case .draw: "Draw"
        case .inflate: "Inflate"
        case .smooth: "Smooth"
        case .flatten: "Flatten"
        case .pinch: "Pinch"
        case .crease: "Crease"
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
        }
    }

    @MainActor var label: String { L(name) }
}

// The brush's circle where the pointer is on the surface (in the body's own coordinates).
struct SculptRing: Equatable {
    var at: SIMD3<Double>
    var normal: SIMD3<Double>
}

// A body being sculpted: the engine's mesh of it as it is now (its points and normals change with every stroke, its
// triangles never), the strokes made on it to undo, and which points changed, in order, for the view to catch up with.
final class SculptSession {
    private let ptr: OpaquePointer
    // The detail it was made at (mm), and its largest side.
    let detail: Double
    let size: Double
    let vertexCount: Int
    let triangleCount: Int
    // Points changed, in the order they changed; how many changes came before the first of them.
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
        vertexCount = d.pointCount
        triangleCount = d.triangleCount
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

    // 3 floats per point each; 3 point numbers per triangle.
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
        let n = Int(bk_sculpt_sync(ptr))
        guard n > 0, let c = bk_sculpt_changed(ptr) else { return }
        // (Long after, the view takes it all again rather than every change one by one.)
        if log.count + n > 2 * vertexCount {
            logStart += log.count
            log.removeAll(keepingCapacity: true)
        }
        log.append(contentsOf: UnsafeBufferPointer(start: c, count: n))
    }

    // The points changed since a reader's place among the changes; nil when those are forgotten (it reads all again).
    func changes(since k: Int) -> ArraySlice<UInt32>? {
        guard k >= logStart, k <= logEnd else { return nil }
        return log[(k - logStart)...]
    }

    // Its mesh as the document keeps it.
    func data() -> SculptData {
        SculptData(positions: Array(UnsafeBufferPointer(start: positions, count: 3 * vertexCount)),
                   indices: Array(UnsafeBufferPointer(start: indices, count: 3 * triangleCount)))
    }

    // Shown as its body's mesh (one smooth face) until the kernel's own takes its place.
    func mesh() -> Mesh {
        var m = Mesh()
        let p = positions, n = normals
        m.vertices = (0..<vertexCount).map { SIMD4(p[3 * $0], p[3 * $0 + 1], p[3 * $0 + 2], 0) }
        m.normals = (0..<vertexCount).map { SIMD4(n[3 * $0], n[3 * $0 + 1], n[3 * $0 + 2], 0) }
        m.indices = Array(UnsafeBufferPointer(start: indices, count: 3 * triangleCount))
        var lo = SIMD3<Float>(repeating: .infinity)
        var hi = -lo
        for v in m.vertices {
            lo = simd_min(lo, v.xyz)
            hi = simd_max(hi, v.xyz)
        }
        var signed = 0.0
        var t = 0
        while t + 2 < m.indices.count {
            let a = m.vertices[Int(m.indices[t])].xyz, b = m.vertices[Int(m.indices[t + 1])].xyz, c = m.vertices[Int(m.indices[t + 2])].xyz
            signed += Double(dot(a, cross(b, c))) / 6
            t += 3
        }
        if vertexCount > 0 {
            m.low = SIMD3<Double>(lo)
            m.high = SIMD3<Double>(hi)
        }
        m.volume = abs(signed)
        m.faceInfo = [(SIMD3(0, 0, 1), (m.low + m.high) / 2)]
        return m
    }
}
