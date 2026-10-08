import Foundation
import simd
#if canImport(ModelIO)
import ModelIO
#endif

// Meshes from other apps and from scanners (STL, OBJ, PLY, another app's 3MF, USDZ) read and made into closed bodies by
// the engine, ready to sculpt, merge, cut and print. (Foundation, simd and ModelIO only: the same on the iPhone.)

// One body made of a file: its mesh about its own middle, and where that middle was in the file (mm).
struct ImportedBody {
    var name: String
    var data: SculptData
    var detail: Double
    var offset: SIMD3<Double>
    var low: SIMD3<Double>, high: SIMD3<Double>  // its points' box, about its middle
    var report: BKScanReport
}

struct ImportResult {
    var bodies: [ImportedBody]
    var metres = false  // the file said no unit and was under 2 units across: read as metres
}

enum ImportError: Error {
    case refused(String)  // the engine's reason ("stl: cut short", "scan: larger than 10 m" …)
    case stopped
}

// An import under way: stopped from any thread (a newer one asked for, the document changed).
final class ImportJob: @unchecked Sendable {
    private let lock = NSLock()
    private var stop = false

    func cancel() {
        lock.lock()
        stop = true
        lock.unlock()
    }

    var cancelled: Bool {
        lock.lock()
        defer { lock.unlock() }
        return stop
    }
}

enum MeshImport {
    static let meshExtensions: Set<String> = ["stl", "obj", "ply", "usdz", "usd", "usdc", "usda"]

    // Whether Bcad imports a file of this kind (a 3MF any app made, too).
    static func canRead(_ url: URL) -> Bool {
        let e = url.pathExtension.lowercased()
        return e == "3mf" || meshExtensions.contains(e)
    }

    // Read, then each part made a closed body (a file over 2 GB refused before it's read).
    static func read(_ url: URL, job: ImportJob) throws -> ImportResult {
        let ext = url.pathExtension.lowercased()
        let size = (try? url.resourceValues(forKeys: [.fileSizeKey]))?.fileSize ?? 0
        guard size <= 2 << 30 else { throw ImportError.refused("scan: too large") }
        if ["usdz", "usd", "usdc", "usda"].contains(ext) { return try scene(url, job: job) }
        let soup: UnsafeMutablePointer<BKScanSoup>?
        if ext == "3mf" {
            soup = try threeMF(url)
        } else {
            let data = try Data(contentsOf: url, options: .alwaysMapped)
            soup = data.withUnsafeBytes { raw in bk_scan_read(raw.bindMemory(to: UInt8.self).baseAddress, Int64(raw.count), ext, nil) }
        }
        guard let soup else { throw ImportError.refused(String(cString: bk_last_error())) }
        defer { bk_scan_soup_free(soup) }
        let s = soup.pointee
        var parts: [(name: String, from: Int, to: Int)] = []
        for k in 0..<Int(s.partCount) {
            parts.append((s.partNames[k].map { String(cString: $0) } ?? "", Int(s.partStart[k]), Int(s.partStart[k + 1])))
        }
        var out = try repair(s.positions, Int(s.vertexCount), s.indices, parts, job: job)
        out.metres = s.unitGuessed != 0
        return out
    }

    // Each part repaired in turn (the engine told to stop as soon as the job is).
    static func repair(_ positions: UnsafePointer<Float>, _ points: Int, _ indices: UnsafePointer<UInt32>, _ parts: [(name: String, from: Int, to: Int)],
                       job: ImportJob) throws -> ImportResult {
        var bodies: [ImportedBody] = []
        for part in parts {
            if job.cancelled { throw ImportError.stopped }
            var o = BKScanOptions()
            bk_scan_options(&o)
            o.context = Unmanaged.passUnretained(job).toOpaque()
            o.progress = { context, _ in
                guard let context else { return 1 }
                return Unmanaged<ImportJob>.fromOpaque(context).takeUnretainedValue().cancelled ? 0 : 1
            }
            var r = BKScanReport()
            guard let m = bk_scan_repair(positions, Int32(points), indices + 3 * part.from, Int32(part.to - part.from), &o, &r) else {
                let why = String(cString: bk_last_error())
                throw why == "scan: stopped" ? ImportError.stopped : ImportError.refused(why)
            }
            defer { bk_sculpt_mesh_free(m) }
            let pos = Array(UnsafeBufferPointer(start: m.pointee.positions, count: 3 * Int(m.pointee.vertexCount)))
            let idx = Array(UnsafeBufferPointer(start: m.pointee.indices, count: 3 * Int(m.pointee.triangleCount)))
            var lo = SIMD3<Double>(repeating: .infinity), hi = -lo
            for i in stride(from: 0, to: pos.count, by: 3) {
                let p = SIMD3<Double>(Double(pos[i]), Double(pos[i + 1]), Double(pos[i + 2]))
                lo = simd_min(lo, p)
                hi = simd_max(hi, p)
            }
            bodies.append(ImportedBody(name: part.name, data: SculptData(positions: pos, indices: idx), detail: r.detail,
                                       offset: SIMD3(r.offset.0, r.offset.1, r.offset.2), low: lo, high: hi, report: r))
        }
        return ImportResult(bodies: bodies)
    }

    // Another app's 3MF: its model files and relationships unpacked, the engine reading the rest.
    static func threeMF(_ url: URL) throws -> UnsafeMutablePointer<BKScanSoup>? {
        let files = try Zip.read(try Data(contentsOf: url, options: .mappedIfSafe), matching: { name in
            name.lowercased().hasSuffix(".model") || name.lowercased() == "_rels/.rels"
        })
        let names = files.keys.sorted()
        let datas = names.map { files[$0] ?? Data() }
        let cNames = names.map { strdup($0) }
        defer { for n in cNames { free(n) } }
        let pointers = cNames.map { UnsafePointer<CChar>($0) }
        let lengths = datas.map { Int64($0.count) }
        return withBytes(datas) { bytes in bk_scan_read_3mf(pointers, bytes, lengths, Int32(names.count), nil) }
    }

    // Every one of `list`'s bytes held still at once.
    private static func withBytes<T>(_ list: [Data], _ held: [UnsafePointer<UInt8>?] = [], _ body: ([UnsafePointer<UInt8>?]) -> T) -> T {
        guard held.count < list.count else { return body(held) }
        return list[held.count].withUnsafeBytes { raw in withBytes(list, held + [raw.bindMemory(to: UInt8.self).baseAddress], body) }
    }

    // A USD scene (what Apple's Object Capture makes): every mesh where the scene places it, turned from y up to z up,
    // in metres where it's under 2 units across (as photo scans are).
    static func scene(_ url: URL, job: ImportJob) throws -> ImportResult {
        #if canImport(ModelIO)
        let asset = MDLAsset(url: url)
        var positions: [Float] = [], indices: [UInt32] = []
        var parts: [(name: String, from: Int, to: Int)] = []
        let yUp = asset.upAxis.y > 0.5
        func visit(_ object: MDLObject) {
            if let mesh = object as? MDLMesh, let at = mesh.vertexAttributeData(forAttributeNamed: MDLVertexAttributePosition, as: .float3) {
                let m = MDLTransform.globalTransform(with: object, atTime: 0)
                let base = UInt32(positions.count / 3), from = indices.count / 3
                for i in 0..<mesh.vertexCount {
                    let p = at.dataStart.advanced(by: i * at.stride).assumingMemoryBound(to: Float.self)
                    var q = (m * SIMD4<Float>(p[0], p[1], p[2], 1)).xyz
                    if yUp { q = SIMD3(q.x, -q.z, q.y) }
                    positions += [q.x, q.y, q.z]
                }
                for case let sub as MDLSubmesh in mesh.submeshes ?? [] where sub.geometryType == .triangles {
                    let map = sub.indexBuffer.map(), n = sub.indexCount
                    for i in 0..<n {
                        let v: UInt32
                        switch sub.indexType {
                        case .uInt8: v = UInt32(map.bytes.load(fromByteOffset: i, as: UInt8.self))
                        case .uInt16: v = UInt32(map.bytes.load(fromByteOffset: 2 * i, as: UInt16.self))
                        default: v = map.bytes.load(fromByteOffset: 4 * i, as: UInt32.self)
                        }
                        indices.append(v < UInt32(mesh.vertexCount) ? base + v : UInt32.max)
                    }
                }
                if indices.count / 3 > from { parts.append((object.name, from, indices.count / 3)) }
            }
            for child in object.children.objects { visit(child) }
        }
        for i in 0..<asset.count { visit(asset.object(at: i)) }
        guard !indices.isEmpty else { throw ImportError.refused("usd: no triangles") }
        // (Under 2 units across: metres.)
        var lo = SIMD3<Float>(repeating: .infinity), hi = -lo
        for i in stride(from: 0, to: positions.count, by: 3) {
            let p = SIMD3(positions[i], positions[i + 1], positions[i + 2])
            lo = simd_min(lo, p)
            hi = simd_max(hi, p)
        }
        let metres = simd_reduce_max(hi - lo) < 2
        if metres { positions = positions.map { $0 * 1000 } }
        // (One body where any part isn't closed is the engine's to tell: a scene's parts are kept as one.)
        let whole = [(name: parts.count == 1 ? parts[0].name : "", from: 0, to: indices.count / 3)]
        var out = try positions.withUnsafeBufferPointer { p in
            try indices.withUnsafeBufferPointer { t in try repair(p.baseAddress!, positions.count / 3, t.baseAddress!, whole, job: job) }
        }
        out.metres = metres
        return out
        #else
        throw ImportError.refused("usd: not read here")
        #endif
    }
}

extension Mesh {
    // A sculpted body's mesh shown as one smooth face until the kernel's own takes its place (light enough for millions
    // of triangles: its points shared, not one per corner).
    init(sculpt d: SculptData) {
        self.init()
        let p = d.positions, count = p.count / 3
        vertices = (0..<count).map { SIMD4(p[3 * $0], p[3 * $0 + 1], p[3 * $0 + 2], 0) }
        indices = d.indices
        // Each point's normal (its triangles' own, by their areas), and the volume.
        var sums = [SIMD3<Float>](repeating: .zero, count: count)
        var signed = 0.0
        var t = 0
        while t + 2 < indices.count {
            let i = Int(indices[t]), j = Int(indices[t + 1]), k = Int(indices[t + 2])
            let a = vertices[i].xyz, b = vertices[j].xyz, c = vertices[k].xyz
            let f = cross(b - a, c - a)
            sums[i] += f
            sums[j] += f
            sums[k] += f
            signed += Double(dot(a, cross(b, c))) / 6
            t += 3
        }
        normals = sums.map { f -> SIMD4<Float> in length(f) > 0 ? SIMD4(normalize(f), 0) : .zero }
        var lo = SIMD3<Float>(repeating: .infinity)
        var hi = -lo
        for v in vertices {
            lo = simd_min(lo, v.xyz)
            hi = simd_max(hi, v.xyz)
        }
        if count > 0 {
            low = SIMD3<Double>(lo)
            high = SIMD3<Double>(hi)
        }
        volume = abs(signed)
        faceInfo = [(SIMD3(0, 0, 1), (low + high) / 2)]
    }
}
