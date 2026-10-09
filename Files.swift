import Foundation
import Compression
import CoreGraphics
import ImageIO
import simd

enum FileError: Error { case corrupt, notBcad, newer, tooLarge }

// MARK: - ZIP (deflate via Apple's Compression framework)

enum Zip {
    private static let crcTable: [UInt32] = (0..<256).map { i -> UInt32 in
        var c = UInt32(i)
        for _ in 0..<8 { c = c & 1 != 0 ? 0xEDB8_8320 ^ (c >> 1) : c >> 1 }
        return c
    }

    static func crc32(_ d: Data) -> UInt32 {
        var c: UInt32 = 0xFFFF_FFFF
        d.withUnsafeBytes { for b in $0 { c = crcTable[Int((c ^ UInt32(b)) & 0xFF)] ^ (c >> 8) } }
        return c ^ 0xFFFF_FFFF
    }

    static func deflate(_ d: Data) -> Data? {
        guard !d.isEmpty else { return nil }
        let cap = d.count + d.count / 8 + 1024
        var out = Data(count: cap)
        let n = out.withUnsafeMutableBytes { o in
            d.withUnsafeBytes { i in
                compression_encode_buffer(o.bindMemory(to: UInt8.self).baseAddress!, cap, i.bindMemory(to: UInt8.self).baseAddress!, d.count, nil, COMPRESSION_ZLIB)
            }
        }
        guard n > 0, n < d.count else { return nil }
        return out.prefix(n)
    }

    // A damaged entry (nothing packed, or more unpacked than deflate can make of it) is refused, not unpacked.
    static func inflate(_ d: Data, size: Int) -> Data? {
        if size == 0 { return Data() }
        guard !d.isEmpty, size <= 1 << 30, size <= d.count * 1032 else { return nil }
        var out = Data(count: size)
        let n = out.withUnsafeMutableBytes { o in
            d.withUnsafeBytes { i -> Int in
                guard let to = o.bindMemory(to: UInt8.self).baseAddress, let from = i.bindMemory(to: UInt8.self).baseAddress else { return 0 }
                return compression_decode_buffer(to, size, from, d.count, nil, COMPRESSION_ZLIB)
            }
        }
        return n == size ? out : nil
    }

    // (No ZIP64: past 4 GiB it's refused.)
    static func write(_ entries: [(String, Data)]) throws -> Data {
        var out = Data(), central = Data()
        func u16(_ v: Int, _ d: inout Data) { d.append(contentsOf: [UInt8(v & 0xFF), UInt8((v >> 8) & 0xFF)]) }
        func u32(_ v: UInt32, _ d: inout Data) { d.append(contentsOf: [UInt8(v & 0xFF), UInt8((v >> 8) & 0xFF), UInt8((v >> 16) & 0xFF), UInt8(v >> 24)]) }
        for (name, data) in entries {
            let packed = deflate(data)
            let body = packed ?? data
            let method = packed == nil ? 0 : 8
            let crc = crc32(data)
            let n = Data(name.utf8)
            guard out.count + 30 + n.count + body.count < 1 << 32, data.count < 1 << 32 else { throw FileError.tooLarge }
            let offset = UInt32(out.count)
            var head = Data()
            u32(0x0403_4B50, &head); u16(20, &head); u16(0x0800, &head); u16(method, &head); u16(0, &head); u16(0x21, &head)
            u32(crc, &head); u32(UInt32(body.count), &head); u32(UInt32(data.count), &head); u16(n.count, &head); u16(0, &head)
            out.append(head); out.append(n); out.append(body)
            u32(0x0201_4B50, &central); u16(20, &central); u16(20, &central); u16(0x0800, &central); u16(method, &central); u16(0, &central); u16(0x21, &central)
            u32(crc, &central); u32(UInt32(body.count), &central); u32(UInt32(data.count), &central); u16(n.count, &central)
            u16(0, &central); u16(0, &central); u16(0, &central); u16(0, &central); u32(0, &central); u32(offset, &central)
            central.append(n)
        }
        guard out.count + central.count < 1 << 32 else { throw FileError.tooLarge }
        let start = UInt32(out.count)
        out.append(central)
        var end = Data()
        u32(0x0605_4B50, &end); u16(0, &end); u16(0, &end); u16(entries.count, &end); u16(entries.count, &end)
        u32(UInt32(central.count), &end); u32(start, &end); u16(0, &end)
        out.append(end)
        return out
    }

    // The entries named (every one when none are; or those `matching` says), each unpacked and checked against its CRC. One
    // larger unpacked than its `limit` is left out; one damaged, larger than 1 GiB, or overlapping another is refused.
    static func read(_ d: Data, only names: Set<String>? = nil, matching: ((String) -> Bool)? = nil, limit: [String: Int] = [:]) throws -> [String: Data] {
        let b = [UInt8](d)
        func u16(_ i: Int) -> Int { i + 1 < b.count ? Int(b[i]) | Int(b[i + 1]) << 8 : 0 }
        func u32(_ i: Int) -> Int { u16(i) | u16(i + 2) << 16 }
        guard b.count >= 22 else { throw FileError.corrupt }
        var e = b.count - 22
        while e >= max(0, b.count - 65_557), u32(e) != 0x0605_4B50 { e -= 1 }
        guard e >= 0, u32(e) == 0x0605_4B50 else { throw FileError.corrupt }
        var p = u32(e + 16)
        var out: [String: Data] = [:]
        var spans: [Range<Int>] = []
        for _ in 0..<u16(e + 10) {
            guard u32(p) == 0x0201_4B50, p + 46 + u16(p + 28) <= b.count else { throw FileError.corrupt }
            let method = u16(p + 10), crc = u32(p + 16), csize = u32(p + 20), usize = u32(p + 24)
            let nl = u16(p + 28), xl = u16(p + 30), cl = u16(p + 32), local = u32(p + 42)
            let name = String(decoding: b[(p + 46)..<(p + 46 + nl)], as: UTF8.self)
            p += 46 + nl + xl + cl
            guard u32(local) == 0x0403_4B50, usize <= 1 << 30 else { throw FileError.corrupt }
            let start = local + 30 + u16(local + 26) + u16(local + 28)
            guard start + csize <= b.count else { throw FileError.corrupt }
            spans.append(local..<(start + csize))
            guard names?.contains(name) ?? true, matching?(name) ?? true, usize <= limit[name] ?? Int.max else { continue }
            let raw = Data(b[start..<(start + csize)])
            let data: Data
            if method == 0, csize == usize { data = raw } else if method == 8, let unpacked = inflate(raw, size: usize) { data = unpacked } else { throw FileError.corrupt }
            guard crc32(data) == UInt32(crc) else { throw FileError.corrupt }
            out[name] = data
        }
        // (Entries sharing their bytes would let a small file unpack to a great deal.)
        spans.sort { $0.lowerBound < $1.lowerBound }
        for (a, b) in zip(spans, spans.dropFirst()) where a.upperBound > b.lowerBound { throw FileError.corrupt }
        return out
    }
}

// MARK: - Mesh helpers

enum Weld {
    // Shared vertices (faces meet exactly on their boundaries) and triangles without degenerate ones.
    static func run(_ m: Mesh) -> (points: [SIMD3<Float>], triangles: [SIMD3<UInt32>]) {
        var index: [SIMD3<Int64>: UInt32] = [:]
        var points: [SIMD3<Float>] = []
        var remap = [UInt32](repeating: 0, count: m.vertices.count)
        for (i, v) in m.vertices.enumerated() {
            let k = SIMD3<Int64>(Int64((Double(v.x) * 1e4).rounded()), Int64((Double(v.y) * 1e4).rounded()), Int64((Double(v.z) * 1e4).rounded()))
            if let j = index[k] { remap[i] = j } else {
                let j = UInt32(points.count)
                index[k] = j
                points.append(SIMD3(v.x, v.y, v.z))
                remap[i] = j
            }
        }
        var tris: [SIMD3<UInt32>] = []
        tris.reserveCapacity(m.indices.count / 3)
        var i = 0
        while i + 2 < m.indices.count {
            let t = SIMD3(remap[Int(m.indices[i])], remap[Int(m.indices[i + 1])], remap[Int(m.indices[i + 2])])
            if t.x != t.y && t.y != t.z && t.x != t.z { tris.append(t) }
            i += 3
        }
        return (points, tris)
    }
}

// MARK: - 3MF (with the editable Bcad model inside)

// Each body an object of its own, named and coloured as in Bcad, its triangles as printers take them; the model's origin
// the bed's front left corner (as the format has it), so slicers put each where it stands on Bcad's bed. Inside too: the
// document itself, to edit again, and a picture of it for file browsers.
enum ThreeMF {
    static let modelPath = "3D/3dmodel.model"
    static let docPath = "Metadata/bcad.json"
    static let thumbnailPath = "Metadata/thumbnail.png"

    // Text as XML takes it: no control characters, the five marks escaped.
    static func escape(_ s: String) -> String {
        let plain = String(String.UnicodeScalarView(s.unicodeScalars.filter { $0.value >= 0x20 && $0.value != 0x7F && $0.value != 0xFFFE && $0.value != 0xFFFF }))
        return plain.replacingOccurrences(of: "&", with: "&amp;").replacingOccurrences(of: "<", with: "&lt;")
            .replacingOccurrences(of: ">", with: "&gt;").replacingOccurrences(of: "\"", with: "&quot;").replacingOccurrences(of: "'", with: "&apos;")
    }

    // A float as written: the shortest text that reads back as the same float.
    private static func num(_ v: Float) -> String {
        var s = v == 0 ? "0" : "\(v)"
        if s.hasSuffix(".0") { s.removeLast(2) }
        return s
    }

    static func hex(_ c: SIMD3<UInt8>) -> String { String(format: "#%02X%02X%02X", c.x, c.y, c.z) }

    // Where Bcad's origin (the bed's middle) lies from the bed's front left corner.
    static func shift(bed: SIMD3<Double>) -> SIMD3<Float> { SIMD3(Float(bed.x / 2), Float(bed.y / 2), 0) }

    // Each object carries its body's id as part number, so opening the file can show it before the kernel rebuilds it.
    static func write(_ url: URL, meshes: [(Solid, SavedMesh)], doc: Document, bed: SIMD3<Double>, thumbnail: Data? = nil) throws {
        var colors: [SIMD3<UInt8>] = []
        for (body, _) in meshes where !colors.contains(body.color) { colors.append(body.color) }
        var xml = """
        <?xml version="1.0" encoding="UTF-8"?>
        <model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
         <metadata name="Application">Bcad</metadata>
         <resources>

        """
        if !colors.isEmpty {
            xml += "  <basematerials id=\"1\">\n"
            for c in colors { xml += "   <base name=\"\(hex(c))\" displaycolor=\"\(hex(c))\"/>\n" }
            xml += "  </basematerials>\n"
        }
        let t = shift(bed: bed)
        let move = "1 0 0 0 1 0 0 0 1 \(num(t.x)) \(num(t.y)) \(num(t.z))"
        var items = ""
        for (i, (body, mesh)) in meshes.enumerated() where !mesh.triangles.isEmpty {
            let id = i + 2, color = colors.firstIndex(of: body.color) ?? 0
            var obj = "  <object id=\"\(id)\" type=\"model\" name=\"\(escape(body.name))\" partnumber=\"\(body.id.uuidString)\" pid=\"1\" pindex=\"\(color)\">\n   <mesh>\n    <vertices>\n"
            obj.reserveCapacity(mesh.points.count * 70 + mesh.triangles.count * 50)
            for p in mesh.points { obj += "     <vertex x=\"\(num(p.x))\" y=\"\(num(p.y))\" z=\"\(num(p.z))\"/>\n" }
            obj += "    </vertices>\n    <triangles>\n"
            for t in mesh.triangles { obj += "     <triangle v1=\"\(t.x)\" v2=\"\(t.y)\" v3=\"\(t.z)\"/>\n" }
            obj += "    </triangles>\n   </mesh>\n  </object>\n"
            xml += obj
            items += "  <item objectid=\"\(id)\" transform=\"\(move)\"/>\n"
        }
        xml += " </resources>\n <build>\n" + items + " </build>\n</model>\n"
        let types = """
        <?xml version="1.0" encoding="UTF-8"?>
        <Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/><Default Extension="json" ContentType="application/json"/><Default Extension="png" ContentType="image/png"/></Types>
        """
        let picture = thumbnail.map { _ in "<Relationship Target=\"/\(thumbnailPath)\" Id=\"rel1\" Type=\"http://schemas.openxmlformats.org/package/2006/relationships/metadata/thumbnail\"/>" } ?? ""
        let rels = """
        <?xml version="1.0" encoding="UTF-8"?>
        <Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/\(modelPath)" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>\(picture)</Relationships>
        """
        let enc = JSONEncoder()
        enc.outputFormatting = [.sortedKeys]
        let json = try enc.encode(doc)
        var entries = [("[Content_Types].xml", Data(types.utf8)), ("_rels/.rels", Data(rels.utf8)), (modelPath, Data(xml.utf8)), (docPath, json)]
        if let thumbnail { entries.append((thumbnailPath, thumbnail)) }
        let zip = try Zip.write(entries)
        try zip.write(to: url, options: .atomic)
    }

    // The document, and the bodies' shapes as saved (world coordinates), by body id. Those are only shown until the
    // engine has made the shapes again: past 512 MiB, or damaged, they're left out.
    static func read(_ url: URL) throws -> (doc: Document, meshes: [UUID: SavedMesh]) {
        let files = try Zip.read(try Data(contentsOf: url, options: .mappedIfSafe), only: [docPath, modelPath], limit: [docPath: 256 << 20, modelPath: 512 << 20])
        guard let json = files[docPath] else { throw FileError.notBcad }
        struct Head: Decodable { var version: Int? }
        if let v = (try? JSONDecoder().decode(Head.self, from: json))?.version, v > Document.version { throw FileError.newer }
        guard let doc = try? JSONDecoder().decode(Document.self, from: json), doc.valid else { throw FileError.corrupt }
        let reader = ModelReader()
        if let model = files[modelPath] {
            let parser = XMLParser(data: model)
            parser.delegate = reader
            if !parser.parse() { reader.meshes = [:] }
        }
        return (doc, reader.meshes)
    }
}

struct SavedMesh {
    var points: [SIMD3<Float>] = []
    var triangles: [SIMD3<UInt32>] = []

    // What its triangles hold (each's signed share from the origin).
    var volume: Double {
        triangles.reduce(0) { v, t in
            let a = SIMD3<Double>(points[Int(t.x)]), b = SIMD3<Double>(points[Int(t.y)]), c = SIMD3<Double>(points[Int(t.z)])
            return v + dot(a, cross(b, c)) / 6
        }
    }
}

// Reads the objects Bcad wrote (those with a body id as part number); one with a bad number or index is left out. (Their
// points are as Bcad places them: the build items' move to the bed's corner is for slicers.)
private final class ModelReader: NSObject, XMLParserDelegate {
    var meshes: [UUID: SavedMesh] = [:]
    private var id: UUID?
    private var mesh = SavedMesh()

    func parser(_ parser: XMLParser, didStartElement name: String, namespaceURI: String?, qualifiedName: String?, attributes a: [String: String] = [:]) {
        switch name {
        case "object":
            id = a["partnumber"].flatMap(UUID.init)
            mesh = SavedMesh()
        case "vertex" where id != nil:
            mesh.points.append(SIMD3(Float(a["x"] ?? "") ?? .nan, Float(a["y"] ?? "") ?? .nan, Float(a["z"] ?? "") ?? .nan))
        case "triangle" where id != nil:
            let v = ["v1", "v2", "v3"].compactMap { a[$0].flatMap { UInt32($0) } }
            if v.count == 3 { mesh.triangles.append(SIMD3(v[0], v[1], v[2])) } else { mesh.triangles.append(SIMD3(repeating: .max)) }
        default: break
        }
    }

    func parser(_ parser: XMLParser, didEndElement name: String, namespaceURI: String?, qualifiedName: String?) {
        guard name == "object", let id else { return }
        let n = UInt32(mesh.points.count)
        if !mesh.triangles.isEmpty, mesh.points.allSatisfy({ $0.x.isFinite && $0.y.isFinite && $0.z.isFinite && simd_reduce_max(simd_abs($0)) < 1e5 }),
           mesh.triangles.allSatisfy({ $0.max() < n }) {
            meshes[id] = mesh
        }
        self.id = nil
    }
}

// MARK: - Checks on a read document (a damaged or hand-edited file must not reach the kernel or the views)

extension SIMD3 where Scalar == Double {
    var finite: Bool { x.isFinite && y.isFinite && z.isFinite }
}

extension Document {
    // As many shapes as anyone would make, none larger than 10 m, nested no deeper than 64 merges.
    var valid: Bool {
        bodies.count <= 10_000 && bodies.allSatisfy { $0.node.valid && $0.place.valid && ($0.link.map { $0.shell.valid && $0.place.valid } ?? true) }
    }
}

extension Placement {
    var valid: Bool {
        let s = simd_abs(scale)
        return move.finite && turn.finite && scale.finite && simd_reduce_min(s) > 1e-9 && simd_reduce_max(s) < 1e6
    }
}

extension Pick {
    var valid: Bool { a.finite && b.finite }
}

extension Node {
    var valid: Bool { valid(depth: 0) }

    func valid(depth: Int) -> Bool {
        guard depth <= 64 else { return false }
        switch self {
        case .primitive(let p):
            let sides = switch p.kind {
            case .prism, .pyramid: (3...24).contains(p.sides)
            case .torus, .ovalTorus: [0, 3, 6].contains(p.sides)
            default: true
            }
            return p.size.count == Primitive.make(p.kind).size.count && p.size.allSatisfy { $0.isFinite && abs($0) <= 10_000 } && sides
        case .fastener(let f):
            return (0..<Int(bk_thread_count())).contains(f.size) && [f.length, f.width, f.height, f.angle, f.seat, f.drive, f.recess, f.depth].allSatisfy { $0.isFinite && abs($0) <= 10_000 }
        case .group(let op, let parts):
            return (0...2).contains(op) && !parts.isEmpty && parts.allSatisfy { $0.node.valid(depth: depth + 1) && $0.place.valid }
        case .split(let n, let plane, _):
            return n.valid(depth: depth + 1) && plane.point.finite && plane.normal.finite
        case .round(let n, let picks, let radius), .cove(let n, let picks, let radius):
            return n.valid(depth: depth + 1) && radius.isFinite && picks.allSatisfy(\.valid)
        case .bevel(let n, let picks, let legs, let corner):
            return n.valid(depth: depth + 1) && legs.x.isFinite && legs.y.isFinite && corner.isFinite && picks.allSatisfy(\.valid)
        case .hollow(let n, let open, let walls, let thickness):
            return n.valid(depth: depth + 1) && thickness.isFinite && open.allSatisfy(\.valid) && walls.allSatisfy { $0.face.valid && $0.thickness.isFinite }
        case .sculpt(let s):
            // Within 10 m, every triangle's corners among its points (whether it's a closed solid is the engine's to tell).
            let d = s.data, n = UInt32(d.pointCount)
            return s.detail.isFinite && s.detail > 0 && d.triangleCount > 0 && d.positions.allSatisfy { $0.isFinite && abs($0) <= 10_000 } &&
                d.indices.allSatisfy { $0 < n }
        case .figure(let f):
            return f.valid
        case .profile(let p):
            return p.valid
        case .feature(let n, let p, let op):
            return n.valid(depth: depth + 1) && p.valid && (0...2).contains(op)
        }
    }
}

// MARK: - STL (binary)

enum STL {
    static func write(_ url: URL, meshes: [SavedMesh]) throws {
        let count = meshes.reduce(0) { $0 + $1.triangles.count }
        var d = Data(capacity: 84 + count * 50)
        var header = Data("Bcad binary STL, millimetres".utf8)
        header.append(Data(count: 80 - header.count))
        d.append(header)
        var n = UInt32(count).littleEndian
        d.append(Data(bytes: &n, count: 4))
        for m in meshes {
            for t in m.triangles {
                let a = m.points[Int(t.x)], b = m.points[Int(t.y)], c = m.points[Int(t.z)]
                let cr = cross(b - a, c - a)
                let nrm = length(cr) > 0 ? normalize(cr) : SIMD3<Float>(0, 0, 0)
                var f: [Float] = [nrm.x, nrm.y, nrm.z, a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z]
                d.append(Data(bytes: &f, count: 48))
                d.append(contentsOf: [0, 0])
            }
        }
        try d.write(to: url, options: .atomic)
    }
}

// MARK: - Thumbnail

// A small picture of the bodies for file browsers: seen from the front, a little from the right and above, each in its
// colour, lit from above left, on nothing (a z-buffer of its own, so it's made off the main thread too).
enum Thumbnail {
    static func png(_ bodies: [(SavedMesh, SIMD3<UInt8>)], size n: Int = 256) -> Data? {
        let yaw: Float = -.pi / 6, pitch: Float = .pi / 5
        func view(_ p: SIMD3<Float>) -> SIMD3<Float> {
            let x = p.x * cos(yaw) - p.y * sin(yaw), y = p.x * sin(yaw) + p.y * cos(yaw)
            // Screen right x, up y, towards the eye z.
            return SIMD3(x, p.z * cos(pitch) + y * sin(pitch), -y * cos(pitch) + p.z * sin(pitch))
        }
        var lo = SIMD3<Float>(repeating: .infinity), hi = SIMD3<Float>(repeating: -.infinity)
        let seen = bodies.map { ($0.0.points.map(view), $0.0.triangles, $0.1) }
        for (pts, _, _) in seen { for p in pts { lo = simd_min(lo, p); hi = simd_max(hi, p) } }
        guard lo.x <= hi.x else { return nil }
        let scale = Float(n) * 0.9 / max(hi.x - lo.x, hi.y - lo.y, 1e-6), middle = (lo + hi) / 2
        var depth = [Float](repeating: -.infinity, count: n * n), rgba = [UInt8](repeating: 0, count: n * n * 4)
        let light = normalize(SIMD3<Float>(-0.4, 0.6, 0.7))
        for (pts, tris, color) in seen {
            let px = pts.map { p in SIMD3(Float(n) / 2 + (p.x - middle.x) * scale, Float(n) / 2 - (p.y - middle.y) * scale, p.z) }
            for t in tris {
                let a = px[Int(t.x)], b = px[Int(t.y)], c = px[Int(t.z)]
                let normal = cross(pts[Int(t.y)] - pts[Int(t.x)], pts[Int(t.z)] - pts[Int(t.x)])
                guard length(normal) > 0, normal.z > 0 else { continue }
                let shade = 0.35 + 0.65 * max(0, dot(normalize(normal), light))
                let area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
                guard area != 0 else { continue }
                let x0 = max(0, Int(min(a.x, b.x, c.x))), x1 = min(n - 1, Int(max(a.x, b.x, c.x)))
                let y0 = max(0, Int(min(a.y, b.y, c.y))), y1 = min(n - 1, Int(max(a.y, b.y, c.y)))
                guard x0 <= x1, y0 <= y1 else { continue }
                for y in y0...y1 {
                    for x in x0...x1 {
                        let q = SIMD2(Float(x) + 0.5, Float(y) + 0.5)
                        let wa = ((b.x - q.x) * (c.y - q.y) - (b.y - q.y) * (c.x - q.x)) / area
                        let wb = ((c.x - q.x) * (a.y - q.y) - (c.y - q.y) * (a.x - q.x)) / area
                        let wc = 1 - wa - wb
                        guard wa >= 0, wb >= 0, wc >= 0 else { continue }
                        let z = wa * a.z + wb * b.z + wc * c.z, i = y * n + x
                        guard z > depth[i] else { continue }
                        depth[i] = z
                        rgba[4 * i] = UInt8(Float(color.x) * shade)
                        rgba[4 * i + 1] = UInt8(Float(color.y) * shade)
                        rgba[4 * i + 2] = UInt8(Float(color.z) * shade)
                        rgba[4 * i + 3] = 255
                    }
                }
            }
        }
        guard let provider = CGDataProvider(data: Data(rgba) as CFData),
              let image = CGImage(width: n, height: n, bitsPerComponent: 8, bitsPerPixel: 32, bytesPerRow: n * 4, space: CGColorSpaceCreateDeviceRGB(),
                                  bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.last.rawValue), provider: provider, decode: nil, shouldInterpolate: false,
                                  intent: .defaultIntent) else { return nil }
        let out = NSMutableData()
        guard let dest = CGImageDestinationCreateWithData(out as CFMutableData, "public.png" as CFString, 1, nil) else { return nil }
        CGImageDestinationAddImage(dest, image, nil)
        return CGImageDestinationFinalize(dest) ? out as Data : nil
    }
}
