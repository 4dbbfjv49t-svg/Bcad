import Foundation
import Compression
import simd

enum FileError: Error { case corrupt, notBcad }

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

    private static func deflate(_ d: Data) -> Data? {
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

    private static func inflate(_ d: Data, size: Int) -> Data? {
        if size == 0 { return Data() }
        var out = Data(count: size)
        let n = out.withUnsafeMutableBytes { o in
            d.withUnsafeBytes { i in
                compression_decode_buffer(o.bindMemory(to: UInt8.self).baseAddress!, size, i.bindMemory(to: UInt8.self).baseAddress!, d.count, nil, COMPRESSION_ZLIB)
            }
        }
        return n == size ? out : nil
    }

    static func write(_ entries: [(String, Data)]) -> Data {
        var out = Data(), central = Data()
        func u16(_ v: Int, _ d: inout Data) { d.append(contentsOf: [UInt8(v & 0xFF), UInt8((v >> 8) & 0xFF)]) }
        func u32(_ v: UInt32, _ d: inout Data) { d.append(contentsOf: [UInt8(v & 0xFF), UInt8((v >> 8) & 0xFF), UInt8((v >> 16) & 0xFF), UInt8(v >> 24)]) }
        for (name, data) in entries {
            let packed = deflate(data)
            let body = packed ?? data
            let method = packed == nil ? 0 : 8
            let crc = crc32(data)
            let n = Data(name.utf8)
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
        let start = UInt32(out.count)
        out.append(central)
        var end = Data()
        u32(0x0605_4B50, &end); u16(0, &end); u16(0, &end); u16(entries.count, &end); u16(entries.count, &end)
        u32(UInt32(central.count), &end); u32(start, &end); u16(0, &end)
        out.append(end)
        return out
    }

    static func read(_ d: Data) throws -> [String: Data] {
        let b = [UInt8](d)
        func u16(_ i: Int) -> Int { i + 1 < b.count ? Int(b[i]) | Int(b[i + 1]) << 8 : 0 }
        func u32(_ i: Int) -> Int { u16(i) | u16(i + 2) << 16 }
        guard b.count >= 22 else { throw FileError.corrupt }
        var e = b.count - 22
        while e >= max(0, b.count - 65_557), u32(e) != 0x0605_4B50 { e -= 1 }
        guard e >= 0, u32(e) == 0x0605_4B50 else { throw FileError.corrupt }
        var p = u32(e + 16)
        var out: [String: Data] = [:]
        for _ in 0..<u16(e + 10) {
            guard u32(p) == 0x0201_4B50, p + 46 + u16(p + 28) <= b.count else { throw FileError.corrupt }
            let method = u16(p + 10), csize = u32(p + 20), usize = u32(p + 24)
            let nl = u16(p + 28), xl = u16(p + 30), cl = u16(p + 32), local = u32(p + 42)
            let name = String(decoding: b[(p + 46)..<(p + 46 + nl)], as: UTF8.self)
            p += 46 + nl + xl + cl
            guard u32(local) == 0x0403_4B50, usize <= 1 << 30 else { throw FileError.corrupt }
            let start = local + 30 + u16(local + 26) + u16(local + 28)
            guard start + csize <= b.count else { throw FileError.corrupt }
            let raw = Data(b[start..<(start + csize)])
            if method == 0 { out[name] = raw } else if method == 8, let data = inflate(raw, size: usize) { out[name] = data } else { throw FileError.corrupt }
        }
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

enum ThreeMF {
    static let modelPath = "3D/3dmodel.model"
    static let docPath = "Metadata/bcad.json"

    private static func escape(_ s: String) -> String {
        s.replacingOccurrences(of: "&", with: "&amp;").replacingOccurrences(of: "<", with: "&lt;")
            .replacingOccurrences(of: ">", with: "&gt;").replacingOccurrences(of: "\"", with: "&quot;")
    }

    private static func num(_ v: Float) -> String {
        var s = String(format: "%.4f", v)
        while s.hasSuffix("0") { s.removeLast() }
        if s.hasSuffix(".") { s.removeLast() }
        return s == "-0" ? "0" : s
    }

    // Each object carries its body's id as part number, so opening the file can show it before the kernel rebuilds it.
    static func write(_ url: URL, meshes: [(Solid, Mesh)], doc: Document) throws {
        var xml = """
        <?xml version="1.0" encoding="UTF-8"?>
        <model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
         <metadata name="Application">Bcad</metadata>
         <resources>

        """
        var items = ""
        for (i, (body, mesh)) in meshes.enumerated() {
            let (pts, tris) = Weld.run(mesh)
            guard !tris.isEmpty else { continue }
            var obj = "  <object id=\"\(i + 1)\" type=\"model\" name=\"\(escape(body.name))\" partnumber=\"\(body.id.uuidString)\">\n   <mesh>\n    <vertices>\n"
            obj.reserveCapacity(pts.count * 60 + tris.count * 50)
            for p in pts { obj += "     <vertex x=\"\(num(p.x))\" y=\"\(num(p.y))\" z=\"\(num(p.z))\"/>\n" }
            obj += "    </vertices>\n    <triangles>\n"
            for t in tris { obj += "     <triangle v1=\"\(t.x)\" v2=\"\(t.y)\" v3=\"\(t.z)\"/>\n" }
            obj += "    </triangles>\n   </mesh>\n  </object>\n"
            xml += obj
            items += "  <item objectid=\"\(i + 1)\"/>\n"
        }
        xml += " </resources>\n <build>\n" + items + " </build>\n</model>\n"
        let types = """
        <?xml version="1.0" encoding="UTF-8"?>
        <Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/><Default Extension="json" ContentType="application/json"/></Types>
        """
        let rels = """
        <?xml version="1.0" encoding="UTF-8"?>
        <Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/\(modelPath)" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>
        """
        let enc = JSONEncoder()
        enc.outputFormatting = [.sortedKeys]
        let json = try enc.encode(doc)
        let zip = Zip.write([("[Content_Types].xml", Data(types.utf8)), ("_rels/.rels", Data(rels.utf8)), (modelPath, Data(xml.utf8)), (docPath, json)])
        try zip.write(to: url, options: .atomic)
    }

    // The document, and the bodies' shapes as saved (world coordinates), by body id.
    static func read(_ url: URL) throws -> (doc: Document, meshes: [UUID: SavedMesh]) {
        let files = try Zip.read(try Data(contentsOf: url))
        guard let json = files[docPath] else { throw FileError.notBcad }
        guard let doc = try? JSONDecoder().decode(Document.self, from: json), doc.valid else { throw FileError.corrupt }
        let reader = ModelReader()
        if let model = files[modelPath] {
            let parser = XMLParser(data: model)
            parser.delegate = reader
            parser.parse()
        }
        return (doc, reader.meshes)
    }
}

struct SavedMesh {
    var points: [SIMD3<Float>] = []
    var triangles: [SIMD3<UInt32>] = []
}

// Reads the objects Bcad wrote (those with a body id as part number); one with a bad number or index is left out.
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
    var valid: Bool { bodies.allSatisfy { $0.node.valid && $0.place.valid } }
}

extension Placement {
    var valid: Bool { move.finite && turn.finite && scale.finite }
}

extension Pick {
    var valid: Bool { a.finite && b.finite }
}

extension Node {
    var valid: Bool {
        switch self {
        case .primitive(let p):
            let sides = switch p.kind {
            case .prism, .pyramid: (3...24).contains(p.sides)
            case .torus, .ovalTorus: [0, 3, 6].contains(p.sides)
            default: true
            }
            return p.size.count == Primitive.make(p.kind).size.count && p.size.allSatisfy(\.isFinite) && sides
        case .fastener(let f):
            return (0..<Int(bk_thread_count())).contains(f.size) && f.length.isFinite
        case .group(let op, let parts):
            return (0...2).contains(op) && !parts.isEmpty && parts.allSatisfy { $0.node.valid && $0.place.valid }
        case .split(let n, let plane, _):
            return n.valid && plane.point.finite && plane.normal.finite
        case .round(let n, let picks, let radius), .cove(let n, let picks, let radius):
            return n.valid && radius.isFinite && picks.allSatisfy(\.valid)
        case .bevel(let n, let picks, let legs, let corner):
            return n.valid && legs.x.isFinite && legs.y.isFinite && corner.isFinite && picks.allSatisfy(\.valid)
        case .hollow(let n, let open, let walls, let thickness):
            return n.valid && thickness.isFinite && open.allSatisfy(\.valid) && walls.allSatisfy { $0.face.valid && $0.thickness.isFinite }
        }
    }
}

// MARK: - STL (binary)

enum STL {
    static func write(_ url: URL, meshes: [Mesh]) throws {
        var tris: [(SIMD3<Float>, SIMD3<Float>, SIMD3<Float>)] = []
        for m in meshes {
            var i = 0
            while i + 2 < m.indices.count {
                let a = m.vertices[Int(m.indices[i])], b = m.vertices[Int(m.indices[i + 1])], c = m.vertices[Int(m.indices[i + 2])]
                tris.append((SIMD3(a.x, a.y, a.z), SIMD3(b.x, b.y, b.z), SIMD3(c.x, c.y, c.z)))
                i += 3
            }
        }
        var d = Data(capacity: 84 + tris.count * 50)
        var header = Data("Bcad binary STL, millimetres".utf8)
        header.append(Data(count: 80 - header.count))
        d.append(header)
        var n = UInt32(tris.count).littleEndian
        d.append(Data(bytes: &n, count: 4))
        for (a, b, c) in tris {
            let cr = cross(b - a, c - a)
            let nrm = length(cr) > 0 ? normalize(cr) : SIMD3<Float>(0, 0, 0)
            var f: [Float] = [nrm.x, nrm.y, nrm.z, a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z]
            d.append(Data(bytes: &f, count: 48))
            d.append(contentsOf: [0, 0])
        }
        try d.write(to: url, options: .atomic)
    }
}
