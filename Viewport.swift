import SwiftUI
import AppKit
import MetalKit
import simd

// MARK: - Camera

struct Camera {
    var target = SIMD3<Float>(0, 0, 0)
    var yaw: Float = -0.6
    var pitch: Float = 0.62
    var distance: Float = 620
    let fov: Float = 0.7

    var eye: SIMD3<Float> { target + distance * SIMD3(cos(pitch) * sin(yaw), -cos(pitch) * cos(yaw), sin(pitch)) }

    // Screen-right in the world, level and from the yaw alone: still defined looking straight down or up.
    var side: SIMD3<Float> { SIMD3(cos(yaw), sin(yaw), 0) }

    var view: simd_float4x4 {
        let e = eye
        let f = normalize(target - e)
        let s = side
        let u = cross(s, f)
        return simd_float4x4(rows: [SIMD4(s, -dot(s, e)), SIMD4(u, -dot(u, e)), SIMD4(-f, dot(f, e)), SIMD4(0, 0, 0, 1)])
    }

    func projection(_ aspect: Float) -> simd_float4x4 {
        let near = max(0.2, distance * 0.005), far = distance * 30 + 3000
        let y = 1 / tan(fov / 2), x = y / aspect, z = far / (near - far)
        return simd_float4x4(rows: [SIMD4(x, 0, 0, 0), SIMD4(0, y, 0, 0), SIMD4(0, 0, z, z * near), SIMD4(0, 0, -1, 0)])
    }

    mutating func preset(_ n: Int) {
        let views: [(Float, Float)] = [(-0.6, 0.62), (0, 0), (.pi, 0), (-.pi / 2, 0), (.pi / 2, 0), (0, .pi / 2), (0, -.pi / 2)]
        (yaw, pitch) = views[max(0, min(6, n))]
    }
}

// Straight views along an axis, named by where the camera stands: Top looks down, North looks south from the +y side.
enum Side: CaseIterable {
    case top, bottom, north, south, west, east

    @MainActor var name: String {
        switch self {
        case .top: L("Top")
        case .bottom: L("Bottom")
        case .north: L("North")
        case .south: L("South")
        case .west: L("West")
        case .east: L("East")
        }
    }

    var yaw: Float {
        switch self {
        case .top, .bottom, .south: 0
        case .north: .pi
        case .west: -.pi / 2
        case .east: .pi / 2
        }
    }

    var pitch: Float { self == .top ? .pi / 2 : self == .bottom ? -.pi / 2 : 0 }
}

extension simd_float4x4 {
    init(_ m: simd_double4x4) {
        self.init(SIMD4<Float>(m.columns.0), SIMD4<Float>(m.columns.1), SIMD4<Float>(m.columns.2), SIMD4<Float>(m.columns.3))
    }
}

extension simd_double4x4 {
    init(_ m: simd_float4x4) {
        self.init(SIMD4<Double>(m.columns.0), SIMD4<Double>(m.columns.1), SIMD4<Double>(m.columns.2), SIMD4<Double>(m.columns.3))
    }
}

// MARK: - GPU types (layouts match the shader)

struct FrameU {
    var viewProj: simd_float4x4
    var eye: SIMD4<Float>
    var light: SIMD4<Float>
    var viewport: SIMD2<Float>
    var glow: Float
    var time: Float
}

struct BodyU {
    var model: simd_float4x4
    var normalM: simd_float4x4
    var color: SIMD4<Float>
    var rim: SIMD4<Float>
    var hoverFace: Int32
    var flags: Int32
    var pad0: Float = 0
    var pad1: Float = 0
}

struct LineV {
    var p: SIMD4<Float>   // this end, w = side
    var q: SIMD4<Float>   // other end, w = width in px
    var color: SIMD4<Float>
}

enum CadShader {
    static let source = """
    #include <metal_stdlib>
    using namespace metal;
    struct FrameU { float4x4 viewProj; float4 eye; float4 light; float2 viewport; float glow; float time; };
    struct BodyU { float4x4 model; float4x4 normalM; float4 color; float4 rim; int hoverFace; int flags; float pad0; float pad1; };
    struct MeshV { float4 pos; float4 nrm; };
    struct MeshOut { float4 position [[position]]; float3 world; float3 normal; float face; };
    vertex MeshOut meshVS(uint vid [[vertex_id]], const device float4 *pos [[buffer(0)]], const device float4 *nrm [[buffer(3)]],
                          constant FrameU &f [[buffer(1)]], constant BodyU &b [[buffer(2)]]) {
        MeshOut o;
        float4 w = b.model * float4(pos[vid].xyz, 1);
        o.position = f.viewProj * w;
        o.world = w.xyz;
        o.normal = (b.normalM * float4(nrm[vid].xyz, 0)).xyz;
        o.face = pos[vid].w;
        return o;
    }
    fragment float4 meshFS(MeshOut in [[stage_in]], constant FrameU &f [[buffer(1)]], constant BodyU &b [[buffer(2)]]) {
        float3 n = normalize(in.normal);
        float3 v = normalize(f.eye.xyz - in.world);
        if (dot(n, v) < 0) n = -n;
        float3 l1 = normalize(f.light.xyz);
        float3 l2 = normalize(float3(-0.4, -0.6, 0.5));
        float diff = max(dot(n, l1), 0.0) * 0.62 + max(dot(n, l2), 0.0) * 0.22 + 0.24;
        float spec = pow(max(dot(n, normalize(l1 + v)), 0.0), 48.0) * 0.28;
        float3 c = b.color.rgb * diff + spec;
        float rim = pow(1.0 - max(dot(n, v), 0.0), 2.2);
        c += b.rim.rgb * rim * b.rim.a;
        if (b.hoverFace >= 0 && abs(in.face - float(b.hoverFace)) < 0.5) c = mix(c, b.rim.rgb, 0.45);
        if (b.flags == 1) c = mix(c, b.rim.rgb, 0.18);
        return float4(c, b.color.a);
    }
    struct LineV { float4 p; float4 q; float4 color; };
    struct LineOut { float4 position [[position]]; float4 color; };
    vertex LineOut lineVS(uint vid [[vertex_id]], const device LineV *v [[buffer(0)]], constant FrameU &f [[buffer(1)]], constant float4x4 &model [[buffer(2)]]) {
        LineV l = v[vid];
        float3 wa = (model * float4(l.p.xyz, 1)).xyz, wb = (model * float4(l.q.xyz, 1)).xyz;
        wa += (f.eye.xyz - wa) * 0.003;
        wb += (f.eye.xyz - wb) * 0.003;
        float4 a = f.viewProj * float4(wa, 1);
        float4 b = f.viewProj * float4(wb, 1);
        float aw = max(a.w, 1e-4), bw = max(b.w, 1e-4);
        float2 sa = a.xy / aw * f.viewport, sb = b.xy / bw * f.viewport;
        float2 d = sb - sa;
        d = length(d) < 1e-5 ? float2(1, 0) : normalize(d);
        float2 nrm = float2(-d.y, d.x);
        float4 o = a;
        o.xy += nrm * l.p.w * l.q.w / f.viewport * a.w;
        LineOut out;
        out.position = o;
        out.color = l.color;
        return out;
    }
    fragment float4 lineFS(LineOut in [[stage_in]]) { return in.color; }
    """
}

// MARK: - Renderer

@MainActor
final class Renderer: NSObject, MTKViewDelegate {
    let device: MTLDevice
    let queue: MTLCommandQueue
    private var meshPipe: MTLRenderPipelineState!
    private var glassPipe: MTLRenderPipelineState!
    private var linePipe: MTLRenderPipelineState!
    private var depthWrite: MTLDepthStencilState!
    private var depthRead: MTLDepthStencilState!
    private var depthOff: MTLDepthStencilState!
    private struct GPUBody { var stamp: Int; var dark: Bool; var pos: MTLBuffer; var nrm: MTLBuffer; var idx: MTLBuffer; var count: Int; var edges: MTLBuffer?; var edgeCount: Int }
    private var bodies: [UUID: GPUBody] = [:]
    // The bed's lines, made again only when the bed, the theme, the accent or the brightness changes.
    private var bedLines: (key: [Float], buffer: MTLBuffer, count: Int)?
    private let start = CACurrentMediaTime()
    // Frames in a row that found no drawable.
    private var missed = 0
    weak var view: CadView?
    let lib = Workbench.shared

    override init() {
        device = MTLCreateSystemDefaultDevice()!
        queue = device.makeCommandQueue()!
        super.init()
        let library = try! device.makeLibrary(source: CadShader.source, options: nil)
        func pipe(_ v: String, _ f: String, blend: Bool) -> MTLRenderPipelineState {
            let d = MTLRenderPipelineDescriptor()
            d.vertexFunction = library.makeFunction(name: v)
            d.fragmentFunction = library.makeFunction(name: f)
            d.colorAttachments[0].pixelFormat = .bgra8Unorm
            d.depthAttachmentPixelFormat = .depth32Float
            d.rasterSampleCount = 4
            if blend {
                let c = d.colorAttachments[0]!
                c.isBlendingEnabled = true
                c.sourceRGBBlendFactor = .sourceAlpha
                c.destinationRGBBlendFactor = .oneMinusSourceAlpha
                c.sourceAlphaBlendFactor = .one
                c.destinationAlphaBlendFactor = .oneMinusSourceAlpha
            }
            return try! device.makeRenderPipelineState(descriptor: d)
        }
        meshPipe = pipe("meshVS", "meshFS", blend: false)
        glassPipe = pipe("meshVS", "meshFS", blend: true)
        linePipe = pipe("lineVS", "lineFS", blend: true)
        func depth(_ compare: MTLCompareFunction, _ write: Bool) -> MTLDepthStencilState {
            let d = MTLDepthStencilDescriptor()
            d.depthCompareFunction = compare
            d.isDepthWriteEnabled = write
            return device.makeDepthStencilState(descriptor: d)!
        }
        depthWrite = depth(.less, true)
        depthRead = depth(.lessEqual, false)
        depthOff = depth(.always, false)
    }

    nonisolated func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}

    // Draws on demand: anything the frame reads from the workbench schedules the next frame when it changes.
    nonisolated func draw(in view: MTKView) {
        MainActor.assumeIsolated {
            withObservationTracking { render(view) } onChange: { [weak view] in
                DispatchQueue.main.async { MainActor.assumeIsolated { (view as? CadView)?.redraw() } }
            }
        }
    }

    private func color(_ c: Color, _ a: Float = 1) -> SIMD4<Float> {
        let n = NSColor(c).usingColorSpace(.sRGB) ?? .white
        return SIMD4(Float(n.redComponent), Float(n.greenComponent), Float(n.blueComponent), a)
    }

    private func upload(_ id: UUID, _ m: Mesh) -> GPUBody? {
        if let g = bodies[id], g.stamp == m.stamp, g.dark == Skin.shared.dark { return g }
        guard !m.vertices.isEmpty, !m.indices.isEmpty,
              let pos = device.makeBuffer(bytes: m.vertices, length: m.vertices.count * 16),
              let nrm = device.makeBuffer(bytes: m.normals, length: m.normals.count * 16),
              let idx = device.makeBuffer(bytes: m.indices, length: m.indices.count * 4) else { bodies[id] = nil; return nil }
        var lines: [LineV] = []
        for e in m.edges { Renderer.polyline(e, width: 1.3, color: SIMD4(Renderer.ink, Skin.shared.dark ? 0.32 : 0.28), into: &lines) }
        let eb = lines.isEmpty ? nil : device.makeBuffer(bytes: lines, length: lines.count * MemoryLayout<LineV>.stride)
        let g = GPUBody(stamp: m.stamp, dark: Skin.shared.dark, pos: pos, nrm: nrm, idx: idx, count: m.indices.count, edges: eb, edgeCount: lines.count)
        bodies[id] = g
        return g
    }

    // Neutral line colour of the current theme.
    static var ink: SIMD3<Float> { Skin.shared.dark ? SIMD3(1, 1, 1) : SIMD3(0.08, 0.09, 0.13) }

    static func segment(_ a: SIMD3<Float>, _ b: SIMD3<Float>, width: Float, color: SIMD4<Float>, into out: inout [LineV]) {
        let v0 = LineV(p: SIMD4(a, 1), q: SIMD4(b, width), color: color)
        let v1 = LineV(p: SIMD4(a, -1), q: SIMD4(b, width), color: color)
        let v2 = LineV(p: SIMD4(b, -1), q: SIMD4(a, width), color: color)
        let v3 = LineV(p: SIMD4(b, 1), q: SIMD4(a, width), color: color)
        out += [v0, v1, v3, v0, v3, v2]
    }

    static func polyline(_ pts: [SIMD3<Float>], width: Float, color: SIMD4<Float>, into out: inout [LineV]) {
        guard pts.count > 1 else { return }
        for i in 1..<pts.count { segment(pts[i - 1], pts[i], width: width, color: color, into: &out) }
    }

    private func drawLines(_ enc: MTLRenderCommandEncoder, _ lines: [LineV], model: simd_float4x4 = matrix_identity_float4x4, depth: MTLDepthStencilState) {
        guard !lines.isEmpty, let buf = device.makeBuffer(bytes: lines, length: lines.count * MemoryLayout<LineV>.stride) else { return }
        drawLines(enc, buf, count: lines.count, model: model, depth: depth)
    }

    private func drawLines(_ enc: MTLRenderCommandEncoder, _ buf: MTLBuffer, count: Int, model: simd_float4x4 = matrix_identity_float4x4, depth: MTLDepthStencilState) {
        var m = model
        enc.setRenderPipelineState(linePipe)
        enc.setDepthStencilState(depth)
        enc.setVertexBuffer(buf, offset: 0, index: 0)
        enc.setVertexBytes(&m, length: 64, index: 2)
        enc.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: count)
    }

    func frame(_ size: CGSize) -> FrameU {
        let aspect = Float(max(1, size.width) / max(1, size.height))
        let cam = lib.camera
        let vp = cam.projection(aspect) * cam.view
        let e = cam.eye
        let light = normalize((e - cam.target) + SIMD3(40, -60, 160))
        return FrameU(viewProj: vp, eye: SIMD4(e, 1), light: SIMD4(light, 0), viewport: SIMD2(Float(size.width), Float(size.height)),
                      glow: Float(lib.brightness), time: Float(CACurrentMediaTime() - start))
    }

    private func render(_ view: MTKView) {
        // Read before anything can bail out, so a frame without a drawable still waits for the next change.
        _ = lib.sceneVersion
        guard let pass = view.currentRenderPassDescriptor, let drawable = view.currentDrawable,
              let cmd = queue.makeCommandBuffer(), let enc = cmd.makeRenderCommandEncoder(descriptor: pass) else {
            // No drawable free yet (the GPU is still on earlier frames): this frame is tried again shortly, or a change made
            // now (a ruler's end, a hover) would wait undrawn for the next change of the scene.
            if missed < 40 {
                missed += 1
                DispatchQueue.main.asyncAfter(deadline: .now() + 0.03) { [weak self] in MainActor.assumeIsolated { self?.view?.redraw() } }
            }
            return
        }
        missed = 0
        if lib.requestFit, fit() { lib.requestFit = false }
        var f = frame(view.bounds.size)
        enc.setVertexBytes(&f, length: MemoryLayout<FrameU>.stride, index: 1)
        enc.setFragmentBytes(&f, length: MemoryLayout<FrameU>.stride, index: 1)
        let accent = color(lib.accent), accent2 = color(lib.accent2), glow = Float(max(0.35, lib.brightness))
        let alive = Set(lib.doc.bodies.map(\.id))
        bodies = bodies.filter { alive.contains($0.key) }

        drawBed(enc, accent: accent)

        let bed = lib.settings.bed
        for b in lib.doc.bodies where !b.hidden {
            guard let m = lib.meshes[b.id], let g = upload(b.id, m) else { continue }
            let model = simd_float4x4(b.place.matrix)
            let n3 = simd_float3x3(SIMD3(model.columns.0.x, model.columns.0.y, model.columns.0.z),
                                   SIMD3(model.columns.1.x, model.columns.1.y, model.columns.1.z),
                                   SIMD3(model.columns.2.x, model.columns.2.y, model.columns.2.z)).inverse.transpose
            let nm = simd_float4x4(SIMD4(n3.columns.0, 0), SIMD4(n3.columns.1, 0), SIMD4(n3.columns.2, 0), SIMD4(0, 0, 0, 1))
            let selected = lib.selection.contains(b.id), hovered = lib.hover.body == b.id
            var rim = SIMD4<Float>(accent.x, accent.y, accent.z, 0)
            if let (lo, hi) = lib.worldBounds(b),
               lo.x < -bed.x / 2 - 0.01 || lo.y < -bed.y / 2 - 0.01 || hi.x > bed.x / 2 + 0.01 || hi.y > bed.y / 2 + 0.01 || lo.z < -0.01 || hi.z > bed.z + 0.01 {
                rim = SIMD4(1, 0.15, 0.25, 1.1 * glow)
            } else if selected {
                rim.w = 0.95 * glow
            } else if hovered && (lib.mode == .select || lib.mode == .thread) {
                rim.w = 0.45 * glow
            }
            let c = SIMD3<Float>(b.color) / 255
            let facePicking = ((lib.mode == .round || lib.mode == .angles) && lib.hover.edge < 0 && lib.hover.corner < 0) || lib.mode == .hollow
            var faceHover = facePicking && hovered ? Int32(lib.hover.face) : -1
            if lib.mode == .measure, let mh = lib.measureHover, mh.snap == .face, mh.body == b.id { faceHover = Int32(mh.index) }
            var u = BodyU(model: model, normalM: nm, color: SIMD4(c, 1), rim: rim, hoverFace: faceHover, flags: selected ? 1 : 0)
            enc.setRenderPipelineState(meshPipe)
            enc.setDepthStencilState(depthWrite)
            enc.setVertexBuffer(g.pos, offset: 0, index: 0)
            enc.setVertexBuffer(g.nrm, offset: 0, index: 3)
            enc.setVertexBytes(&u, length: MemoryLayout<BodyU>.stride, index: 2)
            enc.setFragmentBytes(&u, length: MemoryLayout<BodyU>.stride, index: 2)
            enc.drawIndexedPrimitives(type: .triangle, indexCount: g.count, indexType: .uint32, indexBuffer: g.idx, indexBufferOffset: 0)
            if let eb = g.edges {
                var mm = model
                enc.setRenderPipelineState(linePipe)
                enc.setDepthStencilState(depthRead)
                enc.setVertexBuffer(eb, offset: 0, index: 0)
                enc.setVertexBytes(&mm, length: 64, index: 2)
                enc.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: g.edgeCount)
            }
            if lib.mode == .round || lib.mode == .angles {
                if lib.editBody == b.id {
                    let faces = lib.edgePicks.filter { $0.kind == Int32(BK_PICK_FACE) }.compactMap { Picking.face(m, $0) }
                    tint(enc, m, g, model, nm, [(faces, SIMD4(accent2.x, accent2.y, accent2.z, 0.35))])
                }
                drawRoundMarks(enc, b, m, model, accent: accent, accent2: accent2)
            }
            if lib.mode == .hollow, lib.editBody == b.id { drawHollowMarks(enc, m, g, model, nm, accent2: accent2) }
        }

        if lib.mode == .split, let plane = lib.splitPlane { drawPlane(enc, plane, accent: accent, hatch: accent2) }
        if lib.mode == .select, !lib.selection.isEmpty { drawGizmo(enc) }
        if let g = self.view?.guides, !g.isEmpty {
            let c = color(lib.accent3)
            var lines: [LineV] = []
            for (p, q) in g { Renderer.segment(SIMD3<Float>(p), SIMD3<Float>(q), width: 2, color: SIMD4(c.x, c.y, c.z, 0.95), into: &lines) }
            drawLines(enc, lines, depth: depthOff)
        }
        if lib.mode == .measure { drawMeasure(enc, accent: accent, accent2: accent2) }
        self.view?.placeTags()
        enc.endEncoding()
        // Shown with the window's own changes in one go (the tags over the view among them), rather than on its own, which
        // a window doesn't always put on screen until something else in it changes.
        cmd.commit()
        cmd.waitUntilScheduled()
        drawable.present()
    }

    private func drawBed(_ enc: MTLRenderCommandEncoder, accent: SIMD4<Float>) {
        let bed = lib.settings.bed
        let glow = Float(lib.brightness)
        let key = [Float(bed.x), Float(bed.y), Float(bed.z), Skin.shared.dark ? 1 : 0, accent.x, accent.y, accent.z, glow]
        if let b = bedLines, b.key == key {
            drawLines(enc, b.buffer, count: b.count, depth: depthRead)
            return
        }
        let w = Float(bed.x / 2), d = Float(bed.y / 2), h = Float(bed.z)
        var lines: [LineV] = []
        var x = -w
        while x <= w + 0.001 {
            let major = abs(x.remainder(dividingBy: 50)) < 0.01
            Renderer.segment(SIMD3(x, -d, 0), SIMD3(x, d, 0), width: major ? 1.2 : 0.8, color: SIMD4(Renderer.ink, major ? 0.16 : 0.07), into: &lines)
            x += 10
        }
        var y = -d
        while y <= d + 0.001 {
            let major = abs(y.remainder(dividingBy: 50)) < 0.01
            Renderer.segment(SIMD3(-w, y, 0), SIMD3(w, y, 0), width: major ? 1.2 : 0.8, color: SIMD4(Renderer.ink, major ? 0.16 : 0.07), into: &lines)
            y += 10
        }
        Renderer.polyline([SIMD3(-w, -d, 0), SIMD3(w, -d, 0), SIMD3(w, d, 0), SIMD3(-w, d, 0), SIMD3(-w, -d, 0)], width: 2.2,
                          color: SIMD4(accent.x, accent.y, accent.z, 0.55 + 0.4 * glow), into: &lines)
        for c in [SIMD3<Float>(-w, -d, 0), SIMD3(w, -d, 0), SIMD3(w, d, 0), SIMD3(-w, d, 0)] {
            Renderer.segment(c, c + SIMD3(0, 0, h), width: 0.8, color: SIMD4(Renderer.ink, 0.06), into: &lines)
        }
        Renderer.segment(SIMD3(-w, -d, 0.02), SIMD3(-w + 30, -d, 0.02), width: 2.5, color: SIMD4(1, 0.3, 0.35, 0.9), into: &lines)
        Renderer.segment(SIMD3(-w, -d, 0.02), SIMD3(-w, -d + 30, 0.02), width: 2.5, color: SIMD4(0.35, 1, 0.5, 0.9), into: &lines)
        Renderer.segment(SIMD3(-w, -d, 0.02), SIMD3(-w, -d, 30), width: 2.5, color: SIMD4(0.35, 0.6, 1, 0.9), into: &lines)
        guard let buf = device.makeBuffer(bytes: lines, length: lines.count * MemoryLayout<LineV>.stride) else { return }
        bedLines = (key, buf, lines.count)
        drawLines(enc, buf, count: lines.count, depth: depthRead)
    }

    private func drawRoundMarks(_ enc: MTLRenderCommandEncoder, _ b: Solid, _ m: Mesh, _ model: simd_float4x4, accent: SIMD4<Float>, accent2: SIMD4<Float>) {
        var lines: [LineV] = []
        let hot = SIMD4(accent.x, accent.y, accent.z, 1)
        if lib.hover.body == b.id {
            if lib.hover.corner >= 0, lib.hover.corner < m.corners.count {
                let c = m.corners[lib.hover.corner], r: Float = lib.camera.distance * 0.008
                Renderer.segment(c - SIMD3(r, 0, 0), c + SIMD3(r, 0, 0), width: 5, color: hot, into: &lines)
                Renderer.segment(c - SIMD3(0, r, 0), c + SIMD3(0, r, 0), width: 5, color: hot, into: &lines)
                Renderer.segment(c - SIMD3(0, 0, r), c + SIMD3(0, 0, r), width: 5, color: hot, into: &lines)
            } else if lib.hover.edge >= 0, lib.hover.edge < m.edges.count {
                Renderer.polyline(m.edges[lib.hover.edge], width: 4, color: hot, into: &lines)
            }
        }
        if lib.editBody == b.id {
            let pick = SIMD4(accent2.x, accent2.y, accent2.z, 1)
            for p in lib.edgePicks {
                switch p.kind {
                case Int32(BK_PICK_BODY): for e in m.edges { Renderer.polyline(e, width: 3, color: pick, into: &lines) }
                case Int32(BK_PICK_EDGE): if let e = Picking.nearestEdge(m, SIMD3<Float>(p.a)) { Renderer.polyline(m.edges[e], width: 3.5, color: pick, into: &lines) }
                case Int32(BK_PICK_CORNER): if let e = Picking.cornerEdge(m, p) { Renderer.polyline(m.edges[e], width: 3.5, color: pick, into: &lines) }
                default: for e in Picking.faceEdges(m, p) { Renderer.polyline(m.edges[e], width: 3, color: pick, into: &lines) }
                }
            }
        }
        drawLines(enc, lines, model: model, depth: depthOff)
    }

    // Openings in red, faces with their own wall in the second accent (brighter when being edited).
    private func drawHollowMarks(_ enc: MTLRenderCommandEncoder, _ m: Mesh, _ g: GPUBody, _ model: simd_float4x4, _ nm: simd_float4x4, accent2: SIMD4<Float>) {
        let focus = lib.focusWall.flatMap { lib.hollowWalls.indices.contains($0) ? Picking.face(m, lib.hollowWalls[$0].face) : nil }
        tint(enc, m, g, model, nm, [
            (lib.hollowOpen.compactMap { Picking.face(m, $0) }, SIMD4(1, 0.1, 0.18, 0.8)),
            (lib.hollowWalls.compactMap { Picking.face(m, $0.face) }.filter { $0 != focus }, SIMD4(accent2.x, accent2.y, accent2.z, 0.5)),
            (focus.map { [$0] } ?? [], SIMD4(accent2.x, accent2.y, accent2.z, 0.85))
        ])
    }

    // Faces of a body washed over in a see-through colour, each group in its own.
    private func tint(_ enc: MTLRenderCommandEncoder, _ m: Mesh, _ g: GPUBody, _ model: simd_float4x4, _ nm: simd_float4x4, _ groups: [([Int], SIMD4<Float>)]) {
        for (faces, tint) in groups where !faces.isEmpty {
            let set = Set(faces)
            var idx: [UInt32] = []
            var i = 0
            while i + 2 < m.indices.count {
                if set.contains(Int(m.vertices[Int(m.indices[i])].w)) { idx += [m.indices[i], m.indices[i + 1], m.indices[i + 2]] }
                i += 3
            }
            guard !idx.isEmpty, let ib = device.makeBuffer(bytes: idx, length: idx.count * 4) else { continue }
            var u = BodyU(model: model, normalM: nm, color: tint, rim: SIMD4(0, 0, 0, 0), hoverFace: -1, flags: 0)
            enc.setRenderPipelineState(glassPipe)
            enc.setDepthStencilState(depthRead)
            enc.setVertexBuffer(g.pos, offset: 0, index: 0)
            enc.setVertexBuffer(g.nrm, offset: 0, index: 3)
            enc.setVertexBytes(&u, length: MemoryLayout<BodyU>.stride, index: 2)
            enc.setFragmentBytes(&u, length: MemoryLayout<BodyU>.stride, index: 2)
            enc.drawIndexedPrimitives(type: .triangle, indexCount: idx.count, indexType: .uint32, indexBuffer: ib, indexBufferOffset: 0)
        }
    }

    // MARK: split plane

    // World axes the two tilt rings turn around: tilt x turns around axis c, tilt y around axis a.
    var tiltAxes: (Int, Int) { ((lib.splitAxis + 2) % 3, (lib.splitAxis + 1) % 3) }

    func tiltRing(_ k: Int, _ plane: Plane) -> [SIMD3<Double>] {
        var axis = SIMD3<Double>(0, 0, 0)
        axis[k == 0 ? tiltAxes.0 : tiltAxes.1] = 1
        return circle(around: axis, plane.point, gizmoLength * 0.9)
    }

    func splitArrow(_ plane: Plane) -> (SIMD3<Double>, SIMD3<Double>) { (plane.point, plane.point + plane.normal * gizmoLength) }

    private func drawPlane(_ enc: MTLRenderCommandEncoder, _ plane: Plane, accent: SIMD4<Float>, hatch: SIMD4<Float>) {
        let (c, size) = lib.splitPatch(plane)
        let n = plane.normal
        let t = normalize(abs(n.z) < 0.9 ? cross(n, SIMD3(0, 0, 1)) : cross(n, SIMD3(1, 0, 0)))
        let s = cross(n, t)
        let corners = [c + (t + s) * size, c + (t - s) * size, c + (-t - s) * size, c + (-t + s) * size].map { SIMD3<Float>($0) }
        var tri: [SIMD4<Float>] = [0, 1, 2, 0, 2, 3].map { SIMD4(corners[$0], -1) }
        let nf = SIMD4<Float>(SIMD3<Float>(n), 0)
        var nrm = [SIMD4<Float>](repeating: nf, count: 6)
        var u = BodyU(model: matrix_identity_float4x4, normalM: matrix_identity_float4x4, color: SIMD4(accent.x, accent.y, accent.z, 0.08),
                      rim: SIMD4(0, 0, 0, 0), hoverFace: -1, flags: 0)
        enc.setRenderPipelineState(glassPipe)
        enc.setDepthStencilState(depthRead)
        enc.setVertexBytes(&tri, length: 6 * 16, index: 0)
        enc.setVertexBytes(&nrm, length: 6 * 16, index: 3)
        enc.setVertexBytes(&u, length: MemoryLayout<BodyU>.stride, index: 2)
        enc.setFragmentBytes(&u, length: MemoryLayout<BodyU>.stride, index: 2)
        enc.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: 6)
        // Thin diagonal lines both ways across the plane, a grid in the second colour that sets it apart from the shapes;
        // like the fill, hidden where a shape is in front.
        var grid: [LineV] = []
        let step = size / 7
        var d = -2 * size + step
        while d < 2 * size - 1e-9 {
            let u0 = max(-size, d - size), u1 = min(size, d + size)
            for k in [1.0, -1.0] {
                let p = c + t * u0 + s * (k * (d - u0)), q = c + t * u1 + s * (k * (d - u1))
                Renderer.segment(SIMD3<Float>(p), SIMD3<Float>(q), width: 1, color: SIMD4(hatch.x, hatch.y, hatch.z, 0.4), into: &grid)
            }
            d += step
        }
        drawLines(enc, grid, depth: depthRead)
        var lines: [LineV] = []
        Renderer.polyline(corners + [corners[0]], width: 2.2, color: SIMD4(accent.x, accent.y, accent.z, 0.95), into: &lines)
        let active = view?.splitHandle
        let (a, b) = splitArrow(plane)
        let arrow = active == 2 ? SIMD4<Float>(Renderer.ink, 1) : SIMD4(accent.x, accent.y, accent.z, 1)
        let side = t * gizmoLength * 0.08
        Renderer.segment(SIMD3<Float>(a), SIMD3<Float>(b), width: 3.2, color: arrow, into: &lines)
        Renderer.segment(SIMD3<Float>(b), SIMD3<Float>(b - n * gizmoLength * 0.18 + side), width: 3.2, color: arrow, into: &lines)
        Renderer.segment(SIMD3<Float>(b), SIMD3<Float>(b - n * gizmoLength * 0.18 - side), width: 3.2, color: arrow, into: &lines)
        for k in 0..<2 {
            let color = active == k ? SIMD4<Float>(Renderer.ink, 1) : Renderer.axisColors[k == 0 ? tiltAxes.0 : tiltAxes.1]
            Renderer.polyline(tiltRing(k, plane).map { SIMD3<Float>($0) }, width: 3, color: color, into: &lines)
        }
        drawLines(enc, lines, depth: depthOff)
    }

    // MARK: ruler

    // The ruler: its line (to the pointer while the second end is still to come), the gap between surfaces in the second
    // accent, and a mark for each end in the shape of what it snapped to.
    private func drawMeasure(_ enc: MTLRenderCommandEncoder, accent: SIMD4<Float>, accent2: SIMD4<Float>) {
        var lines: [LineV] = []
        let hot = SIMD4(accent.x, accent.y, accent.z, 1), second = SIMD4(accent2.x, accent2.y, accent2.z, 1)
        let eye = SIMD3<Double>(lib.camera.eye), side = SIMD3<Double>(lib.camera.side)
        let r = Double(lib.camera.distance) * 0.008
        func seg(_ a: SIMD3<Double>, _ b: SIMD3<Double>, _ w: Float, _ c: SIMD4<Float>) {
            Renderer.segment(SIMD3<Float>(a), SIMD3<Float>(b), width: w, color: c, into: &lines)
        }
        func mark(_ e: MeasureEnd, _ c: SIMD4<Float>) {
            let p = e.point, n = unit(eye - p), up = unit(cross(n, side))
            switch e.snap {
            case .corner:
                for a in [SIMD3<Double>(1, 0, 0), SIMD3(0, 1, 0), SIMD3(0, 0, 1)] { seg(p - a * r, p + a * r, 4, c) }
            case .centre:
                Renderer.polyline(circle(around: n, p, r).map { SIMD3<Float>($0) }, width: 3, color: c, into: &lines)
                seg(p - side * r * 0.4, p + side * r * 0.4, 2.5, c)
                seg(p - up * r * 0.4, p + up * r * 0.4, 2.5, c)
            case .midpoint:
                let d = [p + side * r, p + up * r, p - side * r, p - up * r, p + side * r]
                Renderer.polyline(d.map { SIMD3<Float>($0) }, width: 3, color: c, into: &lines)
            case .edge, .face:
                Renderer.polyline(circle(around: n, p, r * 0.45).map { SIMD3<Float>($0) }, width: 4, color: c, into: &lines)
            }
        }
        // The edge the pointer is on, lit along its length.
        if let h = lib.measureHover, h.snap == .edge, let id = h.body, let b = lib.body(id), let m = lib.meshes[id], m.edges.indices.contains(h.index) {
            let mat = b.place.matrix
            let pts = m.edges[h.index].map { SIMD3<Float>((mat * SIMD4(SIMD3<Double>($0), 1)).xyz) }
            Renderer.polyline(pts, width: 3, color: SIMD4(accent.x, accent.y, accent.z, 0.6), into: &lines)
        }
        if let a = lib.measureA, let b = lib.measureB ?? lib.measureHover { seg(a.point, b.point, 2.5, hot) }
        if let g = lib.shownGap {
            seg(g.a, g.b, 2.5, second)
            for q in [g.a, g.b] {
                let n = unit(eye - q), up = unit(cross(n, side))
                seg(q - side * r * 0.5, q + side * r * 0.5, 2.5, second)
                seg(q - up * r * 0.5, q + up * r * 0.5, 2.5, second)
            }
        }
        for e in [lib.measureA, lib.measureB, lib.measureHover] { if let e { mark(e, hot) } }
        drawLines(enc, lines, depth: depthOff)
    }

    // MARK: gizmo

    var gizmoCenter: SIMD3<Double> {
        // Turning: about the selection's middle, which stays put as it turns (the middle of a box round it would move as
        // an uneven shape turns, and turn after turn the shape would wander).
        if lib.gizmo == .rotate { return lib.turnPivot }
        var lo = SIMD3<Double>(repeating: .infinity), hi = SIMD3<Double>(repeating: -.infinity)
        for b in lib.selected { if let (l, h) = lib.worldBounds(b) { lo = simd_min(lo, l); hi = simd_max(hi, h) } }
        return lo.x.isFinite ? (lo + hi) / 2 : .zero
    }

    var gizmoLength: Double { Double(lib.camera.distance) * 0.13 }

    func gizmoAxes() -> [SIMD3<Double>] {
        if lib.gizmo == .scale, lib.selection.count == 1, let b = lib.primary {
            let r = b.place.rotation
            return [r.columns.0, r.columns.1, r.columns.2]
        }
        return [SIMD3(1, 0, 0), SIMD3(0, 1, 0), SIMD3(0, 0, 1)]
    }

    // Which way each move or scale handle points from the gizmo's centre: the way it last pointed, unless the panels over
    // the view (the tool rail, the inspector, the shape bar) or the view's edge cover more of it there than they would on
    // the other side; then it turns round, so it can always be reached. Nothing turns while a handle is held.
    var gizmoSides = SIMD3<Double>(1, 1, 1)

    func turnGizmo() {
        guard let v = view, v.dragAxis == nil else { return }
        let c = gizmoCenter, L = gizmoLength, covers = v.covers(), open = v.bounds.insetBy(dx: 10, dy: 10)
        func hidden(_ a: SIMD3<Double>) -> Int {
            [0.4, 0.6, 0.8, 1.0].filter { t in
                guard let p = v.project(c + a * L * t) else { return true }
                return !open.contains(p) || covers.contains { $0.contains(p) }
            }.count
        }
        for (i, a) in gizmoAxes().enumerated() where hidden(-a * gizmoSides[i]) < hidden(a * gizmoSides[i]) {
            gizmoSides[i] = -gizmoSides[i]
        }
    }

    // The move or scale handles' directions from the gizmo's centre.
    func gizmoHandles() -> [SIMD3<Double>] { gizmoAxes().enumerated().map { $1 * gizmoSides[$0] } }

    static let axisColors: [SIMD4<Float>] = Axis.rgb.map { SIMD4($0, 1) }

    func ring(_ axis: Int, _ c: SIMD3<Double>, _ r: Double) -> [SIMD3<Double>] { circle(around: gizmoAxes()[axis], c, r) }

    func circle(around n: SIMD3<Double>, _ c: SIMD3<Double>, _ r: Double) -> [SIMD3<Double>] {
        let t = normalize(abs(n.z) < 0.9 ? cross(n, SIMD3(0, 0, 1)) : cross(n, SIMD3(1, 0, 0)))
        let s = cross(n, t)
        return (0...64).map { i in let a = Double(i) / 64 * 2 * .pi; return c + (t * cos(a) + s * sin(a)) * r }
    }

    private func drawGizmo(_ enc: MTLRenderCommandEncoder) {
        turnGizmo()
        let c = gizmoCenter, L = gizmoLength, axes = gizmoHandles()
        var lines: [LineV] = []
        let active = view?.dragAxis
        for (i, a) in axes.enumerated() {
            var col = Renderer.axisColors[i]
            if active == i { col = SIMD4(Renderer.ink, 1) }
            switch lib.gizmo {
            case .move, .scale:
                let tip = c + a * L
                Renderer.segment(SIMD3<Float>(c), SIMD3<Float>(tip), width: 3.2, color: col, into: &lines)
                if lib.gizmo == .move {
                    let side = normalize(abs(a.z) < 0.9 ? cross(a, SIMD3(0, 0, 1)) : cross(a, SIMD3(1, 0, 0)))
                    Renderer.segment(SIMD3<Float>(tip), SIMD3<Float>(tip - a * L * 0.18 + side * L * 0.08), width: 3.2, color: col, into: &lines)
                    Renderer.segment(SIMD3<Float>(tip), SIMD3<Float>(tip - a * L * 0.18 - side * L * 0.08), width: 3.2, color: col, into: &lines)
                } else {
                    let q = L * 0.06
                    let side = normalize(abs(a.z) < 0.9 ? cross(a, SIMD3(0, 0, 1)) : cross(a, SIMD3(1, 0, 0)))
                    let up = cross(a, side)
                    let sq = [tip + (side + up) * q, tip + (side - up) * q, tip + (-side - up) * q, tip + (-side + up) * q, tip + (side + up) * q]
                    Renderer.polyline(sq.map { SIMD3<Float>($0) }, width: 3.2, color: col, into: &lines)
                }
            case .rotate:
                Renderer.polyline(ring(i, c, L * 0.9).map { SIMD3<Float>($0) }, width: 3, color: col, into: &lines)
            }
        }
        drawLines(enc, lines, depth: depthOff)
    }

    // MARK: camera helpers

    // Frames the visible shapes (or the bed when there are none). False while shapes are still being built.
    func fit() -> Bool {
        let shown = lib.doc.bodies.filter { !$0.hidden }
        let framing = lib.framing(shown)
        if framing == nil && !shown.isEmpty && lib.building { return false }
        (lib.camera.target, lib.camera.distance) = framing ?? lib.bedFraming
        return true
    }
}

// The axes' colours, x red, y green, z blue: the same on the gizmo, in the inspector and on the split chips, in every style.
enum Axis {
    static let rgb: [SIMD3<Float>] = [SIMD3(1, 0.3, 0.38), SIMD3(0.35, 1, 0.55), SIMD3(0.38, 0.62, 1)]
    static func color(_ i: Int) -> Color { Color(red: Double(rgb[i].x), green: Double(rgb[i].y), blue: Double(rgb[i].z)) }
}

// MARK: - Picking

enum Picking {
    static func rayHit(_ m: Mesh, _ o: SIMD3<Double>, _ d: SIMD3<Double>) -> (t: Double, face: Int)? {
        // bounding box test
        var tmin = -Double.infinity, tmax = Double.infinity
        for a in 0..<3 {
            if abs(d[a]) < 1e-12 { if o[a] < m.low[a] - 1e-6 || o[a] > m.high[a] + 1e-6 { return nil }; continue }
            let t1 = (m.low[a] - o[a]) / d[a], t2 = (m.high[a] - o[a]) / d[a]
            tmin = max(tmin, min(t1, t2))
            tmax = min(tmax, max(t1, t2))
        }
        if tmax < max(tmin, 0) { return nil }
        let of = SIMD3<Float>(o), df = SIMD3<Float>(d)
        var best = Float.infinity, face = -1
        let v = m.vertices
        var i = 0
        while i < m.indices.count {
            let a = v[Int(m.indices[i])], b = v[Int(m.indices[i + 1])], c = v[Int(m.indices[i + 2])]
            let p0 = SIMD3(a.x, a.y, a.z), e1 = SIMD3(b.x, b.y, b.z) - p0, e2 = SIMD3(c.x, c.y, c.z) - p0
            let p = cross(df, e2)
            let det = dot(e1, p)
            if abs(det) > 1e-12 {
                let inv = 1 / det
                let s = of - p0
                let u = dot(s, p) * inv
                if u >= 0 && u <= 1 {
                    let q = cross(s, e1)
                    let w = dot(df, q) * inv
                    if w >= 0 && u + w <= 1 {
                        let t = dot(e2, q) * inv
                        if t > 1e-4 && t < best { best = t; face = Int(a.w) }
                    }
                }
            }
            i += 3
        }
        return face >= 0 ? (Double(best), face) : nil
    }

    static func midpoint(_ e: [SIMD3<Float>]) -> (SIMD3<Float>, SIMD3<Float>) {
        guard e.count > 1 else { return (e.first ?? .zero, SIMD3(0, 0, 1)) }
        var total: Float = 0
        for i in 1..<e.count { total += length(e[i] - e[i - 1]) }
        var run: Float = 0
        for i in 1..<e.count {
            let l = length(e[i] - e[i - 1])
            if run + l >= total / 2 {
                let t = l > 0 ? (total / 2 - run) / l : 0
                return (e[i - 1] + (e[i] - e[i - 1]) * t, l > 0 ? normalize(e[i] - e[i - 1]) : SIMD3(0, 0, 1))
            }
            run += l
        }
        return (e.last!, normalize(e[e.count - 1] - e[e.count - 2]))
    }

    // The mesh face a face pick (normal, centroid) points at, within the kernel's reach for picks.
    static func face(_ m: Mesh, _ pick: Pick) -> Int? {
        var best = max(0.3, simd_length(m.high - m.low) * 0.02), hit: Int?
        for (i, f) in m.faceInfo.enumerated() where dot(f.normal, pick.a) > 0.7 {
            let d = length(f.centroid - pick.b)
            if d < best { best = d; hit = i }
        }
        return hit
    }

    static func nearestEdge(_ m: Mesh, _ p: SIMD3<Float>) -> Int? {
        var best = Float.infinity, hit: Int?
        for (i, e) in m.edges.enumerated() where e.count > 1 {
            let d = length(midpoint(e).0 - p)
            if d < best { best = d; hit = i }
        }
        return best < 1 ? hit : nil
    }

    static func cornerEdge(_ m: Mesh, _ pick: Pick) -> Int? {
        let v = SIMD3<Float>(pick.b), n = normalize(SIMD3<Float>(pick.a))
        var best: Float = 0.3, hit: Int?
        for (i, e) in m.edges.enumerated() where e.count > 1 {
            guard let f = e.first, let l = e.last, min(length(f - v), length(l - v)) < 0.01 else { continue }
            let d = abs(dot(midpoint(e).1, n))
            if d > best { best = d; hit = i }
        }
        return hit
    }

    // A face that meets no other face along any edge: the whole surface of its shape (a sphere's, a torus's).
    static func alone(_ m: Mesh, _ face: Int) -> Bool {
        let f = Int32(face)
        return !m.edgeFaces.isEmpty && !m.edgeFaces.contains { ($0.x == f && $0.y >= 0 && $0.y != f) || ($0.y == f && $0.x >= 0 && $0.x != f) }
    }

    // The edges around a picked face: the ones a rounding of the face works on.
    static func faceEdges(_ m: Mesh, _ pick: Pick) -> [Int] {
        guard let f = face(m, pick).map(Int32.init) else { return [] }
        return m.edgeFaces.indices.filter { m.edgeFaces[$0].x == f || m.edgeFaces[$0].y == f }
    }
}

// MARK: - View

final class CadView: MTKView {
    let lib = Workbench.shared
    var renderer: Renderer!
    var dragAxis: Int?
    var splitHandle: Int?   // 0, 1: tilt rings · 2: the move arrow
    private enum Drag { case none, orbit, pan, body, axis(Int), ring(Int), scaleAxis(Int), round, split, tilt(Int) }
    private var drag: Drag = .none
    private var downAt = CGPoint.zero
    private var last = CGPoint.zero
    private var moved = false
    private var starts: [UUID: Placement] = [:]
    // A lone shape's size as a resize began, along its own axes (a live resize changes its mesh as it goes).
    private var startSize: SIMD3<Double>?
    private var startPlane = SIMD3<Double>(0, 0, 0)
    private var startOffset = 0.0
    private var startTilt = SIMD2<Double>(0, 0)
    private var startRadius = 0.0
    private var accum = 0.0
    private var tracking: NSTrackingArea?
    private var startBox: Box?
    private var others: [Box] = []
    private var marks: [[Mark]] = [[], [], []]
    // The selection's own holes' and pegs' middles as the drag began.
    private var ownRings: [SIMD3<Double>] = []
    private var frameAsked = false
    var guides: [(SIMD3<Double>, SIMD3<Double>)] = []
    // The ruler's length at the middle of its line, and what the pointer snaps to beside it.
    private let lengthTag = Tag(), snapTag = Tag()

    init() {
        let r = Renderer()
        super.init(frame: .zero, device: r.device)
        renderer = r
        r.view = self
        delegate = r
        colorPixelFormat = .bgra8Unorm
        depthStencilPixelFormat = .depth32Float
        sampleCount = 4
        clearColor = MTLClearColor(red: 0, green: 0, blue: 0, alpha: 0)
        layer?.isOpaque = false
        isPaused = true
        enableSetNeedsDisplay = false
        presentsWithTransaction = true
        addSubview(lengthTag)
        addSubview(snapTag)
    }

    // Draws once on the next turn of the main loop, however often it's asked. Frames are drawn here rather than left to
    // MetalKit's own scheduling, which stops for good when the app is launched by opening a file.
    func redraw() {
        guard !frameAsked else { return }
        frameAsked = true
        DispatchQueue.main.async { [weak self] in
            MainActor.assumeIsolated {
                guard let self else { return }
                self.frameAsked = false
                self.draw()
            }
        }
    }

    override func viewDidMoveToWindow() {
        super.viewDidMoveToWindow()
        layer?.isOpaque = false
        redraw()
    }

    override func setFrameSize(_ newSize: NSSize) {
        super.setFrameSize(newSize)
        redraw()
    }

    override func viewDidChangeBackingProperties() {
        super.viewDidChangeBackingProperties()
        redraw()
    }

    required init(coder: NSCoder) { fatalError() }

    override var acceptsFirstResponder: Bool { true }
    override func acceptsFirstMouse(for event: NSEvent?) -> Bool { true }

    override func updateTrackingAreas() {
        super.updateTrackingAreas()
        if let t = tracking { removeTrackingArea(t) }
        let t = NSTrackingArea(rect: bounds, options: [.mouseMoved, .activeInKeyWindow, .inVisibleRect, .mouseEnteredAndExited], owner: self)
        addTrackingArea(t)
        tracking = t
    }

    // MARK: geometry helpers

    private var scale: CGFloat { window?.backingScaleFactor ?? 2 }

    private func vp() -> simd_double4x4 {
        let f = renderer.frame(CGSize(width: bounds.width, height: bounds.height))
        return simd_double4x4(f.viewProj)
    }

    func project(_ p: SIMD3<Double>) -> CGPoint? {
        let c = vp() * SIMD4(p, 1)
        guard c.w > 1e-6 else { return nil }
        return CGPoint(x: (c.x / c.w + 1) / 2 * bounds.width, y: (c.y / c.w + 1) / 2 * bounds.height)
    }

    func ray(_ p: CGPoint) -> (SIMD3<Double>, SIMD3<Double>) {
        let inv = vp().inverse
        let x = Double(p.x / bounds.width) * 2 - 1, y = Double(p.y / bounds.height) * 2 - 1
        let a = inv * SIMD4(x, y, 0, 1), b = inv * SIMD4(x, y, 1, 1)
        let o = SIMD3(a.x, a.y, a.z) / a.w, e = SIMD3(b.x, b.y, b.z) / b.w
        return (o, normalize(e - o))
    }

    private func worldPerPoint(at p: SIMD3<Double>) -> Double {
        let d = length(SIMD3<Double>(lib.camera.eye) - p)
        return 2 * d * tan(Double(lib.camera.fov) / 2) / Double(max(1, bounds.height))
    }

    struct Hit { var body: UUID; var face: Int; var local: SIMD3<Double>; var world: SIMD3<Double>; var distance: Double }

    func hitBody(_ p: CGPoint) -> Hit? {
        let (o, d) = ray(p)
        var best: Hit?
        for b in lib.doc.bodies where !b.hidden {
            guard let m = lib.meshes[b.id], !m.vertices.isEmpty else { continue }
            let inv = b.place.matrix.inverse
            let lo4 = inv * SIMD4(o, 1), ld4 = inv * SIMD4(d, 0)
            let lo = SIMD3(lo4.x, lo4.y, lo4.z), ld = SIMD3(ld4.x, ld4.y, ld4.z)
            guard let (t, face) = Picking.rayHit(m, lo, ld) else { continue }
            let local = lo + ld * t
            let w4 = b.place.matrix * SIMD4(local, 1)
            let world = SIMD3(w4.x, w4.y, w4.z)
            let dist = length(world - o)
            if best == nil || dist < best!.distance { best = Hit(body: b.id, face: face, local: local, world: world, distance: dist) }
        }
        return best
    }

    private func segmentDistance(_ p: CGPoint, _ a: CGPoint, _ b: CGPoint) -> CGFloat {
        let dx = b.x - a.x, dy = b.y - a.y
        let l2 = dx * dx + dy * dy
        let t = l2 > 0 ? max(0, min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2)) : 0
        return hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy))
    }

    private func gizmoHit(_ p: CGPoint) -> Int? {
        guard lib.mode == .select, !lib.selection.isEmpty else { return nil }
        let c = renderer.gizmoCenter, L = renderer.gizmoLength
        var best: CGFloat = 9, hit: Int?
        for (i, a) in renderer.gizmoHandles().enumerated() {
            if lib.gizmo == .rotate {
                let pts = renderer.ring(i, c, L * 0.9).compactMap { project($0) }
                for k in 1..<max(1, pts.count) {
                    let d = segmentDistance(p, pts[k - 1], pts[k])
                    if d < best { best = d; hit = i }
                }
            } else if let s = project(c), let e = project(c + a * L) {
                let d = segmentDistance(p, s, e)
                if d < best { best = d; hit = i }
            }
        }
        return hit
    }

    // MARK: ruler

    // The ruler's end under the pointer: the nearest corner, circle centre or edge middle within 9 points, else the nearest
    // point of an edge within 7, else the face. Only what can be seen counts. free (⌥): the point on the face as it is.
    func measureSnap(_ p: CGPoint, free: Bool = false) -> MeasureEnd? {
        let hit = hitBody(p)
        let face = hit.map { MeasureEnd(snap: .face, point: $0.world, body: $0.body, index: $0.face) }
        if free { return face }
        let (o, d) = ray(p)
        var points: [(gap: CGFloat, end: MeasureEnd, at: CGPoint)] = []
        var edges: [(gap: CGFloat, end: MeasureEnd, at: CGPoint)] = []
        for b in lib.doc.bodies where !b.hidden {
            // Only a shape's exact mesh has its corners and edges (not the saved look shown while a file opens).
            guard let m = lib.meshes[b.id], !m.edges.isEmpty else { continue }
            let mat = b.place.matrix
            func world(_ v: SIMD3<Float>) -> SIMD3<Double> { (mat * SIMD4(SIMD3<Double>(v), 1)).xyz }
            func near(_ w: SIMD3<Double>, _ reach: CGFloat) -> (CGFloat, CGPoint)? {
                guard let s = project(w) else { return nil }
                let g = hypot(s.x - p.x, s.y - p.y)
                return g < reach ? (g, s) : nil
            }
            for c in m.corners {
                let w = world(c)
                if let (g, s) = near(w, 9) { points.append((g, MeasureEnd(snap: .corner, point: w, body: b.id), s)) }
            }
            for r in CadView.rings(m, b.place) {
                if let (g, s) = near(r.center, 9) { points.append((g + 0.5, MeasureEnd(snap: .centre, point: r.center, body: b.id), s)) }
            }
            for (i, e) in m.edges.enumerated() where e.count > 1 {
                // A closed edge (a whole circle) has no middle.
                if length(e[0] - e[e.count - 1]) > 1e-4 {
                    let w = world(Picking.midpoint(e).0)
                    if let (g, s) = near(w, 9) { points.append((g + 1, MeasureEnd(snap: .midpoint, point: w, body: b.id), s)) }
                }
                var best: (CGFloat, SIMD3<Double>, CGPoint)?
                var prev: SIMD3<Double>?
                for v in e {
                    let w = world(v)
                    defer { prev = w }
                    guard let a = prev else { continue }
                    let q = a + (w - a) * closest(o, d, a, w)
                    if let (g, s) = near(q, 7), best == nil || g < best!.0 { best = (g, q, s) }
                }
                if let (g, q, s) = best { edges.append((g, MeasureEnd(snap: .edge, point: q, body: b.id, index: i), s)) }
            }
        }
        // Seen: no shape lies in front of it (a few of the nearest are tried).
        func seen(_ w: SIMD3<Double>, _ s: CGPoint) -> Bool {
            let (so, _) = ray(s)
            let dist = length(w - so)
            guard let h = hitBody(s) else { return true }
            return h.distance >= dist - max(0.3, dist * 0.004)
        }
        for c in points.sorted(by: { $0.gap < $1.gap }).prefix(6) where seen(c.end.point, c.at) { return c.end }
        for c in edges.sorted(by: { $0.gap < $1.gap }).prefix(6) where seen(c.end.point, c.at) { return c.end }
        return face
    }

    // Where on segment a…b the line of the ray (o, unit d) passes closest, 0…1.
    private func closest(_ o: SIMD3<Double>, _ d: SIMD3<Double>, _ a: SIMD3<Double>, _ b: SIMD3<Double>) -> Double {
        let u = b - a, w = a - o
        let uu = dot(u, u), ud = dot(u, d)
        let den = uu - ud * ud
        guard den > 1e-12 else { return 0 }
        return max(0, min(1, (ud * dot(d, w) - dot(u, w)) / den))
    }

    // The panels floating over the 3D view (the tool rail, the inspector, the shape bar), in this view's coordinates.
    func covers() -> [CGRect] {
        let height = window?.contentView?.bounds.height ?? bounds.height
        return [lib.railFrame, lib.inspectorFrame, lib.shapeBarFrame].filter { !$0.isEmpty }.map { r in
            convert(CGRect(x: r.minX, y: height - r.maxY, width: r.width, height: r.height), from: nil).insetBy(dx: -8, dy: -8)
        }
    }

    // The tags follow their points every frame.
    func placeTags() {
        let on = lib.mode == .measure
        if on, let a = lib.measureA, let b = lib.measureB ?? lib.measureHover, let s = project((a.point + b.point) / 2) {
            lengthTag.show(Ruler.mm(length(b.point - a.point)), color: NSColor(lib.accent), at: CGPoint(x: s.x, y: s.y + 16))
        } else {
            lengthTag.isHidden = true
        }
        if on, let h = lib.measureHover, let s = project(h.point) {
            snapTag.show(h.name, color: NSColor(lib.accent2), at: CGPoint(x: s.x + 18, y: s.y - 18), centered: false)
        } else {
            snapTag.isHidden = true
        }
    }

    // Round mode: corner, then edge, then face under the pointer.
    private func roundHover(_ p: CGPoint) -> Hover {
        guard let h = hitBody(p), let b = lib.body(h.body), let m = lib.meshes[h.body] else { return Hover() }
        var hv = Hover(body: h.body, face: h.face, point: h.local)
        let mat = b.place.matrix
        let eye = SIMD3<Double>(lib.camera.eye)
        let limit = h.distance + max(2, h.distance * 0.01)
        func screen(_ v: SIMD3<Float>) -> (CGPoint, Double)? {
            let w = mat * SIMD4(SIMD3<Double>(v), 1)
            let wp = SIMD3(w.x, w.y, w.z)
            guard let s = project(wp) else { return nil }
            return (s, length(wp - eye))
        }
        var best: CGFloat = 9
        for (i, c) in m.corners.enumerated() {
            guard let (s, dist) = screen(c), dist <= limit else { continue }
            let d = hypot(s.x - p.x, s.y - p.y)
            if d < best { best = d; hv.corner = i }
        }
        if hv.corner >= 0 { return hv }
        best = 7
        for (i, e) in m.edges.enumerated() where e.count > 1 {
            var prev: (CGPoint, Double)?
            for v in e {
                let cur = screen(v)
                if let a = prev, let b = cur, min(a.1, b.1) <= limit {
                    let d = segmentDistance(p, a.0, b.0)
                    if d < best { best = d; hv.edge = i }
                }
                prev = cur
            }
        }
        return hv
    }

    private func pick(for hv: Hover) -> Pick? {
        guard let id = hv.body, let m = lib.meshes[id] else { return nil }
        if hv.corner >= 0, hv.face >= 0, hv.face < m.faceInfo.count {
            return Pick(kind: Int32(BK_PICK_CORNER), a: m.faceInfo[hv.face].normal, b: SIMD3<Double>(m.corners[hv.corner]))
        }
        if hv.edge >= 0 {
            let (mid, dir) = Picking.midpoint(m.edges[hv.edge])
            return Pick(kind: Int32(BK_PICK_EDGE), a: SIMD3<Double>(mid), b: SIMD3<Double>(dir))
        }
        if hv.face >= 0, hv.face < m.faceInfo.count {
            let f = m.faceInfo[hv.face]
            return Pick(kind: Int32(BK_PICK_FACE), a: f.normal, b: f.centroid)
        }
        return nil
    }

    // MARK: events

    override func mouseMoved(with e: NSEvent) {
        let p = convert(e.locationInWindow, from: nil)
        var hv: Hover
        if lib.mode == .measure {
            let end = measureSnap(p, free: e.modifierFlags.contains(.option))
            if end != lib.measureHover { lib.measureHover = end }
        }
        if lib.mode == .round || lib.mode == .angles {
            hv = roundHover(p)
        } else {
            let h = hitBody(p)
            hv = Hover(body: h?.body, face: h?.face ?? -1)
        }
        if hv != lib.hover { lib.hover = hv }
    }

    override func mouseExited(with event: NSEvent) {
        if lib.hover != Hover() { lib.hover = Hover() }
        if lib.measureHover != nil { lib.measureHover = nil }
    }

    override func mouseDown(with e: NSEvent) {
        window?.makeFirstResponder(self)
        let p = convert(e.locationInWindow, from: nil)
        downAt = p
        last = p
        moved = false
        accum = 0
        let shift = e.modifierFlags.contains(.shift), cmd = e.modifierFlags.contains(.command)
        switch lib.mode {
        case .measure:
            // A click measures (on mouse up); a drag turns or pans the view.
            drag = shift ? .pan : .orbit
            return
        case .split:
            startOffset = lib.splitOffset
            startTilt = lib.splitTilt
            if let k = splitHandleHit(p) {
                splitHandle = k
                if let plane = lib.splitPlane { startPlane = plane.point }
                drag = k == 2 ? .split : .tilt(k)
            } else if let h = hitBody(p), lib.selection.contains(h.body) {
                drag = .split
            } else {
                drag = shift ? .pan : .orbit
            }
            return
        case .hollow:
            guard let h = hitBody(p), let m = lib.meshes[h.body], m.faceInfo.indices.contains(h.face) else { drag = shift ? .pan : .orbit; return }
            if lib.editBody != h.body { lib.loadHollow(h.body) }
            let f = m.faceInfo[h.face], own = e.modifierFlags.contains(.option)
            drag = .none
            // A face that is the whole surface (a sphere's, a torus's) leaves nothing to keep when opened.
            if !own && Picking.alone(m, h.face) {
                lib.flash(L("A shape with one surface can't have an opening"))
                return
            }
            lib.pickHollowFace(Pick(kind: Int32(BK_PICK_FACE), a: f.normal, b: f.centroid), ownWall: own)
            return
        case .round:
            let hv = roundHover(p)
            lib.hover = hv
            guard let id = hv.body, let pk = pick(for: hv) else { drag = shift ? .pan : .orbit; return }
            if lib.editBody != id {
                lib.clearPreview()
                lib.editBody = id
                lib.edgePicks = []
            }
            if shift {
                if let i = lib.edgePicks.firstIndex(of: pk) { lib.edgePicks.remove(at: i) } else { lib.edgePicks.append(pk) }
                drag = .none
                return
            }
            if !lib.edgePicks.contains(pk) && !lib.edgePicks.contains(where: { $0.kind == Int32(BK_PICK_BODY) }) { lib.edgePicks = [pk] }
            drag = .round
            startRadius = 0
            startPlane = hv.point
            return
        case .angles:
            let hv = roundHover(p)
            lib.hover = hv
            guard let id = hv.body, let pk = pick(for: hv) else { drag = shift ? .pan : .orbit; return }
            if !lib.selection.contains(id) { lib.selection = [id] }
            if lib.editBody != id {
                lib.editBody = id
                lib.edgePicks = []
            }
            if shift {
                if let i = lib.edgePicks.firstIndex(of: pk) { lib.edgePicks.remove(at: i) } else { lib.edgePicks.append(pk) }
            } else {
                lib.edgePicks = [pk]
            }
            drag = .none
            return
        case .select, .thread:
            break
        }
        if let axis = gizmoHit(p) {
            lib.begin()
            starts = Dictionary(uniqueKeysWithValues: lib.selected.map { ($0.id, $0.place) })
            startSize = nil
            if lib.gizmo == .scale {
                lib.beginResize()
                if lib.selection.count == 1, let b = lib.primary, let m = lib.meshes[b.id] { startSize = m.size * b.place.scale }
            }
            startPlane = renderer.gizmoCenter
            captureBoxes()
            dragAxis = axis
            drag = lib.gizmo == .move ? .axis(axis) : lib.gizmo == .rotate ? .ring(axis) : .scaleAxis(axis)
            return
        }
        if let h = hitBody(p) {
            if shift || cmd {
                if let i = lib.selection.firstIndex(of: h.body) { lib.selection.remove(at: i) } else { lib.selection.append(h.body) }
                drag = .none
                return
            }
            if !lib.selection.contains(h.body) { lib.selection = [h.body] }
            lib.begin()
            starts = Dictionary(uniqueKeysWithValues: lib.selected.map { ($0.id, $0.place) })
            startPlane = h.world
            captureBoxes()
            drag = .body
            return
        }
        drag = shift ? .pan : .orbit
    }

    override func mouseDragged(with e: NSEvent) {
        let p = convert(e.locationInWindow, from: nil)
        let dx = p.x - last.x, dy = p.y - last.y
        if hypot(p.x - downAt.x, p.y - downAt.y) > 3 { moved = true }
        last = p
        let free = e.modifierFlags.contains(.command)
        defer { redraw() }
        // Shapes stay exactly where they are until the pointer has really moved: a click's jitter would snap them to a mark.
        switch drag {
        case .body, .axis, .ring, .scaleAxis: if !moved { return }
        default: break
        }
        switch drag {
        case .none: break
        case .orbit: orbit(dx, dy)
        case .pan: pan(dx, dy)
        case .body:
            let (o0, d0) = ray(downAt), (o1, d1) = ray(p)
            let z = startPlane.z
            guard abs(d0.z) > 1e-6, abs(d1.z) > 1e-6 else { return }
            let a = o0 + d0 * ((z - o0.z) / d0.z), b = o1 + d1 * ((z - o1.z) / d1.z)
            var delta = b - a
            delta.z = 0
            guides = []
            if !free {
                for axis in 0..<2 { delta[axis] = place(delta[axis], axis: axis, shift: delta) }
            }
            for (id, s) in starts { lib.mutate(id) { $0.place.move = s.move + delta } }
            lib.sceneVersion += 1
        case .axis(let i):
            let a = renderer.gizmoAxes()[i]
            accum = travel(p, a, dx, dy)
            guides = []
            var shift = SIMD3<Double>(0, 0, 0)
            shift[i] = accum
            let v = free ? accum : place(accum, axis: i, shift: shift)
            for (id, s) in starts { lib.mutate(id) { $0.place.move = s.move + a * v } }
            lib.sceneVersion += 1
        case .ring(let i):
            guard let c = project(startPlane) else { return }
            let a0 = atan2(Double(downAt.y - c.y), Double(downAt.x - c.x)), a1 = atan2(Double(p.y - c.y), Double(p.x - c.x))
            var ang = (a1 - a0) * 180 / .pi
            let axis = renderer.gizmoAxes()[i]
            if dot(axis, SIMD3<Double>(lib.camera.eye) - startPlane) < 0 { ang = -ang }
            if !free { ang = (ang / lib.settings.turnStep).rounded() * lib.settings.turnStep }
            rotate(axis, ang)
        case .scaleAxis(let i):
            // Along the handle: dragging it outwards grows the side it is on.
            let low = renderer.gizmoSides[i] < 0
            accum = travel(p, renderer.gizmoHandles()[i], dx, dy)
            guides = []
            let uniform = lib.settings.uniform || e.modifierFlags.contains(.shift)
            let symmetric = lib.settings.symmetric || e.modifierFlags.contains(.option)
            // The dragged side follows the pointer; symmetric, the other side comes along the other way.
            var s0 = startBox.map { $0.hi[i] - $0.lo[i] } ?? renderer.gizmoLength
            if let size = startSize { s0 = size[i] }
            guard s0 > 0 else { return }
            var f = max(0.02, 1 + accum * (symmetric ? 2 : 1) / s0)
            if !free, lib.selection.count == 1, let b = lib.primary, let st = starts[b.id] {
                f = resize(s0 * f, axis: i, start: st, symmetric: symmetric, low: low) / s0
            }
            lib.stretch(starts, axis: i, by: f, uniform: uniform, symmetric: symmetric, low: low)
        case .round:
            let wpp = worldPerPoint(at: startPlane)
            startRadius += Double(dy) * wpp * 0.6
            let r = max(0.01, (abs(startRadius) * 100).rounded() / 100)
            lib.roundRadius = r
            lib.previewRound()
        case .split:
            guard let plane = lib.splitPlane else { return }
            startOffset += along(plane.normal, dx, dy)
            let v = free ? startOffset : (startOffset / lib.settings.snap).rounded() * lib.settings.snap
            lib.splitOffset = (v * 100).rounded() / 100
        case .tilt(let k):
            guard let c = project(startPlane) else { return }
            let a0 = atan2(Double(downAt.y - c.y), Double(downAt.x - c.x)), a1 = atan2(Double(p.y - c.y), Double(p.x - c.x))
            var ang = (a1 - a0) * 180 / .pi
            ang = ang.remainder(dividingBy: 360)
            var axis = SIMD3<Double>(0, 0, 0)
            axis[k == 0 ? renderer.tiltAxes.0 : renderer.tiltAxes.1] = 1
            if dot(axis, SIMD3<Double>(lib.camera.eye) - startPlane) < 0 { ang = -ang }
            if !free { ang = (ang / lib.settings.turnStep).rounded() * lib.settings.turnStep }
            var t = startTilt
            if k == 0 { t.x = startTilt.x + ang } else { t.y = startTilt.y - ang }
            lib.splitTilt = SIMD2(min(80, max(-80, (t.x * 100).rounded() / 100)), min(80, max(-80, (t.y * 100).rounded() / 100)))
        }
    }

    // The split plane's handle under the pointer: a tilt ring (0, 1) or the move arrow (2).
    private func splitHandleHit(_ p: CGPoint) -> Int? {
        guard let plane = lib.splitPlane else { return nil }
        var best: CGFloat = 9, hit: Int?
        for k in 0..<2 {
            let pts = renderer.tiltRing(k, plane).compactMap { project($0) }
            for i in 1..<max(1, pts.count) {
                let d = segmentDistance(p, pts[i - 1], pts[i])
                if d < best { best = d; hit = k }
            }
        }
        let (a, b) = renderer.splitArrow(plane)
        if let s = project(a), let e = project(b), segmentDistance(p, s, e) < best { hit = 2 }
        return hit
    }

    override func mouseUp(with e: NSEvent) {
        switch drag {
        case .orbit, .pan:
            if !moved && lib.mode == .measure, let end = measureSnap(convert(e.locationInWindow, from: nil), free: e.modifierFlags.contains(.option)) {
                lib.measure(end)
            }
            if !moved && (lib.mode == .select || lib.mode == .thread) && !e.modifierFlags.contains(.shift) { lib.selection = [] }
        case .round:
            if moved { lib.commitRound() } else { lib.clearPreview() }
        case .scaleAxis:
            if !moved { lib.undoLastIfUnchanged() } else { lib.finishScale() }
        case .ring, .body, .axis:
            if !moved { lib.undoLastIfUnchanged() } else { lib.finishTransform() }
        default: break
        }
        drag = .none
        dragAxis = nil
        splitHandle = nil
        guides = []
        redraw()
    }

    override func rightMouseDown(with e: NSEvent) { last = convert(e.locationInWindow, from: nil) }
    override func rightMouseDragged(with e: NSEvent) {
        let p = convert(e.locationInWindow, from: nil)
        orbit(p.x - last.x, p.y - last.y)
        last = p
        redraw()
    }
    override func otherMouseDown(with e: NSEvent) { last = convert(e.locationInWindow, from: nil) }
    override func otherMouseDragged(with e: NSEvent) {
        let p = convert(e.locationInWindow, from: nil)
        pan(p.x - last.x, p.y - last.y)
        last = p
        redraw()
    }

    override func scrollWheel(with e: NSEvent) {
        if e.hasPreciseScrollingDeltas && !e.modifierFlags.contains(.option) {
            pan(e.scrollingDeltaX, -e.scrollingDeltaY)
        } else {
            let d = e.hasPreciseScrollingDeltas ? e.scrollingDeltaY * 0.01 : e.scrollingDeltaY * 0.1
            zoom(Float(d), at: convert(e.locationInWindow, from: nil))
        }
        redraw()
    }

    override func magnify(with e: NSEvent) {
        zoom(Float(e.magnification) * 1.5, at: convert(e.locationInWindow, from: nil))
        redraw()
    }

    override func rotate(with e: NSEvent) {
        lib.camera.yaw -= Float(e.rotation) * .pi / 180
        redraw()
    }

    private func orbit(_ dx: CGFloat, _ dy: CGFloat) {
        lib.camera.yaw -= Float(dx) * 0.008
        lib.camera.pitch = max(-1.55, min(1.55, lib.camera.pitch - Float(dy) * 0.008))
    }

    private func pan(_ dx: CGFloat, _ dy: CGFloat) {
        let cam = lib.camera
        let f = normalize(cam.target - cam.eye)
        let r = cam.side
        let u = cross(r, f)
        let wpp = Float(worldPerPoint(at: SIMD3<Double>(cam.target)))
        lib.camera.target += (-r * Float(dx) - u * Float(dy)) * wpp
    }

    private func zoom(_ amount: Float, at p: CGPoint) {
        let k = exp(-amount)
        let old = lib.camera.distance
        let new = max(5, min(8000, old * k))
        if let h = hitBody(p) {
            let t = SIMD3<Float>(h.world)
            lib.camera.target += (t - lib.camera.target) * (1 - new / old)
        }
        lib.camera.distance = new
    }

    // MARK: linking

    struct Box {
        var lo: SIMD3<Double>
        var hi: SIMD3<Double>
        var mid: SIMD3<Double> { (lo + hi) / 2 }
        func moved(_ d: SIMD3<Double>) -> Box { Box(lo: lo + d, hi: hi + d) }
    }

    // A place along one axis a moving side or middle can line up with: what it belongs to, and a hole's or peg's rim to show.
    struct Mark {
        var value: Double
        var box: Box?
        var ring: [SIMD3<Double>] = []
    }

    // Where the other shapes can be lined up with, gathered as a drag starts: their sides and middles, the middles and rims of
    // their holes, pegs and round edges, their flat faces square to an axis (steps, the walls of a square hole) and those
    // faces' middles; then the bed's edges, middle, floor and top.
    private func captureBoxes() {
        var lo = SIMD3<Double>(repeating: .infinity), hi = SIMD3<Double>(repeating: -.infinity)
        for b in lib.selected { if let (l, h) = lib.worldBounds(b) { lo = simd_min(lo, l); hi = simd_max(hi, h) } }
        startBox = lo.x.isFinite ? Box(lo: lo, hi: hi) : nil
        var found: [[Mark]] = [[], [], []]
        others = []
        for b in lib.doc.bodies where !b.hidden && !lib.selection.contains(b.id) {
            guard let (l, h) = lib.worldBounds(b), let m = lib.meshes[b.id] else { continue }
            let box = Box(lo: l, hi: h)
            others.append(box)
            for a in 0..<3 { found[a] += [Mark(value: l[a], box: box), Mark(value: h[a], box: box), Mark(value: box.mid[a], box: box)] }
            for r in Self.rings(m, b.place) {
                let rim = renderer.circle(around: r.axis, r.center, r.radius)
                for a in 0..<3 {
                    found[a].append(Mark(value: r.center[a], box: box, ring: rim))
                    let reach = r.radius * sqrt(max(0, 1 - r.axis[a] * r.axis[a]))
                    if reach > 0.01 { found[a] += [Mark(value: r.center[a] - reach, box: box, ring: rim), Mark(value: r.center[a] + reach, box: box, ring: rim)] }
                }
            }
            let mat = b.place.matrix
            for f in m.faceInfo {
                let n = unit(b.place.rotation * (f.normal / b.place.scale)), c = (mat * SIMD4(f.centroid, 1)).xyz
                guard (0..<3).contains(where: { abs(n[$0]) > 0.999 }) else { continue }
                for k in 0..<3 { found[k].append(Mark(value: c[k], box: box)) }
            }
        }
        let bed = lib.settings.bed
        for a in 0..<2 { found[a] += [-bed[a] / 2, 0, bed[a] / 2].map { Mark(value: $0, box: nil) } }
        found[2] += [0, bed.z].map { Mark(value: $0, box: nil) }
        marks = found
        ownRings = lib.selected.flatMap { b in lib.meshes[b.id].map { Self.rings($0, b.place).map(\.center) } ?? [] }
    }

    // A shape's circles in the world, each once (a hole's rim often comes in two halves).
    static func rings(_ m: Mesh, _ place: Placement) -> [Ring] {
        let mat = place.matrix, grow = (place.scale.x + place.scale.y + place.scale.z) / 3
        var out: [Ring] = []
        for r in m.circles {
            let w = Ring(center: (mat * SIMD4(r.center, 1)).xyz, axis: unit(place.rotation * (r.axis / place.scale)), radius: r.radius * grow)
            if !out.contains(where: { simd_distance($0.center, w.center) < 1e-4 && abs($0.radius - w.radius) < 1e-4 }) { out.append(w) }
        }
        return out
    }

    private var linkReach: Double { 10 * worldPerPoint(at: startBox?.mid ?? startPlane) }

    // A move along one world axis: lines a side, the middle or one of its own holes' middles up with the nearest mark in reach
    // (marks first, then 10 mm grid lines), otherwise steps by the snap step.
    private func place(_ value: Double, axis a: Int, shift: SIMD3<Double>) -> Double {
        let step = (value / lib.settings.snap).rounded() * lib.settings.snap
        guard lib.settings.autoLink, let box = startBox?.moved(shift) else { return step }
        let reach = linkReach
        let features = [box.lo[a], box.hi[a], box.mid[a]] + ownRings.map { $0[a] + shift[a] }
        var best: (score: Double, move: Double, mark: Mark)?
        for f in features {
            for m in marks[a] {
                let d = m.value - f
                if abs(d) < reach, best == nil || abs(d) < best!.score { best = (abs(d), d, m) }
            }
            let g = (f / 10).rounded() * 10
            if abs(g - f) < reach * 0.5, best == nil || abs(g - f) * 2 < best!.score { best = (abs(g - f) * 2, g - f, Mark(value: g)) }
        }
        guard let best else { return step }
        guide(on: a, at: best.mark.value, box, best.mark)
        return value + best.move
    }

    // A resize along one axis: links the size to another shape's width, depth or height, or the moving side to a mark in reach
    // (with symmetric resizing, the size the middle reaching out both ways would make).
    private func resize(_ size: Double, axis i: Int, start: Placement, symmetric: Bool, low: Bool = false) -> Double {
        let step = max(lib.settings.snap, (size / lib.settings.snap).rounded() * lib.settings.snap)
        guard lib.settings.autoLink, let box = startBox else { return step }
        let reach = linkReach
        var best: (score: Double, size: Double, target: Box?)?
        for o in others {
            for k in 0..<3 {
                let s = o.hi[k] - o.lo[k]
                if abs(s - size) < reach, best == nil || abs(s - size) < best!.score { best = (abs(s - size), s, o) }
            }
        }
        if start.turn == SIMD3(0, 0, 0) {
            for m in marks[i] {
                let s = symmetric ? 2 * abs(m.value - box.mid[i]) : low ? box.hi[i] - m.value : m.value - box.lo[i]
                if s > 0, abs(s - size) < reach, best == nil || abs(s - size) < best!.score { best = (abs(s - size), s, m.box) }
            }
        }
        guard let best else { return step }
        if let t = best.target {
            let axis = (0..<3).min { abs((t.hi[$0] - t.lo[$0]) - best.size) < abs((t.hi[$1] - t.lo[$1]) - best.size) } ?? i
            var end = t.lo
            end[axis] = t.hi[axis]
            guides.append((t.lo, end))
        }
        return best.size
    }

    // A guide line across the linked plane, spanning the moving box and what it lines up with, and the rim it lines up with.
    private func guide(on a: Int, at v: Double, _ box: Box, _ mark: Mark) {
        let span = a == 0 ? 1 : 0
        var lo = box.lo, hi = box.hi
        if let t = mark.box { lo = simd_min(lo, t.lo); hi = simd_max(hi, t.hi) } else { lo -= 20; hi += 20 }
        var p = SIMD3<Double>(box.mid.x, box.mid.y, a == 2 ? v : box.lo.z)
        p[a] = v
        var q = p
        p[span] = lo[span]
        q[span] = hi[span]
        guides.append((p, q))
        for k in mark.ring.indices.dropFirst() { guides.append((mark.ring[k - 1], mark.ring[k])) }
    }

    // How far along axis a, through the gizmo as the drag began, the pointer has gone since the press (mm): exactly the point
    // under it, or stepped from its movement while the axis points straight at the viewer.
    private func travel(_ p: CGPoint, _ a: SIMD3<Double>, _ dx: CGFloat, _ dy: CGFloat) -> Double {
        func reach(_ q: CGPoint) -> Double? {
            let (o, d) = ray(q)
            let w = o - startPlane, b = dot(d, a), across = 1 - b * b
            return across > 1e-4 ? (dot(a, w) - b * dot(d, w)) / across : nil
        }
        if let now = reach(p), let then = reach(downAt) { return now - then }
        return accum + along(a, dx, dy)
    }

    // Mouse movement along a world direction, in mm.
    private func along(_ axis: SIMD3<Double>, _ dx: CGFloat, _ dy: CGFloat) -> Double {
        let c = renderer.gizmoCenter
        guard let s = project(c), let e = project(c + axis * renderer.gizmoLength) else { return 0 }
        let v = SIMD2(Double(e.x - s.x), Double(e.y - s.y))
        let l = length(v)
        guard l > 1 else { return 0 }
        return dot(SIMD2(Double(dx), Double(dy)), v / l) / l * renderer.gizmoLength
    }

    private func rotate(_ axis: SIMD3<Double>, _ degrees: Double) {
        let q = simd_quatd(angle: degrees * .pi / 180, axis: normalize(axis))
        let r = simd_double3x3(q)
        let c = startPlane
        for (id, s) in starts {
            lib.mutate(id) { b in
                b.place.turn = Placement.euler(r * s.rotation)
                b.place.move = c + r * (s.move - c)
            }
        }
        lib.sceneVersion += 1
    }
}

struct Viewport: NSViewRepresentable {
    func makeNSView(context: Context) -> CadView { CadView() }
    func updateNSView(_ v: CadView, context: Context) {}
}

// A small label over the 3D view (the ruler's length, what the pointer snaps to). It lets clicks through.
final class Tag: NSView {
    private let label = NSTextField(labelWithString: "")

    init() {
        super.init(frame: .zero)
        wantsLayer = true
        layer?.cornerRadius = 7
        layer?.backgroundColor = NSColor(white: 0.05, alpha: 0.82).cgColor
        label.font = .monospacedDigitSystemFont(ofSize: 12, weight: .semibold)
        addSubview(label)
        isHidden = true
    }

    required init?(coder: NSCoder) { nil }

    override func hitTest(_ point: NSPoint) -> NSView? { nil }

    // At p: its middle, or its left edge when not centred.
    func show(_ text: String, color: NSColor, at p: CGPoint, centered: Bool = true) {
        if label.stringValue != text { label.stringValue = text }
        label.textColor = color
        label.sizeToFit()
        let size = CGSize(width: ceil(label.frame.width) + 14, height: ceil(label.frame.height) + 6)
        label.frame.origin = CGPoint(x: 7, y: 3)
        frame = CGRect(x: (centered ? p.x - size.width / 2 : p.x).rounded(), y: (p.y - size.height / 2).rounded(), width: size.width, height: size.height)
        isHidden = false
    }
}
