#if SELFTEST
import AppKit
import SwiftUI
import simd

// Kernel, file and editing checks: bash test/selftest.sh
@MainActor
enum SelfTest {
    static func run(_ dir: URL) -> Bool {
        setvbuf(stdout, nil, _IONBF, 0)
        var ok = true
        func check(_ name: String, _ cond: Bool, _ note: String = "") {
            print(cond ? "✓" : "✗", name, note)
            if !cond { ok = false }
        }
        let k = Kernel.shared
        k.clearance = 0.2
        func mesh(_ n: Node) -> Mesh? { k.mesh(n) }
        func manifold(_ m: Mesh) -> Bool {
            let (_, tris) = Weld.run(m)
            var count: [SIMD2<UInt32>: Int] = [:]
            for t in tris { for (a, b) in [(t.x, t.y), (t.y, t.z), (t.z, t.x)] { count[SIMD2(min(a, b), max(a, b)), default: 0] += 1 } }
            return !tris.isEmpty && count.values.allSatisfy { $0 == 2 }
        }

        let prims: [Primitive] = [.make(.box), .make(.cylinder), .make(.cone), .make(.sphere), .make(.torus), .make(.wedge)]
            + [3, 5, 6, 8].map { .make(.prism, sides: $0) } + [3, 4, 6, 8].map { .make(.pyramid, sides: $0) }
        for p in prims {
            let m = mesh(.primitive(p))
            check(p.name, m?.valid == true && (m?.volume ?? 0) > 0, String(format: "%.2f mm³", m?.volume ?? 0))
        }

        // Tori: a polygon tube turned round (Pappus), any tube taken along an oval (its area times the oval's length).
        func ovalLength(_ a: Double, _ b: Double, _ degrees: Double) -> Double {
            let t = degrees * .pi / 180
            var length = 0.0, last = SIMD2(a, 0.0)
            for k in 1...20000 {
                let s = 2 * Double.pi * Double(k) / 20000, p = SIMD2(a * cos(s) + b * cos(t) * sin(s), b * sin(t) * sin(s))
                length += simd_distance(p, last)
                last = p
            }
            return length
        }
        let triangle = sqrt(3) / 4 * 64, hexagon = 3 * sqrt(3) / 8 * 64, circle = Double.pi * 16
        let tori: [(Primitive, Double, Bool)] = [
            (.make(.torus, sides: 3), triangle * 2 * .pi * 11, true), (.make(.torus, sides: 6), hexagon * 2 * .pi * 11, true),
            (Primitive(kind: .ovalTorus, sides: 0, size: [40, 30, 90, 8]), circle * ovalLength(16, 11, 90), false),
            (Primitive(kind: .ovalTorus, sides: 6, size: [40, 30, 60, 8]), hexagon * ovalLength(16, 11, 60), false)
        ]
        for (p, want, closed) in tori {
            let m = mesh(.primitive(p))
            check(p.name, m?.valid == true && abs((m?.volume ?? 0) - want) < want * 0.002 && (!closed || manifold(m!)),
                  String(format: "%.2f / %.2f mm³", m?.volume ?? 0, want))
        }
        _ = k.takeProblems()
        check("too thick a tube for an oval refused", mesh(.primitive(Primitive(kind: .ovalTorus, sides: 0, size: [40, 12, 90, 8]))) == nil && k.takeProblems().contains("bend"))
        let every = prims + tori.map(\.0) + [.make(.hemisphere), .make(.bowl), .make(.ring), .make(.glass), .make(.oval), Primitive(kind: .oval, size: [20, 12, 60, 20]),
                                              Primitive(kind: .cone, size: [8, 20, 15]), .make(.ovalTorus, sides: 3)]
        let mismatched = every.filter { p in
            guard let m = mesh(.primitive(p)) else { return true }
            return simd_reduce_max(simd_abs(m.size - p.extent)) > 0.02
        }
        check("every shape's size is known before it's built", mismatched.isEmpty, mismatched.map(\.name).joined(separator: ", "))

        let shaped: [(Primitive, Double)] = [
            (.make(.hemisphere), 2.0 / 3 * .pi * 1000), (.make(.bowl), 2.0 / 3 * .pi * (8000 - 5832)),
            (.make(.ring), .pi * 125 * 5), (.make(.glass), .pi * 2747)
        ]
        for (p, want) in shaped {
            let m = mesh(.primitive(p))
            check(p.name, m?.valid == true && abs((m?.volume ?? 0) - want) < want * 0.001 && manifold(m!), String(format: "%.2f / %.2f mm³", m?.volume ?? 0, want))
        }

        let box = Node.primitive(.make(.box))
        check("cube volume", abs((mesh(box)?.volume ?? 0) - 8000) < 0.5)
        let base = Part(node: box, place: Placement()), shifted = Part(node: box, place: Placement(move: SIMD3(10, 0, 0)))
        let vol = { (op: Int32) in mesh(.group(op: op, parts: [base, shifted]))?.volume ?? 0 }
        check("merge", abs(vol(Int32(BK_UNION)) - 12000) < 1)
        check("subtract", abs(vol(Int32(BK_SUBTRACT)) - 4000) < 1)
        check("intersect", abs(vol(Int32(BK_INTERSECT)) - 4000) < 1)
        let touching = Part(node: box, place: Placement(move: SIMD3(20, 7, 3)))
        _ = k.takeProblems()
        let merged = k.shape(.group(op: Int32(BK_UNION), parts: [base, touching]))
        check("merge is one solid", merged.map { $0.with { bk_piece_count($0) } } == 1 && k.takeProblems().isEmpty)
        _ = k.shape(.group(op: Int32(BK_UNION), parts: [base, Part(node: box, place: Placement(move: SIMD3(40, 0, 0)))]))
        check("separate pieces reported", k.takeProblems().contains("pieces"))

        let plane = Plane(point: SIMD3(0, 0, 3), normal: SIMD3(0, 0, 1))
        let up = mesh(.split(of: box, plane: plane, side: 0))?.volume ?? 0, down = mesh(.split(of: box, plane: plane, side: 1))?.volume ?? 0
        check("split halves", abs(up + down - 8000) < 1 && abs(up - 2800) < 1, String(format: "%.2f + %.2f", up, down))
        let tilted = Plane(point: SIMD3(1, 2, 3), normal: normalize(SIMD3(0.3, -0.2, 1)))
        for (name, whole) in [("cylinder", Node.primitive(.make(.cylinder))), ("M8 bolt", Node.fastener(Fastener(nut: false, size: 4, length: 30, threadOnly: false)))] {
            let all = mesh(whole)?.volume ?? 0
            let a = mesh(.split(of: whole, plane: tilted, side: 0))?.volume ?? 0, b = mesh(.split(of: whole, plane: tilted, side: 1))?.volume ?? 0
            check("tilted split keeps all of the \(name)", a > 0 && b > 0 && abs(a + b - all) < all * 0.000_1, String(format: "%.3f + %.3f = %.3f", a, b, all))
        }

        let oneEdge = 8000 - (4 - Double.pi) * 20
        let edge = Pick(kind: Int32(BK_PICK_EDGE), a: SIMD3(0, -10, 10), b: SIMD3(1, 0, 0))
        let e = mesh(.round(of: box, picks: [edge], radius: 2))
        check("round edge", abs((e?.volume ?? 0) - oneEdge) < 1 && e?.valid == true)
        let corner = Pick(kind: Int32(BK_PICK_CORNER), a: SIMD3(0, 0, 1), b: SIMD3(10, 10, 10))
        let c = mesh(.round(of: box, picks: [corner], radius: 2))
        check("round corner", abs((c?.volume ?? 0) - oneEdge) < 1 && c?.valid == true)
        let face = Pick(kind: Int32(BK_PICK_FACE), a: SIMD3(0, 0, 1), b: SIMD3(0, 0, 10))
        let f = mesh(.round(of: box, picks: [face], radius: 2))
        check("round face", f?.valid == true && (f?.volume ?? 8000) < 8000 - 60)
        let all = mesh(.round(of: box, picks: [Pick(kind: Int32(BK_PICK_BODY), a: .zero, b: .zero)], radius: 2))
        check("round body", all?.valid == true && (all?.volume ?? 8000) < 8000 - 150 && manifold(all!))
        let twice = Node.round(of: .round(of: box, picks: [edge], radius: 2), picks: [Pick(kind: Int32(BK_PICK_BODY), a: .zero, b: .zero)], radius: 3)
        let t2 = mesh(twice)
        check("round all edges after a rounding", t2?.valid == true && manifold(t2!) && (t2?.volume ?? 8000) < (e?.volume ?? 0))
        let skewed = Solid(name: "Skewed", color: Palette.colors[0], node: twice, place: Placement(move: SIMD3(-3, -5, 13), turn: SIMD3(0, 0, -60), scale: SIMD3(1.75, 1.3, 1.3)))
        check("world mesh of a scaled, rotated, rounded body", k.worldMesh(skewed)?.valid == true)
        _ = k.takeProblems()
        let tooBig = mesh(.round(of: box, picks: [edge], radius: 40))
        check("too large radius reported", k.takeProblems().contains { $0.hasPrefix("max:") } && tooBig != nil)

        for size in [0, Int(bk_thread_count()) - 1] {
            for nut in [false, true] {
                for only in [false, true] {
                    let fs = Fastener(nut: nut, size: size, length: nut ? bk_thread_default_length(Int32(size), 1) : 30, threadOnly: only)
                    let t0 = Date()
                    let m = mesh(.fastener(fs))
                    let h = m?.size.z ?? 0
                    let lengthOK = nut || only ? abs(h - fs.length) < 0.01 : h > fs.length
                    let sizeOK = m.map { simd_reduce_max(simd_abs($0.size - fs.extent(clearance: 0.2))) < 0.05 } == true
                    check(fs.name, m?.valid == true && lengthOK && sizeOK && manifold(m!), String(format: "h %.2f · %.1f s", h, Date().timeIntervalSince(t0)))
                }
            }
        }

        let top = Pick(kind: Int32(BK_PICK_FACE), a: SIMD3(0, 0, 1), b: SIMD3(0, 0, 10))
        let bottom = Pick(kind: Int32(BK_PICK_FACE), a: SIMD3(0, 0, -1), b: SIMD3(0, 0, -10))
        let hollows: [(String, Node, Double)] = [
            ("hollow closed", .hollow(of: box, open: [], walls: [], thickness: 2), 8000 - 4096),
            ("hollow open top", .hollow(of: box, open: [top], walls: [], thickness: 2), 8000 - 16 * 16 * 18),
            ("hollow open top, 5 mm bottom", .hollow(of: box, open: [top], walls: [Wall(face: bottom, thickness: 5)], thickness: 2), 8000 - 16 * 16 * 15)
        ]
        for (name, node, want) in hollows {
            let m = mesh(node)
            check(name, m?.valid == true && abs((m?.volume ?? 0) - want) < 0.5, String(format: "%.2f / %.2f mm³", m?.volume ?? 0, want))
        }
        let cylTop = Pick(kind: Int32(BK_PICK_FACE), a: SIMD3(0, 0, 1), b: SIMD3(0, 0, 10))
        let cup = mesh(.hollow(of: .primitive(.make(.cylinder)), open: [cylTop], walls: [], thickness: 1.5))
        check("hollow cylinder", cup?.valid == true && abs((cup?.volume ?? 0) - .pi * (100 * 20 - 8.5 * 8.5 * 18.5)) < 1)
        let lump = Node.group(op: Int32(BK_UNION), parts: [base, Part(node: .primitive(Primitive(kind: .cylinder, size: [12, 30])), place: Placement(move: SIMD3(10, 0, 5)))])
        _ = k.takeProblems()
        let shell = mesh(.hollow(of: lump, open: [], walls: [], thickness: 1.5))
        check("hollow merged shape", shell?.valid == true && !k.takeProblems().contains("hollow") && (shell?.volume ?? 0) < (mesh(lump)?.volume ?? 0))
        _ = mesh(.hollow(of: box, open: [top], walls: [], thickness: 12))
        check("too thick walls reported", k.takeProblems().contains("hollow"))

        let oval = mesh(.primitive(Primitive(kind: .oval, size: [20, 12, 90, 20])))
        check("oval cylinder", oval?.valid == true && abs((oval?.volume ?? 0) - .pi * 10 * 6 * 20) < 1, String(format: "%.2f mm³", oval?.volume ?? 0))
        let skew = mesh(.primitive(Primitive(kind: .oval, size: [20, 12, 60, 20])))
        let skewWant = Double.pi * 10 * 6 * sin(Double.pi / 3) * 20
        check("skewed oval cylinder", skew?.valid == true && abs((skew?.volume ?? 0) - skewWant) < 1, String(format: "%.2f / %.2f mm³", skew?.volume ?? 0, skewWant))
        let bevel = mesh(.bevel(of: box, picks: [edge], legs: SIMD2(2, 4), corner: 0))
        check("bevel 2 × 4 mm", bevel?.valid == true && abs((bevel?.volume ?? 0) - (8000 - 80)) < 0.5, String(format: "%.2f mm³", bevel?.volume ?? 0))
        let soft = mesh(.bevel(of: box, picks: [edge], legs: SIMD2(2, 2), corner: 0.5))
        check("softened bevel", soft?.valid == true && (soft?.volume ?? 8000) < 8000 - 40, String(format: "%.2f mm³", soft?.volume ?? 0))
        let cove = mesh(.cove(of: box, picks: [edge], radius: 3))
        let coveWant = 8000 - Double.pi * 9 / 4 * 20
        check("inward rounding", cove?.valid == true && abs((cove?.volume ?? 0) - coveWant) < 1, String(format: "%.2f / %.2f mm³", cove?.volume ?? 0, coveWant))
        let coves = mesh(.cove(of: box, picks: [Pick(kind: Int32(BK_PICK_BODY), a: .zero, b: .zero)], radius: 1))
        check("inward rounding on all edges", coves?.valid == true && (coves?.volume ?? 8000) < 8000 && manifold(coves!))
        let holed = mesh(.group(op: Int32(BK_SUBTRACT), parts: [base, Part(node: .primitive(Primitive(kind: .cylinder, size: [8, 30])), place: Placement())]))
        let rims = holed?.circles.filter { abs($0.radius - 4) < 1e-6 && abs($0.center.x) < 1e-6 && abs($0.center.y) < 1e-6 && abs(abs($0.center.z) - 10) < 1e-6 } ?? []
        check("a hole's rims are found", rims.count >= 2, "\(holed?.circles.count ?? 0) circles")
        let topEdges = mesh(box).map { Picking.faceEdges($0, Pick(kind: Int32(BK_PICK_FACE), a: SIMD3(0, 0, 1), b: SIMD3(0, 0, 10))).count }
        check("a face knows its edges", topEdges == 4, "\(topEdges ?? 0) edges")
        let cut = k.section(box, edge)
        check("cut through an edge", cut.map { abs($0.angle - 90) < 0.01 && $0.loops.count == 1 } == true, cut.map { String(format: "%.1f° · %d loops", $0.angle, $0.loops.count) } ?? "none")

        let block = Part(node: .primitive(Primitive(kind: .box, size: [30, 30, 20])), place: Placement())
        let bolt = Part(node: .fastener(Fastener(nut: false, size: 4, length: 30, threadOnly: true)), place: Placement())
        let t0 = Date()
        let hole = mesh(.group(op: Int32(BK_SUBTRACT), parts: [block, bolt]))
        check("threaded hole", hole?.valid == true && (hole?.volume ?? 0) < 18000 - 600 && manifold(hole!), String(format: "%.1f s · %.2f mm³", Date().timeIntervalSince(t0), hole?.volume ?? 0))

        var doc = Document()
        let mixed = SIMD3<UInt8>(12, 34, 56)
        doc.bodies = [Solid(name: "Cube", color: mixed, node: .round(of: box, picks: [edge], radius: 2)),
                      Solid(name: "M8 bolt", color: Palette.colors[1], node: .fastener(Fastener(nut: false, size: 4, length: 30, threadOnly: false)), place: Placement(move: SIMD3(40, 0, 15))),
                      Solid(name: "Box", color: Palette.colors[2], node: .hollow(of: box, open: [top], walls: [Wall(face: bottom, thickness: 5)], thickness: 2), place: Placement(move: SIMD3(-40, 0, 10))),
                      Solid(name: "Oval", color: Palette.colors[3], node: .cove(of: .primitive(Primitive(kind: .oval, size: [20, 12, 70, 20])),
                                                                 picks: [Pick(kind: Int32(BK_PICK_BODY), a: .zero, b: .zero)], radius: 1), place: Placement(move: SIMD3(0, 40, 10))),
                      Solid(name: "Bevelled", color: Palette.colors[4], node: .bevel(of: box, picks: [edge], legs: SIMD2(2, 3), corner: 0.4), place: Placement(move: SIMD3(0, -40, 10))),
                      Solid(name: "Hex ring", color: Palette.colors[5], node: .primitive(Primitive(kind: .ovalTorus, sides: 6, size: [40, 30, 60, 8])), place: Placement(move: SIMD3(60, 40, 3.46)))]
        let meshes = doc.bodies.compactMap { b in k.worldMesh(b).map { (b, $0) } }
        let u3 = dir.appendingPathComponent("test.3mf")
        try? ThreeMF.write(u3, meshes: meshes, doc: doc)
        check("3mf reopens editable", (try? ThreeMF.read(u3))?.doc == doc)
        check("a mixed colour is saved", (try? ThreeMF.read(u3))?.doc.bodies.first?.color == mixed)
        check("3mf carries every body's shape", (try? ThreeMF.read(u3))?.meshes.count == doc.bodies.count)
        let parts = (try? Zip.read(Data(contentsOf: u3))) ?? [:]
        check("3mf model parses", parts[ThreeMF.modelPath].map { XMLParser(data: $0).parse() } == true)
        let stl = dir.appendingPathComponent("test.stl")
        try? STL.write(stl, meshes: meshes.map(\.1))
        let stlSize = (try? Data(contentsOf: stl).count) ?? 0
        check("stl", stlSize == 84 + 50 * meshes.reduce(0) { $0 + $1.1.indices.count / 3 })
        let step = dir.appendingPathComponent("test.step")
        let wrote = k.exportStep(doc.bodies, to: step.path)
        check("step", wrote && ((try? String(contentsOf: step, encoding: .utf8))?.hasPrefix("ISO-10303-21") ?? false))

        // Sizes the kernel must refuse at once (they crashed or hung it), and results with nothing left in them.
        let t1 = Date()
        let refused = [Primitive(kind: .box, size: [.nan, 20, 20]), Primitive(kind: .oval, size: [20, 12, 90, 0]), Primitive(kind: .torus, size: [10, 30]),
                       Primitive(kind: .cone, size: [0, 0, 20])].allSatisfy { mesh(.primitive($0)) == nil }
        check("bad sizes refused", refused && Date().timeIntervalSince(t1) < 5)
        _ = k.takeProblems()
        let apart = Part(node: box, place: Placement(move: SIMD3(100, 0, 0)))
        check("nothing left reported", mesh(.group(op: Int32(BK_SUBTRACT), parts: [base, base])) == nil && mesh(.group(op: Int32(BK_INTERSECT), parts: [base, apart])) == nil
              && mesh(.split(of: box, plane: Plane(point: SIMD3(0, 0, 50), normal: SIMD3(0, 0, 1)), side: 0)) == nil && k.takeProblems().contains("empty"))
        let broken = Part(node: .primitive(Primitive(kind: .box, size: [0, 20, 20])), place: Placement())
        check("subtract with a broken part fails", mesh(.group(op: Int32(BK_SUBTRACT), parts: [broken, shifted])) == nil)
        _ = k.takeProblems()

        // Damaged files are refused, never crash.
        var bad = [UInt8]((try? Data(contentsOf: u3)) ?? Data())
        let eocd = bad.count - 22, cd = Int(bad[eocd + 16]) | Int(bad[eocd + 17]) << 8 | Int(bad[eocd + 18]) << 16 | Int(bad[eocd + 19]) << 24
        bad[cd + 28] = 0xFF
        bad[cd + 29] = 0xFF
        let damaged = dir.appendingPathComponent("damaged.3mf")
        try? Data(bad).write(to: damaged)
        check("damaged 3mf refused", (try? ThreeMF.read(damaged)) == nil)
        var odd = Document()
        odd.bodies = [Solid(name: "Odd", color: Palette.colors[0], node: .primitive(Primitive(kind: .box, size: [20])))]
        let oddURL = dir.appendingPathComponent("odd.3mf")
        try? ThreeMF.write(oddURL, meshes: [], doc: odd)
        check("3mf with wrong sizes refused", (try? ThreeMF.read(oddURL)) == nil)

        // Opening and editing (last: the workbench builds on the kernel's thread from here on).
        let lib = Workbench.shared
        lib.open(u3)
        check("open loads the file unchanged", lib.doc == doc && !lib.dirty)
        let shown = zip(doc.bodies, meshes).allSatisfy { b, m in
            guard let p = lib.meshes[b.id], !p.vertices.isEmpty else { return false }
            return abs(p.volume - m.1.volume) < m.1.volume * 0.02
        }
        check("open shows the saved shapes at once", shown)
        lib.begin()
        lib.undoLastIfUnchanged()
        check("a click changes nothing", !lib.dirty)
        if let id = lib.doc.bodies.first?.id { lib.setPlace(id) { $0.move.x += 5 } }
        check("a move is a change", lib.dirty)
        lib.undo()
        check("undo back to the saved file", !lib.dirty)
        if let id = lib.doc.bodies.first?.id {
            lib.selection = [id]
            lib.enter(.hollow)
            lib.angleEdit = AngleEdit(body: id, picks: [], section: Section(loops: [], angle: 90, point: .zero, direction: SIMD3(0, 0, 1)))
        }
        lib.open(u3)
        check("open leaves the tools behind", lib.mode == .select && lib.angleEdit == nil && lib.editBody == nil && lib.selection.isEmpty)
        let copy = dir.appendingPathComponent("saved.3mf")
        lib.fileURL = copy
        var saved: Bool?
        let asked = Date()
        lib.saveDocument { saved = $0 }
        let returned = Date().timeIntervalSince(asked) < 0.5
        while saved == nil && Date().timeIntervalSince(asked) < 120 { RunLoop.main.run(until: Date().addingTimeInterval(0.05)) }
        check("save works in the background", returned && saved == true && (try? ThreeMF.read(copy))?.doc == lib.doc && !lib.dirty)

        // Resizing: a new size keeps the left, front and bottom sides (or the middle), roundings go along, drags stretch one
        // side, several shapes stretch along one axis only, and nothing drops to the bed.
        func settle() {
            let t = Date()
            repeat { RunLoop.main.run(until: Date().addingTimeInterval(0.02)) } while lib.building && Date().timeIntervalSince(t) < 120
        }
        func bounds(_ id: UUID) -> (SIMD3<Double>, SIMD3<Double>) { lib.body(id).flatMap { lib.worldBounds($0) } ?? (.zero, .zero) }
        func near(_ a: SIMD3<Double>, _ b: SIMD3<Double>, _ tolerance: Double = 0.01) -> Bool { simd_reduce_max(simd_abs(a - b)) < tolerance }
        func use(_ bodies: [Solid]) {
            lib.selection = []
            lib.doc.bodies = bodies
            lib.rebuildScene()
            settle()
        }
        lib.settings = Settings()
        let rounded = Solid(name: "Rounded", color: Palette.colors[0], node: .round(of: box, picks: [edge], radius: 2), place: Placement(move: SIMD3(0, 0, 30)))
        use([rounded])
        lib.reshape(rounded.id) { n in
            guard case .primitive(var p) = n else { return n }
            p.size = [40, 20, 40]
            return .primitive(p)
        }
        settle()
        let (wideLo, wideHi) = bounds(rounded.id)
        check("a new size keeps the left, front and bottom sides", near(wideLo, SIMD3(-10, -10, 20)) && near(wideHi, SIMD3(30, 10, 60)))
        let wide = lib.meshes[rounded.id]?.volume ?? 0, wideWant = 32000 - (4 - Double.pi) * 40
        check("a rounding goes along with a new size", abs(wide - wideWant) < 1, String(format: "%.2f / %.2f mm³", wide, wideWant))
        lib.settings.symmetric = true
        lib.reshape(rounded.id) { n in
            guard case .primitive(var p) = n else { return n }
            p.size = [20, 20, 20]
            return .primitive(p)
        }
        settle()
        let (backLo, backHi) = bounds(rounded.id)
        check("a symmetric new size keeps the middle", near((backLo + backHi) / 2, (wideLo + wideHi) / 2) && near(backHi - backLo, SIMD3(20, 20, 20)))
        lib.settings.symmetric = false
        lib.selection = [rounded.id]
        if let start = lib.body(rounded.id)?.place { lib.stretch([rounded.id: start], axis: 2, by: 1.5, uniform: false, symmetric: false) }
        lib.finishScale()
        settle()
        let (tallLo, tallHi) = bounds(rounded.id)
        let baked = lib.body(rounded.id).map { $0.place.scale == SIMD3(1, 1, 1) && $0.node.base == .primitive(Primitive(kind: .box, size: [20, 20, 30])) } == true
        check("a resize drag keeps the bottom and stays up", near(tallLo, SIMD3(backLo.x, backLo.y, backLo.z)) && abs(tallHi.z - backLo.z - 30) < 0.01 && baked)
        let left = Solid(name: "Left", color: Palette.colors[0], node: box, place: Placement(move: SIMD3(-20, 0, 10)))
        let right = Solid(name: "Right", color: Palette.colors[1], node: .primitive(.make(.cylinder)), place: Placement(move: SIMD3(20, 0, 10)))
        use([left, right])
        lib.stretch([left.id: left.place, right.id: right.place], axis: 0, by: 2, uniform: false, symmetric: false)
        check("several shapes stretch along one axis only", near(bounds(left.id).0, SIMD3(-30, -10, 0)) && near(bounds(left.id).1, SIMD3(10, 10, 20))
              && near(bounds(right.id).0, SIMD3(50, -10, 0)) && near(bounds(right.id).1, SIMD3(90, 10, 20)))
        lib.selection = []
        lib.addThread()
        settle()
        check("⌘B adds a bolt and opens Thread", lib.primary.map { if case .fastener = $0.node { true } else { false } } == true && lib.screen == .thread)

        // Real mouse events on the 3D view (offscreen): handles, ⌥ for symmetric, two shapes, a face in Angles.
        let view = CadView()
        view.frame = NSRect(x: 0, y: 0, width: 900, height: 700)
        func event(_ type: NSEvent.EventType, _ p: CGPoint, _ mods: NSEvent.ModifierFlags) -> NSEvent {
            NSEvent.mouseEvent(with: type, location: p, modifierFlags: mods, timestamp: 0, windowNumber: 0, context: nil, eventNumber: 0, clickCount: 1, pressure: 1)!
        }
        func drag(_ from: CGPoint, _ to: CGPoint, _ mods: NSEvent.ModifierFlags = []) {
            view.mouseDown(with: event(.leftMouseDown, from, mods))
            for k in 1...12 {
                let t = CGFloat(k) / 12
                view.mouseDragged(with: event(.leftMouseDragged, CGPoint(x: from.x + (to.x - from.x) * t, y: from.y + (to.y - from.y) * t), mods))
            }
            view.mouseUp(with: event(.leftMouseUp, to, mods))
            settle()
        }
        // From the handle of axis i to where it lies `mm` further along that axis.
        func pull(_ i: Int, _ mm: Double, _ mods: NSEvent.ModifierFlags = []) {
            let r = view.renderer!, c = r.gizmoCenter, a = r.gizmoAxes()[i], at = r.gizmoLength * 0.95
            guard let from = view.project(c + a * at), let to = view.project(c + a * (at + mm)) else { return }
            drag(from, to, mods)
        }
        let cube = Solid(name: "Cube", color: Palette.colors[0], node: box, place: Placement(move: SIMD3(0, 0, 10)))
        use([cube])
        lib.camera = Camera()
        lib.camera.distance = 150
        lib.selection = [cube.id]
        lib.choose(.resize)
        pull(2, 10)
        let (pullLo, pullHi) = bounds(cube.id)
        check("dragging the top handle keeps the bottom", abs(pullLo.z) < 0.01 && abs(pullHi.z - 30) < 0.6, String(format: "z %.2f … %.2f", pullLo.z, pullHi.z))
        pull(2, 5, .option)
        let (optLo, optHi) = bounds(cube.id)
        check("⌥-dragging keeps the middle", abs((optLo.z + optHi.z) / 2 - (pullLo.z + pullHi.z) / 2) < 0.01 && abs(optHi.z - optLo.z - (pullHi.z - pullLo.z) - 10) < 0.6,
              String(format: "z %.2f … %.2f", optLo.z, optHi.z))
        let other = Solid(name: "Other", color: Palette.colors[1], node: box, place: Placement(move: SIMD3(30, 0, 10)))
        use([cube, other])
        lib.selection = [cube.id, other.id]
        lib.choose(.move)
        lib.choose(.resize)
        pull(0, 10)
        let pair = [cube.id, other.id].map { lib.body($0)?.node.base }
        let widened = pair.allSatisfy { if case .primitive(let p) = $0 { p.size[0] > 20.5 && p.size[1] == 20 && p.size[2] == 20 } else { false } }
        check("dragging the side handle of two shapes changes that side only", widened, pair.map { "\($0.map { "\($0)" } ?? "none")" }.joined(separator: " · "))
        use([cube])
        lib.selection = [cube.id]
        lib.choose(.angles)
        if let p = view.project(SIMD3(0, 0, 20)) {
            view.mouseDown(with: event(.leftMouseDown, p, []))
            view.mouseUp(with: event(.leftMouseUp, p, []))
        }
        let picked = lib.edgePicks.first
        let lit = picked.flatMap { pk in lib.meshes[cube.id].map { Picking.faceEdges($0, pk).count } } ?? 0
        check("a face clicked in Angles is picked with its edges", lib.edgePicks.count == 1 && picked?.kind == Int32(BK_PICK_FACE) && lit == 4, "\(lit) edges")
        // The workbench's build must end before the process does: OpenCascade tears itself down at exit.
        k.queue.sync {}
        print(ok ? "ALL OK" : "FAILURES")
        return ok
    }
}

// TEMPORARY: which part of the interface makes the shape bar's quick button lose a click in its middle. Real pointer
// events (CGEvent) on copies of the interface with one piece left out at a time.
@MainActor
enum ClickProbe {
    static var frames: [String: CGRect] = [:]
    static var taps = 0
    static var window: NSWindow?

    static func run() {
        setvbuf(stdout, nil, _IONBF, 0)
        let app = NSApplication.shared
        app.setActivationPolicy(.regular)
        print("accessibility trusted:", AXIsProcessTrusted())
        Thread.detachNewThread { drive() }
        DispatchQueue.main.asyncAfter(deadline: .now() + 500) { print("✗ probe timed out"); exit(1) }
        app.run()
    }

    static func show(_ v: AnyView) {
        window?.close()
        frames = [:]
        let w = NSWindow(contentRect: NSRect(x: 0, y: 0, width: 1024, height: 674), styleMask: [.titled, .fullSizeContentView], backing: .buffered, defer: false)
        w.isReleasedWhenClosed = false
        w.contentView = NSHostingView(rootView: v.environment(Workbench.shared).skinEnvironment())
        w.setFrameTopLeftPoint(NSPoint(x: 0, y: (NSScreen.main?.frame.height ?? 768) - 30))
        w.makeKeyAndOrderFront(nil)
        NSApp.activate(ignoringOtherApps: true)
        window = w
    }

    // A spot in the probed button (fractions of its frame), on the screen (CG: top-left origin).
    static func spot(_ key: String, _ fx: CGFloat, _ fy: CGFloat) -> CGPoint? {
        guard let w = window, let f = frames[key], let c = w.contentView else { return nil }
        let x = f.minX + f.width * fx, y = f.minY + f.height * fy
        let screen = w.convertPoint(toScreen: c.convert(NSPoint(x: x, y: c.isFlipped ? y : c.bounds.height - y), to: nil))
        return CGPoint(x: screen.x, y: (NSScreen.screens.first?.frame.height ?? 768) - screen.y)
    }

    nonisolated static func onMain<T: Sendable>(_ f: @MainActor () -> T) -> T { DispatchQueue.main.sync { MainActor.assumeIsolated(f) } }

    nonisolated static func drive() {
        func post(_ type: CGEventType, _ p: CGPoint) {
            CGEvent(mouseEventSource: nil, mouseType: type, mouseCursorPosition: p, mouseButton: .left)?.post(tap: .cghidEventTap)
            usleep(25_000)
        }
        func total() -> Int { onMain { Workbench.shared.doc.bodies.count + taps } }
        func click(_ key: String, _ fx: CGFloat, _ fy: CGFloat) -> String {
            guard let p = onMain({ spot(key, fx, fy) }) else { return "?" }
            let before = total()
            post(.mouseMoved, p)
            usleep(600_000)
            post(.leftMouseDown, p)
            usleep(60_000)
            post(.leftMouseUp, p)
            usleep(500_000)
            return total() > before ? "✓" : "✗"
        }
        func variant(_ name: String, _ key: String = "root-blocks", _ make: @escaping @MainActor () -> AnyView) {
            onMain { show(make()) }
            sleep(2)
            onMain {
                NSApp.activate(ignoringOtherApps: true)
                window?.makeKeyAndOrderFront(nil)
            }
            sleep(1)
            var line = "\(name) \(onMain { frames[key].map { "\($0)" } ?? "no frame" }):"
            let h = onMain { frames[key]?.height ?? 34 }
            for dy in [-6.0, -1, -0.5, 0, 0.5, 1, 6] as [CGFloat] { line += String(format: " %+.1f", dy) + click(key, 0.5, 0.5 + dy / h) }
            line += " left" + click(key, 0.15, 0.5)
            print(line)
        }
        sleep(2)
        func one<L: View>(_ name: String, top: Bool = false, size: CGFloat = 34, @ViewBuilder _ label: @escaping () -> L,
                          _ wrap: @escaping @MainActor (AnyView) -> AnyView = { $0 }) {
            variant(name, "target") {
                let b = wrap(AnyView(Button { ClickProbe.taps += 1 } label: { label() }.buttonStyle(NeonButtonStyle(size: size)))).probed("target")
                return top ? AnyView(TopHarness { b }) : AnyView(Harness { b })
            }
        }
        variant("whole interface") { AnyView(RootView()) }
        for size in [16, 24, 28, 30, 32, 34, 36] as [CGFloat] {
            one("size \(Int(size))", size: size) { Image(systemName: "cube") }
            one("size \(Int(size)) at the top", top: true, size: size) { Image(systemName: "cube") }
        }
        onMain {
            Kernel.shared.queue.sync {}
            exit(0)
        }
    }
}

extension View {
    func probed(_ name: String) -> some View {
        onGeometryChange(for: CGRect.self) { $0.frame(in: .global) } action: { ClickProbe.frames[name] = $0 }
    }
}

// The probed view at the bottom middle, laid out like the real window.
struct Harness<Content: View>: View {
    @ViewBuilder let content: Content

    var body: some View {
        VStack(spacing: 10) {
            Spacer()
            content
        }
        .padding(.bottom, 16)
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .ignoresSafeArea()
    }
}

struct TopHarness<Content: View>: View {
    @ViewBuilder let content: Content

    var body: some View {
        VStack(spacing: 10) {
            content
            Spacer()
        }
        .padding(.top, 60)
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .ignoresSafeArea()
    }
}

struct ProbeBar: View {
    var arrows = true
    var menus = true
    var glass = true
    var symbols = false
    @State private var open: ShapeGroup?

    var body: some View {
        let bar = HStack(spacing: 8) {
            ForEach(ShapeGroup.allCases, id: \.self) { g in
                HStack(spacing: 1) {
                    Group {
                        if symbols {
                            Button { ClickProbe.taps += 1 } label: { Image(systemName: "cube") }
                        } else {
                            Button { ClickProbe.taps += 1 } label: { ShapeIcon(prim: g.members[0].primitive, size: 15) }
                        }
                    }
                    .buttonStyle(NeonButtonStyle(size: 34))
                    .help("x")
                    .probed(g == .blocks ? "target" : "other")
                    if arrows {
                        Button { withAnimation(Neon.glide) { open = open == g ? nil : g } } label: {
                            Image(systemName: "chevron.up").font(.ui(size: 8, weight: .black))
                        }
                        .buttonStyle(NeonButtonStyle(lit: open == g, size: 16))
                        .rotationEffect(.degrees(open == g ? 180 : 0))
                    }
                }
                .overlay(alignment: .bottom) {
                    if menus && open == g {
                        Text("menu").padding(6).glassBar(16).fixedSize().offset(y: -54).transition(.menu)
                    }
                }
            }
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 8)
        if glass { bar.glassBar(22) } else { bar }
    }
}

// RootView with pieces left out.
struct RootCopy: View {
    @Environment(Workbench.self) private var lib
    let leave: Set<String>

    var body: some View {
        let look = Skin.shared
        let root = ZStack {
            if !leave.contains("backdrop") { Backdrop() }
            if !leave.contains("3D view") {
                Viewport().padding(look.rtl ? .trailing : .leading, lib.drawerOpen ? 320 * look.scale : 0)
            }
            if leave.contains("scaled") { ui } else { ScaledUI { ui } }
        }
        .background(WindowConfigurator())
        .ignoresSafeArea()
        if leave.contains("animations") {
            root
        } else {
            root
                .animation(Neon.glide, value: lib.busy)
                .animation(Neon.glide, value: lib.mode)
                .animation(Neon.glide, value: lib.selection.isEmpty)
                .animation(Neon.glide, value: lib.drawerOpen)
        }
    }

    private var ui: some View {
        ZStack {
            HStack(spacing: 0) {
                if lib.drawerOpen && !leave.contains("drawer") {
                    Drawer().transition(.move(edge: .leading).combined(with: .haze))
                }
                ZStack {
                    VStack(spacing: 10) {
                        Spacer()
                        ShapeBar()
                    }
                    .padding(.bottom, 16)
                    if !leave.contains("rail") {
                        HStack(alignment: .top, spacing: 10) {
                            ToolRail()
                        }
                        .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topTrailing)
                        .padding(.top, 14)
                        .padding(.trailing, 14)
                    }
                }
            }
        }
    }
}
#endif
