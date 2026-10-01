#if SELFTEST
import Foundation
import simd

// Kernel and file checks: bash test/selftest.sh
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
        let skewed = Solid(name: "Skewed", color: 0, node: twice, place: Placement(move: SIMD3(-3, -5, 13), turn: SIMD3(0, 0, -60), scale: SIMD3(1.75, 1.3, 1.3)))
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
                    check(fs.name, m?.valid == true && lengthOK && manifold(m!), String(format: "h %.2f · %.1f s", h, Date().timeIntervalSince(t0)))
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
        let cut = k.section(box, edge)
        check("cut through an edge", cut.map { abs($0.angle - 90) < 0.01 && $0.loops.count == 1 } == true, cut.map { String(format: "%.1f° · %d loops", $0.angle, $0.loops.count) } ?? "none")

        let block = Part(node: .primitive(Primitive(kind: .box, size: [30, 30, 20])), place: Placement())
        let bolt = Part(node: .fastener(Fastener(nut: false, size: 4, length: 30, threadOnly: true)), place: Placement())
        let t0 = Date()
        let hole = mesh(.group(op: Int32(BK_SUBTRACT), parts: [block, bolt]))
        check("threaded hole", hole?.valid == true && (hole?.volume ?? 0) < 18000 - 600 && manifold(hole!), String(format: "%.1f s · %.2f mm³", Date().timeIntervalSince(t0), hole?.volume ?? 0))

        var doc = Document()
        doc.bodies = [Solid(name: "Cube", color: 0, node: .round(of: box, picks: [edge], radius: 2)),
                      Solid(name: "M8 bolt", color: 1, node: .fastener(Fastener(nut: false, size: 4, length: 30, threadOnly: false)), place: Placement(move: SIMD3(40, 0, 15))),
                      Solid(name: "Box", color: 2, node: .hollow(of: box, open: [top], walls: [Wall(face: bottom, thickness: 5)], thickness: 2), place: Placement(move: SIMD3(-40, 0, 10))),
                      Solid(name: "Oval", color: 3, node: .cove(of: .primitive(Primitive(kind: .oval, size: [20, 12, 70, 20])),
                                                                 picks: [Pick(kind: Int32(BK_PICK_BODY), a: .zero, b: .zero)], radius: 1), place: Placement(move: SIMD3(0, 40, 10))),
                      Solid(name: "Bevelled", color: 4, node: .bevel(of: box, picks: [edge], legs: SIMD2(2, 3), corner: 0.4), place: Placement(move: SIMD3(0, -40, 10)))]
        let meshes = doc.bodies.compactMap { b in k.worldMesh(b).map { (b.name, $0) } }
        let u3 = dir.appendingPathComponent("test.3mf")
        try? ThreeMF.write(u3, meshes: meshes, doc: doc)
        check("3mf reopens editable", (try? ThreeMF.read(u3)) == doc)
        let parts = (try? Zip.read(Data(contentsOf: u3))) ?? [:]
        check("3mf model parses", parts[ThreeMF.modelPath].map { XMLParser(data: $0).parse() } == true)
        let stl = dir.appendingPathComponent("test.stl")
        try? STL.write(stl, meshes: meshes.map(\.1))
        let stlSize = (try? Data(contentsOf: stl).count) ?? 0
        check("stl", stlSize == 84 + 50 * meshes.reduce(0) { $0 + $1.1.indices.count / 3 })
        let step = dir.appendingPathComponent("test.step")
        let wrote = k.exportStep(doc.bodies, to: step.path)
        check("step", wrote && ((try? String(contentsOf: step, encoding: .utf8))?.hasPrefix("ISO-10303-21") ?? false))
        print(ok ? "ALL OK" : "FAILURES")
        return ok
    }
}
#endif
