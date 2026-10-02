#if SELFTEST
import AppKit
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
        let occt = String(cString: bk_occt_version())
        check("OpenCascade's version is known for the acknowledgements", occt.split(separator: ".").count == 3, occt)
        func mesh(_ n: Node) -> Mesh? { k.mesh(n) }
        func manifold(_ m: Mesh) -> Bool {
            let (_, tris) = Weld.run(m)
            var count: [SIMD2<UInt32>: Int] = [:]
            for t in tris { for (a, b) in [(t.x, t.y), (t.y, t.z), (t.z, t.x)] { count[SIMD2(min(a, b), max(a, b)), default: 0] += 1 } }
            return !tris.isEmpty && count.values.allSatisfy { $0 == 2 }
        }
        // The ruler's kernel half: the gap between two faces, a point to a face, an edge to an edge, a stretched placement,
        // a cylinder's side, and a face that isn't there.
        do {
            let cubeNode = Node.primitive(.make(.box)), canNode = Node.primitive(.make(.cylinder))
            let cm = mesh(cubeNode), cyl = mesh(canNode)
            let px = cm?.faceInfo.firstIndex { $0.normal.x > 0.9 } ?? -1, nx = cm?.faceInfo.firstIndex { $0.normal.x < -0.9 } ?? -1
            let side = cyl?.faceInfo.firstIndex { abs($0.normal.z) < 0.1 } ?? -1
            func at(_ x: Double, _ sx: Double = 1) -> [Double] { Placement(move: SIMD3(x, 0, 0), scale: SIMD3(sx, 1, 1)).kernel }
            func face(_ n: Node, _ f: Int, _ place: [Double]) -> GapEnd { GapEnd(kind: Int32(BK_END_FACE), index: Int32(f), node: n, place: place, point: [0, 0, 0]) }
            let gaps = [
                k.distance(face(cubeNode, px, at(0)), face(cubeNode, nx, at(25)))?.distance,
                k.distance(GapEnd(kind: Int32(BK_END_POINT), point: [10, 10, 10]), face(cubeNode, nx, at(25)))?.distance,
                k.distance(GapEnd(kind: Int32(BK_END_EDGE), index: 0, node: cubeNode, place: at(0), point: [0, 0, 0]),
                           GapEnd(kind: Int32(BK_END_EDGE), index: 0, node: cubeNode, place: at(30), point: [0, 0, 0]))?.distance,
                k.distance(face(cubeNode, px, at(0, 2)), face(cubeNode, nx, at(45)))?.distance,
                k.distance(face(cubeNode, px, at(0)), face(canNode, side, at(30)))?.distance
            ]
            let want: [Double] = [5, 5, 30, 15, 10]
            let fine = gaps.count == want.count && zip(gaps, want).allSatisfy { g, w in g.map { abs($0 - w) < 1e-6 } ?? false }
            check("the ruler's gap between faces, points and edges, placed and stretched", fine && k.distance(face(cubeNode, 99, at(0)), face(cubeNode, nx, at(25))) == nil,
                  gaps.map { $0.map { String(format: "%.3f", $0) } ?? "–" }.joined(separator: " "))
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
        for (name, whole) in [("cylinder", Node.primitive(.make(.cylinder))), ("M8 bolt", Node.fastener(Fastener(kind: .hex, size: 4)))] {
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

        // Every bolt head and nut: its standard sizes fit at every thread; built at the smallest and largest, it's one closed
        // solid of the size known beforehand, as long as its length (plus a head that isn't countersunk).
        let outside = Fastener.Kind.allCases.flatMap { kind in
            (0..<Int(bk_thread_count())).compactMap { size -> String? in
                let fs = Fastener(kind: kind, size: size)
                return kind.fields.allSatisfy { fs.range($0)?.contains(fs[$0]) == true } ? nil : "\(kind) M\(size)"
            }
        }
        check("every head's and nut's standard sizes fit", outside.isEmpty, outside.joined(separator: ", "))
        for size in [0, Int(bk_thread_count()) - 1] {
            for kind in Fastener.Kind.allCases {
                let fs = Fastener(kind: kind, size: size)
                let t0 = Date()
                let m = mesh(.fastener(fs))
                let h = m?.size.z ?? 0
                let tall = fs.length + (kind.nut || kind.countersunk || kind == .rod ? 0 : fs.height)
                let sizeOK = m.map { simd_reduce_max(simd_abs($0.size - fs.extent(clearance: 0.2))) < 0.05 } == true
                check("\(fs.name) · \(kind)", m?.valid == true && abs(h - tall) < 0.01 && sizeOK && manifold(m!), String(format: "h %.2f · %.1f s", h, Date().timeIntervalSince(t0)))
            }
        }
        // A size may go past what the others allow as they are, and they follow: a lower socket head gets a shallower socket,
        // a narrower countersunk head a smaller Torx.
        let socketHead = Fastener(kind: .socket, size: 4), lower = socketHead.setting(.height, 3)
        check("a lower head takes a shallower socket with it", socketHead.range(.height, loose: true)?.contains(3) == true && socketHead.range(.height)?.contains(3) == false
              && lower.height == 3 && lower.depth <= 2.7 + 1e-9 && mesh(.fastener(lower))?.valid == true, String(format: "depth %.2f", lower.depth))
        let sunk = Fastener(kind: .torxCone, size: 4).setting(.drive, 50), narrower = sunk.setting(.width, 12)
        check("a narrower countersunk head takes a smaller Torx with it", sunk.drive == 50 && narrower.width == 12 && narrower.drive < 50
              && mesh(.fastener(narrower))?.valid == true, String(format: "T%.0f", narrower.drive))
        let phHead = Fastener(kind: .phCone, size: 4).setting(.drive, 2)
        check("a Phillips size brings its recess", phHead.drive == 2 && phHead.recess == 5 && mesh(.fastener(phHead))?.valid == true)
        _ = k.takeProblems()
        var misfit = Fastener(kind: .torx, size: 4)
        misfit.depth = 20
        check("sizes that don't fit are refused and said", mesh(.fastener(misfit)) == nil && k.takeProblems().contains { $0.hasPrefix("bolt") })
        // Bolts and nuts in files from earlier versions: hex or plain, with the standard sizes.
        let earlierBolt = try? JSONDecoder().decode(Fastener.self, from: Data(#"{"nut":false,"size":4,"length":30,"threadOnly":false}"#.utf8))
        let earlierSleeve = try? JSONDecoder().decode(Fastener.self, from: Data(#"{"nut":true,"size":0,"length":2.4,"threadOnly":true}"#.utf8))
        check("earlier bolts and nuts open as hex and plain ones", earlierBolt == Fastener(kind: .hex, size: 4) && earlierSleeve?.kind == .sleeve
              && earlierSleeve?.width == 1.6 && earlierSleeve?.length == 2.4)

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

        // Rounded shapes: hollowed with walls as thick as the rounding (the defaults), closed and with the top open; a
        // bevel and a rounding beside an earlier rounding; different roundings, and a wall of its own, hollowed.
        let roundAll = Node.round(of: box, picks: [Pick(kind: Int32(BK_PICK_BODY), a: .zero, b: .zero)], radius: 2)
        let roundAllVolume = mesh(roundAll)?.volume ?? 0
        let roundShell = mesh(.hollow(of: roundAll, open: [], walls: [], thickness: 2))
        check("rounded cube hollowed", roundShell?.valid == true && abs((roundShell?.volume ?? 0) - (roundAllVolume - 4096)) < 1 && !k.takeProblems().contains("hollow"),
              String(format: "%.2f / %.2f mm³", roundShell?.volume ?? 0, roundAllVolume - 4096))
        let roundCup = mesh(.hollow(of: roundAll, open: [top], walls: [], thickness: 2))
        check("rounded cube hollowed with its top open", roundCup?.valid == true && (roundCup?.volume ?? 0) > 0 && (roundCup?.volume ?? 0) < (roundShell?.volume ?? 0)
              && !k.takeProblems().contains("hollow"), String(format: "%.2f mm³", roundCup?.volume ?? 0))
        let thinCup = mesh(.hollow(of: roundAll, open: [top], walls: [], thickness: 1))
        check("rounded cube hollowed with thinner walls than its rounding", thinCup?.valid == true && (thinCup?.volume ?? 0) > 0 && !k.takeProblems().contains("hollow"))
        let uprights = [(-10.0, -10.0), (-10, 10), (10, -10), (10, 10)].map { Pick(kind: Int32(BK_PICK_EDGE), a: SIMD3($0.0, $0.1, 0), b: SIMD3(0, 0, 1)) }
        let upright = Node.round(of: box, picks: uprights, radius: 2)
        let uprightVolume = mesh(upright)?.volume ?? 0
        let bevelBeside = mesh(.bevel(of: upright, picks: [edge], legs: SIMD2(2, 2), corner: 0))
        check("bevel beside a rounding", bevelBeside?.valid == true && (bevelBeside?.volume ?? 8000) < uprightVolume - 30 && !k.takeProblems().contains("bevel"),
              String(format: "%.2f mm³", bevelBeside?.volume ?? 0))
        let roundBeside = mesh(.round(of: upright, picks: [face], radius: 2))
        check("rounding beside a rounding", roundBeside?.valid == true && (roundBeside?.volume ?? 8000) < uprightVolume - 30 && k.takeProblems().isEmpty)
        // Different roundings meeting at the corners (1 mm up the sides, 2 mm round the top) hollow in moments, not minutes.
        let started = Date()
        let mixedCup = mesh(.hollow(of: .round(of: .round(of: box, picks: uprights, radius: 1), picks: [top], radius: 2), open: [top], walls: [], thickness: 2))
        let took = Date().timeIntervalSince(started)
        check("a cube rounded 1 mm up its sides and 2 mm round its top is hollowed", mixedCup?.valid == true && abs((mixedCup?.volume ?? 0) - 3309.6) < 1
              && !k.takeProblems().contains("hollow") && took < 30, String(format: "%.2f mm³ · %.1f s", mixedCup?.volume ?? 0, took))
        // A face with a wall of its own beside roundings gets just that wall (4 mm on +x, 2 mm elsewhere).
        let side = Pick(kind: Int32(BK_PICK_FACE), a: SIMD3(1, 0, 0), b: SIMD3(10, 0, 0))
        let ownWall = mesh(.hollow(of: roundAll, open: [], walls: [Wall(face: side, thickness: 4)], thickness: 2))
        check("a rounded cube with a thicker wall of its own", ownWall?.valid == true && abs((ownWall?.volume ?? 0) - (roundAllVolume - 14 * 16 * 16)) < 1
              && !k.takeProblems().contains("hollow"), String(format: "%.2f / %.2f mm³", ownWall?.volume ?? 0, roundAllVolume - 14 * 16 * 16))

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
        let bolt = Part(node: .fastener(Fastener(kind: .rod, size: 4)), place: Placement())
        let t0 = Date()
        let hole = mesh(.group(op: Int32(BK_SUBTRACT), parts: [block, bolt]))
        check("threaded hole", hole?.valid == true && (hole?.volume ?? 0) < 18000 - 600 && manifold(hole!), String(format: "%.1f s · %.2f mm³", Date().timeIntervalSince(t0), hole?.volume ?? 0))

        var doc = Document()
        let mixed = SIMD3<UInt8>(12, 34, 56)
        doc.bodies = [Solid(name: "Cube", color: mixed, node: .round(of: box, picks: [edge], radius: 2)),
                      Solid(name: "M8 bolt", color: Palette.colors[1], node: .fastener(Fastener(kind: .hex, size: 4)), place: Placement(move: SIMD3(40, 0, 15))),
                      Solid(name: "M5 Torx", color: Palette.colors[6], node: .fastener(Fastener(kind: .torxCone, size: 2)), place: Placement(move: SIMD3(-60, 40, 10))),
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

        // Renaming: a saved file is renamed where it is and a name already taken is refused; an unsaved document keeps the
        // name for saving.
        lib.renameDocument("renamed")
        let renamed = dir.appendingPathComponent("renamed.3mf")
        check("renaming a saved document renames its file", lib.fileURL == renamed && lib.title == "renamed"
              && FileManager.default.fileExists(atPath: renamed.path) && !FileManager.default.fileExists(atPath: copy.path))
        try? Data().write(to: dir.appendingPathComponent("taken.3mf"))
        lib.note = nil
        lib.renameDocument("taken")
        check("a name already taken is refused", lib.fileURL == renamed && lib.note == L("A file named “{name}” already exists", ["name": "taken.3mf"]), lib.note ?? "")
        lib.fileURL = nil
        lib.renameDocument(" Bracket / v2 ")
        check("an unsaved document keeps a new name", lib.title == "Bracket - v2", lib.title)
        lib.docName = nil

        // Resizing: a new size keeps the left, front and bottom sides (or the middle), roundings go along, drags stretch one
        // side, several shapes stretch along one axis only, and nothing drops to the bed.
        func settle() {
            let t = Date()
            repeat { RunLoop.main.run(until: Date().addingTimeInterval(0.02)) } while (lib.building || lib.trying) && Date().timeIntervalSince(t) < 120
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

        // Turning a raised shape leaves it where it is, as resizing does, with dropping onto the bed on.
        let raisedBox = Solid(name: "Raised", color: Palette.colors[2], node: box, place: Placement(move: SIMD3(0, 0, 40)))
        use([raisedBox])
        lib.settings.dropToBed = true
        lib.selection = [raisedBox.id]
        lib.gizmo = .rotate
        lib.setPlace(raisedBox.id) { $0.turn.x = 45 }
        lib.finishTransform()
        settle()
        lib.gizmo = .move
        let turnedLow = bounds(raisedBox.id).0.z
        check("a turned shape stays up", abs(turnedLow - (40 - 10 * sqrt(2))) < 0.01, String(format: "bottom at %.2f", turnedLow))
        // A turned shape's box takes in every point of its mesh, and follows a new turn.
        let turnedBolt = Solid(name: "Turned bolt", color: Palette.colors[3], node: .fastener(Fastener(kind: .hex, size: 0)),
                               place: Placement(move: SIMD3(0, 0, 30), turn: SIMD3(33, 21, 0)))
        use([turnedBolt])
        func everyPoint(of b: Solid) -> (SIMD3<Double>, SIMD3<Double>) {
            var lo = SIMD3<Double>(repeating: .infinity), hi = SIMD3<Double>(repeating: -.infinity)
            for v in lib.meshes[b.id]?.vertices ?? [] {
                let w = b.place.matrix * SIMD4(Double(v.x), Double(v.y), Double(v.z), 1)
                lo = simd_min(lo, SIMD3(w.x, w.y, w.z))
                hi = simd_max(hi, SIMD3(w.x, w.y, w.z))
            }
            return (lo, hi)
        }
        let first = bounds(turnedBolt.id), firstWant = everyPoint(of: turnedBolt)
        lib.setPlace(turnedBolt.id) { $0.turn.z = 50 }
        let second = bounds(turnedBolt.id), secondWant = lib.body(turnedBolt.id).map(everyPoint) ?? (.zero, .zero)
        check("a turned shape's box holds all of it and follows a new turn", near(first.0, firstWant.0, 1e-9) && near(first.1, firstWant.1, 1e-9)
              && near(second.0, secondWant.0, 1e-9) && near(second.1, secondWant.1, 1e-9) && !near(first.0, second.0))
        lib.selection = []
        lib.addThread()
        settle()
        check("⌘B adds a bolt and opens Thread", lib.primary.map { if case .fastener = $0.node { true } else { false } } == true && lib.screen == .thread)

        // A treatment that doesn't fit never enters the document (it's said once); one that fits goes in.
        let plain = Solid(name: "Plain", color: Palette.colors[0], node: box, place: Placement(move: SIMD3(0, 0, 10)))
        use([plain])
        lib.selection = [plain.id]
        lib.enter(.hollow)
        lib.hollowOpen = [top]
        lib.hollowThickness = 12
        lib.note = nil
        let unchanged = lib.doc
        lib.commitHollow()
        settle()
        check("walls that don't fit change nothing and say so", lib.doc == unchanged && lib.mode == .hollow
              && lib.note == L("These walls don't fit this shape — try thinner walls"), lib.note ?? "no message")
        lib.cancelMode()
        let softCube = Solid(name: "Soft", color: Palette.colors[1], node: roundAll, place: Placement(move: SIMD3(0, 0, 10)))
        use([softCube])
        lib.selection = [softCube.id]
        lib.enter(.hollow)
        lib.hollowOpen = [top]
        lib.hollowThickness = 2
        lib.note = nil
        lib.commitHollow()
        settle()
        let cupped = lib.body(softCube.id).map { if case .hollow = $0.node { true } else { false } } == true
        check("a rounded cube is hollowed with its top open", cupped && lib.mode == .select && lib.note == nil
              && (lib.meshes[softCube.id]?.volume ?? 0) < roundAllVolume - 1000, lib.note ?? "")

        // Hollow on a hollowed shape changes that hollow (beneath a later rounding too) rather than hollowing it again;
        // a sphere's one surface can't be opened.
        let shut = Solid(name: "Shut", color: Palette.colors[2], node: .hollow(of: box, open: [], walls: [], thickness: 2), place: Placement(move: SIMD3(0, 0, 10)))
        use([shut])
        lib.selection = [shut.id]
        lib.hollowThickness = 1
        lib.enter(.hollow)
        let loaded = lib.hollowThickness == 2 && lib.hollowOpen.isEmpty && lib.editBody == shut.id
        lib.pickHollowFace(top, ownWall: false)
        lib.note = nil
        lib.commitHollow()
        settle()
        check("a hollowed box gets its top opened later", loaded && lib.body(shut.id)?.node == .hollow(of: box, open: [top], walls: [], thickness: 2)
              && abs((lib.meshes[shut.id]?.volume ?? 0) - (8000 - 16 * 16 * 18)) < 0.5 && lib.note == nil, lib.note ?? "")
        lib.enter(.hollow)
        lib.pickHollowFace(Pick(kind: top.kind, a: top.a, b: top.b + SIMD3(0.01, 0.01, 0)), ownWall: false)
        check("clicking an opening again closes it", lib.hollowOpen.isEmpty && lib.hollowThickness == 2)
        lib.cancelMode()
        let roundedShut = Solid(name: "Rounded shut", color: Palette.colors[3], node: .round(of: .hollow(of: box, open: [], walls: [], thickness: 2), picks: uprights, radius: 1),
                                place: Placement(move: SIMD3(0, 0, 10)))
        use([roundedShut])
        lib.selection = [roundedShut.id]
        lib.enter(.hollow)
        lib.pickHollowFace(top, ownWall: false)
        lib.note = nil
        lib.commitHollow()
        settle()
        check("a hollow beneath a rounding is opened in place",
              lib.body(roundedShut.id)?.node == .round(of: .hollow(of: box, open: [top], walls: [], thickness: 2), picks: uprights, radius: 1) && lib.note == nil, lib.note ?? "")
        let ball = k.queue.sync { k.mesh(.primitive(.make(.sphere))) }, cubeMesh = k.queue.sync { k.mesh(box) }
        check("a sphere's one surface can't be opened, a box's faces can", ball.map { Picking.alone($0, 0) } == true
              && cubeMesh.map { m in !m.faceInfo.isEmpty && m.faceInfo.indices.allSatisfy { !Picking.alone(m, $0) } } == true)

        // The straight views look along an axis at the selection, or at every shape when nothing is selected.
        func steady(_ c: Camera) -> Bool { [c.view.columns.0, c.view.columns.1, c.view.columns.2, c.view.columns.3].allSatisfy { simd_reduce_max(simd_abs($0)).isFinite } }
        // Angles the same way round, whatever whole turns lie between them.
        func turned(_ a: Float, _ b: Float) -> Bool { abs(remainder(a - b, 2 * .pi)) < 1e-4 }
        // Until the glide has ended: facing the way asked and no longer moving (a slow machine takes longer than half a second).
        func flown(_ yaw: Float, _ pitch: Float) {
            let t = Date()
            var last = lib.camera
            repeat {
                RunLoop.main.run(until: Date().addingTimeInterval(0.05))
                let now = lib.camera
                if turned(now.yaw, yaw) && abs(now.pitch - pitch) < 1e-4 && now.target == last.target && now.distance == last.distance { return }
                last = now
            } while Date().timeIntervalSince(t) < 15
        }
        let westBox = Solid(name: "West", color: Palette.colors[4], node: box, place: Placement(move: SIMD3(-40, 0, 10)))
        let eastBox = Solid(name: "East", color: Palette.colors[5], node: box, place: Placement(move: SIMD3(40, 0, 10)))
        use([westBox, eastBox])
        lib.selection = [eastBox.id]
        lib.look(from: .top)
        flown(0, .pi / 2)
        let fromTop = lib.camera
        lib.selection = []
        lib.look(from: .north)
        flown(.pi, 0)
        let fromNorth = lib.camera
        var below = Camera()
        below.pitch = -.pi / 2
        check("straight views look along the axes, at the selection or at everything",
              turned(fromTop.yaw, 0) && abs(fromTop.pitch - .pi / 2) < 1e-4 && near(SIMD3<Double>(fromTop.target), SIMD3(40, 0, 10), 0.01) && steady(fromTop)
              && turned(fromNorth.yaw, .pi) && abs(fromNorth.pitch) < 1e-4 && near(SIMD3<Double>(fromNorth.target), SIMD3(0, 0, 10), 0.01)
              && fromNorth.distance > fromTop.distance && steady(below),
              String(format: "top %.3f/%.3f, north %.3f/%.3f", fromTop.yaw, fromTop.pitch, fromNorth.yaw, fromNorth.pitch))

        // The split plane's square holds every cut through the selection, however far the plane is tilted and moved.
        lib.selection = [westBox.id, eastBox.id]
        var uncovered = 0
        let lo = simd_min(bounds(westBox.id).0, bounds(eastBox.id).0), hi = simd_max(bounds(westBox.id).1, bounds(eastBox.id).1)
        let corners = (0..<8).map { i in SIMD3(i & 1 == 0 ? lo.x : hi.x, i & 2 == 0 ? lo.y : hi.y, i & 4 == 0 ? lo.z : hi.z) }
        for axis in 0..<3 {
            for tilt in [SIMD2(0.0, 0), SIMD2(80, 0), SIMD2(-45, 70), SIMD2(80, -80)] {
                for offset in [-30.0, 0, 25] {
                    lib.splitAxis = axis
                    lib.splitTilt = tilt
                    lib.splitOffset = offset
                    guard let plane = lib.splitPlane else { uncovered += 1; continue }
                    let (c, half) = lib.splitPatch(plane)
                    for i in 0..<8 {
                        for j in 0..<8 where i < j && (i ^ j).nonzeroBitCount == 1 {
                            let a = simd_dot(corners[i] - plane.point, plane.normal), b = simd_dot(corners[j] - plane.point, plane.normal)
                            guard a * b <= 0, a != b else { continue }
                            let q = corners[i] + (corners[j] - corners[i]) * (a / (a - b))
                            if simd_length(q - c) > half + 1e-9 { uncovered += 1 }
                        }
                    }
                }
            }
        }
        lib.splitAxis = 2
        lib.splitTilt = .zero
        lib.splitOffset = 0
        check("the split plane covers every cut through the selection", uncovered == 0, "\(uncovered) points outside")

        // Angles on a shape with rounded edges: a 2 mm bevel beside the rounding goes in.
        let upstanding = Solid(name: "Upright", color: Palette.colors[3], node: upright, place: Placement(move: SIMD3(0, 0, 10)))
        use([upstanding])
        lib.selection = [upstanding.id]
        lib.choose(.angles)
        if let section = k.queue.sync({ k.section(upright, edge) }) {
            var a = AngleEdit(body: upstanding.id, picks: [edge], section: section)
            a.treatment = .angled
            a.legs = SIMD2(2, 2)
            lib.angleEdit = a
            lib.note = nil
            lib.applyAngles()
            settle()
        }
        let bevelled = lib.body(upstanding.id).map { if case .bevel = $0.node { true } else { false } } == true
        check("an Angles bevel beside a rounding goes in", bevelled && lib.angleEdit == nil && lib.note == nil, lib.note ?? "")

        // The angle editor opens on a side face of a rounded, hollowed cup.
        let cupBody = Solid(name: "Cup", color: Palette.colors[4], node: .hollow(of: .round(of: box, picks: [top], radius: 2), open: [top], walls: [], thickness: 2),
                        place: Placement(move: SIMD3(0, 0, 10)))
        use([cupBody])
        lib.selection = [cupBody.id]
        lib.choose(.move)
        lib.choose(.angles)
        lib.edgePicks = [Pick(kind: Int32(BK_PICK_FACE), a: SIMD3(-1, 0, 0), b: SIMD3(-10, 0, -1))]
        let opening = Date()
        lib.workWithAngles()
        while lib.angleEdit == nil && Date().timeIntervalSince(opening) < 60 { RunLoop.main.run(until: Date().addingTimeInterval(0.02)) }
        check("the angle editor opens on a rounded, hollowed cup", lib.angleEdit != nil, String(format: "%.1f s · %@", Date().timeIntervalSince(opening), lib.note ?? ""))
        lib.closeAngles()
        lib.cancelMode()

        // Problems left by other work (a save, a cut) aren't said at the next rebuild.
        _ = k.queue.sync { k.mesh(.hollow(of: box, open: [top], walls: [], thickness: 13)) }
        lib.note = nil
        use([Solid(name: "Fresh", color: Palette.colors[2], node: box, place: Placement(move: SIMD3(0, 0, 10)))])
        check("nothing left over from other work is said later", lib.note == nil, lib.note ?? "")

        // A file from an earlier version keeps colours as palette numbers: it opens, with those colours.
        let earlier = Document(bodies: [Solid(name: "Old", color: Palette.colors[2], node: roundAll, place: Placement(move: SIMD3(0, 0, 10)))])
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.sortedKeys]
        let oldJSON = String(decoding: (try? encoder.encode(earlier)) ?? Data(), as: UTF8.self).replacingOccurrences(of: "\"color\":[255,184,51]", with: "\"color\":2")
        let oldURL = dir.appendingPathComponent("old.3mf")
        try? Zip.write([(ThreeMF.docPath, Data(oldJSON.utf8))]).write(to: oldURL)
        lib.open(oldURL)
        settle()
        check("a file from an earlier version opens", oldJSON.contains("\"color\":2") && lib.doc.bodies.first?.color == Palette.colors[2]
              && lib.doc.bodies.first?.node == roundAll && lib.doc.bodies.first.map { !(lib.meshes[$0.id]?.vertices.isEmpty ?? true) } == true)

        // A rounded and hollowed cube, a ring and a torus saved together reopen with every shape shown and nothing said.
        let kept = Document(bodies: [
            Solid(name: "Cup", color: Palette.colors[0], node: .hollow(of: roundAll, open: [top], walls: [], thickness: 2), place: Placement(move: SIMD3(0, 0, 10))),
            Solid(name: "Ring", color: Palette.colors[1], node: .primitive(.make(.ring)), place: Placement(move: SIMD3(30, 0, 2.5))),
            Solid(name: "Torus", color: Palette.colors[2], node: .primitive(.make(.torus)), place: Placement(move: SIMD3(-30, 0, 4)))
        ])
        let keptMeshes = k.queue.sync { kept.bodies.compactMap { b in k.worldMesh(b).map { (b, $0) } } }
        let keptURL = dir.appendingPathComponent("kept.3mf")
        try? ThreeMF.write(keptURL, meshes: keptMeshes, doc: kept)
        lib.note = nil
        lib.open(keptURL)
        settle()
        check("a saved rounded cup, ring and torus reopen", keptMeshes.count == 3 && lib.doc == kept && lib.note == nil
              && kept.bodies.allSatisfy { !(lib.meshes[$0.id]?.vertices.isEmpty ?? true) }, lib.note ?? "")

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
        // From the handle of axis i to where it lies `mm` further out along it (the handle points the way it is drawn).
        func pull(_ i: Int, _ mm: Double, _ mods: NSEvent.ModifierFlags = []) {
            let r = view.renderer!
            r.turnGizmo()
            let c = r.gizmoCenter, a = r.gizmoHandles()[i], at = r.gizmoLength * 0.95
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
        // Seen from behind, the side handles turn round with the view: x's points to -x, and pulling it grows that side.
        use([cube])
        lib.selection = [cube.id]
        lib.choose(.move)
        lib.choose(.resize)
        lib.camera.yaw = .pi - 0.6
        let r = view.renderer!
        r.turnGizmo()
        let (behindLo, behindHi) = bounds(cube.id)
        pull(0, 10)
        let (turnLo, turnHi) = bounds(cube.id)
        check("from behind the handles face the view, and the near side grows", r.gizmoSides.x < 0 && r.gizmoSides.y < 0 && r.gizmoSides.z > 0
              && abs(turnHi.x - behindHi.x) < 0.01 && abs(turnLo.x - (behindLo.x - 10)) < 0.6,
              String(format: "sides %.0f %.0f %.0f · x %.2f … %.2f", r.gizmoSides.x, r.gizmoSides.y, r.gizmoSides.z, turnLo.x, turnHi.x))
        lib.camera = Camera()
        lib.camera.distance = 150
        r.turnGizmo()
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
        // Sizes in mm and in percent on shapes that were edited after they were made: a rounded, hollowed box and a merge.
        let edited = Solid(name: "Edited", color: Palette.colors[3], node: .round(of: .hollow(of: box, open: [top], walls: [], thickness: 2), picks: [edge], radius: 1),
                           place: Placement(move: SIMD3(0, 0, 10)))
        let merged = Solid(name: "Merged", color: Palette.colors[4], node: .group(op: Int32(BK_UNION), parts: [
            Part(node: box, place: Placement()), Part(node: .primitive(.make(.cylinder)), place: Placement(move: SIMD3(15, 0, 0)))
        ]), place: Placement(move: SIMD3(60, 0, 10)))
        use([edited, merged])
        lib.settings = Settings()
        var resized: [String] = []
        for s in [edited, merged] {
            // As the Size row does: the size shown is the mesh's along the shape's axes, times its scale.
            guard let b = lib.body(s.id), let m = lib.meshes[s.id] else { resized.append("\(s.name): no mesh"); continue }
            lib.rescale(s.id, axis: 0, by: 40 / (m.size.x * b.place.scale.x))
            settle()
            let (lo1, hi1) = bounds(s.id)
            let before = hi1.y - lo1.y
            // As the Scale row does: 50 % of the shape as it is now (its y scale is still 100 %).
            if let b2 = lib.body(s.id) { lib.rescale(s.id, axis: 1, by: 50 / 100 / b2.place.scale.y) }
            settle()
            let (lo2, hi2) = bounds(s.id)
            if abs(hi1.x - lo1.x - 40) > 0.05 || abs(hi2.y - lo2.y - before / 2) > 0.05 || abs(hi2.x - lo2.x - 40) > 0.05 {
                resized.append(String(format: "%@: %.2f × %.2f (from %.2f)", s.name, hi2.x - lo2.x, hi2.y - lo2.y, before))
            }
        }
        check("an edited shape and a merge resize to a size in mm and by percent", resized.isEmpty, resized.joined(separator: " · "))
        // The ruler, through the pointer: it snaps to what can be seen, two corners give their distance, two faces the gap
        // between them as well, Esc clears and then leaves.
        func click(_ w: SIMD3<Double>, _ mods: NSEvent.ModifierFlags = []) {
            guard let p = view.project(w) else { return }
            view.mouseDown(with: event(.leftMouseDown, p, mods))
            view.mouseUp(with: event(.leftMouseUp, p, mods))
        }
        func snap(_ w: SIMD3<Double>, by d: CGPoint = CGPoint(x: 4, y: 3)) -> MeasureEnd? {
            view.project(w).flatMap { view.measureSnap(CGPoint(x: $0.x + d.x, y: $0.y + d.y)) }
        }
        func gapSettled() {
            k.queue.sync {}
            RunLoop.main.run(until: Date().addingTimeInterval(0.05))
        }
        lib.camera = Camera()
        lib.camera.distance = 150
        use([cube])
        lib.enter(.measure)
        let seenCorner = snap(SIMD3(10, -10, 20))
        let hidden = snap(SIMD3(10, 10, 0), by: .zero)
        check("the ruler snaps to a corner it can see, not to one behind", lib.mode == .measure && seenCorner?.snap == .corner
              && near(seenCorner?.point ?? .zero, SIMD3(10, -10, 20), 1e-4) && !(hidden.map { near($0.point, SIMD3(10, 10, 0), 0.5) } ?? false),
              "\(String(describing: seenCorner?.snap)) · behind: \(String(describing: hidden?.snap))")
        let onEdge = snap(SIMD3(5, -10, 20), by: CGPoint(x: 0, y: 3))
        check("between corners it snaps to the edge", onEdge?.snap == .edge && abs((onEdge?.point.x ?? 0) - 5) < 0.5
              && near(SIMD3(0, onEdge?.point.y ?? 0, onEdge?.point.z ?? 0), SIMD3(0, -10, 20), 0.01), "\(String(describing: onEdge))")
        click(SIMD3(10, -10, 20))
        click(SIMD3(-10, 10, 0))
        gapSettled()
        let span = lib.measureA.flatMap { a in lib.measureB.map { simd_length($0.point - a.point) } } ?? 0
        check("two corners measure their distance", abs(span - (1200.0).squareRoot()) < 0.01 && lib.gap == nil, String(format: "%.3f mm", span))
        let other2 = Solid(name: "Other", color: Palette.colors[1], node: box, place: Placement(move: SIMD3(30, 0, 10)))
        use([cube, other2])
        click(SIMD3(0, -10, 10))
        click(SIMD3(30, -10, 15))
        gapSettled()
        check("two faces measure the gap between them too", lib.measureA?.snap == .face && lib.measureB?.snap == .face
              && abs((lib.shownGap?.distance ?? 0) - 10) < 0.01,
              String(format: "%@ · gap %.3f mm", Ruler.mm(lib.measureA.flatMap { a in lib.measureB.map { simd_length($0.point - a.point) } } ?? 0), lib.gap?.distance ?? -1))
        let can = Solid(name: "Can", color: Palette.colors[2], node: .primitive(.make(.cylinder)), place: Placement(move: SIMD3(0, 0, 10)))
        use([can])
        let centre = snap(SIMD3(0, 0, 20))
        check("a circle's centre is a snap", centre?.snap == .centre && near(centre?.point ?? .zero, SIMD3(0, 0, 20), 1e-4), "\(String(describing: centre?.snap))")
        click(SIMD3(0, 0, 20))
        lib.cancelMode()
        let cleared = lib.measureA == nil && lib.mode == .measure
        lib.cancelMode()
        check("Esc clears the ruler, then leaves it", cleared && lib.mode == .select)
        // The workbench's build must end before the process does: OpenCascade tears itself down at exit.
        k.queue.sync {}
        print(ok ? "ALL OK" : "FAILURES")
        return ok
    }
}
#endif
