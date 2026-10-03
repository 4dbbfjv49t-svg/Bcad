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
        // On Bcad's own engine (./build.sh --selftest --engine own) what only OpenCascade does yet is skipped, and said.
        let own = occt.isEmpty
        func skipped(_ what: String) { print("–", what, "(not in Bcad's own engine yet)") }
        check(own ? "Bcad's own engine has no OpenCascade to acknowledge" : "OpenCascade's version is known for the acknowledgements",
              own || occt.split(separator: ".").count == 3, occt)
        // As exact as each engine tells a shape's box: Bcad's own to the last digits, OpenCascade's to a tenth of a micron.
        let tight = own ? 1e-9 : 1e-4
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
            // An upright edge (engines number edges differently).
            let standing = Int32(cm?.edges.firstIndex { e in (e.map(\.z).max() ?? 0) - (e.map(\.z).min() ?? 0) > 1 } ?? 0)
            func at(_ x: Double, _ sx: Double = 1) -> [Double] { Placement(move: SIMD3(x, 0, 0), scale: SIMD3(sx, 1, 1)).kernel }
            func face(_ n: Node, _ f: Int, _ place: [Double]) -> GapEnd { GapEnd(kind: Int32(BK_END_FACE), index: Int32(f), node: n, place: place, point: [0, 0, 0]) }
            let gaps = [
                k.distance(face(cubeNode, px, at(0)), face(cubeNode, nx, at(25)))?.distance,
                k.distance(GapEnd(kind: Int32(BK_END_POINT), point: [10, 10, 10]), face(cubeNode, nx, at(25)))?.distance,
                k.distance(GapEnd(kind: Int32(BK_END_EDGE), index: standing, node: cubeNode, place: at(0), point: [0, 0, 0]),
                           GapEnd(kind: Int32(BK_END_EDGE), index: standing, node: cubeNode, place: at(30), point: [0, 0, 0]))?.distance,
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
        check("shapes apart merge without complaint", k.takeProblems().isEmpty)
        let atCorner = k.shape(.group(op: Int32(BK_UNION), parts: [base, Part(node: box, place: Placement(move: SIMD3(20, 20, 20)))]))
        check("shapes touching at a corner merge into one piece", atCorner.map { $0.with { bk_piece_count($0) } } == 1 && k.takeProblems().isEmpty)

        let plane = Plane(point: SIMD3(0, 0, 3), normal: SIMD3(0, 0, 1))
        let up = mesh(.split(of: box, plane: plane, side: 0))?.volume ?? 0, down = mesh(.split(of: box, plane: plane, side: 1))?.volume ?? 0
        check("split halves", abs(up + down - 8000) < 1 && abs(up - 2800) < 1, String(format: "%.2f + %.2f", up, down))
        let tilted = Plane(point: SIMD3(1, 2, 3), normal: normalize(SIMD3(0.3, -0.2, 1)))
        let splitWholes: [(String, Node)] = [("cylinder", .primitive(.make(.cylinder)))] + (own ? [] : [("M8 bolt", .fastener(Fastener(kind: .hex, size: 4)))])
        for (name, whole) in splitWholes {
            let all = mesh(whole)?.volume ?? 0
            let a = mesh(.split(of: whole, plane: tilted, side: 0))?.volume ?? 0, b = mesh(.split(of: whole, plane: tilted, side: 1))?.volume ?? 0
            check("tilted split keeps all of the \(name)", a > 0 && b > 0 && abs(a + b - all) < all * 0.000_1, String(format: "%.3f + %.3f = %.3f", a, b, all))
        }

        let edge = Pick(kind: Int32(BK_PICK_EDGE), a: SIMD3(0, -10, 10), b: SIMD3(1, 0, 0))
        let face = Pick(kind: Int32(BK_PICK_FACE), a: SIMD3(0, 0, 1), b: SIMD3(0, 0, 10))
        let oneEdge = 8000 - (4 - Double.pi) * 20
        let e = mesh(.round(of: box, picks: [edge], radius: 2))
        check("round edge", abs((e?.volume ?? 0) - oneEdge) < 1 && e?.valid == true)
        let corner = Pick(kind: Int32(BK_PICK_CORNER), a: SIMD3(0, 0, 1), b: SIMD3(10, 10, 10))
        let c = mesh(.round(of: box, picks: [corner], radius: 2))
        check("round corner", abs((c?.volume ?? 0) - oneEdge) < 1 && c?.valid == true)
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
        if own { skipped("bolts and nuts") } else {
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
        }
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
        let roundAll = Node.round(of: box, picks: [Pick(kind: Int32(BK_PICK_BODY), a: .zero, b: .zero)], radius: 2)
        let uprights = [(-10.0, -10.0), (-10, 10), (10, -10), (10, 10)].map { Pick(kind: Int32(BK_PICK_EDGE), a: SIMD3($0.0, $0.1, 0), b: SIMD3(0, 0, 1)) }
        let upright = Node.round(of: box, picks: uprights, radius: 2)
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
        let roundAllVolume = mesh(roundAll)?.volume ?? 0
        let roundShell = mesh(.hollow(of: roundAll, open: [], walls: [], thickness: 2))
        check("rounded cube hollowed", roundShell?.valid == true && abs((roundShell?.volume ?? 0) - (roundAllVolume - 4096)) < 1 && !k.takeProblems().contains("hollow"),
              String(format: "%.2f / %.2f mm³", roundShell?.volume ?? 0, roundAllVolume - 4096))
        let roundCup = mesh(.hollow(of: roundAll, open: [top], walls: [], thickness: 2))
        check("rounded cube hollowed with its top open", roundCup?.valid == true && (roundCup?.volume ?? 0) > 0 && (roundCup?.volume ?? 0) < (roundShell?.volume ?? 0)
              && !k.takeProblems().contains("hollow"), String(format: "%.2f mm³", roundCup?.volume ?? 0))
        let thinCup = mesh(.hollow(of: roundAll, open: [top], walls: [], thickness: 1))
        check("rounded cube hollowed with thinner walls than its rounding", thinCup?.valid == true && (thinCup?.volume ?? 0) > 0 && !k.takeProblems().contains("hollow"))
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
        if own { skipped("threaded holes") } else {
            let block = Part(node: .primitive(Primitive(kind: .box, size: [30, 30, 20])), place: Placement())
            let bolt = Part(node: .fastener(Fastener(kind: .rod, size: 4)), place: Placement())
            let t0 = Date()
            let hole = mesh(.group(op: Int32(BK_SUBTRACT), parts: [block, bolt]))
            check("threaded hole", hole?.valid == true && (hole?.volume ?? 0) < 18000 - 600 && manifold(hole!), String(format: "%.1f s · %.2f mm³", Date().timeIntervalSince(t0), hole?.volume ?? 0))
        }

        var doc = Document()
        let mixed = SIMD3<UInt8>(12, 34, 56)
        // Treated shapes (hollowed, inward-rounded, bevelled) on either engine; bolts on OpenCascade's, a holed block and a
        // half sphere on Bcad's own.
        let treated = [Solid(name: "Box", color: Palette.colors[2], node: .hollow(of: box, open: [top], walls: [Wall(face: bottom, thickness: 5)], thickness: 2), place: Placement(move: SIMD3(-40, 0, 10))),
                       Solid(name: "Oval", color: Palette.colors[3], node: .cove(of: .primitive(Primitive(kind: .oval, size: [20, 12, 70, 20])),
                                                                  picks: [Pick(kind: Int32(BK_PICK_BODY), a: .zero, b: .zero)], radius: 1), place: Placement(move: SIMD3(0, 40, 10))),
                       Solid(name: "Bevelled", color: Palette.colors[4], node: .bevel(of: box, picks: [edge], legs: SIMD2(2, 3), corner: 0.4), place: Placement(move: SIMD3(0, -40, 10))),
                       Solid(name: "Hex ring", color: Palette.colors[5], node: .primitive(Primitive(kind: .ovalTorus, sides: 6, size: [40, 30, 60, 8])), place: Placement(move: SIMD3(60, 40, 3.46)))]
        let roundedCube = Solid(name: "Cube", color: mixed, node: .round(of: box, picks: [edge], radius: 2))
        doc.bodies = own ? [roundedCube,
                            Solid(name: "Holed", color: Palette.colors[1], node: .group(op: Int32(BK_SUBTRACT), parts: [
                                base, Part(node: .primitive(Primitive(kind: .cylinder, size: [8, 30])), place: Placement())
                            ]), place: Placement(move: SIMD3(40, 0, 10))),
                            Solid(name: "Half", color: Palette.colors[6], node: .split(of: .primitive(.make(.sphere)), plane: tilted, side: 0), place: Placement(move: SIMD3(-80, 0, 10)))] + treated
            : [roundedCube,
               Solid(name: "M8 bolt", color: Palette.colors[1], node: .fastener(Fastener(kind: .hex, size: 4)), place: Placement(move: SIMD3(40, 0, 15))),
               Solid(name: "M5 Torx", color: Palette.colors[6], node: .fastener(Fastener(kind: .torxCone, size: 2)), place: Placement(move: SIMD3(-60, 40, 10)))] + treated
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
        if own { skipped("STEP") } else {
            let step = dir.appendingPathComponent("test.step")
            let wrote = k.exportStep(doc.bodies, to: step.path)
            check("step", wrote && ((try? String(contentsOf: step, encoding: .utf8))?.hasPrefix("ISO-10303-21") ?? false))
        }

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
        let rounded = Solid(name: "Rounded", color: Palette.colors[0], node: own ? box : .round(of: box, picks: [edge], radius: 2), place: Placement(move: SIMD3(0, 0, 30)))
        use([rounded])
        lib.reshape(rounded.id) { n in
            guard case .primitive(var p) = n else { return n }
            p.size = [40, 20, 40]
            return .primitive(p)
        }
        settle()
        let (wideLo, wideHi) = bounds(rounded.id)
        check("a new size keeps the left, front and bottom sides", near(wideLo, SIMD3(-10, -10, 20)) && near(wideHi, SIMD3(30, 10, 60)))
        let wide = lib.meshes[rounded.id]?.volume ?? 0, wideWant = 32000 - (own ? 0 : (4 - Double.pi) * 40)
        check(own ? "a new size is built" : "a rounding goes along with a new size", abs(wide - wideWant) < 1, String(format: "%.2f / %.2f mm³", wide, wideWant))
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

        // Resizing keeps exactly the side it should, however the sizes come out: a cylinder pulled wider becomes an oval with
        // its left side where it was, a sphere pulled taller stays round on its bottom, ⌥ keeps the middle, ⇧ the corner, and
        // sizes rounded to 0.01 mm move nothing. On Bcad's own engine what a resize shows as it goes is what it makes.
        func prim(_ id: UUID) -> Primitive? {
            if case .primitive(let p)? = lib.body(id)?.node { return p }
            return nil
        }
        func spans(_ b: (SIMD3<Double>, SIMD3<Double>)) -> String {
            String(format: "%.6f %.6f %.6f … %.6f %.6f %.6f", b.0.x, b.0.y, b.0.z, b.1.x, b.1.y, b.1.z)
        }
        func afterResize(_ s: Solid, axis i: Int, by f: Double, uniform: Bool = false, symmetric: Bool = false, low: Bool = false)
            -> (shown: (SIMD3<Double>, SIMD3<Double>), made: (SIMD3<Double>, SIMD3<Double>)) {
            use([s])
            lib.selection = [s.id]
            lib.beginResize()
            lib.stretch([s.id: s.place], axis: i, by: f, uniform: uniform, symmetric: symmetric, low: low)
            let shown = bounds(s.id)
            lib.finishScale()
            settle()
            return (shown, bounds(s.id))
        }
        let wideCan = Solid(name: "Can", color: Palette.colors[2], node: .primitive(.make(.cylinder)), place: Placement(move: SIMD3(0, 0, 10)))
        let canResize = afterResize(wideCan, axis: 0, by: 1.5)
        let madeOval = prim(wideCan.id).map { $0.kind == .oval && $0.size == [30, 20, 90, 20] } == true && lib.body(wideCan.id)?.place.scale == SIMD3(1, 1, 1)
        check("a cylinder pulled wider becomes an oval with its left side where it was", madeOval
              && near(canResize.made.0, SIMD3(-10, -10, 0), tight) && near(canResize.made.1, SIMD3(20, 10, 20), tight), spans(canResize.made))
        let tallBall = Solid(name: "Ball", color: Palette.colors[3], node: .primitive(.make(.sphere)), place: Placement(move: SIMD3(0, 0, 10)))
        let ballResize = afterResize(tallBall, axis: 2, by: 1.5)
        check("a sphere pulled taller stays round and keeps its bottom", prim(tallBall.id)?.size == [30]
              && near(ballResize.made.0, SIMD3(-15, -15, 0), tight) && near(ballResize.made.1, SIMD3(15, 15, 30), tight), spans(ballResize.made))
        if own {
            check("what a resize shows as it goes is what it makes", near(canResize.shown.0, canResize.made.0, 1e-9) && near(canResize.shown.1, canResize.made.1, 1e-9)
                  && near(ballResize.shown.0, ballResize.made.0, 1e-9) && near(ballResize.shown.1, ballResize.made.1, 1e-9), spans(ballResize.shown))
        }
        let offBlock = Solid(name: "Block", color: Palette.colors[4], node: box, place: Placement(move: SIMD3(5, 5, 10)))
        let middleResize = afterResize(offBlock, axis: 0, by: 1.5, symmetric: true)
        check("a symmetric resize keeps the middle", near(middleResize.made.0, SIMD3(-10, -5, 0), tight) && near(middleResize.made.1, SIMD3(20, 15, 20), tight),
              spans(middleResize.made))
        let cornerResize = afterResize(offBlock, axis: 0, by: 1.5, uniform: true)
        check("a uniform resize keeps the corner", near(cornerResize.made.0, SIMD3(-5, -5, 0), tight) && near(cornerResize.made.1, SIMD3(25, 25, 30), tight),
              spans(cornerResize.made))
        let oddResize = afterResize(offBlock, axis: 0, by: 1.23456, low: true)
        check("sizes rounded to 0.01 mm leave the kept side where it was", prim(offBlock.id)?.size == [24.69, 20, 20]
              && abs(oddResize.made.1.x - 15) < tight && abs(oddResize.made.0.x - (15 - 24.69)) < tight, spans(oddResize.made))

        // Settling: a turned shape goes down onto the bed exactly (its box from the shape, not its mesh); a new shape is on the
        // bed at once, and so is a copy made before it was built.
        func boxed() {
            k.queue.sync {}
            RunLoop.main.run(until: Date().addingTimeInterval(0.05))
        }
        let leaning = Solid(name: "Leaning", color: Palette.colors[5], node: .primitive(.make(.cylinder)), place: Placement(move: SIMD3(0, 0, 40), turn: SIMD3(30, 0, 0)))
        use([leaning])
        lib.settings.dropToBed = true
        lib.selection = [leaning.id]
        lib.dropToBed()
        _ = bounds(leaning.id)
        boxed()
        // Its lowest point: half its height and its radius, turned 30° over.
        let leanLow = bounds(leaning.id).0.z, leanDepth = 10 * cos(Double.pi / 6) + 10 * sin(Double.pi / 6)
        check("a turned cylinder goes down onto the bed exactly", abs(leanLow) < tight && abs((lib.body(leaning.id)?.place.move.z ?? 0) - leanDepth) < tight,
              String(format: "bottom at %.9f", leanLow))
        use([])
        lib.add(.primitive(.make(.box)), name: "Cube")
        let fresh = lib.selection.first
        let onBedAtOnce = fresh.flatMap { lib.body($0)?.place.move.z } == 10
        lib.duplicate()
        let twinOfFresh = lib.selection.first
        settle()
        let freshLow = fresh.map { bounds($0).0 } ?? .zero, twinLow = twinOfFresh.map { bounds($0).0 } ?? .zero
        check("a new shape is on the bed at once, and so is a copy made straight away", onBedAtOnce && twinOfFresh != fresh && lib.doc.bodies.count == 2
              && abs(freshLow.z) < tight && abs(twinLow.z) < tight && near(twinLow, freshLow + SIMD3(10, 0, 0), tight),
              String(format: "bottoms at %.6f and %.6f", freshLow.z, twinLow.z))

        // A merged shape stands where its middle is (its Position), and a turn typed in turns it there.
        let pair2 = Solid(name: "Merged", color: Palette.colors[6], node: .group(op: Int32(BK_UNION), parts: [
            Part(node: box, place: Placement(move: SIMD3(60, 0, 10))), Part(node: .primitive(.make(.cylinder)), place: Placement(move: SIMD3(75, 0, 10)))
        ]), place: Placement())
        use([pair2])
        let pairMiddle = lib.body(pair2.id).map(lib.middle) ?? .zero
        lib.turn(pair2.id, to: SIMD3(0, 0, 90))
        let pairTurned = lib.body(pair2.id).map(lib.middle) ?? .zero
        _ = bounds(pair2.id)
        boxed()
        let pairBox = bounds(pair2.id)
        check("a merged shape's position is its middle, and it turns there", near(pairMiddle, SIMD3(67.5, 0, 10), tight) && near(pairTurned, pairMiddle, 1e-9)
              && near(pairBox.0, SIMD3(57.5, -17.5, 0), tight) && near(pairBox.1, SIMD3(77.5, 17.5, 20), tight), spans(pairBox))

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
        // A turned shape's box is the shape's own (a cone's, worked out here), not just round its mesh: it holds every point of
        // it, and follows a new turn.
        let turnedCone = Solid(name: "Turned cone", color: Palette.colors[3], node: .primitive(.make(.cone)), place: Placement(move: SIMD3(0, 0, 30), turn: SIMD3(33, 21, 0)))
        use([turnedCone])
        func everyPoint(of b: Solid) -> (SIMD3<Double>, SIMD3<Double>) {
            var lo = SIMD3<Double>(repeating: .infinity), hi = SIMD3<Double>(repeating: -.infinity)
            for v in lib.meshes[b.id]?.vertices ?? [] {
                let w = b.place.matrix * SIMD4(Double(v.x), Double(v.y), Double(v.z), 1)
                lo = simd_min(lo, SIMD3(w.x, w.y, w.z))
                hi = simd_max(hi, SIMD3(w.x, w.y, w.z))
            }
            return (lo, hi)
        }
        func coneBox(_ b: Solid) -> (SIMD3<Double>, SIMD3<Double>) {
            guard let m = lib.meshes[b.id] else { return (.zero, .zero) }
            let r = (m.high.x - m.low.x) / 2, c = (m.low + m.high) / 2, foot = SIMD3(c.x, c.y, m.low.z), tip = SIMD3(c.x, c.y, m.high.z)
            // How far the cone reaches along u (in its own coordinates): to its tip, or to its foot's rim.
            func reach(_ u: SIMD3<Double>) -> Double { max(simd_dot(tip, u), simd_dot(foot, u) + r * (u.x * u.x + u.y * u.y).squareRoot()) }
            let rot = b.place.rotation
            var lo = b.place.move, hi = b.place.move
            for i in 0..<3 {
                let u = SIMD3(rot[0][i], rot[1][i], rot[2][i])
                hi[i] += reach(u)
                lo[i] -= reach(-u)
            }
            return (lo, hi)
        }
        func holds(_ box: (SIMD3<Double>, SIMD3<Double>), _ b: Solid) -> Bool {
            let points = everyPoint(of: b), want = coneBox(b)
            return simd_reduce_max(box.0 - points.0) < 1e-5 && simd_reduce_max(points.1 - box.1) < 1e-5 && near(box.0, want.0, tight) && near(box.1, want.1, tight)
        }
        _ = bounds(turnedCone.id)
        boxed()
        let first = bounds(turnedCone.id), firstHolds = holds(first, turnedCone)
        lib.setPlace(turnedCone.id) { $0.turn.z = 50 }
        _ = bounds(turnedCone.id)
        boxed()
        let second = bounds(turnedCone.id), secondHolds = lib.body(turnedCone.id).map { holds(second, $0) } == true
        check("a turned shape's box is the shape's own, holds every point of it and follows a new turn", firstHolds && secondHolds && !near(first.0, second.0),
              spans(first) + " · " + spans(second))
        if own { skipped("⌘B: bolts") } else {
            lib.selection = []
            lib.addThread()
            settle()
            check("⌘B adds a bolt and opens Thread", lib.primary.map { if case .fastener = $0.node { true } else { false } } == true && lib.screen == .thread)
        }

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

        // Angles is the one way to round, bevel or round inward, its key R (once the rounding tool's): on a shape with rounded
        // edges, a 2 mm bevel beside the rounding goes in.
        let upstanding = Solid(name: "Upright", color: Palette.colors[3], node: upright, place: Placement(move: SIMD3(0, 0, 10)))
        use([upstanding])
        lib.selection = [upstanding.id]
        lib.choose(.move)
        lib.perform(.angles)
        check("R opens Angles", lib.settings.key(.angles) == "KeyR" && lib.screen == .angles && lib.mode == .angles && lib.editBody == upstanding.id)
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
        let oldNode = roundAll
        let earlier = Document(bodies: [Solid(name: "Old", color: Palette.colors[2], node: oldNode, place: Placement(move: SIMD3(0, 0, 10)))])
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.sortedKeys]
        let oldJSON = String(decoding: (try? encoder.encode(earlier)) ?? Data(), as: UTF8.self).replacingOccurrences(of: "\"color\":[255,184,51]", with: "\"color\":2")
        let oldURL = dir.appendingPathComponent("old.3mf")
        try? Zip.write([(ThreeMF.docPath, Data(oldJSON.utf8))]).write(to: oldURL)
        lib.open(oldURL)
        settle()
        check("a file from an earlier version opens", oldJSON.contains("\"color\":2") && lib.doc.bodies.first?.color == Palette.colors[2]
              && lib.doc.bodies.first?.node == oldNode && lib.doc.bodies.first.map { !(lib.meshes[$0.id]?.vertices.isEmpty ?? true) } == true)

        // A rounded and hollowed cube, a ring and a torus saved together reopen with every shape shown and nothing said.
        let cupNode: Node = .hollow(of: roundAll, open: [top], walls: [], thickness: 2)
        let kept = Document(bodies: [
            Solid(name: "Cup", color: Palette.colors[0], node: cupNode, place: Placement(move: SIMD3(0, 0, 10))),
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
        // A handle a panel over the view covers turns round to the free side, and pulling it there grows that side.
        use([cube])
        lib.selection = [cube.id]
        lib.choose(.move)
        lib.choose(.resize)
        let r = view.renderer!
        r.gizmoSides = SIMD3(1, 1, 1)
        if let tip = view.project(r.gizmoCenter + SIMD3(1, 0, 0) * r.gizmoLength * 0.9) {
            lib.inspectorFrame = CGRect(x: tip.x - 25, y: view.bounds.height - tip.y - 25, width: 50, height: 50)
        }
        r.turnGizmo()
        let sides = r.gizmoSides
        let (behindLo, behindHi) = bounds(cube.id)
        pull(0, 10)
        let (turnLo, turnHi) = bounds(cube.id)
        let turned: Bool = sides.x < 0 && sides.y > 0 && sides.z > 0
        let grown: Bool = abs(turnHi.x - behindHi.x) < 0.01 && abs(turnLo.x - (behindLo.x - 10)) < 0.6
        check("a handle under a panel turns to the free side, and pulling it grows that side", turned && grown,
              String(format: "sides %.0f %.0f %.0f · x %.2f … %.2f", sides.x, sides.y, sides.z, turnLo.x, turnHi.x))
        lib.inspectorFrame = .zero
        r.gizmoSides = SIMD3(1, 1, 1)
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
        // Turning by a ring: about the shape's middle, which stays where it is, so an uneven shape (a wedge, its box's middle
        // moving as it turns) turned one way and back again is where it was.
        let wedge = Solid(name: "Wedge", color: Palette.colors[5], node: .primitive(.make(.wedge)), place: Placement(move: SIMD3(4, -3, 10)))
        use([wedge])
        lib.selection = [wedge.id]
        lib.choose(.move)
        lib.choose(.rotate)
        // Along the flat ring (about z), from one of its points to another.
        func turnByRing(from a: Int, to b: Int) {
            let r = view.renderer!, points = r.ring(2, r.gizmoCenter, r.gizmoLength * 0.9)
            guard let p = view.project(points[a]), let q = view.project(points[b]) else { return }
            drag(p, q)
        }
        func square(_ m: simd_double3x3) -> Bool { simd_reduce_max(simd_abs(m.columns.0 - SIMD3(1, 0, 0)) + simd_abs(m.columns.1 - SIMD3(0, 1, 0)) + simd_abs(m.columns.2 - SIMD3(0, 0, 1))) < 1e-9 }
        let wedgeMiddle = lib.body(wedge.id).map(lib.middle) ?? .zero
        turnByRing(from: 4, to: 12)
        let wedgeOnce = lib.body(wedge.id)
        turnByRing(from: 12, to: 4)
        let wedgeBack = lib.body(wedge.id)
        let turnedOnce = wedgeOnce.map { !square($0.place.rotation) } == true, middleKept = wedgeOnce.map { near(lib.middle($0), wedgeMiddle, 1e-9) } == true
        let backAgain = wedgeBack.map { square($0.place.rotation) && near($0.place.move, wedge.place.move, 1e-9) } == true
        check("a shape turned by its ring turns about its middle, and back again it's where it was", turnedOnce && middleKept && backAgain,
              String(format: "turned %.1f° · back at %.6f %.6f %.6f", wedgeOnce?.place.turn.z ?? 0, wedgeBack?.place.move.x ?? 0, wedgeBack?.place.move.y ?? 0,
                     wedgeBack?.place.move.z ?? 0))
        lib.choose(.move)
        // Linking: a cube dragged near a turned ball lines its side up with the ball's side exactly (the ball's box from the
        // ball itself, not from its mesh), and off the 10 mm grid.
        let turnedBall = Solid(name: "Ball", color: Palette.colors[2], node: .primitive(.make(.sphere)), place: Placement(move: SIMD3(3.37, 0, 10), turn: SIMD3(33, 21, 0)))
        let linker = Solid(name: "Linker", color: Palette.colors[1], node: box, place: Placement(move: SIMD3(-40, 0, 10)))
        use([turnedBall, linker])
        _ = bounds(turnedBall.id)
        boxed()
        let ballSide = bounds(turnedBall.id).0.x
        if let from = view.project(SIMD3(-40, 0, 20)), let to = view.project(SIMD3(-40 + (ballSide - 0.4) - (-30), 0, 20)) { drag(from, to) }
        let linkedSide = bounds(linker.id).1.x
        check("a dragged shape lines up with a turned shape's side exactly", abs(ballSide - (3.37 - 10)) < tight && abs(linkedSide - ballSide) < 1e-9
              && abs(bounds(linker.id).0.z) < 1e-9, String(format: "side at %.9f, ball's at %.9f", linkedSide, ballSide))
        // A click's jitter on a shape or on a handle leaves it exactly where it is (it would snap to a grid line or a step).
        let jittery = Solid(name: "Jittery", color: Palette.colors[3], node: .primitive(Primitive(kind: .box, size: [20.4, 20, 20])),
                            place: Placement(move: SIMD3(-40.3, 0.2, 10)))
        use([turnedBall, jittery])
        if let p = view.project(SIMD3(-40.3, 0.2, 20)) { drag(p, CGPoint(x: p.x + 2, y: p.y + 1)) }
        let clickedOn = lib.selection == [jittery.id]
        lib.choose(.resize)
        let jitterGizmo = view.renderer!
        jitterGizmo.turnGizmo()
        if let tip = view.project(jitterGizmo.gizmoCenter + jitterGizmo.gizmoHandles()[0] * jitterGizmo.gizmoLength * 0.95) { drag(tip, CGPoint(x: tip.x + 2, y: tip.y)) }
        lib.choose(.move)
        check("a click's jitter on a shape or a handle leaves it where it is", clickedOn && lib.body(jittery.id) == jittery,
              "\(String(describing: lib.body(jittery.id)?.place.move)) · \(String(describing: lib.body(jittery.id)?.node))")
        // Between files: copied shapes paste into another document where they were (beside the copies when pasted back into
        // their own), and a Bcad file dropped on the window adds its shapes to the one open.
        use([cube])
        lib.selection = [cube.id]
        let copied = lib.copySelection()
        lib.paste()
        settle()
        let twin = lib.doc.bodies.last
        let (cubeLo, cubeHi) = bounds(cube.id), (twinLo, _) = bounds(twin?.id ?? cube.id)
        let twinSame: Bool = twin?.node == cube.node && twin?.id != cube.id && lib.selection == [twin?.id ?? cube.id]
        let twinBeside: Bool = twinLo.x >= cubeHi.x && abs(twinLo.z - cubeLo.z) < 0.01
        check("a shape pasted into its own file lands beside it", copied && lib.doc.bodies.count == 2 && twinSame && twinBeside,
              String(format: "%d shapes, twin from x %.2f", lib.doc.bodies.count, twinLo.x))
        let elsewhere = Solid(name: "Elsewhere", color: Palette.colors[1], node: box, place: Placement(move: SIMD3(-60, 0, 10)))
        use([elsewhere])
        lib.paste()
        settle()
        let landed = lib.doc.bodies.last
        let landedSame: Bool = landed?.node == cube.node && near(landed?.place.move ?? .zero, cube.place.move)
        check("copied shapes paste into another file where they were", lib.doc.bodies.count == 2 && landedSame, "\(String(describing: landed?.place.move))")
        let fileBefore = lib.fileURL
        use([elsewhere])
        let addedFile = lib.addFiles([keptURL])
        settle()
        let addedNodes: [Node] = lib.doc.bodies.dropFirst().map(\.node)
        let keptNodes: [Node] = kept.bodies.map(\.node)
        let allShown: Bool = lib.doc.bodies.allSatisfy { !(lib.meshes[$0.id]?.vertices.isEmpty ?? true) }
        let stays: Bool = lib.doc.bodies.first == elsewhere && lib.fileURL == fileBefore
        check("a dropped Bcad file adds its shapes to the open one", addedFile && lib.doc.bodies.count == 1 + kept.bodies.count
              && stays && addedNodes == keptNodes && allShown, "\(lib.doc.bodies.count) shapes")
        let notAdded = lib.addFiles([dir.appendingPathComponent("notes.txt")])
        let saysSo: Bool = lib.note == L("Only 3MF files made by Bcad can be added")
        check("a file that isn't a Bcad 3MF adds nothing and says so", !notAdded && lib.doc.bodies.count == 1 + kept.bodies.count && saysSo, lib.note ?? "")
        // Sizes in mm and in percent on shapes that were edited after they were made: a rounded, hollowed box and a merge.
        // (On Bcad's own engine, a box split across.)
        let editedNode: Node = own ? .split(of: box, plane: Plane(point: SIMD3(0, 0, 3), normal: SIMD3(0, 0, 1)), side: 1)
            : .round(of: .hollow(of: box, open: [top], walls: [], thickness: 2), picks: [edge], radius: 1)
        let edited = Solid(name: "Edited", color: Palette.colors[3], node: editedNode, place: Placement(move: SIMD3(0, 0, 10)))
        let mergedShape = Solid(name: "Merged", color: Palette.colors[4], node: .group(op: Int32(BK_UNION), parts: [
            Part(node: box, place: Placement()), Part(node: .primitive(.make(.cylinder)), place: Placement(move: SIMD3(15, 0, 0)))
        ]), place: Placement(move: SIMD3(60, 0, 10)))
        use([edited, mergedShape])
        lib.settings = Settings()
        var resized: [String] = []
        for s in [edited, mergedShape] {
            // As the Size row does: the size shown is the mesh's along the shape's axes, times its scale.
            guard let b = lib.body(s.id), let m = lib.meshes[s.id] else { resized.append("\(s.name): no mesh"); continue }
            lib.rescale(s.id, axis: 0, by: 40 / (m.size.x * b.place.scale.x))
            settle()
            let (lo1, hi1) = bounds(s.id)
            let before: Double = hi1.y - lo1.y
            // As the Scale row does: 50 % of the shape as it is now (its y scale is still 100 %).
            if let b2 = lib.body(s.id) { lib.rescale(s.id, axis: 1, by: 50 / 100 / b2.place.scale.y) }
            settle()
            let (lo2, hi2) = bounds(s.id)
            let width1: Double = hi1.x - lo1.x, width2: Double = hi2.x - lo2.x, depth2: Double = hi2.y - lo2.y
            let wrong: Bool = abs(width1 - 40) > 0.05 || abs(depth2 - before / 2) > 0.05 || abs(width2 - 40) > 0.05
            if wrong {
                resized.append(String(format: "%@: %.2f × %.2f (from %.2f)", s.name, width2, depth2, before))
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
        // Merging: any number of shapes, apart or touching; a merge merged with another shape takes it in as a part of its
        // own; switched off (on its layer), its parts are shapes again to edit, and switched on they merge again with the
        // layers the merge had (its rounding), as edited — after saving and opening too.
        lib.cancelMode()
        let parts3 = (0..<3).map { i in Solid(name: "Part \(i)", color: Palette.colors[i], node: box, place: Placement(move: SIMD3(Double(i) * 40, 0, 10))) }
        use(parts3)
        lib.note = nil
        lib.selection = [parts3[0].id, parts3[1].id]
        lib.combine(Int32(BK_UNION))
        settle()
        let firstTwo = lib.selection.first.flatMap { lib.body($0) }
        lib.selection = [firstTwo?.id, parts3[2].id].compactMap { $0 }
        lib.combine(Int32(BK_UNION))
        settle()
        let all3 = lib.selection.first.flatMap { lib.body($0) }
        let threeParts: Bool = { if case .group(_, let p) = all3?.node { return p.count == 3 } else { return false } }()
        check("shapes apart merge, and a merge merged again takes the new shape in", threeParts && lib.doc.bodies.count == 1 && lib.note == nil, lib.note ?? "")
        if let m = all3 {
            lib.doc.bodies = [Solid(id: m.id, name: m.name, color: m.color, node: .round(of: m.node, picks: [Pick(kind: Int32(BK_PICK_BODY), a: .zero, b: .zero)], radius: 1), place: m.place)]
            lib.rebuildScene()
            settle()
            lib.unmerge(m.id)
            settle()
        }
        let switchedOff = lib.doc.bodies
        let linked = switchedOff.count == 3 && Set(switchedOff.compactMap { $0.link?.id }).count == 1 && switchedOff.allSatisfy { $0.node == box }
        check("a merge switched off: its parts shapes again, the merge kept", linked, "\(switchedOff.count) shapes")
        let unmergedFile = dir.appendingPathComponent("unmerged.3mf")
        try? ThreeMF.write(unmergedFile, meshes: [], doc: lib.doc)
        check("a merge switched off is kept in its file", (try? ThreeMF.read(unmergedFile))?.doc == lib.doc)
        if let first = switchedOff.first {
            lib.setPlace(first.id) { $0.move.z += 5 }
            lib.remerge(first.id)
            settle()
        }
        let again = lib.doc.bodies.first
        let remerged: Bool = {
            guard lib.doc.bodies.count == 1, case .round(.group(_, let p), _, let r) = again?.node else { return false }
            return p.count == 3 && r == 1 && abs(p[0].place.move.z - 15) < 1e-9
        }()
        check("switched on again: merged as edited, its rounding back", remerged && lib.note == nil, lib.note ?? "")

        // Working on a rounding again: a click on its face finds it and the edge it was made on (sharp); applied, it's made
        // anew from that edge (2 mm becomes 1 mm, or a bevel in its place), and "all edges" replaces it too.
        let roundEdge = Pick(kind: Int32(BK_PICK_EDGE), a: SIMD3(0, -10, 10), b: SIMD3(1, 0, 0))
        let once = Solid(name: "Once", color: Palette.colors[0], node: .round(of: box, picks: [roundEdge], radius: 2), place: Placement(move: SIMD3(0, 0, 10)))
        use([once])
        let onRounding = SIMD3<Double>(0, -8 - 2 * 0.5.squareRoot(), 8 + 2 * 0.5.squareRoot())
        let spot = k.queue.sync { k.treatedAt(once.node, onRounding) }
        check("a click on a rounding's face finds the rounding and its sharp edge", spot?.level == 0 && spot?.path.isEmpty == true && spot?.rest.isEmpty == true
              && simd_length((spot?.edges.first?.a ?? .zero) - SIMD3(0, -10, 10)) < 0.5, "\(String(describing: spot?.edges))")
        func applyEdit(_ change: (inout AngleEdit) -> Void) {
            guard let spot else { return }
            var e = AngleEdit(body: once.id, picks: [roundEdge], section: Section(loops: [], angle: 90, point: .zero, direction: SIMD3(1, 0, 0)))
            e.edits = [spot]
            e.fresh = []
            e.adopt(spot.layer)
            change(&e)
            lib.angleEdit = e
            lib.note = nil
            lib.applyAngles()
            settle()
        }
        let sharpEdge = spot?.edges.first ?? roundEdge
        applyEdit { $0.radius = 1 }
        check("worked on again: the rounding made anew from its sharp edge", lib.body(once.id)?.node == .round(of: box, picks: [sharpEdge], radius: 1)
              && abs((lib.meshes[once.id]?.volume ?? 0) - (8000 - (1 - Double.pi / 4) * 20)) < 0.5, String(format: "%.2f mm³", lib.meshes[once.id]?.volume ?? 0))
        use([once])
        applyEdit { $0.treatment = .angled; $0.legs = SIMD2(1, 1) }
        check("a rounding worked on as a bevel is replaced by it", lib.body(once.id)?.node == .bevel(of: box, picks: [sharpEdge], legs: SIMD2(1, 1), corner: 0)
              && abs((lib.meshes[once.id]?.volume ?? 0) - 7990) < 0.5, String(format: "%.2f mm³", lib.meshes[once.id]?.volume ?? 0))
        use([once])
        let everyPick = Pick(kind: Int32(BK_PICK_BODY), a: .zero, b: .zero)
        var allEdges = AngleEdit(body: once.id, picks: [everyPick], section: Section(loops: [], angle: 90, point: .zero, direction: SIMD3(1, 0, 0)))
        allEdges.radius = 1
        lib.angleEdit = allEdges
        lib.applyAngles(whole: true)
        settle()
        check("all edges: an earlier rounding is replaced, not rounded over", lib.body(once.id)?.node == .round(of: box, picks: [everyPick], radius: 1))

        // A click on the line where a rounding meets a face (smoothly), or on a bevel's border, finds that treatment and the
        // edge it was made on; an edge no treatment made is worked on as it is.
        let seam = k.queue.sync { k.treatedAt(once.node, SIMD3(0, -8, 10), edge: true) }
        check("a click on a rounding's seam finds the rounding and its sharp edge", seam?.level == 0 && seam?.path.isEmpty == true
              && simd_length((seam?.edges.first?.a ?? .zero) - SIMD3(0, -10, 10)) < 0.5, "\(String(describing: seam?.edges))")
        let bevelled = Node.bevel(of: box, picks: [roundEdge], legs: SIMD2(2, 2), corner: 0)
        let border = k.queue.sync { k.treatedAt(bevelled, SIMD3(0, -10, 8), edge: true) }
        check("a click on a bevel's border finds the bevel and its sharp edge", border?.layer == bevelled
              && simd_length((border?.edges.first?.a ?? .zero) - SIMD3(0, -10, 10)) < 0.5, "\(String(describing: border?.edges))")
        let untouched = k.queue.sync { k.treatedAt(once.node, SIMD3(0, 10, 10), edge: true) }
        check("an edge no treatment made is worked on as it is", untouched == nil)
        let seamPick = Pick(kind: Int32(BK_PICK_EDGE), a: SIMD3(0, -8, 10), b: SIMD3(1, 0, 0)), backPick = Pick(kind: Int32(BK_PICK_EDGE), a: SIMD3(0, 10, 10), b: SIMD3(1, 0, 0))
        let smoothSeam = k.queue.sync { k.smooth(once.node, seamPick) }, sharpBack = k.queue.sync { k.smooth(once.node, backPick) }
        check("a smooth seam is told from a corner", smoothSeam && !sharpBack, "seam \(smoothSeam), corner \(sharpBack)")

        // "All edges" of a merge of two rounded boxes: the parts' own roundings are left out, the merge rounded all round.
        let roundedBox = Node.round(of: box, picks: [everyPick], radius: 2)
        let pair = Solid(name: "Pair", color: Palette.colors[1], node: .group(op: Int32(BK_UNION), parts: [Part(node: roundedBox, place: Placement()),
                         Part(node: roundedBox, place: Placement(move: SIMD3(15, 0, 0)))]), place: Placement(move: SIMD3(0, 0, 10)))
        use([pair])
        var pairEdges = AngleEdit(body: pair.id, picks: [everyPick], section: Section(loops: [], angle: 90, point: .zero, direction: SIMD3(1, 0, 0)))
        pairEdges.radius = 1
        lib.angleEdit = pairEdges
        lib.note = nil
        lib.applyAngles(whole: true)
        settle()
        let plainParts: Bool = {
            guard case .round(.group(_, let parts), _, let r) = lib.body(pair.id)?.node else { return false }
            return r == 1 && parts.count == 2 && parts.allSatisfy { $0.node == box }
        }()
        check("all edges of a merge: its parts' roundings are replaced too", plainParts && lib.note == nil, lib.note ?? "")

        // The workbench's build must end before the process does: OpenCascade tears itself down at exit.
        k.queue.sync {}
        print(ok ? "ALL OK" : "FAILURES")
        return ok
    }
}
#endif
