import SwiftUI
import AppKit
import simd

// MARK: - Root

struct RootView: View {
    @Environment(Workbench.self) private var lib

    var body: some View {
        let look = Skin.shared
        ZStack {
            Backdrop()
            // The 3D view keeps full resolution and never mirrors: it sits outside the scaled interface, beside the drawer.
            Viewport()
                .padding(look.rtl ? .trailing : .leading, lib.drawerInset * look.scale)
            ScaledUI {
                ZStack {
                    HStack(spacing: 0) {
                        // A plain slide: blurring the panel on its way in draws its frosted glass out of place.
                        if lib.drawerOpen {
                            Drawer().transition(Neon.calm ? .opacity : .move(edge: .leading))
                        }
                        ZStack {
                            if lib.angleEdit != nil {
                                AngleEditor().transition(.scale(scale: 0.4).combined(with: .opacity))
                            } else {
                                // The button grows and shrinks where it is, not from the middle of the window.
                                ZStack {
                                    if !lib.drawerOpen {
                                        Button { withAnimation(Neon.glide) { lib.drawerOpen = true } } label: {
                                            Image(systemName: "sidebar.left").accessibilityLabel(L("Show the side panel"))
                                        }
                                        .buttonStyle(NeonButtonStyle(tint: lib.accent, size: 32))
                                        .help(L("Show the side panel"))
                                        .transition(.scale.combined(with: .opacity))
                                    }
                                }
                                .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topLeading)
                                .padding(.top, 40)
                                .padding(.leading, 14)
                                VStack(spacing: 10) {
                                    Spacer()
                                    if tool {
                                        ModeBar().transition(.move(edge: .bottom).combined(with: .haze))
                                    }
                                    ShapeBar()
                                }
                                .padding(.bottom, 16)
                                // The tools sit beside the inspector, or at the right edge while it's away.
                                HStack(alignment: .top, spacing: 10) {
                                    ToolRail()
                                    if !lib.selection.isEmpty && !tool {
                                        Inspector().transition(.move(edge: .trailing).combined(with: .haze))
                                    }
                                }
                                .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topTrailing)
                                .padding(.top, 14)
                                .padding(.trailing, 14)
                            }
                        }
                    }
                    // Work under way shows with a spinner; a message (a result, a problem) without one.
                    if let busy = lib.busy {
                        BusyToast(text: busy)
                            .frame(maxHeight: .infinity, alignment: .top)
                            .padding(.top, 14)
                            .transition(.move(edge: .top).combined(with: .haze))
                    } else if let note = lib.note {
                        NoteToast(text: note)
                            .frame(maxHeight: .infinity, alignment: .top)
                            .padding(.top, 14)
                            .transition(.move(edge: .top).combined(with: .haze))
                    }
                }
            }
        }
        .background(WindowConfigurator())
        .ignoresSafeArea()
        .animation(Neon.glide, value: lib.busy)
        .animation(Neon.glide, value: lib.note)
        .animation(Neon.glide, value: lib.mode)
        .animation(Neon.glide, value: lib.selection.isEmpty)
        .animation(Neon.glide, value: lib.drawerOpen)
    }

    // A tool (split, round, hollow) is out: its bar shows instead of the inspector.
    private var tool: Bool { [.round, .split, .hollow].contains(lib.mode) }
}

// A short message at the top of the window, in the look of the busy note but without its spinner.
struct NoteToast: View {
    let text: String

    var body: some View {
        let look = Skin.shared
        Text(text)
            .font(.ui(size: 12.5, weight: .semibold, design: .rounded))
            .foregroundStyle(Ink.text.opacity(0.85))
            .padding(.horizontal, 16)
            .frame(height: 36)
            .background(RoundedRectangle(cornerRadius: 11, style: .continuous).fill(Ink.void.opacity(0.6)))
            .background(RoundedRectangle(cornerRadius: 11, style: .continuous).fill(.ultraThinMaterial))
            .halo(look.accent, 14)
            .allowsHitTesting(false)
    }
}

extension View {
    func glassBar(_ r: CGFloat = 20) -> some View {
        let shape = RoundedRectangle(cornerRadius: r, style: .continuous)
        return self
            .background { shape.fill(Ink.void.opacity(0.45)).contentShape(shape).onTapGesture {} }
            .background(shape.fill(.ultraThinMaterial))
            .overlay(shape.strokeBorder(Ink.text.opacity(0.07), lineWidth: 1).allowsHitTesting(false))
    }
}

// MARK: - Fields

// A millimetre (or degree, percent) value with two decimals, or as many as `digits`. ↑/↓ step by 1, ⇧ by 10, ⌥ by 0.1.
struct MMField: View {
    @Environment(Workbench.self) private var lib
    let value: Double
    var unit: String?
    var range: ClosedRange<Double> = -100000...100000
    var width: CGFloat = 64
    var tint: Color?
    // How strongly the field is tinted at rest.
    var fill = 0.07
    var digits = 2
    let set: (Double) -> Void
    @State private var text = ""
    @State private var blink = false
    @State private var hover = false
    @FocusState private var focused: Bool

    static func format(_ v: Double, digits: Int = 2) -> String {
        let k = pow(10, Double(digits)), r = (v * k).rounded() / k
        return String(format: "%.\(digits)f", r == 0 ? 0 : r)
    }

    var body: some View {
        let t = tint ?? lib.accent
        HStack(spacing: 4) {
            TextField("", text: $text)
                .textFieldStyle(.plain)
                .font(.ui(size: 12.5, weight: .bold, design: .rounded))
                .monospacedDigit()
                .multilineTextAlignment(.center)
                .focused($focused)
                .onSubmit {
                    commit()
                    focused = false
                }
                .onKeyPress(keys: [.upArrow, .downArrow]) { press in
                    let step = press.modifiers.contains(.shift) ? 10 : press.modifiers.contains(.option) ? 0.1 : 1
                    let now = Double(text.replacingOccurrences(of: ",", with: ".")) ?? value
                    text = Self.format(now + (press.key == .upArrow ? step : -step), digits: digits)
                    commit()
                    return .handled
                }
                .frame(width: width, height: 26)
                .background(RoundedRectangle(cornerRadius: 8, style: .continuous).fill(t.opacity(focused ? (blink ? 0.3 : max(0.12, fill)) : (hover ? fill + 0.07 : fill))))
                .glow(t, focused ? (blink ? 12 : 3) : (hover ? 7 : 0))
                .scaleEffect(hover && !focused ? 1.05 : 1)
                .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
            if let unit {
                Text(unit).font(.ui(size: 11, weight: .medium, design: .rounded)).foregroundStyle(Ink.text.opacity(0.45)).fixedSize()
            }
        }
        .onAppear { text = Self.format(value, digits: digits) }
        .onChange(of: value) { if !focused { text = Self.format(value, digits: digits) } }
        .onChange(of: focused) {
            if focused {
                lib.endCapture()
                withAnimation(.easeInOut(duration: 0.5).repeatForever(autoreverses: true)) { blink = true }
            } else {
                withAnimation(.easeOut(duration: 0.2)) { blink = false }
                commit()
            }
        }
        .onDisappear(perform: commit)
    }

    private func commit() {
        let s = text.replacingOccurrences(of: ",", with: ".").trimmingCharacters(in: .whitespaces)
        guard let n = Double(s), n.isFinite else { text = Self.format(value, digits: digits); return }
        let k = pow(10, Double(digits)), v = (min(range.upperBound, max(range.lowerBound, n)) * k).rounded() / k
        if abs(v - value) > 0.000_1 { set(v) }
        text = Self.format(v, digits: digits)
    }
}

struct Chip: View {
    @Environment(Workbench.self) private var lib
    let text: String
    let chosen: Bool
    var tint: Color?
    let action: () -> Void
    @State private var hover = false
    @State private var clicks = 0

    var body: some View {
        let t = tint ?? lib.accent
        let shape = RoundedRectangle(cornerRadius: 8, style: .continuous)
        Text(text)
            .font(.ui(size: 12, weight: .bold, design: .rounded))
            .foregroundStyle(chosen ? Color.black : (hover ? t : Ink.text.opacity(0.8)))
            .lineLimit(1)
            .padding(.horizontal, 8)
            .frame(minWidth: 34)
            .frame(height: 28)
            .background(shape.fill(chosen ? t : t.opacity(hover ? 0.18 : 0.08)))
            .glow(t, chosen ? 10 : (hover ? 7 : 0))
            .pop(clicks, scale: 1.12)
            .contentShape(shape)
            .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
            .onTapGesture {
                clicks += 1
                withAnimation(Neon.spring) { action() }
            }
            .accessibilityElement(children: .combine)
            .accessibilityAddTraits(chosen ? [.isButton, .isSelected] : .isButton)
            .accessibilityAction(.default) { withAnimation(Neon.spring) { action() } }
    }
}

struct ToolButton: View {
    @Environment(Workbench.self) private var lib
    let icon: String
    let title: String
    var key: String?
    var tint: Color?
    var lit = false
    let size: CGFloat
    let action: () -> Void

    var body: some View {
        Button(action: action) { Image(systemName: icon) }
            .buttonStyle(NeonButtonStyle(tint: tint ?? lib.accent, lit: lit, size: size))
            .help(key.map { "\(title) (\($0))" } ?? title)
            .accessibilityLabel(title)
    }
}

// MARK: - Toolbars

struct ShapeBar: View {
    @State private var open: ShapeGroup?

    var body: some View {
        HStack(spacing: 8) {
            ForEach(ShapeGroup.allCases, id: \.self) { g in ShapeGroupButton(group: g, open: $open) }
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 8)
        .glassBar(22)
    }
}

// A group's quick shape with an arrow that opens the group's row: the program's default first, then the rest.
struct ShapeGroupButton: View {
    @Environment(Workbench.self) private var lib
    let group: ShapeGroup
    @Binding var open: ShapeGroup?

    var body: some View {
        let k = lib.quick(group)
        HStack(spacing: 1) {
            // Not 28 or 34 points: at those sizes a hovered button ignores clicks along its middle line.
            Button { lib.addShape(k) } label: { ShapeIcon(prim: k.primitive, size: 15) }
                .buttonStyle(NeonButtonStyle(tint: lib.accent, size: 36))
                .help(L(k.primitive.name))
                .accessibilityLabel(L(k.primitive.name))
            Button { withAnimation(Neon.glide) { open = open == group ? nil : group } } label: {
                Image(systemName: "chevron.up").font(.ui(size: 8, weight: .black))
            }
            .buttonStyle(NeonButtonStyle(tint: lib.accent, lit: open == group, size: 16))
            .rotationEffect(.degrees(open == group ? 180 : 0))
            .help(L("More shapes"))
            .accessibilityLabel(L("More shapes"))
        }
        .overlay(alignment: .bottom) {
            if open == group {
                HStack(spacing: 6) {
                    ShapeChoice(group: group, kind: group.members[0], open: $open)
                    Rectangle().fill(Ink.text.opacity(0.18)).frame(width: 1, height: 22).padding(.horizontal, 3)
                    ForEach(group.members.dropFirst(), id: \.self) { m in ShapeChoice(group: group, kind: m, open: $open) }
                }
                .padding(6)
                .glassBar(16)
                .fixedSize()
                .offset(y: -54)
                .transition(.menu)
            }
        }
    }
}

// A shape's picture: a symbol, or for a torus with a polygon tube (and any oval torus) its tube cut through.
struct ShapeIcon: View {
    let prim: Primitive
    let size: CGFloat

    var body: some View {
        if let name = prim.symbol {
            Image(systemName: name)
        } else {
            TubeGlyph(sides: prim.sides).frame(width: size * 1.25, height: size)
        }
    }
}

extension Primitive {
    var symbol: String? {
        switch kind {
        case .box: "cube"
        case .cylinder: "cylinder"
        case .cone: "cone"
        case .sphere: "circle.fill"
        case .torus: sides == 0 ? "circle.circle" : nil
        case .ovalTorus: nil
        case .wedge: "righttriangle"
        case .prism: ["3": "triangle", "5": "pentagon", "8": "octagon"][String(sides)] ?? "hexagon"
        case .pyramid: "pyramid"
        case .hemisphere: "circle.tophalf.filled"
        case .bowl: "circle.bottomhalf.filled"
        case .ring: "smallcircle.circle"
        case .glass: "wineglass"
        case .oval: "oval"
        }
    }
}

// A torus cut through its middle: the tube's outline either side of the axis (a circle, a triangle point up, a flat hexagon).
struct TubeGlyph: View {
    let sides: Int

    var body: some View {
        Canvas { ctx, size in
            let h = min(size.height, size.width * 0.4), cy = size.height / 2
            var tubes = Path()
            for cx in [h / 2, size.width - h / 2] {
                let r = h / 2, t = r * sqrt(3) / 2
                switch sides {
                case 3:
                    tubes.move(to: CGPoint(x: cx - r, y: cy + t))
                    tubes.addLine(to: CGPoint(x: cx + r, y: cy + t))
                    tubes.addLine(to: CGPoint(x: cx, y: cy - t))
                    tubes.closeSubpath()
                case 6:
                    tubes.move(to: CGPoint(x: cx - r, y: cy))
                    for (dx, dy) in [(-r / 2, t), (r / 2, t), (r, 0), (r / 2, -t), (-r / 2, -t)] { tubes.addLine(to: CGPoint(x: cx + dx, y: cy + dy)) }
                    tubes.closeSubpath()
                default:
                    tubes.addEllipse(in: CGRect(x: cx - r, y: cy - r, width: h, height: h))
                }
            }
            ctx.fill(tubes, with: .foreground)
            var axis = Path()
            axis.move(to: CGPoint(x: size.width / 2, y: cy - h * 0.7))
            axis.addLine(to: CGPoint(x: size.width / 2, y: cy + h * 0.7))
            ctx.stroke(axis, with: .foreground, style: StrokeStyle(lineWidth: 1.2, lineCap: .round, dash: [1.5, 2]))
        }
        .accessibilityHidden(true)
    }
}

// A bolt with its thread: the Thread tab's picture (there's no symbol for a thread).
struct ThreadGlyph: Shape {
    func path(in r: CGRect) -> Path {
        var p = Path()
        let w = r.width, h = r.height
        p.addRoundedRect(in: CGRect(x: r.minX + w * 0.16, y: r.minY + h * 0.04, width: w * 0.68, height: h * 0.22), cornerSize: CGSize(width: w * 0.06, height: w * 0.06))
        let left = r.minX + w * 0.3, right = r.maxX - w * 0.3, top = r.minY + h * 0.26, bottom = r.maxY - h * 0.02
        p.move(to: CGPoint(x: left, y: top))
        p.addLine(to: CGPoint(x: left, y: bottom))
        p.addLine(to: CGPoint(x: right, y: bottom))
        p.addLine(to: CGPoint(x: right, y: top))
        var y = top + h * 0.17
        while y < bottom - h * 0.02 {
            p.move(to: CGPoint(x: left, y: y))
            p.addLine(to: CGPoint(x: right, y: y - h * 0.11))
            y += h * 0.17
        }
        return p
    }
}

// A click adds the shape; holding it for a second makes it the group's quick shape (a ring fills while holding).
struct ShapeChoice: View {
    @Environment(Workbench.self) private var lib
    let group: ShapeGroup
    let kind: ShapeKind
    @Binding var open: ShapeGroup?
    @GestureState private var pressing = false
    @State private var held = false
    @State private var chosen = 0

    var body: some View {
        let quick = lib.quick(group) == kind
        let name = L(kind.primitive.name)
        Button {
            if held { held = false; return }
            withAnimation(Neon.glide) { open = nil }
            lib.addShape(kind)
        } label: {
            ShapeIcon(prim: kind.primitive, size: 14)
        }
        .buttonStyle(NeonButtonStyle(tint: lib.accent, lit: quick, size: 32))
        .overlay {
            Circle()
                .trim(from: 0, to: pressing ? 1 : 0)
                .stroke(lib.accent3, style: StrokeStyle(lineWidth: 2.5, lineCap: .round))
                .rotationEffect(.degrees(-90))
                .frame(width: 40, height: 40)
                .animation(pressing ? .linear(duration: 1) : .easeOut(duration: 0.15), value: pressing)
                .allowsHitTesting(false)
        }
        .pop(chosen, scale: 1.2)
        .simultaneousGesture(LongPressGesture(minimumDuration: 1)
            .updating($pressing) { v, state, _ in state = v }
            .onEnded { _ in
                held = true
                chosen += 1
                lib.setQuick(group, kind)
            })
        .help(quick ? name : name + " · " + L("Hold to keep it on the button"))
        .accessibilityLabel(name)
        .accessibilityAction(named: L("Keep it on the button")) { lib.setQuick(group, kind) }
    }
}

struct ToolRail: View {
    @Environment(Workbench.self) private var lib
    @State private var views = false

    var body: some View {
        let s = lib.settings
        VStack(spacing: 6) {
            ToolButton(icon: "arrow.down.to.line", title: Action.drop.label, key: Keys.label(s.key(.drop)), tint: lib.accent3, size: 30) { lib.perform(.drop) }
            divider
            ToolButton(icon: "scissors", title: Action.split.label, key: Keys.label(s.key(.split)), tint: lib.accent2, lit: lib.mode == .split, size: 30) { lib.perform(.split) }
            ToolButton(icon: "app", title: Action.round.label, key: Keys.label(s.key(.round)), tint: lib.accent2, lit: lib.mode == .round, size: 30) { lib.perform(.round) }
            ToolButton(icon: "square.dashed.inset.filled", title: Action.hollow.label, key: Keys.label(s.key(.hollow)), tint: lib.accent2,
                       lit: lib.mode == .hollow, size: 30) { lib.perform(.hollow) }
            divider
            ToolButton(icon: "arrow.uturn.backward", title: L("Undo"), key: "⌘Z", size: 30) { lib.undo() }
            ToolButton(icon: "arrow.uturn.forward", title: L("Redo"), key: "⇧⌘Z", size: 30) { lib.redo() }
            ToolButton(icon: "viewfinder", title: Action.frame.label, key: Keys.label(s.key(.frame)), size: 30) { lib.perform(.frame) }
            // Straight views along an axis, opening beside the rail.
            Button { withAnimation(Neon.glide) { views.toggle() } } label: {
                Image(systemName: "chevron.left").font(.ui(size: 8, weight: .black))
            }
            .buttonStyle(NeonButtonStyle(tint: lib.accent, lit: views, size: 16))
            .rotationEffect(.degrees(views ? 180 : 0))
            .help(L("Views"))
            .accessibilityLabel(L("Views"))
            .overlay(alignment: .trailing) {
                if views {
                    Grid(alignment: .leading, horizontalSpacing: 6, verticalSpacing: 6) {
                        ForEach([[Side.top, .bottom], [.north, .south], [.west, .east]], id: \.self) { pair in
                            GridRow {
                                ForEach(pair, id: \.self) { side in
                                    Chip(text: side.name, chosen: false) {
                                        lib.look(from: side)
                                        withAnimation(Neon.glide) { views = false }
                                    }
                                }
                            }
                        }
                    }
                    .padding(8)
                    .glassBar(14)
                    .fixedSize()
                    .offset(x: -38)
                    .transition(.menu)
                }
            }
        }
        .padding(.vertical, 10)
        .padding(.horizontal, 7)
        .glassBar(22)
    }

    private var divider: some View { Rectangle().fill(Ink.text.opacity(0.12)).frame(width: 22, height: 1) }
}

struct ModeBar: View {
    @Environment(Workbench.self) private var lib

    private var title: String {
        switch lib.mode {
        case .round: L("Round edges")
        case .hollow: L("Hollow")
        default: L("Split")
        }
    }

    private var hint: String {
        switch lib.mode {
        case .round: lib.edgePicks.isEmpty ? L("Click an edge, a corner or a face · ⇧ adds more") : L("Drag up or down on the pick to round it · Enter applies")
        case .hollow: L("Click faces to open them · ⌥-click a face for its own wall")
        default: L("Drag the arrow to move the plane · drag a ring to tilt it")
        }
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack(spacing: 8) {
                Text(title).foregroundStyle(lib.accent2)
                Text(hint).foregroundStyle(Ink.text.opacity(0.55)).lineLimit(1).contentTransition(.opacity)
            }
            .padding(.leading, 4)
            HStack(spacing: 10) {
                switch lib.mode {
                case .round: round
                case .hollow: hollow
                default: split
                }
                // Not 28 or 34 points: at those sizes a hovered button ignores clicks along its middle line.
                ToolButton(icon: "xmark", title: L("Cancel"), key: "Esc", tint: Neon.red, size: 30) { lib.cancelMode() }
            }
        }
        .font(.ui(size: 12.5, weight: .semibold, design: .rounded))
        .foregroundStyle(Ink.text.opacity(0.85))
        .padding(.horizontal, 14)
        .padding(.vertical, 8)
        .glassBar(18)
        .halo(lib.accent2, 8)
    }

    @ViewBuilder private var round: some View {
        let all = lib.edgePicks.contains { $0.kind == Int32(BK_PICK_BODY) }
        Chip(text: L("All edges") + " · A", chosen: all, tint: lib.accent2) {
            if let b = lib.editBody ?? lib.selection.last {
                lib.editBody = b
                lib.edgePicks = [Pick(kind: Int32(BK_PICK_BODY), a: .zero, b: .zero)]
            }
        }
        MMField(value: lib.roundRadius, unit: L("mm"), range: 0.01...1000, width: 58) { v in
            lib.roundRadius = v
            lib.previewRound()
        }
        Button(L("Apply")) { lib.commitRound() }
            .buttonStyle(PillStyle(tint: lib.accent2))
            .frame(width: 90)
            .disabled(lib.edgePicks.isEmpty)
    }

    @ViewBuilder private var hollow: some View {
        Text(L("Walls")).foregroundStyle(Ink.text.opacity(0.55))
        MMField(value: lib.hollowThickness, unit: L("mm"), range: 0.01...1000, width: 58) { lib.hollowThickness = $0 }
        if !lib.hollowOpen.isEmpty {
            Chip(text: L("{n} openings", ["n": lib.hollowOpen.count]) + "  ×", chosen: true, tint: Neon.red) { lib.hollowOpen = [] }
                .help(L("Close all openings"))
                .transition(.scale.combined(with: .haze))
        }
        if let i = lib.focusWall, lib.hollowWalls.indices.contains(i) {
            Text(L("This face")).foregroundStyle(lib.accent2)
            MMField(value: lib.hollowWalls[i].thickness, unit: L("mm"), range: 0.01...1000, width: 58) { lib.hollowWalls[i].thickness = $0 }
                .id(i)
            ToolButton(icon: "xmark", title: L("Use the default wall here"), tint: lib.accent2, size: 24) { lib.removeWall(i) }
        } else if !lib.hollowWalls.isEmpty {
            Text(L("{n} own walls", ["n": lib.hollowWalls.count])).foregroundStyle(lib.accent2)
        }
        Button(L("Apply")) { lib.commitHollow() }
            .buttonStyle(PillStyle(tint: lib.accent2))
            .frame(width: 90)
            .disabled(lib.editBody == nil)
    }

    @ViewBuilder private var split: some View {
        HStack(spacing: 4) {
            ForEach(0..<3, id: \.self) { i in
                Chip(text: ["X", "Y", "Z"][i], chosen: lib.splitAxis == i, tint: Axis.color(i)) { lib.splitAxis = i }
            }
        }
        Text(L("Offset")).foregroundStyle(Ink.text.opacity(0.55))
        MMField(value: lib.splitOffset, unit: L("mm"), width: 60) { lib.splitOffset = $0 }
        Text(L("Tilt")).foregroundStyle(Ink.text.opacity(0.55))
        MMField(value: lib.splitTilt.x, unit: "°", range: -80...80, width: 52) { lib.splitTilt.x = $0 }
        MMField(value: lib.splitTilt.y, unit: "°", range: -80...80, width: 52) { lib.splitTilt.y = $0 }
        Button(L("Split")) { lib.split() }
            .buttonStyle(PillStyle(tint: lib.accent2))
            .frame(width: 90)
    }
}

// MARK: - Drawer

struct Drawer: View {
    @Environment(Workbench.self) private var lib
    @State private var langOpen = false
    @State private var styleOpen = false

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack(spacing: 6) {
                Button { lib.toggleSettings() } label: {
                    Image(systemName: lib.showSettings ? "chevron.left" : "gearshape.fill").contentTransition(.symbolEffect(.replace))
                }
                .buttonStyle(NeonButtonStyle(tint: lib.accent, lit: lib.showSettings, size: 32))
                .accessibilityLabel(lib.showSettings ? L("Back") : L("Settings"))
                .help(lib.showSettings ? L("Back to shapes") : L("Settings"))
                if lib.showSettings {
                    Text(L("Settings"))
                        .font(.ui(size: 15, weight: .bold, design: .rounded))
                        .foregroundStyle(Ink.text)
                        .lineLimit(1)
                        .transition(.move(edge: .trailing).combined(with: .haze))
                    Spacer(minLength: 0)
                } else {
                    HStack(spacing: 6) {
                        Text(lib.title)
                            .font(.ui(size: 15, weight: .bold, design: .rounded))
                            .foregroundStyle(Ink.text)
                            .lineLimit(1)
                        if lib.dirty {
                            Circle().fill(lib.accent).frame(width: 6, height: 6).glow(lib.accent, 5).transition(.scale)
                        }
                    }
                    .transition(.move(edge: .leading).combined(with: .haze))
                    Spacer(minLength: 4)
                    Button { withAnimation(Neon.glide) { lib.drawerOpen = false } } label: { Image(systemName: "sidebar.left").accessibilityLabel(L("Hide the side panel")) }
                        .buttonStyle(NeonButtonStyle(tint: lib.accent, size: 32))
                        .help(L("Hide the side panel"))
                }
            }
            .zIndex(2)
            if lib.showSettings {
                SettingsPane(langOpen: $langOpen, styleOpen: $styleOpen).transition(.move(edge: .trailing).combined(with: .haze))
            } else {
                ObjectList().transition(.haze)
                if lib.selection.count == 1, let b = lib.primary {
                    LayerStack(item: b)
                        .id(b.id)
                        .transition(.move(edge: .bottom).combined(with: .haze))
                }
            }
        }
        .animation(Neon.glide, value: lib.showSettings)
        .animation(Neon.glide, value: lib.selection)
        .padding(.top, 40)
        .padding(.horizontal, 12)
        .padding(.bottom, 12)
        .frame(width: 320)
        .frame(maxHeight: .infinity, alignment: .top)
        .background(LinearGradient(colors: [Ink.text.opacity(0.05), Color.black.opacity(0.12)], startPoint: .top, endPoint: .bottom))
        .background(.ultraThinMaterial.opacity(0.35))
        .overlay(alignment: .trailing) {
            Rectangle().fill(lib.accent.opacity(0.22)).frame(width: 1).glow(lib.accent, 5).allowsHitTesting(false)
        }
    }
}

struct ObjectList: View {
    @Environment(Workbench.self) private var lib

    var body: some View {
        if lib.doc.bodies.isEmpty {
            VStack(spacing: 10) {
                Image(systemName: "cube.transparent").font(.ui(size: 34, weight: .light)).foregroundStyle(lib.accent).halo(lib.accent, 12)
                Text(L("Add a shape from the bar below"))
                    .font(.ui(size: 13, weight: .medium, design: .rounded))
                    .foregroundStyle(Ink.text.opacity(0.5))
                    .multilineTextAlignment(.center)
            }
            .frame(maxWidth: .infinity, maxHeight: .infinity)
        } else {
            ScrollView {
                LazyVStack(spacing: 3) {
                    ForEach(Array(lib.doc.bodies.enumerated()), id: \.element.id) { i, b in
                        ObjectRow(item: b).cascade(i)
                    }
                }
                .padding(.vertical, 4)
            }
            .scrollIndicators(.never)
            .frame(maxHeight: .infinity)
        }
    }
}

struct ObjectRow: View {
    @Environment(Workbench.self) private var lib
    let item: Solid
    @State private var hover = false
    @State private var editing = false
    @State private var text = ""
    @State private var flash = 0
    @FocusState private var focused: Bool

    @ViewBuilder private var icon: some View {
        switch item.node.base {
        case .primitive(let p): ShapeIcon(prim: p, size: 10)
        case .fastener(let f): Image(systemName: f.nut ? "circle.hexagonpath" : "screwdriver")
        case .group: Image(systemName: "square.on.square")
        default: Image(systemName: "cube")
        }
    }

    var body: some View {
        let chosen = lib.selection.contains(item.id)
        let color = Palette.color(item.color)
        HStack(spacing: 10) {
            Circle().fill(color).frame(width: 10, height: 10).glow(color, chosen ? 6 : 0)
            if editing {
                TextField("", text: $text)
                    .textFieldStyle(.plain)
                    .focused($focused)
                    .onSubmit { focused = false }
                    .onChange(of: focused) { if !focused { finish() } }
            } else {
                Text(item.name).lineLimit(1)
            }
            Spacer(minLength: 4)
            icon.font(.ui(size: 11, weight: .semibold)).foregroundStyle(Ink.text.opacity(0.35)).accessibilityHidden(true)
            Button {
                lib.commit { d in if let i = d.bodies.firstIndex(where: { $0.id == item.id }) { d.bodies[i].hidden.toggle() } }
                lib.selection.removeAll { $0 == item.id }
            } label: {
                Image(systemName: item.hidden ? "eye.slash" : "eye").contentTransition(.symbolEffect(.replace)).accessibilityLabel(item.hidden ? L("Show") : L("Hide"))
            }
            .buttonStyle(.plain)
            .foregroundStyle(item.hidden ? Ink.text.opacity(0.35) : lib.accent.opacity(hover ? 1 : 0.6))
            .help(item.hidden ? L("Show") : L("Hide"))
        }
        .font(.ui(size: 13, weight: .medium, design: .rounded))
        .foregroundStyle(chosen ? Ink.text : Ink.text.opacity(item.hidden ? 0.35 : 0.75))
        .padding(.horizontal, 10)
        .frame(height: 34)
        .background(RoundedRectangle(cornerRadius: 9, style: .continuous).fill(chosen ? lib.accent.opacity(0.18) : .clear))
        .rowMotion(hover, tint: lib.accent, flash: flash)
        .contentShape(Rectangle())
        .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
        .onTapGesture {
            flash += 1
            NSApp.keyWindow?.makeFirstResponder(nil)
            let mods = NSApp.currentEvent?.modifierFlags ?? NSEvent.modifierFlags
            if mods.contains(.command) || mods.contains(.shift) {
                if let i = lib.selection.firstIndex(of: item.id) { lib.selection.remove(at: i) } else { lib.selection.append(item.id) }
            } else {
                lib.selection = [item.id]
            }
        }
        .accessibilityElement(children: .contain)
        .accessibilityLabel(item.name)
        .accessibilityAddTraits(lib.selection.contains(item.id) ? [.isButton, .isSelected] : .isButton)
        .accessibilityAction(.default) { lib.selection = [item.id] }
        .accessibilityAction(named: L("Add to selection")) {
            if let i = lib.selection.firstIndex(of: item.id) { lib.selection.remove(at: i) } else { lib.selection.append(item.id) }
        }
        .simultaneousGesture(TapGesture(count: 2).onEnded {
            text = item.name
            editing = true
            focused = true
        })
    }

    private func finish() {
        editing = false
        let t = text.trimmingCharacters(in: .whitespaces)
        guard !t.isEmpty, t != item.name else { return }
        lib.begin()
        lib.mutate(item.id) { $0.name = t }
    }
}

struct LayerStack: View {
    @Environment(Workbench.self) private var lib
    let item: Solid

    var body: some View {
        let layers = lib.stack(item)
        VStack(alignment: .leading, spacing: 6) {
            SettingsTitle(text: L("Layers"))
            ForEach(Array(layers.enumerated()), id: \.offset) { level, node in
                LayerRow(item: item, level: level, node: node)
            }
        }
    }
}

struct LayerRow: View {
    @Environment(Workbench.self) private var lib
    let item: Solid
    let level: Int
    let node: Node

    var body: some View {
        switch node {
        case .round(_, let picks, let radius):
            let all = picks.contains { $0.kind == Int32(BK_PICK_BODY) }
            SettingLine(title: L("Rounding"), detail: all ? L("All edges") : L("{n} picks", ["n": picks.count])) {
                HStack(spacing: 6) {
                    MMField(value: radius, unit: L("mm"), range: 0.01...1000, width: 58) { v in
                        lib.editLayer(item.id, level: level) { n in
                            if case .round(let of, let p, _) = n { return .round(of: of, picks: p, radius: v) }
                            return n
                        }
                    }
                    remove
                }
            }
        case .cove(_, let picks, let radius):
            let all = picks.contains { $0.kind == Int32(BK_PICK_BODY) }
            SettingLine(title: L("Inward rounding"), detail: all ? L("All edges") : L("{n} picks", ["n": picks.count])) {
                HStack(spacing: 6) {
                    MMField(value: radius, unit: L("mm"), range: 0.01...1000, width: 58) { v in
                        lib.editLayer(item.id, level: level) { n in
                            if case .cove(let of, let p, _) = n { return .cove(of: of, picks: p, radius: v) }
                            return n
                        }
                    }
                    remove
                }
            }
        case .bevel(_, let picks, let legs, _):
            let all = picks.contains { $0.kind == Int32(BK_PICK_BODY) }
            SettingLine(title: L("Bevel"), detail: (all ? L("All edges") : L("{n} picks", ["n": picks.count])) + " · "
                        + MMField.format(legs.x) + " × " + MMField.format(legs.y) + " " + L("mm")) {
                remove
            }
        case .split(_, let plane, let side):
            SettingLine(title: L("Split"), detail: side == 0 ? L("Upper side") : L("Lower side")) {
                HStack(spacing: 6) {
                    MMField(value: dot(plane.point, plane.normal), unit: L("mm"), width: 58) { v in
                        lib.editLayer(item.id, level: level) { n in
                            if case .split(let of, let p, let s) = n { return .split(of: of, plane: Plane(point: p.normal * v, normal: p.normal), side: s) }
                            return n
                        }
                    }
                    .help(L("Plane offset"))
                    remove
                }
            }
        case .hollow(_, let open, let walls, let thickness):
            SettingLine(title: L("Hollow"), detail: L("{n} openings", ["n": open.count]) + " · " + L("{n} own walls", ["n": walls.count])) {
                HStack(spacing: 6) {
                    MMField(value: thickness, unit: L("mm"), range: 0.01...1000, width: 58) { v in
                        lib.editLayer(item.id, level: level) { n in
                            if case .hollow(let of, let o, let w, _) = n { return .hollow(of: of, open: o, walls: w, thickness: v) }
                            return n
                        }
                    }
                    .help(L("Walls"))
                    remove
                }
            }
        case .group(let op, let parts):
            SettingLine(title: op == BK_UNION ? L("Merge") : op == BK_SUBTRACT ? L("Subtract") : L("Intersect"),
                        detail: L("{n} shapes", ["n": parts.count])) {
                Button(L("Ungroup")) { lib.ungroup() }.buttonStyle(PillStyle(tint: lib.accent)).frame(width: 110)
            }
        case .primitive(let p):
            SettingLine(title: L(p.name)) { EmptyView() }
        case .fastener(let f):
            SettingLine(title: f.name) { EmptyView() }
        }
    }

    private var remove: some View {
        ToolButton(icon: "xmark", title: L("Remove"), tint: Neon.red, size: 24) { lib.removeLayer(item.id, level: level) }
    }
}

// MARK: - Inspector

struct Inspector: View {
    @Environment(Workbench.self) private var lib

    var body: some View {
        let items = lib.selected
        VStack(alignment: .leading, spacing: 10) {
            NamesRow(items: items)
            ScreenSwitch().padding(.horizontal, 12)
            Snug {
                ScrollView {
                    VStack(alignment: .leading, spacing: 8) {
                        ZStack(alignment: .topLeading) {
                            ScreenContent(items: items)
                                .id(lib.screen)
                                .transition(slide)
                        }
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .clipped()
                        if items.count == 1, let b = items.first {
                            Info(item: b)
                        } else {
                            Combine()
                        }
                    }
                    .padding(.horizontal, 14)
                    .padding(.bottom, 14)
                }
                .scrollIndicators(.never)
                .scrollBounceBehavior(.basedOnSize)
            }
        }
        .padding(.top, 14)
        .frame(width: 300)
        .glassBar(20)
        .shadow(color: lib.accent.opacity(0.12 * Skin.shared.glow), radius: 24)
        .onGeometryChange(for: CGRect.self) { $0.frame(in: .global) } action: { lib.inspectorFrame = $0 }
    }

    private var slide: AnyTransition {
        let forward = lib.screenStep > 0
        return .asymmetric(insertion: .move(edge: forward ? .trailing : .leading).combined(with: .opacity),
                           removal: .move(edge: forward ? .leading : .trailing).combined(with: .opacity))
    }
}

// Its content's own height, but never more than there's room for (the content scrolls then): a panel as tall as it needs.
struct Snug: Layout {
    func sizeThatFits(proposal: ProposedViewSize, subviews: Subviews, cache: inout ()) -> CGSize {
        guard let s = subviews.first else { return .zero }
        let ideal = s.sizeThatFits(ProposedViewSize(width: proposal.width, height: nil))
        return CGSize(width: proposal.width ?? ideal.width, height: min(ideal.height, proposal.height ?? ideal.height))
    }

    func placeSubviews(in bounds: CGRect, proposal: ProposedViewSize, subviews: Subviews, cache: inout ()) {
        subviews.first?.place(at: bounds.origin, anchor: .topLeading, proposal: ProposedViewSize(bounds.size))
    }
}

// The selected shapes' names; a double click edits one (its text selected). Scrolls sideways when they don't fit.
struct NamesRow: View {
    @Environment(Workbench.self) private var lib
    let items: [Solid]

    var body: some View {
        ScrollView(.horizontal) {
            HStack(spacing: 6) {
                ForEach(items) { NameChip(item: $0, solo: items.count == 1) }
            }
            .padding(.horizontal, 14)
            .padding(.vertical, 2)
        }
        .scrollIndicators(.never)
        .onGeometryChange(for: CGRect.self) { $0.frame(in: .global) } action: { lib.namesFrame = $0 }
    }
}

struct NameChip: View {
    @Environment(Workbench.self) private var lib
    let item: Solid
    let solo: Bool
    @State private var editing = false
    @State private var draft = ""
    @State private var hover = false
    @FocusState private var focused: Bool

    var body: some View {
        let size: CGFloat = solo ? 15 : 13
        Group {
            if editing {
                TextField("", text: $draft)
                    .textFieldStyle(.plain)
                    .font(.ui(size: size, weight: .bold, design: .rounded))
                    .foregroundStyle(Ink.text)
                    .fixedSize()
                    .frame(minWidth: 60, alignment: .leading)
                    .focused($focused)
                    .onSubmit(commit)
                    .onExitCommand { editing = false }
                    .onChange(of: focused) { if !focused && editing { commit() } }
                    .task {
                        draft = item.name
                        try? await Task.sleep(for: .milliseconds(40))
                        focused = true
                        try? await Task.sleep(for: .milliseconds(30))
                        NSApp.sendAction(#selector(NSText.selectAll(_:)), to: nil, from: nil)
                    }
            } else {
                Text(item.name)
                    .font(.ui(size: size, weight: .bold, design: .rounded))
                    .foregroundStyle(Ink.text)
                    .lineLimit(1)
                    .fixedSize()
                    .halo(lib.accent, solo ? 8 : 0)
                    .onTapGesture(count: 2) { editing = true }
                    .help(L("Double-click to rename"))
            }
        }
        .padding(.horizontal, solo ? 0 : 9)
        .frame(height: solo ? 26 : 24)
        .background {
            if !solo || editing {
                RoundedRectangle(cornerRadius: 8, style: .continuous).fill(lib.accent.opacity(editing ? 0.16 : (hover ? 0.14 : 0.08)))
                    .padding(.horizontal, solo ? -6 : 0)
            }
        }
        .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
        .accessibilityElement(children: .combine)
        .accessibilityAction(named: L("Rename")) { editing = true }
    }

    private func commit() {
        lib.rename(item.id, draft)
        editing = false
    }
}

// Move · Resize · Rotate · Angles · Thread: the lit segment slides to the chosen screen (a two-finger swipe over the panel does too).
struct ScreenSwitch: View {
    @Environment(Workbench.self) private var lib
    @Namespace private var lane
    @State private var hover: Screen?
    @State private var waves = 0

    var body: some View {
        let shape = RoundedRectangle(cornerRadius: 12, style: .continuous)
        HStack(spacing: 2) {
            ForEach(Screen.allCases, id: \.self) { sc in
                let on = sc == lib.screen
                VStack(spacing: 2) {
                    Group {
                        if let icon = sc.icon {
                            Image(systemName: icon).font(.ui(size: 15, weight: .bold))
                        } else {
                            ThreadGlyph().stroke(style: StrokeStyle(lineWidth: 1.7, lineCap: .round, lineJoin: .round)).frame(width: 14, height: 16)
                        }
                    }
                    .frame(height: 18)
                    Text(L(sc.title)).font(.ui(size: 11.5, weight: .bold, design: .rounded)).lineLimit(1).minimumScaleFactor(0.7)
                }
                .foregroundStyle(on ? Color.black : (hover == sc ? lib.accent : Ink.text.opacity(0.75)))
                .frame(maxWidth: .infinity)
                .frame(height: 40)
                .background {
                    if on {
                        RoundedRectangle(cornerRadius: 8, style: .continuous)
                            .fill(lib.accent)
                            .glow(lib.accent, 12)
                            .matchedGeometryEffect(id: "lit", in: lane)
                    } else if hover == sc {
                        RoundedRectangle(cornerRadius: 8, style: .continuous).fill(lib.accent.opacity(0.12)).transition(.opacity)
                    }
                }
                .contentShape(Rectangle())
                .onHover { h in withAnimation(Neon.hover(h)) { hover = h ? sc : (hover == sc ? nil : hover) } }
                .onTapGesture {
                    guard !on else { return }
                    waves += 1
                    lib.choose(sc)
                }
                .help(key(sc).map { L(sc.title) + " (\($0))" } ?? L(sc.title))
                .accessibilityElement(children: .combine)
                .accessibilityAddTraits(on ? [.isButton, .isSelected] : .isButton)
                .accessibilityAction(.default) { lib.choose(sc) }
            }
        }
        .padding(3)
        .background(shape.fill(Ink.text.opacity(0.06)))
        .pressWave(waves, shape: shape, tint: lib.accent)
    }

    private func key(_ sc: Screen) -> String? {
        let s = lib.settings
        switch sc {
        case .move: return s.isOn(.move) ? Keys.label(s.key(.move)) : nil
        case .resize: return s.isOn(.scale) ? Keys.label(s.key(.scale)) : nil
        case .rotate: return s.isOn(.rotate) ? Keys.label(s.key(.rotate)) : nil
        case .angles, .thread: return nil
        }
    }
}

struct ScreenContent: View {
    @Environment(Workbench.self) private var lib
    let items: [Solid]

    var body: some View {
        let one = items.count == 1 ? items.first : nil
        VStack(alignment: .leading, spacing: 8) {
            switch lib.screen {
            case .move:
                if let b = one {
                    SettingsTitle(text: L("Position") + " · " + L("mm"))
                    AxisRow(values: b.place.move) { i, v in lib.setPlace(b.id) { $0.move[i] = v } }
                    Rectangle().fill(Ink.text.opacity(0.12)).frame(height: 1).padding(.horizontal, 6).padding(.top, 6)
                    ColourLine(item: b).id(b.id)
                } else {
                    Hint(text: L("Drag the arrows to move all selected shapes"))
                }
            case .resize:
                if let b = one {
                    switch b.node.base {
                    case .primitive(let p): PrimitiveSizes(id: b.id, prim: p)
                    case .fastener(let f): FastenerLength(id: b.id, f: f)
                    default: EmptyView()
                    }
                    if !plain(b) || b.place.scale != SIMD3(1, 1, 1) {
                        SettingsTitle(text: L("Scale") + " · %")
                        AxisRow(values: b.place.scale * 100, range: 1...100000) { i, v in lib.rescale(b.id, axis: i, by: v / 100 / b.place.scale[i]) }
                    }
                } else {
                    Hint(text: L("Drag the handles to resize all selected shapes"))
                }
                ProportionsLine()
                SymmetryLine()
            case .rotate:
                if let b = one {
                    SettingsTitle(text: L("Rotation") + " · °")
                    AxisRow(values: b.place.turn, range: -360...360) { i, v in lib.setPlace(b.id) { $0.turn[i] = v } }
                } else {
                    Hint(text: L("Drag the rings to turn all selected shapes"))
                }
            case .angles:
                AnglesScreen()
            case .thread:
                ThreadScreen(items: items)
            }
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }

    // A primitive or a fastener, rounded or not: resizing changes its sizes rather than stretching it.
    private func plain(_ b: Solid) -> Bool {
        switch b.node.base {
        case .primitive, .fastener: true
        default: false
        }
    }
}

struct Hint: View {
    let text: String
    var body: some View {
        Text(text)
            .font(.ui(size: 11.5, weight: .medium, design: .rounded))
            .foregroundStyle(Ink.text.opacity(0.5))
            .fixedSize(horizontal: false, vertical: true)
            .padding(.leading, 6)
    }
}

// Uniform scaling, remembered for every shape.
struct ProportionsLine: View {
    @Environment(Workbench.self) private var lib

    var body: some View {
        SettingLine(title: L("Keep proportions"), detail: L("or hold ⇧ while resizing")) {
            HStack(spacing: 8) {
                Image(systemName: lib.settings.uniform ? "link" : "link.badge.plus")
                    .font(.ui(size: 13, weight: .bold))
                    .foregroundStyle(lib.settings.uniform ? lib.accent3 : Ink.text.opacity(0.4))
                    .contentTransition(.symbolEffect(.replace))
                NeonToggle(state: lib.settings.uniform) { lib.updateSettings { $0.uniform.toggle() } }
                    .accessibilityLabel(L("Keep proportions"))
            }
        }
    }
}

// Symmetric resizing, remembered for every shape: both sides of a size move and the middle stays; otherwise the left,
// front or bottom side stays.
struct SymmetryLine: View {
    @Environment(Workbench.self) private var lib

    var body: some View {
        let on = lib.settings.symmetric
        SettingLine(title: L("Symmetric resizing"), detail: L("or hold ⌥ while resizing")) {
            HStack(spacing: 8) {
                Image(systemName: on ? "arrow.left.and.right" : "arrow.right")
                    .font(.ui(size: 13, weight: .bold))
                    .foregroundStyle(on ? lib.accent3 : Ink.text.opacity(0.4))
                    .contentTransition(.symbolEffect(.replace))
                NeonToggle(state: on) { lib.updateSettings { $0.symmetric.toggle() } }
                    .accessibilityLabel(L("Symmetric resizing"))
            }
        }
    }
}

// The palette, and at its end a button that opens a mixer for any colour; closing it keeps the colour.
struct ColourLine: View {
    @Environment(Workbench.self) private var lib
    let item: Solid
    @State private var mixing = false

    var body: some View {
        let c = item.color
        let mixed = !Palette.colors.contains(c)
        VStack(alignment: .leading, spacing: 8) {
            SettingsTitle(text: L("Colour"))
            HStack(spacing: 7) {
                ForEach(Palette.colors.indices, id: \.self) { i in dot(i) }
                Button { toggle() } label: { Image(systemName: "paintpalette.fill") }
                    .buttonStyle(NeonButtonStyle(tint: mixed ? Palette.color(c) : lib.accent, lit: mixing, size: 24))
                    .overlay(RoundedRectangle(cornerRadius: 7, style: .continuous).strokeBorder(Ink.text, lineWidth: mixed ? 2 : 0).allowsHitTesting(false))
                    .help(L("Mix a colour"))
                    .accessibilityLabel(L("Mix a colour"))
            }
            .padding(.leading, 6)
            if mixing {
                ColourMixer(item: item).transition(.menu)
            }
        }
        .onDisappear { if mixing { lib.undoLastIfUnchanged() } }
    }

    private func dot(_ i: Int) -> some View {
        let c = Palette.colors[i], color = Palette.color(c), on = item.color == c
        return Circle().fill(color)
            .frame(width: 20, height: 20)
            .overlay(Circle().strokeBorder(Ink.text, lineWidth: on ? 2 : 0))
            .glow(color, on ? 8 : 0)
            .contentShape(Circle())
            .onTapGesture { recolor(c) }
            .accessibilityElement()
            .accessibilityLabel(L("Colour"))
            .accessibilityValue("\(i + 1)")
            .accessibilityAddTraits(on ? [.isButton, .isSelected] : .isButton)
            .accessibilityAction(.default) { recolor(c) }
    }

    // While the mixer is open its one undo step covers every change; otherwise a colour is a step of its own.
    private func recolor(_ c: SIMD3<UInt8>) {
        if mixing { lib.paint(item.id, c) } else { lib.commit { d in if let k = d.bodies.firstIndex(where: { $0.id == item.id }) { d.bodies[k].color = c } } }
    }

    private func toggle() {
        withAnimation(Neon.glide) { mixing.toggle() }
        if mixing { lib.begin() } else { lib.undoLastIfUnchanged() }
    }
}

// Red, green and blue from 0 to 255: a slider for each, then the three as numbers.
struct ColourMixer: View {
    @Environment(Workbench.self) private var lib
    let item: Solid

    var body: some View {
        let c = item.color
        VStack(spacing: 10) {
            ForEach(0..<3, id: \.self) { k in ChannelSlider(color: c, channel: k) { set(k, $0) } }
            HStack(spacing: 6) {
                ForEach(0..<3, id: \.self) { k in
                    HStack(spacing: 3) {
                        Text(["R", "G", "B"][k]).font(.ui(size: 10.5, weight: .black, design: .rounded)).foregroundStyle(ChannelSlider.tint(k))
                        MMField(value: Double(c[k]), range: 0...255, width: 52, tint: ChannelSlider.tint(k), digits: 0) { set(k, $0) }
                            .accessibilityLabel(ChannelSlider.name(k))
                    }
                    .frame(maxWidth: .infinity)
                }
            }
        }
        .padding(10)
        .background(RoundedRectangle(cornerRadius: 12, style: .continuous).fill(Ink.text.opacity(0.05)))
    }

    private func set(_ k: Int, _ v: Double) {
        var c = item.color
        c[k] = UInt8(min(255, max(0, v.rounded())))
        lib.paint(item.id, c)
    }
}

// One channel of a colour: the track runs from the colour without it to the colour with all of it.
struct ChannelSlider: View {
    let color: SIMD3<UInt8>
    let channel: Int
    let set: (Double) -> Void
    @State private var dragging = false
    @State private var hover = false

    static func tint(_ k: Int) -> Color { [Color(red: 1, green: 0.3, blue: 0.35), Color(red: 0.3, green: 0.9, blue: 0.4), Color(red: 0.35, green: 0.55, blue: 1)][k] }
    @MainActor static func name(_ k: Int) -> String { [L("Red"), L("Green"), L("Blue")][k] }

    var body: some View {
        var none = color, full = color
        none[channel] = 0
        full[channel] = 255
        let f = CGFloat(color[channel]) / 255
        return GeometryReader { g in
            let knob: CGFloat = dragging ? 16 : (hover ? 14 : 12), room = max(1, g.size.width - knob)
            ZStack(alignment: .leading) {
                Capsule()
                    .fill(LinearGradient(colors: [Palette.color(none), Palette.color(full)], startPoint: .leading, endPoint: .trailing))
                    .overlay(Capsule().strokeBorder(Ink.text.opacity(0.18), lineWidth: 1))
                    .frame(height: hover || dragging ? 9 : 7)
                SliderKnob(size: knob, tint: Palette.color(color), active: dragging)
                    .offset(x: f * room)
            }
            .frame(width: g.size.width, height: g.size.height)
            .contentShape(Rectangle())
            .gesture(DragGesture(minimumDistance: 0)
                .onChanged { v in
                    dragging = true
                    set(Double(min(1, max(0, (v.location.x - knob / 2) / room))) * 255)
                }
                .onEnded { _ in dragging = false })
        }
        .frame(height: 22)
        .environment(\.layoutDirection, .leftToRight)
        .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
        .animation(Neon.pop, value: dragging)
        .accessibilityElement()
        .accessibilityLabel(Self.name(channel))
        .accessibilityValue("\(color[channel])")
        .accessibilityAdjustableAction { d in set(Double(color[channel]) + (d == .increment ? 5 : -5)) }
    }
}

struct Combine: View {
    @Environment(Workbench.self) private var lib

    var body: some View {
        HStack(spacing: 6) {
            Button(L("Merge")) { lib.combine(Int32(BK_UNION)) }.buttonStyle(PillStyle(tint: lib.accent2))
            Button(L("Subtract")) { lib.combine(Int32(BK_SUBTRACT)) }.buttonStyle(PillStyle(tint: lib.accent2))
        }
        HStack(spacing: 6) {
            Button(L("Intersect")) { lib.combine(Int32(BK_INTERSECT)) }.buttonStyle(PillStyle(tint: lib.accent2))
            Button(L("Duplicate")) { lib.duplicate() }.buttonStyle(PillStyle(tint: lib.accent))
        }
        Hint(text: L("Merge and subtract keep the first selected shape as the base"))
    }
}

// Picking the edges to work on: just the hint until something is picked, then the way into the angle editor.
struct AnglesScreen: View {
    @Environment(Workbench.self) private var lib

    var body: some View {
        let picks = lib.edgePicks
        let all = picks.contains { $0.kind == Int32(BK_PICK_BODY) }
        VStack(alignment: .leading, spacing: 10) {
            Hint(text: picks.isEmpty ? L("Click an edge, a corner or a face · ⇧ adds more") : (all ? L("All edges") : L("{n} picks", ["n": picks.count])))
                .contentTransition(.opacity)
            if !picks.isEmpty {
                Button { lib.workWithAngles() } label: {
                    Label(L("Work with angles"), systemImage: "angle")
                }
                .buttonStyle(PillStyle(tint: lib.accent2))
                .transition(.scale(scale: 0.9, anchor: .top).combined(with: .haze))
            }
        }
        .animation(Neon.spring, value: picks.isEmpty)
    }
}

struct AxisRow: View {
    let values: SIMD3<Double>
    var range: ClosedRange<Double> = -100000...100000
    let set: (Int, Double) -> Void

    var body: some View {
        HStack(spacing: 6) {
            ForEach(0..<3, id: \.self) { i in
                HStack(spacing: 3) {
                    Text(["X", "Y", "Z"][i]).font(.ui(size: 10.5, weight: .black, design: .rounded)).foregroundStyle(Axis.color(i))
                    MMField(value: values[i], range: range, width: 64, tint: Axis.color(i)) { set(i, $0) }
                }
            }
        }
        .padding(.leading, 6)
    }
}

// A size; one running along an axis is washed in that axis's colour, row and field, whatever the style or theme.
struct SizeLine: View {
    let title: String
    let axis: Int?
    let value: Double
    var unit: String?
    let range: ClosedRange<Double>
    let set: (Double) -> Void

    var body: some View {
        let tint = axis.map(Axis.color)
        HStack(spacing: 10) {
            Text(title).font(.ui(size: 13, weight: .medium, design: .rounded)).foregroundStyle(Ink.text).lineLimit(2)
            Spacer(minLength: 4)
            MMField(value: value, unit: unit, range: range, tint: tint, fill: tint == nil ? 0.07 : 0.24, set: set)
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 8)
        .frame(minHeight: 44)
        .background(RoundedRectangle(cornerRadius: 11, style: .continuous).fill(tint.map { $0.opacity(0.13) } ?? Ink.text.opacity(0.05)))
    }
}

struct PrimitiveSizes: View {
    @Environment(Workbench.self) private var lib
    let id: UUID
    let prim: Primitive

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            SettingsTitle(text: L("Size") + " · " + L("mm"))
            if prim.kind == .prism || prim.kind == .pyramid {
                SettingLine(title: L("Sides")) {
                    HStack(spacing: 4) {
                        ToolButton(icon: "minus", title: L("Fewer sides"), size: 24) { sides(prim.sides - 1) }
                        Text("\(prim.sides)").font(.ui(size: 13, weight: .bold, design: .rounded)).monospacedDigit().frame(width: 24)
                        ToolButton(icon: "plus", title: L("More sides"), size: 24) { sides(prim.sides + 1) }
                    }
                }
            }
            if [.cylinder, .oval, .torus, .ovalTorus].contains(prim.kind) {
                Segmented(options: [false, true], label: { $0 ? L("Oval") : L("Round") }, icon: { $0 ? "oval" : "circle" },
                          current: prim.kind == .oval || prim.kind == .ovalTorus) { oval in shape(oval) }
            }
            ForEach(Array(prim.fields.enumerated()), id: \.offset) { i, field in
                SizeLine(title: L(field), axis: prim.axes[i], value: prim.size[i], unit: prim.degrees.contains(i) ? "°" : nil,
                         range: prim.range(i, limit: lib.settings.longest)) { v in
                    lib.reshape(id) { n in
                        guard case .primitive(var p) = n else { return n }
                        if lib.settings.uniform, !p.degrees.contains(i), p.size[i] > 0 {
                            let k = v / p.size[i]
                            p.size = p.size.enumerated().map { j, x in p.degrees.contains(j) || x == 0 ? x : max(0.01, (x * k * 100).rounded() / 100) }
                        }
                        p.size[i] = v
                        return .primitive(p)
                    }
                }
            }
        }
    }

    // A round cylinder or torus becomes an oval with both diameters equal and square to each other, and back.
    private func shape(_ oval: Bool) {
        lib.reshape(id) { node in
            guard case .primitive(var p) = node else { return node }
            switch (oval, p.kind) {
            case (true, .cylinder): p = Primitive(kind: .oval, size: [p.size[0], p.size[0], 90, p.size[1]])
            case (false, .oval): p = Primitive(kind: .cylinder, size: [max(p.size[0], p.size[1]), p.size[3]])
            case (true, .torus):
                p = Primitive(kind: .ovalTorus, sides: p.sides, size: [p.size[0], p.size[0], 90, p.size[1]])
                if !p.bends(1.06) { p.size[3] = p.range(3, limit: .infinity).upperBound }
            case (false, .ovalTorus): p = Primitive(kind: .torus, sides: p.sides, size: [max(p.size[0], p.size[1]), p.size[3]])
            default: break
            }
            return .primitive(p)
        }
    }

    private func sides(_ n: Int) {
        guard (3...24).contains(n) else { return }
        lib.reshape(id) { node in
            guard case .primitive(var p) = node else { return node }
            p.sides = n
            return .primitive(p)
        }
    }
}

// A bolt's or nut's length, its height; the rest of it is on the Thread tab.
struct FastenerLength: View {
    @Environment(Workbench.self) private var lib
    let id: UUID
    let f: Fastener

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            SettingsTitle(text: L("Size") + " · " + L("mm"))
            SizeLine(title: L("Length"), axis: 2, value: f.length, range: 1...lib.settings.longest) { v in
                var n = f
                n.length = v
                lib.reshape(id) { _ in .fastener(n) }
            }
        }
    }
}

// A selected bolt or nut changes as it's set; for anything else the same choices make a new one.
struct ThreadScreen: View {
    @Environment(Workbench.self) private var lib
    let items: [Solid]

    var body: some View {
        if items.count == 1, let b = items.first, case .fastener(let f) = b.node.base {
            ThreadControls(f: f, compact: true) { n in
                let old = f.name
                lib.reshape(b.id) { _ in .fastener(n) }
                lib.mutate(b.id) { if $0.name == old { $0.name = n.name } }
            }
        } else {
            ThreadControls(f: lib.thread, compact: true) { lib.thread = $0 }
            Button(lib.thread.nut ? L("Add nut") : L("Add bolt")) { lib.addFastener(lib.thread) }
                .buttonStyle(PillStyle(tint: lib.accent))
                .contentTransition(.interpolate)
        }
    }
}

struct Info: View {
    @Environment(Workbench.self) private var lib
    let item: Solid

    var body: some View {
        let m = lib.meshes[item.id]
        let s = item.place.scale
        VStack(alignment: .leading, spacing: 3) {
            if let (lo, hi) = lib.worldBounds(item) {
                let d = hi - lo
                Text(L("Size") + "  " + [d.x, d.y, d.z].map { MMField.format($0) }.joined(separator: " × ") + " " + L("mm"))
            }
            if let m {
                Text(L("Volume") + "  " + MMField.format(m.volume * abs(s.x * s.y * s.z)) + " " + L("mm³"))
                if !m.valid {
                    Text(L("This shape isn't a closed solid")).foregroundStyle(Neon.red)
                }
            }
        }
        .font(.ui(size: 11, weight: .medium, design: .rounded))
        .monospacedDigit()
        .foregroundStyle(Ink.text.opacity(0.5))
        .padding(.leading, 6)
        .padding(.top, 4)
    }
}

// MARK: - Thread

// Animated segmented choice: the lit segment slides under the chosen option.
struct Segmented<Option: Hashable>: View {
    @Environment(Workbench.self) private var lib
    let options: [Option]
    let label: (Option) -> String
    let icon: (Option) -> String
    let current: Option
    let choose: (Option) -> Void
    @Namespace private var lane
    @State private var hover: Option?
    @State private var waves = 0

    var body: some View {
        let shape = RoundedRectangle(cornerRadius: 12, style: .continuous)
        HStack(spacing: 4) {
            ForEach(options, id: \.self) { o in
                let on = o == current
                HStack(spacing: 7) {
                    Image(systemName: icon(o)).font(.ui(size: 12, weight: .bold))
                    Text(label(o)).font(.ui(size: 13, weight: .bold, design: .rounded)).lineLimit(1)
                }
                .foregroundStyle(on ? Color.black : (hover == o ? lib.accent : Ink.text.opacity(0.75)))
                .frame(maxWidth: .infinity)
                .frame(height: 32)
                .background {
                    if on {
                        RoundedRectangle(cornerRadius: 9, style: .continuous)
                            .fill(lib.accent)
                            .glow(lib.accent, 12)
                            .matchedGeometryEffect(id: "lit", in: lane)
                    } else if hover == o {
                        RoundedRectangle(cornerRadius: 9, style: .continuous).fill(lib.accent.opacity(0.12)).transition(.opacity)
                    }
                }
                .scaleEffect(hover == o && !on ? 1.03 : 1)
                .contentShape(Rectangle())
                .onHover { h in withAnimation(Neon.hover(h)) { hover = h ? o : (hover == o ? nil : hover) } }
                .onTapGesture {
                    guard !on else { return }
                    waves += 1
                    withAnimation(.spring(response: 0.42, dampingFraction: 0.72)) { choose(o) }
                }
                .accessibilityElement(children: .combine)
                .accessibilityAddTraits(on ? [.isButton, .isSelected] : .isButton)
                .accessibilityAction(.default) { if !on { withAnimation(.spring(response: 0.42, dampingFraction: 0.72)) { choose(o) } } }
            }
        }
        .padding(4)
        .background(shape.fill(Ink.text.opacity(0.06)))
        .pressWave(waves, shape: shape, tint: lib.accent)
    }
}

// Bolt | Nut, Plain | Hexed, thread size and length: the Thread tab's choices.
struct ThreadControls: View {
    @Environment(Workbench.self) private var lib
    let f: Fastener
    var compact = false
    let change: (Fastener) -> Void
    @State private var sizesOpen = false
    @State private var hover = false

    private var name: String { String(cString: bk_thread_name(Int32(f.size))) }

    private var detail: String {
        switch (f.nut, f.threadOnly) {
        case (false, false): L("Length is measured under the head")
        case (false, true): L("The thread takes the whole length")
        case (true, false): L("A hex nut of this height")
        case (true, true): L("A thin cylinder around the thread instead of a hex nut")
        }
    }

    var body: some View {
        VStack(spacing: 10) {
            Segmented(options: [false, true], label: { $0 ? L("Nut") : L("Bolt") }, icon: { $0 ? "circle.hexagonpath" : "screwdriver" },
                      current: f.nut) { nut in
                var n = f
                n.nut = nut
                n.length = bk_thread_default_length(Int32(n.size), nut ? 1 : 0)
                change(n)
            }
            Segmented(options: [true, false], label: { $0 ? L("Plain") : L("Hexed") }, icon: { $0 ? "circle" : "hexagon" },
                      current: f.threadOnly) { only in
                var n = f
                n.threadOnly = only
                change(n)
            }
            HStack(spacing: 8) {
                HStack(spacing: 8) {
                    if !compact { Text(L("Thread")).foregroundStyle(Ink.text.opacity(0.55)) }
                    Text(name).contentTransition(.numericText())
                    Spacer(minLength: 0)
                    Image(systemName: "chevron.down")
                        .font(.ui(size: 10, weight: .bold))
                        .rotationEffect(.degrees(sizesOpen ? 180 : 0))
                }
                .font(.ui(size: 13, weight: .bold, design: .rounded))
                .foregroundStyle(hover || sizesOpen ? lib.accent : Ink.text)
                .padding(.horizontal, 12)
                .frame(height: 36)
                .background(RoundedRectangle(cornerRadius: 11, style: .continuous).fill(Ink.text.opacity(hover || sizesOpen ? 0.1 : 0.06)))
                .glow(lib.accent, hover ? 10 : (sizesOpen ? 5 : 0))
                .contentShape(Rectangle())
                .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
                .onTapGesture { withAnimation(Neon.glide) { sizesOpen.toggle() } }
                .accessibilityElement(children: .combine)
                .accessibilityLabel(L("Thread"))
                .accessibilityValue(name)
                .accessibilityAddTraits(.isButton)
                .accessibilityAction(.default) { withAnimation(Neon.glide) { sizesOpen.toggle() } }
                Text(L("Length")).font(.ui(size: 13, weight: .semibold, design: .rounded)).foregroundStyle(Ink.text.opacity(0.55))
                MMField(value: f.length, unit: L("mm"), range: 1...lib.settings.longest, width: compact ? 54 : 60) { v in
                    var n = f
                    n.length = v
                    change(n)
                }
            }
            if sizesOpen {
                LazyVGrid(columns: Array(repeating: GridItem(.flexible(), spacing: 6), count: 5), spacing: 6) {
                    ForEach(0..<Int(bk_thread_count()), id: \.self) { i in
                        Chip(text: String(cString: bk_thread_name(Int32(i))), chosen: i == f.size) {
                            var n = f
                            n.size = i
                            n.length = bk_thread_default_length(Int32(i), f.nut ? 1 : 0)
                            change(n)
                            withAnimation(Neon.glide) { sizesOpen = false }
                        }
                        .cascade(i)
                    }
                }
                .padding(6)
                .background(RoundedRectangle(cornerRadius: 12, style: .continuous).fill(Ink.void.opacity(0.5)))
                .transition(.menu)
            }
            Text(detail)
                .font(.ui(size: 10.5, design: .rounded))
                .foregroundStyle(Ink.text.opacity(0.45))
                .frame(maxWidth: .infinity, alignment: .leading)
                .padding(.leading, 6)
                .id(detail)
                .transition(.haze)
        }
        .animation(.spring(response: 0.42, dampingFraction: 0.78), value: f)
    }
}

// MARK: - Settings

struct SettingsPane: View {
    @Environment(Workbench.self) private var lib
    @Binding var langOpen: Bool
    @Binding var styleOpen: Bool

    private var fixed: [(String, String)] {
        [(L("New · Open · Save"), "⌘N  ⌘O  ⌘S"), (L("Export STL · STEP"), "⇧⌘E  ⌥⌘E"), (L("Undo · Redo"), "⌘Z  ⇧⌘Z"),
         (L("Duplicate · Delete"), "⌘D  ⌫"), (L("Select all"), "⌘A"), (L("Merge"), "⌘U"), (L("Subtract · Intersect"), "⌘⌫  ⌘I"),
         (L("Ungroup"), "⇧⌘G"), (L("Add thread"), "⌘B"), (L("Nudge"), "← → ↑ ↓  PgUp PgDn"), (L("Nudge ×10"), "⇧ + ←→↑↓"),
         (L("Views: iso, front, back, left, right, top, bottom"), "0–6"), (L("Split axis"), "X  Y  Z"), (L("Round all edges"), "A"), (L("A face with its own wall (Hollow)"), "⌥ click"),
         (L("Apply · Cancel"), "Enter  Esc"), (L("Add to selection"), "⇧/⌘ click"), (L("Orbit"), L("drag empty space")),
         (L("Pan"), L("two-finger scroll · ⇧ drag")), (L("Zoom"), L("pinch · ⌥ scroll")), (L("Move freely"), L("hold ⌘ while dragging")),
         (L("Keep proportions"), L("hold ⇧ while resizing")), (L("Symmetric resizing"), L("hold ⌥ while resizing"))]
    }

    var body: some View {
        let s = lib.settings
        ScrollView {
            VStack(alignment: .leading, spacing: 8) {
                SettingsHead(langOpen: $langOpen, styleOpen: $styleOpen)
                SettingsTitle(text: L("Shortcuts"))
                ForEach(Action.allCases, id: \.self) { a in
                    ShortcutLine(title: a.label, on: s.isOn(a), toggle: { lib.toggleShortcut(a) }) { KeyField(action: a) }
                }
                SettingsTitle(text: L("Editing"))
                SettingLine(title: L("Snap step"), detail: L("Dragging moves in these steps")) {
                    MMField(value: s.snap, unit: L("mm"), range: 0.01...100, width: 58) { v in lib.updateSettings { $0.snap = v } }
                }
                SettingLine(title: L("Rotation step")) {
                    MMField(value: s.turnStep, unit: "°", range: 0.1...90, width: 58) { v in lib.updateSettings { $0.turnStep = v } }
                }
                SettingLine(title: L("Automatic linking"), detail: L("Dragging links to nearby surfaces, the sizes of other shapes and the bed grid · hold ⌘ to drag freely")) {
                    NeonToggle(state: s.autoLink) { lib.updateSettings { $0.autoLink.toggle() } }
                        .accessibilityLabel(L("Automatic linking"))
                }
                SettingLine(title: L("Drop onto the bed"), detail: L("New and rotated shapes rest on the bed")) {
                    NeonToggle(state: s.dropToBed) { lib.updateSettings { $0.dropToBed.toggle() } }
                        .accessibilityLabel(L("Drop onto the bed"))
                }
                SettingsTitle(text: L("Print bed"))
                ForEach(0..<3, id: \.self) { i in
                    SettingLine(title: [L("Width"), L("Depth"), L("Height")][i]) {
                        MMField(value: s.bed[i], unit: L("mm"), range: 10...5000, width: 64) { v in lib.updateSettings { $0.bed[i] = v } }
                    }
                }
                SettingsTitle(text: L("Threads"))
                SettingLine(title: L("Thread clearance"), detail: L("Thins bolt threads and widens nut threads so printed parts fit · 0.00 is the exact ISO size")) {
                    MMField(value: s.clearance, unit: L("mm"), range: 0...2, width: 58) { v in lib.updateSettings { $0.clearance = v } }
                }
                SettingsTitle(text: L("Other shortcuts"))
                ForEach(fixed, id: \.0) { title, keys in
                    HStack {
                        Text(title).foregroundStyle(Ink.text.opacity(0.75))
                        Spacer(minLength: 8)
                        Text(keys).foregroundStyle(lib.accent).multilineTextAlignment(.trailing)
                    }
                    .font(.ui(size: 11.5, weight: .medium, design: .rounded))
                    .padding(.horizontal, 12)
                    .padding(.vertical, 3)
                }
                Button(L("Restore defaults")) { lib.restoreDefaults() }
                    .buttonStyle(PillStyle(tint: lib.accent2))
                    .padding(.top, 10)
            }
            .padding(.bottom, 12)
        }
        .scrollIndicators(.never)
    }
}

// MARK: - Angles

// A small picture of a block's top-right corner: sharp, rounded outward or inward, or bevelled at an angle to the top face.
struct CornerGlyph: View {
    enum Look: Hashable {
        case sharp, outbound, inbound, bevel(Double), softBevel(Double)
    }

    let look: Look
    let tint: Color

    var body: some View {
        Canvas { ctx, size in
            let w = size.width, h = size.height, m: CGFloat = 5
            let x0 = m, y0 = m + 3, x1 = w - m, y1 = h - m
            let r = min(x1 - x0, y1 - y0) * 0.5
            var p = Path()
            p.move(to: CGPoint(x: x0, y: y1))
            p.addLine(to: CGPoint(x: x0, y: y0))
            switch look {
            case .sharp:
                p.addLine(to: CGPoint(x: x1, y: y0))
            case .outbound:
                p.addLine(to: CGPoint(x: x1 - r, y: y0))
                p.addArc(center: CGPoint(x: x1 - r, y: y0 + r), radius: r, startAngle: .degrees(-90), endAngle: .degrees(0), clockwise: false)
            case .inbound:
                p.addLine(to: CGPoint(x: x1 - r, y: y0))
                p.addArc(center: CGPoint(x: x1, y: y0), radius: r, startAngle: .degrees(180), endAngle: .degrees(90), clockwise: true)
            case .bevel(let a), .softBevel(let a):
                let t = tan(max(5, min(85, a)) * .pi / 180)
                let legA = min(r * 1.25, r * 1.25 / max(1, t)), legB = legA * t
                let pa = CGPoint(x: x1 - legA, y: y0), pb = CGPoint(x: x1, y: y0 + min(legB, y1 - y0 - 2))
                if case .softBevel = look {
                    let k: CGFloat = 0.35
                    p.addLine(to: CGPoint(x: pa.x - legA * k * 0.5, y: y0))
                    p.addQuadCurve(to: CGPoint(x: pa.x + (pb.x - pa.x) * k * 0.5, y: pa.y + (pb.y - pa.y) * k * 0.5), control: pa)
                    p.addLine(to: CGPoint(x: pb.x - (pb.x - pa.x) * k * 0.5, y: pb.y - (pb.y - pa.y) * k * 0.5))
                    p.addQuadCurve(to: CGPoint(x: x1, y: pb.y + legB * k * 0.5), control: pb)
                } else {
                    p.addLine(to: pa)
                    p.addLine(to: pb)
                }
            }
            p.addLine(to: CGPoint(x: x1, y: y1))
            p.closeSubpath()
            ctx.fill(p, with: .color(tint.opacity(0.3)))
            ctx.stroke(p, with: .color(tint.opacity(0.9)), style: StrokeStyle(lineWidth: 1.5, lineJoin: .round))
        }
        .accessibilityHidden(true)
    }
}

// Choices shown as small pictures with a caption; the lit segment slides to the chosen one.
struct PictureSegmented<Option: Hashable>: View {
    @Environment(Workbench.self) private var lib
    let options: [Option]
    let current: Option
    let label: (Option) -> String
    let picture: (Option) -> CornerGlyph.Look
    var enabled: (Option) -> Bool = { _ in true }
    let choose: (Option) -> Void
    @Namespace private var lane
    @State private var hover: Option?

    var body: some View {
        HStack(spacing: 3) {
            ForEach(options, id: \.self) { o in
                let on = o == current, ok = enabled(o)
                VStack(spacing: 2) {
                    CornerGlyph(look: picture(o), tint: on ? .black : Ink.text).frame(width: 40, height: 28)
                    Text(label(o)).font(.ui(size: 10.5, weight: .bold, design: .rounded)).lineLimit(1).minimumScaleFactor(0.7)
                        .foregroundStyle(on ? Color.black : (hover == o ? lib.accent : Ink.text.opacity(0.75)))
                }
                .frame(maxWidth: .infinity)
                .padding(.vertical, 5)
                .background {
                    if on {
                        RoundedRectangle(cornerRadius: 9, style: .continuous).fill(lib.accent).glow(lib.accent, 10)
                            .matchedGeometryEffect(id: "lit", in: lane)
                    } else if hover == o {
                        RoundedRectangle(cornerRadius: 9, style: .continuous).fill(lib.accent.opacity(0.12)).transition(.opacity)
                    }
                }
                .opacity(ok ? 1 : 0.35)
                .contentShape(Rectangle())
                .onHover { h in withAnimation(Neon.hover(h)) { hover = h && ok ? o : (hover == o ? nil : hover) } }
                .onTapGesture { if ok && !on { choose(o) } }
                .accessibilityElement(children: .combine)
                .accessibilityLabel(label(o))
                .accessibilityAddTraits(on ? [.isButton, .isSelected] : .isButton)
                .accessibilityAction(.default) { if ok { choose(o) } }
            }
        }
        .padding(4)
        .background(RoundedRectangle(cornerRadius: 12, style: .continuous).fill(Ink.text.opacity(0.06)))
    }
}

// A millimetre value with − and + stepping one millimetre; each greys out at its end of the range.
struct StepperLine: View {
    @Environment(Workbench.self) private var lib
    let title: String
    let value: Double
    let range: ClosedRange<Double>
    let set: (Double) -> Void

    var body: some View {
        SettingLine(title: title) {
            HStack(spacing: 5) {
                step(-1, "minus", enabled: value > range.lowerBound + 0.001)
                MMField(value: value, unit: L("mm"), range: range, width: 58, set: set)
                step(1, "plus", enabled: value < range.upperBound - 0.001)
            }
        }
    }

    private func step(_ d: Double, _ icon: String, enabled: Bool) -> some View {
        ToolButton(icon: icon, title: d < 0 ? L("Smaller") : L("Larger"), size: 24) {
            set(min(range.upperBound, max(range.lowerBound, value + d)))
        }
        .disabled(!enabled)
        .opacity(enabled ? 1 : 0.3)
        .saturation(enabled ? 1 : 0)
    }
}

// The angle editor: the cut through the shape fills the view, the controls sit beside it.
struct AngleEditor: View {
    @Environment(Workbench.self) private var lib

    var body: some View {
        if let e = lib.angleEdit {
            let tint = lib.body(e.body).map { Palette.color($0.color) } ?? lib.accent
            ZStack(alignment: .trailing) {
                SectionView(edit: e, window: e.window, tint: tint)
                    .ignoresSafeArea()
                AnglePanel(edit: e)
                    .padding(.trailing, 14)
                    .padding(.vertical, 14)
            }
        }
    }
}

extension AngleEdit {
    // How much of the cut the view shows (millimetres from its centre), so the worked-on corner stays large.
    var window: Double {
        let reach: Double
        switch treatment {
        case .angled: reach = max(legs.x, legs.y)
        case .rounded: reach = rounding == .outbound ? radius / max(0.05, tan(phi / 2)) : radius
        }
        return max(1.5, reach * 2.4)
    }
}

// The cut drawn in 2D: material filled with the shape's colour, the part that goes away marked in red, the new outline lit.
struct SectionView: View, Animatable {
    var edit: AngleEdit
    var window: Double
    let tint: Color

    var animatableData: Double {
        get { window }
        set { window = newValue }
    }

    var body: some View {
        let look = Skin.shared
        let accent = look.accent, accent2 = look.accent2, accent3 = look.accent3, glow = look.glow
        let ink = Ink.text, void = Ink.void
        let faceA = L("Face A"), faceB = L("Face B")
        let bold = Font.ui(size: 12, weight: .heavy, design: .rounded), small = Font.ui(size: 11.5, weight: .bold, design: .rounded)
        Canvas { ctx, size in
            let e = edit, sec = e.section
            let room = max(240, size.width - 380)
            let k = min(room, size.height) * 0.46 / window
            let bis = simd_normalize(sec.dirA + sec.dirB)
            let focus = bis * window * 0.3
            let center = CGPoint(x: room / 2, y: size.height / 2)
            func pt(_ p: SIMD2<Double>) -> CGPoint { CGPoint(x: center.x + (p.x - focus.x) * k, y: center.y - (p.y - focus.y) * k) }

            ctx.fill(Path(CGRect(origin: .zero, size: size)), with: .color(void))
            // millimetre grid, every 1, 2, 5 or 10 … mm depending on the zoom
            let raw = window / 6
            let mag = pow(10, floor(log10(raw)))
            let gridStep = [1.0, 2, 5, 10].map { $0 * mag }.first { $0 >= raw } ?? raw
            var grid = Path()
            let span = window * 3
            var g = (focus.x - span).rounded(.down) - (focus.x - span).truncatingRemainder(dividingBy: gridStep)
            while g < focus.x + span { grid.move(to: pt(SIMD2(g, focus.y - span))); grid.addLine(to: pt(SIMD2(g, focus.y + span))); g += gridStep }
            g = (focus.y - span).rounded(.down) - (focus.y - span).truncatingRemainder(dividingBy: gridStep)
            while g < focus.y + span { grid.move(to: pt(SIMD2(focus.x - span, g))); grid.addLine(to: pt(SIMD2(focus.x + span, g))); g += gridStep }
            ctx.stroke(grid, with: .color(ink.opacity(0.06)), lineWidth: 1)

            var body = Path()
            for loop in sec.loops where loop.count > 2 {
                body.move(to: pt(loop[0]))
                for q in loop.dropFirst() { body.addLine(to: pt(q)) }
                body.closeSubpath()
            }
            ctx.fill(body, with: .color(tint.opacity(0.78)), style: FillStyle(eoFill: true))
            ctx.stroke(body, with: .color(ink.opacity(0.75)), lineWidth: 1.5)

            // what the treatment removes, and the new outline
            let (cut, line) = Self.shapes(e, pt)
            ctx.fill(cut, with: .color(Color(red: 1, green: 0.13, blue: 0.2).opacity(0.32)))
            var lit = ctx
            if glow > 0 { lit.addFilter(.shadow(color: accent.opacity(0.8 * glow), radius: 6)) }
            lit.stroke(line, with: .color(accent), style: StrokeStyle(lineWidth: 3, lineCap: .round, lineJoin: .round))

            // face names and the corner angle
            func label(_ text: String, _ at: SIMD2<Double>, _ color: Color) {
                ctx.draw(Text(text).font(bold).foregroundColor(color), at: pt(at))
            }
            label(faceA, sec.dirA * window * 0.85 + SIMD2(0, window * 0.07), accent3)
            label(faceB, sec.dirB * window * 0.85 + SIMD2(window * 0.1, 0), accent3)
            label(String(format: "%.0f°", sec.angle), bis * window * 0.62, ink.opacity(0.6))
            Self.dimensions(ctx, e, window, pt, accent2, small)
        }
        .accessibilityElement()
        .accessibilityLabel(L("Cut through the edge"))
    }

    private nonisolated static func shapes(_ e: AngleEdit, _ pt: (SIMD2<Double>) -> CGPoint) -> (Path, Path) {
        let sec = e.section, a = sec.dirA, b = sec.dirB, phi = e.phi
        var cut = Path(), line = Path()
        switch (e.treatment, e.rounding) {
        case (.rounded, .outbound):
            let t = e.radius / max(0.05, tan(phi / 2))
            let c = simd_normalize(a + b) * (e.radius / max(0.05, sin(phi / 2)))
            let ta = a * t, tb = b * t
            let arc = arcPoints(c, from: ta, to: tb, radius: e.radius, towards: .zero)
            cut.move(to: pt(.zero)); cut.addLine(to: pt(ta)); for q in arc { cut.addLine(to: pt(q)) }; cut.closeSubpath()
            line.move(to: pt(ta)); for q in arc { line.addLine(to: pt(q)) }
        case (.rounded, .inbound):
            let arc = arcPoints(.zero, from: a * e.radius, to: b * e.radius, radius: e.radius, towards: simd_normalize(a + b))
            cut.move(to: pt(.zero)); cut.addLine(to: pt(a * e.radius)); for q in arc { cut.addLine(to: pt(q)) }; cut.closeSubpath()
            line.move(to: pt(a * e.radius)); for q in arc { line.addLine(to: pt(q)) }
        case (.angled, _):
            let pa = a * e.legs.x, pb = b * e.legs.y
            cut.move(to: pt(.zero)); cut.addLine(to: pt(pa)); cut.addLine(to: pt(pb)); cut.closeSubpath()
            if e.roundedCorners {
                let rc = e.cornerRadius, bevel = simd_normalize(pb - pa)
                line.move(to: pt(pa + a * rc * 1.2))
                line.addQuadCurve(to: pt(pa + bevel * rc), control: pt(pa))
                line.addLine(to: pt(pb - bevel * rc))
                line.addQuadCurve(to: pt(pb + b * rc * 1.2), control: pt(pb))
            } else {
                line.move(to: pt(pa)); line.addLine(to: pt(pb))
            }
        }
        return (cut, line)
    }

    // Points of the arc around `c` from `p` to `q`, bending the way `towards` lies.
    private nonisolated static func arcPoints(_ c: SIMD2<Double>, from p: SIMD2<Double>, to q: SIMD2<Double>, radius: Double, towards: SIMD2<Double>) -> [SIMD2<Double>] {
        let a0 = atan2(p.y - c.y, p.x - c.x)
        var a1 = atan2(q.y - c.y, q.x - c.x)
        var d = a1 - a0
        while d > .pi { d -= 2 * .pi }
        while d < -.pi { d += 2 * .pi }
        let mid = c + SIMD2(cos(a0 + d / 2), sin(a0 + d / 2)) * radius
        let alt = c + SIMD2(cos(a0 + d / 2 + .pi), sin(a0 + d / 2 + .pi)) * radius
        if simd_distance(alt, towards) < simd_distance(mid, towards) { d += d > 0 ? -2 * .pi : 2 * .pi }
        a1 = a0 + d
        return (0...32).map { i in
            let t = a0 + (a1 - a0) * Double(i) / 32
            return c + SIMD2(cos(t), sin(t)) * radius
        }
    }

    private nonisolated static func dimensions(_ ctx: GraphicsContext, _ e: AngleEdit, _ window: Double, _ pt: (SIMD2<Double>) -> CGPoint,
                                               _ color: Color, _ font: Font) {
        let sec = e.section, a = sec.dirA, b = sec.dirB
        func text(_ s: String, _ at: SIMD2<Double>) {
            ctx.draw(Text(s).font(font).foregroundColor(color), at: pt(at))
        }
        func mm(_ v: Double) -> String { String(format: "%.2f", (v * 100).rounded() / 100) }
        let off = window * 0.08
        switch e.treatment {
        case .angled:
            let pa = a * e.legs.x, pb = b * e.legs.y
            let outward = -simd_normalize(a + b)
            text("A " + mm(e.legs.x), pa * 0.5 + SIMD2(0, off))
            text("B " + mm(e.legs.y), pb * 0.5 + SIMD2(off * 1.4, 0))
            text(mm(e.hypotenuse), (pa + pb) / 2 + outward * off * 1.6)
            text(String(format: "%.1f°", e.angleA), pa + a * off * 2.2 + SIMD2(0, -off))
        case .rounded:
            let c = e.rounding == .outbound ? simd_normalize(a + b) * (e.radius / max(0.05, sin(e.phi / 2))) : .zero
            text("R " + mm(e.radius), c + simd_normalize(a + b) * off * (e.rounding == .outbound ? -1.4 : 2.4))
        }
    }
}

// Controls of the angle editor.
struct AnglePanel: View {
    @Environment(Workbench.self) private var lib
    let edit: AngleEdit
    @State private var more = false

    var body: some View {
        let e = edit
        VStack(alignment: .leading, spacing: 10) {
            VStack(alignment: .leading, spacing: 2) {
                Text(L("Work with angles")).font(.ui(size: 15, weight: .bold, design: .rounded)).foregroundStyle(Ink.text).halo(lib.accent, 8)
                Text(L("{n} picks", ["n": e.picks.count]) + " · " + String(format: "%.0f°", e.section.angle))
                    .font(.ui(size: 11, weight: .medium, design: .rounded)).foregroundStyle(Ink.text.opacity(0.5))
            }
            PictureSegmented(options: [AngleEdit.Treatment.rounded, .angled], current: e.treatment,
                             label: { $0 == .rounded ? L("Rounded corner") : L("Angled corner") },
                             picture: { $0 == .rounded ? .outbound : .bevel(45) }) { t in lib.updateAngles { $0.treatment = t } }
            ZStack(alignment: .topLeading) {
                if e.treatment == .rounded {
                    rounded(e).transition(.move(edge: .leading).combined(with: .opacity))
                } else {
                    angled(e).transition(.move(edge: .trailing).combined(with: .opacity))
                }
            }
            .clipped()
            HStack(spacing: 8) {
                Button(L("Apply")) { lib.applyAngles() }
                    .buttonStyle(PillStyle(tint: lib.accent2))
                ToolButton(icon: "chevron.up", title: L("More ways to apply"), tint: lib.accent2, lit: more, size: 32) {
                    withAnimation(Neon.glide) { more.toggle() }
                }
                Button(L("Cancel")) { lib.closeAngles() }
                    .buttonStyle(PillStyle(tint: Neon.red))
            }
            .overlay(alignment: .top) {
                if more {
                    VStack(spacing: 6) {
                        Button(L("Apply to the whole shape")) { lib.applyAngles(whole: true) }
                            .buttonStyle(PillStyle(tint: lib.accent2))
                        Button(L("Apply to all selected shapes")) { lib.applyAngles(everyShape: true) }
                            .buttonStyle(PillStyle(tint: lib.accent2))
                            .disabled(lib.selection.count < 2)
                            .opacity(lib.selection.count < 2 ? 0.35 : 1)
                    }
                    .padding(8)
                    .glassBar(14)
                    .offset(y: -96)
                    .transition(.menu)
                }
            }
            Text(L("Esc cancels · Enter applies")).font(.ui(size: 10.5, weight: .medium, design: .rounded)).foregroundStyle(Ink.text.opacity(0.4))
        }
        .padding(14)
        .frame(width: 330)
        .glassBar(20)
        .animation(.spring(response: 0.42, dampingFraction: 0.84), value: e.treatment)
    }

    @ViewBuilder private func rounded(_ e: AngleEdit) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            PictureSegmented(options: [AngleEdit.Rounding.outbound, .inbound], current: e.rounding,
                             label: { $0 == .outbound ? L("Outward") : L("Inward") },
                             picture: { $0 == .outbound ? .outbound : .inbound }) { r in
                lib.updateAngles {
                    $0.rounding = r
                    $0.radius = min($0.radius, $0.radiusRange.upperBound)
                }
            }
            StepperLine(title: L("Radius"), value: e.radius, range: e.radiusRange) { v in lib.updateAngles { $0.radius = v } }
        }
    }

    @ViewBuilder private func angled(_ e: AngleEdit) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            AnglePresets(edit: e)
            SettingLine(title: L("Depth"), detail: L("Square to the bevel")) {
                MMField(value: e.depth, unit: L("mm"), range: 0.01...1000, width: 58) { v in lib.updateAngles { $0.setDepth(v) } }
            }
            HStack(spacing: 6) {
                leg("A", e.legs.x) { v in lib.updateAngles { $0.setLeg(0, v) } }
                leg("B", e.legs.y) { v in lib.updateAngles { $0.setLeg(1, v) } }
                leg("⟋", e.hypotenuse) { v in lib.updateAngles { $0.setHypotenuse(v) } }
            }
            .help(L("Along face A, along face B and across the bevel · each keeps the others in step"))
            PictureSegmented(options: [false, true], current: e.roundedCorners,
                             label: { $0 ? L("Softened") : L("Sharp") },
                             picture: { $0 ? .softBevel(e.angleA) : .bevel(e.angleA) }) { v in lib.updateAngles { $0.roundedCorners = v } }
        }
    }

    private func leg(_ name: String, _ value: Double, _ set: @escaping (Double) -> Void) -> some View {
        VStack(spacing: 3) {
            Text(name).font(.ui(size: 10.5, weight: .black, design: .rounded)).foregroundStyle(lib.accent3)
            MMField(value: value, range: 0.01...1000, width: 62, set: set)
        }
        .frame(maxWidth: .infinity)
    }
}

// −60° · −30° · the editable middle angle · +30° · +60°: plus leans the bevel to face A, minus to face B.
struct AnglePresets: View {
    @Environment(Workbench.self) private var lib
    let edit: AngleEdit

    var body: some View {
        let e = edit
        let side = 180 - e.section.angle
        HStack(spacing: 3) {
            preset(-60, e, side)
            preset(-30, e, side)
            VStack(spacing: 2) {
                CornerGlyph(look: .bevel(e.angleA), tint: Ink.text).frame(width: 40, height: 28)
                MMField(value: e.angleA, unit: "°", range: e.angleRange, width: 44) { v in lib.updateAngles { $0.setAngle(v) } }
            }
            .frame(maxWidth: .infinity)
            .padding(.vertical, 5)
            .background(RoundedRectangle(cornerRadius: 9, style: .continuous).fill(lib.accent.opacity(0.1)))
            preset(30, e, side)
            preset(60, e, side)
        }
        .padding(4)
        .background(RoundedRectangle(cornerRadius: 12, style: .continuous).fill(Ink.text.opacity(0.06)))
    }

    private func preset(_ signed: Double, _ e: AngleEdit, _ side: Double) -> some View {
        let toA = signed > 0 ? signed : side - abs(signed)
        let ok = e.angleRange.contains(toA)
        let on = abs(e.angleA - toA) < 0.05
        return VStack(spacing: 2) {
            CornerGlyph(look: .bevel(toA), tint: on ? .black : Ink.text).frame(width: 34, height: 26)
            Text((signed > 0 ? "+" : "−") + String(format: "%.0f°", abs(signed)))
                .font(.ui(size: 10.5, weight: .bold, design: .rounded))
                .foregroundStyle(on ? Color.black : Ink.text.opacity(0.75))
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 5)
        .background {
            if on { RoundedRectangle(cornerRadius: 9, style: .continuous).fill(lib.accent).glow(lib.accent, 10) }
        }
        .opacity(ok ? 1 : 0.3)
        .contentShape(Rectangle())
        .onTapGesture { if ok { lib.updateAngles { $0.setAngle(toA) } } }
        .help(signed > 0 ? L("Leans to face A") : L("Leans to face B"))
        .accessibilityElement(children: .combine)
        .accessibilityAddTraits(on ? [.isButton, .isSelected] : .isButton)
        .accessibilityAction(.default) { if ok { lib.updateAngles { $0.setAngle(toA) } } }
    }
}
