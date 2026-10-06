import SwiftUI
import AppKit

// MARK: - Shared look · begin
// Identical in BPlayer, BSynth and Bcad: keep the three copies the same.
// Styles, light, motion, theme, interface and font scale, languages, controls and settings rows. Each app plugs in through
// `DesignHost`, which `Skin` reads live (so colours follow the app's state without any syncing).

struct SkinSettings: Codable, Equatable {
    var simplified = false
    var reduceMotion = false
    var dark = true
    var scale = 1.0
    var fontSize = 13.0

    static let scales: ClosedRange<Double> = 0.8...1.2
    static let fontSizes: ClosedRange<Double> = 11...16

    init() {}

    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        simplified = (try? c.decode(Bool.self, forKey: .simplified)) ?? false
        reduceMotion = (try? c.decode(Bool.self, forKey: .reduceMotion)) ?? false
        dark = (try? c.decode(Bool.self, forKey: .dark)) ?? true
        scale = min(1.2, max(0.8, (try? c.decode(Double.self, forKey: .scale)) ?? 1))
        fontSize = min(16, max(11, (try? c.decode(Double.self, forKey: .fontSize)) ?? 13))
    }
}

// What the shared settings rows ask of the app.
@MainActor
protocol DesignHost: AnyObject {
    // The chosen style, and the one whose colours are showing (BPlayer: a playing playlist's own style wins).
    var style: Style { get }
    var palette: Style { get }
    var brightness: Double { get }
    func setLanguage(_ id: String)
    func setStyle(_ s: Style)
    func setBrightness(_ v: Double)
    func setLook(_ change: (inout SkinSettings) -> Void)
    func endCapture()
}

@Observable @MainActor
final class Skin {
    static let shared = Skin()
    var values = SkinSettings()
    @ObservationIgnored weak var host: (any DesignHost)?

    var palette: Style { host?.palette ?? .classic }
    var brightness: Double { host?.brightness ?? 0.8 }
    var simplified: Bool { values.simplified }
    var reduceMotion: Bool { values.reduceMotion }
    var dark: Bool { values.dark }
    var scale: Double { values.scale }
    var fontScale: Double { values.fontSize / 13 }
    var rtl: Bool { L10n.shared.id == "ar" }
    var direction: LayoutDirection { rtl ? .rightToLeft : .leftToRight }
    var scheme: ColorScheme { values.dark ? .dark : .light }

    // Simplified UI is neutral monochrome; `mark` keeps the style colour as a thin accent on the selected item.
    var accent: Color { simplified ? Ink.text.opacity(0.82) : palette.accents.0 }
    var accent2: Color { simplified ? Ink.text.opacity(0.5) : palette.accents.1 }
    var accent3: Color { simplified ? Ink.text.opacity(0.66) : palette.accents.2 }
    var mark: Color { palette.accents.0 }
    // Light interactive elements give off: the brightness setting, none in Simplified UI.
    var glow: Double { simplified ? 0 : brightness }

    func apply(_ v: SkinSettings) {
        values = v
        let name: NSAppearance.Name = v.dark ? .darkAqua : .aqua
        let app = NSApplication.shared
        if app.appearance?.name != name { app.appearance = NSAppearance(named: name) }
    }
}

extension NSAppearance {
    var isDark: Bool { bestMatch(from: [.darkAqua, .aqua]) == .darkAqua }
}

// Neutral colours that follow the theme; style colours never change with it.
enum Ink {
    static let text = Color(nsColor: NSColor(name: nil) { $0.isDark ? .white : NSColor(srgbRed: 0.08, green: 0.09, blue: 0.13, alpha: 1) })
    static let void = Color(nsColor: NSColor(name: nil) {
        $0.isDark ? NSColor(srgbRed: 0.02, green: 0.03, blue: 0.07, alpha: 1) : NSColor(srgbRed: 0.93, green: 0.94, blue: 0.97, alpha: 1)
    })
}

extension Font {
    // Every interface font goes through here so the Font size setting reaches all text.
    @MainActor static func ui(size: CGFloat, weight: Font.Weight? = nil, design: Font.Design? = nil) -> Font {
        .system(size: size * CGFloat(Skin.shared.fontScale), weight: weight, design: design)
    }
}

enum Neon {
    static let cyan = Color(red: 0.0, green: 0.94, blue: 1.0)
    static let magenta = Color(red: 1.0, green: 0.16, blue: 0.43)
    static let amber = Color(red: 1.0, green: 0.69, blue: 0.0)
    static let red = Color(red: 1.0, green: 0.13, blue: 0.2)
    static let green = Color(red: 0.22, green: 1.0, blue: 0.45)
    static let quick = Animation.easeOut(duration: 0.14)
    // Reduce animations swaps every spring for a short ease; things still appear and move, without the flourish.
    @MainActor static var calm: Bool { Skin.shared.reduceMotion }
    @MainActor static var spring: Animation { calm ? quick : .spring(response: 0.35, dampingFraction: 0.62) }
    @MainActor static var glide: Animation { calm ? quick : .spring(response: 0.55, dampingFraction: 0.82) }
    @MainActor static var lift: Animation { calm ? quick : .spring(response: 0.5, dampingFraction: 0.86) }
    @MainActor static var settle: Animation { calm ? quick : .spring(response: 0.55, dampingFraction: 0.48) }
    @MainActor static var pop: Animation { calm ? quick : .spring(response: 0.28, dampingFraction: 0.42) }
    @MainActor static func hover(_ on: Bool) -> Animation { calm ? quick : (on ? spring : settle) }
}

enum Style: String, Codable, CaseIterable, Identifiable {
    case classic, mango, strawberry, grapes, blueberry, dragonfruit, coconut, lime, cinnamon, blackberry
    var id: String { rawValue }
    @MainActor var label: String { L(name) }

    var name: String {
        switch self {
        case .classic: "Classic"
        case .mango: "Mango"
        case .strawberry: "Strawberry"
        case .grapes: "Grapes"
        case .blueberry: "Blueberry"
        case .dragonfruit: "Dragonfruit"
        case .coconut: "Coconut"
        case .lime: "Lime"
        case .cinnamon: "Cinnamon"
        case .blackberry: "Blackberry"
        }
    }

    var accents: (Color, Color, Color) {
        switch self {
        case .classic: (Neon.cyan, Neon.magenta, Neon.amber)
        case .mango: (Color(red: 1.0, green: 0.6, blue: 0.05), Color(red: 1.0, green: 0.25, blue: 0.05), Color(red: 1.0, green: 0.85, blue: 0.2))
        case .strawberry: (Color(red: 1.0, green: 0.2, blue: 0.35), Color(red: 0.85, green: 0.05, blue: 0.15), Color(red: 1.0, green: 0.75, blue: 0.8))
        case .grapes: (Color(red: 0.65, green: 0.25, blue: 1.0), Color(red: 0.35, green: 0.05, blue: 0.6), Color(red: 0.9, green: 0.4, blue: 0.95))
        case .blueberry: (Color(red: 0.25, green: 0.45, blue: 1.0), Color(red: 0.2, green: 0.1, blue: 0.55), Color(red: 0.55, green: 0.7, blue: 1.0))
        case .dragonfruit: (Color(red: 1.0, green: 0.15, blue: 0.65), Color(red: 0.4, green: 0.9, blue: 0.3), Color(red: 1.0, green: 0.6, blue: 0.85))
        case .coconut: (Color(red: 0.93, green: 0.92, blue: 0.88), Color(red: 0.55, green: 0.35, blue: 0.2), Color(red: 0.72, green: 0.88, blue: 1.0))
        case .lime: (Color(red: 0.55, green: 1.0, blue: 0.15), Color(red: 0.15, green: 0.55, blue: 0.1), Color(red: 0.85, green: 1.0, blue: 0.3))
        case .cinnamon: (Color(red: 0.85, green: 0.45, blue: 0.1), Color(red: 0.55, green: 0.18, blue: 0.05), Color(red: 0.95, green: 0.75, blue: 0.45))
        case .blackberry: (Color(red: 0.5, green: 0.2, blue: 0.65), Color(red: 0.55, green: 0.05, blue: 0.35), Color(red: 0.4, green: 0.3, blue: 0.85))
        }
    }
}

enum Languages {
    // English, Ukrainian, then by native name with Latin scripts first; Chinese and Japanese last.
    static let all: [(id: String, name: String, flag: String)] = [
        ("en", "English", "🇬🇧"), ("uk", "Українська", "🇺🇦"), ("cs", "Čeština", "🇨🇿"), ("de", "Deutsch", "🇩🇪"),
        ("es", "Español", "🇪🇸"), ("fr", "Français", "🇫🇷"), ("it", "Italiano", "🇮🇹"), ("hu", "Magyar", "🇭🇺"),
        ("nl", "Nederlands", "🇳🇱"), ("nb", "Norsk", "🇳🇴"), ("pl", "Polski", "🇵🇱"), ("ro", "Română", "🇷🇴"),
        ("fi", "Suomi", "🇫🇮"), ("sv", "Svenska", "🇸🇪"), ("kk", "Қазақша", "🇰🇿"), ("ka", "ქართული", "🇬🇪"),
        ("ar", "العربية", "🇸🇦"), ("hi", "हिन्दी", "🇮🇳"), ("zh-Hans", "中文", "🇨🇳"), ("ja", "日本語", "🇯🇵")
    ]

    // Unicode CLDR plural categories for whole numbers.
    static func plural(_ lang: String, _ n: Int) -> String {
        let m10 = n % 10, m100 = n % 100
        switch lang {
        case "uk":
            if m10 == 1 && m100 != 11 { return "one" }
            if (2...4).contains(m10) && !(12...14).contains(m100) { return "few" }
            return "many"
        case "pl":
            if n == 1 { return "one" }
            if (2...4).contains(m10) && !(12...14).contains(m100) { return "few" }
            return "many"
        case "cs": return n == 1 ? "one" : (2...4).contains(n) ? "few" : "other"
        case "ro": return n == 1 ? "one" : (n == 0 || (2...19).contains(m100)) ? "few" : "other"
        case "ar":
            switch n {
            case 0: return "zero"
            case 1: return "one"
            case 2: return "two"
            default: return (3...10).contains(m100) ? "few" : (11...99).contains(m100) ? "many" : "other"
            }
        case "fr", "hi": return n == 0 || n == 1 ? "one" : "other"
        case "zh-Hans", "ja": return "other"
        default: return n == 1 ? "one" : "other"
        }
    }
}

// Interface text in 20 languages. The English text is the key; i18n.json holds the rest.
@Observable @MainActor
final class L10n {
    static let shared = L10n()
    var id = "en"

    nonisolated(unsafe) static let table: [String: [String: Any]] = {
        let url = Bundle.main.url(forResource: "i18n", withExtension: "json")
            ?? ProcessInfo.processInfo.environment["APP_STRINGS"].map { URL(fileURLWithPath: $0) }
        guard let url, let data = try? Data(contentsOf: url),
              let o = try? JSONSerialization.jsonObject(with: data) as? [String: [String: Any]] else { return [:] }
        return o
    }()

    static func valid(_ id: String?) -> String { Languages.all.contains { $0.id == id } ? id! : "en" }

    nonisolated static func text(_ key: String, _ lang: String, _ args: [String: Any]) -> String {
        let entry = table[key]
        var v: Any? = entry?[lang] ?? entry?["en"]
        if let forms = v as? [String: String] {
            let n = args["n"] as? Int ?? 0
            v = forms[Languages.plural(lang, n)] ?? forms["other"] ?? forms["many"] ?? forms.values.first
        }
        var out = v as? String ?? key
        for (k, a) in args { out = out.replacingOccurrences(of: "{\(k)}", with: "\(a)") }
        return out
    }
}

@MainActor func L(_ key: String, _ args: [String: Any] = [:]) -> String { L10n.text(key, L10n.shared.id, args) }

// Whether a name is how `key` reads in any language: one Bcad gave (it follows its shape's kind, whatever the language
// is now), not one typed in.
@MainActor func inAnyLanguage(_ name: String, _ key: String, _ args: [String: Any] = [:]) -> Bool {
    name == L10n.text(key, "en", args) || Languages.all.contains { L10n.text(key, $0.id, args) == name }
}

// Lays the interface out smaller or larger and draws it scaled, so every control, gap and text follows Interface scale.
// Arabic mirrors inside; the scaling frame itself always runs left to right.
struct ScaledUI<Content: View>: View {
    @ViewBuilder let content: Content

    var body: some View {
        let look = Skin.shared
        GeometryReader { g in
            let s = CGFloat(look.scale)
            content
                .environment(\.layoutDirection, look.direction)
                .frame(width: g.size.width / s, height: g.size.height / s)
                .scaleEffect(s, anchor: UnitPoint(x: 0, y: 0))
                .frame(width: g.size.width, height: g.size.height, alignment: .topLeading)
        }
        .environment(\.layoutDirection, .leftToRight)
    }
}

// Interface scale for content that sizes itself (a menu-bar panel): lays out at size ÷ scale and reports size × scale.
struct ScaledFit: Layout {
    let scale: CGFloat

    func sizeThatFits(proposal: ProposedViewSize, subviews: Subviews, cache: inout ()) -> CGSize {
        guard let v = subviews.first else { return .zero }
        let size = v.sizeThatFits(ProposedViewSize(width: proposal.width.map { $0 / scale }, height: proposal.height.map { $0 / scale }))
        return CGSize(width: size.width * scale, height: size.height * scale)
    }

    func placeSubviews(in bounds: CGRect, proposal: ProposedViewSize, subviews: Subviews, cache: inout ()) {
        subviews.first?.place(at: bounds.origin, anchor: .topLeading,
                              proposal: ProposedViewSize(width: bounds.width / scale, height: bounds.height / scale))
    }
}

extension View {
    @MainActor func scaledFit() -> some View {
        let skin = Skin.shared
        let s = CGFloat(skin.scale)
        return ScaledFit(scale: s) {
            self.environment(\.layoutDirection, skin.direction)
                .scaleEffect(s, anchor: UnitPoint(x: 0, y: 0))
        }
        .environment(\.layoutDirection, .leftToRight)
    }
}

extension View {
    // The theme and motion settings for a window's content.
    func skinEnvironment() -> some View {
        modifier(SkinEnvironment())
    }
}

struct SkinEnvironment: ViewModifier {
    func body(content: Content) -> some View {
        let look = Skin.shared
        content
            .preferredColorScheme(look.scheme)
            .symbolEffectsRemoved(look.reduceMotion)
    }
}

struct Glow: ViewModifier {
    let color: Color
    let r: CGFloat
    let fixed: Bool

    func body(content: Content) -> some View {
        let look = Skin.shared
        let g = look.simplified ? 0 : (fixed ? 1 : look.brightness)
        return content
            .shadow(color: color.opacity(r > 0 ? 0.85 * g : 0), radius: r * 0.22)
            .shadow(color: color.opacity(r > 0 ? 0.5 * g : 0), radius: r * 0.6)
            .shadow(color: color.opacity(r > 0 ? 0.3 * g : 0), radius: r)
    }
}

extension View {
    // Light an interactive element gives off; the brightness setting scales it.
    func glow(_ c: Color, _ r: CGFloat) -> some View { modifier(Glow(color: c, r: r, fixed: false)) }
    // Decorative light (titles, status lights, artwork, overlays); brightness never changes it.
    func halo(_ c: Color, _ r: CGFloat) -> some View { modifier(Glow(color: c, r: r, fixed: true)) }
}

struct BlurFade: ViewModifier {
    let radius: CGFloat
    let opacity: Double
    func body(content: Content) -> some View { content.blur(radius: radius).opacity(opacity) }
}

extension AnyTransition {
    @MainActor static var haze: AnyTransition {
        Neon.calm ? .opacity : .modifier(active: BlurFade(radius: 18, opacity: 0), identity: BlurFade(radius: 0, opacity: 1))
    }
    @MainActor static var menu: AnyTransition {
        Neon.calm ? .opacity
            : .asymmetric(insertion: .move(edge: .top).combined(with: .haze), removal: .modifier(active: Fold(amount: 0), identity: Fold(amount: 1)))
    }
}

struct RippleRing<S: InsettableShape>: View {
    let shape: S
    let tint: Color
    let done: () -> Void
    @State private var go = false

    var body: some View {
        shape.strokeBorder(tint, lineWidth: go ? 0.5 : 2.5)
            .glow(tint, 6)
            .scaleEffect(go ? 1.45 : 1)
            .opacity(go ? 0 : 0.9)
            .onAppear {
                withAnimation(.easeOut(duration: 0.55)) { go = true }
                Task {
                    try? await Task.sleep(for: .milliseconds(620))
                    done()
                }
            }
    }
}

struct Ripple<S: InsettableShape>: ViewModifier {
    let trigger: Int
    let shape: S
    let tint: Color
    @State private var rings: [Int] = []

    func body(content: Content) -> some View {
        content
            .overlay {
                ZStack {
                    ForEach(rings, id: \.self) { id in
                        RippleRing(shape: shape, tint: tint) { rings.removeAll { $0 == id } }
                    }
                }
                .allowsHitTesting(false)
            }
            .onChange(of: trigger) { if !Neon.calm { rings.append(trigger) } }
    }
}

struct PressWave<S: Shape>: ViewModifier {
    let trigger: Int
    let shape: S
    let tint: Color
    @State private var waves: [Int] = []

    func body(content: Content) -> some View {
        content
            .overlay {
                GeometryReader { g in
                    ZStack {
                        ForEach(waves, id: \.self) { id in
                            WaveCircle(size: max(g.size.width, g.size.height) * 1.3, tint: tint) { waves.removeAll { $0 == id } }
                        }
                    }
                    .frame(width: g.size.width, height: g.size.height)
                }
                .clipShape(shape)
                .allowsHitTesting(false)
            }
            .onChange(of: trigger) { if !Neon.calm { waves.append(trigger) } }
    }
}

struct WaveCircle: View {
    let size: CGFloat
    let tint: Color
    let done: () -> Void
    @State private var go = false

    var body: some View {
        Circle()
            .fill(tint.opacity(0.45))
            .frame(width: size, height: size)
            .scaleEffect(go ? 1 : 0.05)
            .opacity(go ? 0 : 1)
            .onAppear {
                withAnimation(.easeOut(duration: 0.5)) { go = true }
                Task {
                    try? await Task.sleep(for: .milliseconds(560))
                    done()
                }
            }
    }
}

struct Sheen<S: Shape>: ViewModifier {
    let active: Bool
    let shape: S
    @State private var x: CGFloat = -1.2

    func body(content: Content) -> some View {
        content
            .overlay {
                GeometryReader { g in
                    LinearGradient(colors: [.clear, .white.opacity(0.5), .clear], startPoint: .leading, endPoint: .trailing)
                        .frame(width: g.size.width * 0.35, height: g.size.height * 2)
                        .rotationEffect(.degrees(20))
                        .offset(x: x * g.size.width, y: -g.size.height / 2)
                }
                .clipShape(shape)
                .blendMode(.plusLighter)
                .allowsHitTesting(false)
            }
            .onChange(of: active) {
                guard active, !Neon.calm else { return }
                x = -0.5
                withAnimation(.easeOut(duration: 0.65)) { x = 1.3 }
            }
    }
}

struct Cascade: ViewModifier {
    let index: Int
    @State private var shown = false

    func body(content: Content) -> some View {
        let on = shown || Neon.calm
        content
            .opacity(on ? 1 : 0)
            .offset(y: on ? 0 : -10)
            .scaleEffect(on ? 1 : 0.96, anchor: .top)
            .onAppear { withAnimation(Neon.spring.delay(Double(min(index, 12)) * 0.03)) { shown = true } }
    }
}

struct RowMotion: ViewModifier {
    let hover: Bool
    let tint: Color
    let flash: Int
    var radius: CGFloat = 9
    @State private var lit = false

    func body(content: Content) -> some View {
        content
            .overlay(alignment: .leading) {
                Capsule()
                    .fill(tint)
                    .frame(width: 3, height: hover ? 16 : 0)
                    .glow(tint, hover ? 6 : 0)
                    .padding(.leading, 2)
                    .allowsHitTesting(false)
            }
            .background(RoundedRectangle(cornerRadius: radius, style: .continuous).fill(tint.opacity(lit ? 0.3 : 0)).allowsHitTesting(false))
            .onChange(of: flash) {
                guard !Neon.calm else { return }
                withAnimation(.easeOut(duration: 0.08)) { lit = true }
                Task {
                    try? await Task.sleep(for: .milliseconds(110))
                    withAnimation(.easeOut(duration: 0.5)) { lit = false }
                }
            }
    }
}

struct Pop: ViewModifier {
    let trigger: Int
    var scale: CGFloat = 1.3
    @State private var up = false

    func body(content: Content) -> some View {
        content
            .scaleEffect(up ? scale : 1)
            .onChange(of: trigger) {
                guard !Neon.calm else { return }
                withAnimation(.easeOut(duration: 0.09)) { up = true }
                Task {
                    try? await Task.sleep(for: .milliseconds(100))
                    withAnimation(Neon.pop) { up = false }
                }
            }
    }
}

struct Fold: ViewModifier {
    let amount: Double

    func body(content: Content) -> some View {
        content
            .scaleEffect(x: 1, y: 0.35 + 0.65 * amount, anchor: .top)
            .opacity(amount)
            .blur(radius: (1 - amount) * 10)
    }
}

// A looping highlight (a field waiting for a key, an open dropdown): pulses, or holds still with Reduce animations.
struct Blink: ViewModifier {
    let active: Bool
    @Binding var on: Bool

    func body(content: Content) -> some View {
        content.onChange(of: [active, Neon.calm], initial: true) {
            if active && !Neon.calm {
                on = false
                withAnimation(.easeInOut(duration: 0.5).repeatForever(autoreverses: true)) { on = true }
            } else {
                var t = Transaction(animation: active ? nil : .easeOut(duration: 0.2))
                t.disablesAnimations = active
                withTransaction(t) { on = active }
            }
        }
    }
}

extension View {
    func ripple<S: InsettableShape>(_ trigger: Int, shape: S, tint: Color) -> some View { modifier(Ripple(trigger: trigger, shape: shape, tint: tint)) }
    func pressWave<S: Shape>(_ trigger: Int, shape: S, tint: Color) -> some View { modifier(PressWave(trigger: trigger, shape: shape, tint: tint)) }
    func sheen<S: Shape>(_ active: Bool, shape: S) -> some View { modifier(Sheen(active: active, shape: shape)) }
    func cascade(_ index: Int) -> some View { modifier(Cascade(index: index)) }
    func glowRoom(_ overflow: Bool, _ room: CGFloat = 24) -> some View {
        contentMargins(.horizontal, room, for: .scrollContent)
            .scrollClipDisabled(!overflow)
            .padding(.horizontal, -room)
    }
    func rowMotion(_ hover: Bool, tint: Color, flash: Int, radius: CGFloat = 9) -> some View { modifier(RowMotion(hover: hover, tint: tint, flash: flash, radius: radius)) }
    func pop(_ trigger: Int, scale: CGFloat = 1.3) -> some View { modifier(Pop(trigger: trigger, scale: scale)) }
    func blink(_ active: Bool, _ on: Binding<Bool>) -> some View { modifier(Blink(active: active, on: on)) }
}

enum Keys {
    static let codes: [UInt16: String] = {
        var m: [UInt16: String] = [:]
        let letters: [(UInt16, String)] = [(0, "A"), (1, "S"), (2, "D"), (3, "F"), (4, "H"), (5, "G"), (6, "Z"), (7, "X"), (8, "C"), (9, "V"), (11, "B"),
                                           (12, "Q"), (13, "W"), (14, "E"), (15, "R"), (16, "Y"), (17, "T"), (31, "O"), (32, "U"), (34, "I"), (35, "P"),
                                           (37, "L"), (38, "J"), (40, "K"), (45, "N"), (46, "M")]
        for (k, l) in letters { m[k] = "Key" + l }
        let digits: [(UInt16, String)] = [(29, "0"), (18, "1"), (19, "2"), (20, "3"), (21, "4"), (23, "5"), (22, "6"), (26, "7"), (28, "8"), (25, "9")]
        for (k, d) in digits { m[k] = "Digit" + d }
        let other: [(UInt16, String)] = [(24, "Equal"), (27, "Minus"), (30, "BracketRight"), (33, "BracketLeft"), (39, "Quote"), (41, "Semicolon"),
                                         (42, "Backslash"), (43, "Comma"), (44, "Slash"), (47, "Period"), (50, "Backquote"), (49, "Space"), (36, "Enter"),
                                         (76, "Enter"), (48, "Tab"), (51, "Backspace"), (117, "Delete"), (123, "ArrowLeft"), (124, "ArrowRight"),
                                         (125, "ArrowDown"), (126, "ArrowUp"), (115, "Home"), (119, "End"), (116, "PageUp"), (121, "PageDown"),
                                         (122, "F1"), (120, "F2"), (99, "F3"), (118, "F4"), (96, "F5"), (97, "F6"), (98, "F7"), (100, "F8"),
                                         (101, "F9"), (109, "F10"), (103, "F11"), (111, "F12")]
        for (k, o) in other { m[k] = o }
        return m
    }()

    @MainActor static func label(_ code: String) -> String {
        if ["Space", "Enter", "Tab", "Home", "End", "PageUp", "PageDown"].contains(code) { return L(code == "PageUp" ? "PgUp" : code == "PageDown" ? "PgDn" : code) }
        if code.hasPrefix("Key") { return String(code.dropFirst(3)) }
        if code.hasPrefix("Digit") { return String(code.dropFirst(5)) }
        let named = ["Equal": "=", "Minus": "-", "BracketRight": "]", "BracketLeft": "[", "Quote": "'", "Semicolon": ";", "Backslash": "\\",
                     "Comma": ",", "Slash": "/", "Period": ".", "Backquote": "`", "Backspace": "⌫", "Delete": "⌦", "ArrowLeft": "←",
                     "ArrowRight": "→", "ArrowDown": "↓", "ArrowUp": "↑"]
        return named[code] ?? code
    }
}

@MainActor
enum MenuText {
    private enum Key { case app(String), kit(table: String, key: String) }
    private final class Box { let key: Key; init(_ key: Key) { self.key = key } }
    private static let tag = UnsafeMutableRawPointer.allocate(byteCount: 1, alignment: 1)
    private static var watch: NSKeyValueObservation?
    // A title set here posts didChangeItem, and menus may answer with changes of their own; those are not retitled again.
    private static var busy = false
    private static let app = Bundle.main.object(forInfoDictionaryKey: "CFBundleName") as? String ?? ProcessInfo.processInfo.processName
    private static let actions: [String: String] = [
        "orderFrontStandardAboutPanel:": "About \(app)", "hide:": "Hide \(app)", "hideOtherApplications:": "Hide Others",
        "unhideAllApplications:": "Show All", "terminate:": "Quit \(app)", "performClose:": "Close", "undo:": "Undo", "redo:": "Redo",
        "cut:": "Cut", "copy:": "Copy", "paste:": "Paste", "pasteAsPlainText:": "Paste and Match Style", "delete:": "Delete",
        "selectAll:": "Select All", "startDictation:": "Start Dictation…", "orderFrontCharacterPalette:": "Emoji & Symbols",
        "performMiniaturize:": "Minimize", "performZoom:": "Zoom", "arrangeInFront:": "Bring All to Front", "toggleTabOverview:": "Show All Tabs",
        "showHelp:": "\(app) Help"
    ]
    // table ▸ language ▸ English key ▸ text; MenuCommands holds the window items, InputManager the Edit menu extras.
    private static let kit: [String: [String: [String: String]]] = {
        var tables: [String: [String: [String: String]]] = [:]
        let bundle = Bundle(for: NSApplication.self)
        for name in ["MenuCommands", "InputManager"] {
            guard let url = bundle.url(forResource: name, withExtension: "loctable"),
                  let all = NSDictionary(contentsOf: url) as? [String: Any] else { continue }
            tables[name] = all.compactMapValues { $0 as? [String: String] }
        }
        return tables
    }()
    private static let launch = Bundle(for: NSApplication.self).preferredLocalizations.first ?? "en"

    static func install() {
        let center = NotificationCenter.default
        for name in [NSMenu.didAddItemNotification, NSMenu.didChangeItemNotification] {
            center.addObserver(forName: name, object: nil, queue: nil) { note in
                MainActor.assumeIsolated {
                    guard let menu = note.object as? NSMenu, let i = note.userInfo?["NSMenuItemIndex"] as? Int, i < menu.numberOfItems else { return }
                    retitle(menu.item(at: i)!, in: menu)
                }
            }
        }
        center.addObserver(forName: NSMenu.didBeginTrackingNotification, object: nil, queue: nil) { _ in MainActor.assumeIsolated { refresh() } }
        // SwiftUI swaps in a freshly built main menu when commands change.
        watch = NSApp.observe(\.mainMenu, options: [.new]) { _, _ in MainActor.assumeIsolated { refresh() } }
        apply()
    }

    static func apply() {
        UserDefaults.standard.set([L10n.shared.id], forKey: "AppleLanguages")
        refresh()
        DispatchQueue.main.async { MainActor.assumeIsolated { refresh() } }
    }

    private static func refresh() {
        guard let main = NSApp.mainMenu else { return }
        walk(main)
    }

    private static func walk(_ menu: NSMenu) {
        for item in menu.items {
            retitle(item, in: menu)
            if let sub = item.submenu { walk(sub) }
        }
    }

    // AppKit's own tables use a few older language codes.
    private static func kitLanguage(_ id: String) -> [String] {
        switch id {
        case "nb": ["nb", "no"]
        case "zh-Hans": ["zh_CN", "zh-Hans"]
        default: [id]
        }
    }

    private static func key(_ item: NSMenuItem, in menu: NSMenu) -> Key? {
        let action = item.action.map(NSStringFromSelector)
        // The app's own commands are titled by SwiftUI through L().
        if action == "menuAction:" { return nil }
        if let sub = item.submenu, menu === NSApp.mainMenu {
            if sub === NSApp.windowsMenu { return .app("Window") }
            if sub === NSApp.helpMenu { return .app("Help") }
            if sub.items.contains(where: { $0.action.map(NSStringFromSelector) == "undo:" }) { return .app("Edit") }
            if let i = menu.items.firstIndex(of: item), i > 0, let before = menu.items[i - 1].submenu,
               before.items.contains(where: { $0.action.map(NSStringFromSelector) == "undo:" }) { return .app("View") }
        }
        if let sub = item.submenu, sub === NSApp.servicesMenu { return .app("Services") }
        if !item.isAlternate, let action {
            if action == "toggleFullScreen:" {
                return .app(NSApp.keyWindow?.styleMask.contains(.fullScreen) == true ? "Exit Full Screen" : "Enter Full Screen")
            }
            if let k = actions[action] { return .app(k) }
        }
        if let box = objc_getAssociatedObject(item, tag) as? Box { return box.key }
        let known: Key? = L10n.table[item.title] != nil ? .app(item.title) : kitKey(item.title, in: menu)
        guard let found = known else { return nil }
        objc_setAssociatedObject(item, tag, Box(found), .OBJC_ASSOCIATION_RETAIN_NONATOMIC)
        return found
    }

    // "Remplir" is both Fill (Window menu) and AutoFill (Edit menu), so the menu decides which table answers first.
    private static func kitKey(_ title: String, in menu: NSMenu) -> Key? {
        let window = sequence(first: menu, next: { $0.supermenu }).contains { $0 === NSApp.windowsMenu }
        for table in window ? ["MenuCommands"] : ["InputManager", "MenuCommands"] {
            for lang in [launch, "en"] {
                if let hit = kit[table]?[lang]?.first(where: { $0.value == title && !$0.key.contains("%@") }) {
                    return .kit(table: table, key: hit.key)
                }
            }
        }
        return nil
    }

    private static func text(_ key: Key) -> String? {
        switch key {
        case .app(let k): return L(k)
        case .kit(let table, let k):
            for lang in kitLanguage(L10n.shared.id) { if let t = kit[table]?[lang]?[k] { return t } }
            return kit[table]?["en"]?[k]
        }
    }

    private static func retitle(_ item: NSMenuItem, in menu: NSMenu) {
        if busy || (menu === NSApp.mainMenu && item === menu.items.first) { return }
        guard let k = key(item, in: menu), let t = text(k) else { return }
        busy = true
        defer { busy = false }
        if item.title != t { item.title = t }
        if let sub = item.submenu, sub.title != t { sub.title = t }
    }
}

struct WindowConfigurator: NSViewRepresentable {
    func makeNSView(context: Context) -> NSView {
        let v = NSView()
        DispatchQueue.main.async {
            guard let w = v.window else { return }
            w.isOpaque = false
            w.backgroundColor = .clear
            w.titlebarAppearsTransparent = true
            w.titleVisibility = .hidden
            w.acceptsMouseMovedEvents = true
            w.styleMask.insert(.fullSizeContentView)
            w.collectionBehavior.insert(.fullScreenPrimary)
            w.isReleasedWhenClosed = false
            // The close button quits as ⌘W does: unsaved changes asked about first, Cancel keeping the window.
            if let close = w.standardWindowButton(.closeButton) {
                close.target = NSApp
                close.action = #selector(NSApplication.terminate(_:))
            }
        }
        return v
    }

    func updateNSView(_ nsView: NSView, context: Context) {}
}

final class PassVisualEffectView: NSVisualEffectView {
    override func hitTest(_ point: NSPoint) -> NSView? { nil }
}

// The frosted window base: dark HUD glass, or light glass in the light theme.
struct VisualEffect: NSViewRepresentable {
    let dark: Bool

    func makeNSView(context: Context) -> NSVisualEffectView {
        let v = PassVisualEffectView()
        v.blendingMode = .behindWindow
        v.state = .active
        v.material = dark ? .hudWindow : .popover
        return v
    }

    func updateNSView(_ v: NSVisualEffectView, context: Context) {
        v.material = dark ? .hudWindow : .popover
    }
}

struct CheckShape: Shape {
    func path(in r: CGRect) -> Path {
        var p = Path()
        p.move(to: CGPoint(x: r.minX + r.width * 0.18, y: r.minY + r.height * 0.52))
        p.addLine(to: CGPoint(x: r.minX + r.width * 0.42, y: r.minY + r.height * 0.76))
        p.addLine(to: CGPoint(x: r.minX + r.width * 0.84, y: r.minY + r.height * 0.28))
        return p
    }
}

struct SliderKnob: View {
    let size: CGFloat
    let tint: Color
    let active: Bool

    var body: some View {
        Circle()
            .fill(Color.white)
            .frame(width: size, height: size)
            .shadow(color: .black.opacity(0.3), radius: 1.5, y: 0.5)
            .glow(tint, active ? 14 : 8)
            .background {
                Circle()
                    .strokeBorder(tint.opacity(0.75), lineWidth: 1.5)
                    .frame(width: size + (active ? 14 : 0), height: size + (active ? 14 : 0))
                    .glow(tint, active ? 8 : 0)
                    .opacity(active ? 1 : 0)
            }
    }
}

struct NeonButtonStyle: ButtonStyle {
    var tint: Color = Neon.cyan
    var lit = false
    var size: CGFloat = 36
    var spin = false
    func makeBody(configuration: Configuration) -> some View {
        NeonBody(configuration: configuration, tint: tint, lit: lit, size: size, spin: spin)
    }
}

struct NeonBody: View {
    let configuration: ButtonStyle.Configuration
    let tint: Color
    let lit: Bool
    let size: CGFloat
    let spin: Bool
    @State private var hover = false
    @State private var hops = 0
    @State private var clicks = 0

    var body: some View {
        let calm = Neon.calm
        let shape = RoundedRectangle(cornerRadius: size * (hover && !calm ? 0.36 : 0.28), style: .continuous)
        configuration.label
            .font(.ui(size: size * 0.4, weight: .semibold))
            .foregroundStyle(hover || lit ? Ink.text : Ink.text.opacity(0.78))
            .contentTransition(.symbolEffect(.replace))
            .symbolEffect(.bounce.up.byLayer, options: .speed(1.5), value: hops)
            .symbolEffect(.bounce.down.byLayer, value: clicks)
            .rotationEffect(.degrees(spin && hover && !calm ? 90 : 0))
            .frame(width: size, height: size)
            .background(shape.fill(tint.opacity(hover ? 0.55 : (lit ? 0.24 : 0))).blur(radius: size * 0.35))
            .background(shape.fill(tint.opacity(hover ? 0.28 : (lit ? 0.14 : 0.06))))
            .glow(tint, hover ? size * 0.55 : (lit ? size * 0.3 : 0))
            .hueRotation(.degrees(hover && !calm ? 14 : 0))
            .scaleEffect(configuration.isPressed ? (calm ? 0.95 : 0.86) : (hover && !calm ? 1.14 : 1))
            .ripple(clicks, shape: RoundedRectangle(cornerRadius: size * 0.3, style: .continuous), tint: tint)
            .contentShape(shape)
            .onHover { h in
                withAnimation(Neon.hover(h)) { hover = h }
                if h && !Neon.calm { hops += 1 }
            }
            .onChange(of: configuration.isPressed) { was, now in if was && !now && !Neon.calm { clicks += 1 } }
            .animation(calm ? Neon.quick : .spring(response: 0.25, dampingFraction: 0.5), value: configuration.isPressed)
    }
}

struct PillStyle: ButtonStyle {
    var tint: Color = Neon.cyan
    func makeBody(configuration: Configuration) -> some View { PillBody(configuration: configuration, tint: tint) }
}

struct PillBody: View {
    let configuration: ButtonStyle.Configuration
    let tint: Color
    @State private var hover = false
    @State private var clicks = 0

    var body: some View {
        let calm = Neon.calm
        let shape = RoundedRectangle(cornerRadius: hover && !calm ? 13 : 10, style: .continuous)
        configuration.label
            .font(.ui(size: 13, weight: .semibold, design: .rounded))
            .lineLimit(1)
            .minimumScaleFactor(0.75)
            .foregroundStyle(hover ? Color.black : tint)
            .padding(.horizontal, 12)
            .frame(maxWidth: .infinity)
            .frame(height: 36)
            .background(shape.fill(hover ? tint : tint.opacity(0.13)).sheen(hover, shape: shape).pressWave(clicks, shape: shape, tint: .white))
            .glow(tint, hover ? 18 : 0)
            .onChange(of: configuration.isPressed) { was, now in if was && !now { clicks += 1 } }
            .scaleEffect(configuration.isPressed ? (calm ? 0.97 : 0.94) : (hover && !calm ? 1.04 : 1))
            .contentShape(shape)
            .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
            .animation(calm ? Neon.quick : .spring(response: 0.25, dampingFraction: 0.5), value: configuration.isPressed)
    }
}

struct NeonToggle: View {
    let state: Bool?
    let action: () -> Void
    @State private var hover = false
    @State private var stretch = false
    @State private var pulses = 0

    var body: some View {
        let look = Skin.shared
        let on = state == true
        let track = RoundedRectangle(cornerRadius: 8, style: .continuous)
        let kw: CGFloat = stretch ? 29 : 20
        ZStack(alignment: .leading) {
            track
                .fill(on ? look.accent.opacity(0.85) : Ink.text.opacity(state == nil ? 0.18 : 0.1))
                .frame(width: 46, height: 26)
                .overlay(track.strokeBorder(look.mark, lineWidth: on && look.simplified ? 1.5 : 0))
                .glow(look.accent, on ? (hover ? 16 : 10) : (hover ? 6 : 0))
                .ripple(pulses, shape: track, tint: look.accent)
            RoundedRectangle(cornerRadius: 6, style: .continuous)
                .fill(Color.white)
                .frame(width: kw, height: stretch ? 17 : 20)
                .shadow(color: .black.opacity(0.35), radius: 2, y: 1)
                .opacity(state == nil ? 0.65 : 1)
                .scaleEffect(hover && !Neon.calm ? 1.12 : 1)
                .offset(x: on ? 43 - kw : (state == nil ? 23 - kw / 2 : 3))
        }
        .frame(width: 46, height: 26)
        .environment(\.layoutDirection, .leftToRight)
        .scaleEffect(x: look.rtl ? -1 : 1)
        .contentShape(track)
        .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
        .onTapGesture {
            withAnimation(Neon.calm ? Neon.quick : .spring(response: 0.35, dampingFraction: 0.65)) { action() }
            guard !Neon.calm else { return }
            pulses += 1
            withAnimation(.easeOut(duration: 0.12)) { stretch = true }
            Task {
                try? await Task.sleep(for: .milliseconds(150))
                withAnimation(Neon.pop) { stretch = false }
            }
        }
        .accessibilityElement()
        .accessibilityAddTraits(.isButton)
        .accessibilityValue(on ? L("On") : (state == nil ? L("Mixed") : L("Off")))
        .accessibilityAction(.default, action)
    }
}

// Sun and moon without words: the knob carries the current theme's sign.
struct ThemeSwitch: View {
    @State private var hover = false

    var body: some View {
        let look = Skin.shared
        let dark = look.dark
        let track = Capsule()
        ZStack {
            track.fill(Ink.text.opacity(hover ? 0.12 : 0.07))
            HStack {
                Image(systemName: "sun.max.fill").opacity(dark ? 0.45 : 0)
                Spacer(minLength: 0)
                Image(systemName: "moon.fill").opacity(dark ? 0 : 0.45)
            }
            .font(.ui(size: 11, weight: .bold))
            .foregroundStyle(Ink.text)
            .padding(.horizontal, 9)
            Circle()
                .fill(Color.white)
                .frame(width: 26, height: 26)
                .shadow(color: .black.opacity(0.3), radius: 2, y: 1)
                .overlay {
                    Image(systemName: dark ? "moon.fill" : "sun.max.fill")
                        .font(.ui(size: 12, weight: .bold))
                        .foregroundStyle(dark ? Color(red: 0.28, green: 0.3, blue: 0.55) : Color(red: 0.95, green: 0.6, blue: 0.05))
                        .contentTransition(.symbolEffect(.replace))
                }
                .glow(look.accent, hover ? 10 : 4)
                .frame(maxWidth: .infinity, alignment: dark ? .trailing : .leading)
                .padding(.horizontal, 5)
        }
        .frame(width: 66, height: 36)
        .environment(\.layoutDirection, .leftToRight)
        .contentShape(track)
        .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
        .onTapGesture { flip() }
        .help(dark ? L("Dark theme") : L("Light theme"))
        .accessibilityElement()
        .accessibilityLabel(dark ? L("Dark theme") : L("Light theme"))
        .accessibilityAddTraits(.isButton)
        .accessibilityAction(.default) { flip() }
    }

    private func flip() {
        withAnimation(Neon.glide) { Skin.shared.host?.setLook { $0.dark.toggle() } }
    }
}

enum CheckMark { case none, check, dash }

struct NeonCheck: View {
    let mark: CheckMark
    var lit = false
    var size: CGFloat = 20
    let action: () -> Void
    @State private var hover = false
    @State private var clicks = 0

    var body: some View {
        let look = Skin.shared
        let calm = Neon.calm
        let filled = mark != .none
        let shape = RoundedRectangle(cornerRadius: size * 0.32, style: .continuous)
        ZStack {
            shape.fill(filled ? look.accent : Ink.text.opacity(hover ? 0.16 : 0.08))
            shape.fill(look.accent.opacity(lit && !filled ? 0.45 : 0)).blur(radius: size * 0.22).padding(size * 0.18)
            CheckShape()
                .trim(from: 0, to: mark == .check ? 1 : 0)
                .stroke(Color.black, style: StrokeStyle(lineWidth: size * 0.14, lineCap: .round, lineJoin: .round))
                .padding(size * 0.1)
                .opacity(mark == .check ? 1 : 0)
            RoundedRectangle(cornerRadius: size * 0.05, style: .continuous)
                .fill(Color.black)
                .frame(width: size * 0.5, height: size * 0.14)
                .scaleEffect(x: mark == .dash ? 1 : 0.01)
                .opacity(mark == .dash ? 1 : 0)
        }
        .frame(width: size, height: size)
        .overlay(shape.strokeBorder(look.mark, lineWidth: filled && look.simplified ? 1.5 : 0))
        .glow(look.accent, filled ? (hover ? 14 : 9) : (hover || lit ? 7 : 0))
        .ripple(clicks, shape: shape, tint: look.accent)
        .pop(clicks)
        .rotationEffect(.degrees(hover && !calm ? -8 : 0))
        .scaleEffect(hover && !calm ? 1.14 : 1)
        .contentShape(shape)
        .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
        .onTapGesture {
            clicks += 1
            action()
        }
        .animation(calm ? Neon.quick : .spring(response: 0.35, dampingFraction: 0.55), value: mark)
        .animation(Neon.spring, value: lit)
        .accessibilityElement()
        .accessibilityAddTraits(.isButton)
        .accessibilityValue(mark == .check ? L("Checked") : (mark == .dash ? L("Mixed") : L("Unchecked")))
        .accessibilityAction(.default, action)
    }
}

struct Scanlines: View {
    var body: some View {
        Canvas { ctx, size in
            var p = Path()
            var y: CGFloat = 0
            while y < size.height {
                p.addRect(CGRect(x: 0, y: y, width: size.width, height: 1))
                y += 3
            }
            ctx.fill(p, with: .color(.black))
        }
        .allowsHitTesting(false)
    }
}

struct TronGrid: View {
    var phase: Double
    var tint: Color = Neon.cyan
    var body: some View {
        Canvas { ctx, size in
            let horizon = size.height * 0.56
            let cx = size.width / 2
            var p = Path()
            for i in -16...16 {
                p.move(to: CGPoint(x: cx + CGFloat(i) * 8, y: horizon))
                p.addLine(to: CGPoint(x: cx + CGFloat(i) * size.width * 0.16, y: size.height))
            }
            for j in 0..<16 {
                let z = (CGFloat(j) + CGFloat(phase)) / 16
                let y = horizon + (size.height - horizon) * z * z
                p.move(to: CGPoint(x: 0, y: y))
                p.addLine(to: CGPoint(x: size.width, y: y))
            }
            ctx.addFilter(.shadow(color: tint, radius: 3))
            ctx.stroke(p, with: .linearGradient(Gradient(colors: [tint.opacity(0), tint.opacity(0.75)]),
                                                startPoint: CGPoint(x: cx, y: horizon), endPoint: CGPoint(x: cx, y: size.height)), lineWidth: 1)
        }
        .allowsHitTesting(false)
    }
}

struct Backdrop: View {
    @Environment(\.controlActiveState) private var active
    // The drift's clock: standing still while the window is behind others, going on from there after (no jump).
    @State private var stoppedAt: Date?
    @State private var skipped: TimeInterval = 0

    var body: some View {
        let look = Skin.shared
        let still = Neon.calm || active == .inactive
        ZStack {
            VisualEffect(dark: look.dark)
            Ink.void.opacity(look.simplified ? 0.94 : 0.8)
            if !look.simplified {
                // (Twenty steps a second: smooth for something this soft and slow, and a fraction of the drawing.)
                TimelineView(.animation(minimumInterval: 1.0 / 20, paused: still)) { tl in
                    let t = (stoppedAt ?? tl.date).timeIntervalSinceReferenceDate - skipped
                    // There and back, 18 s each way, easing in and out (none in reduced motion).
                    let k = Neon.calm ? 0 : (1 - cos(t * .pi / 18)) / 2
                    let at = { (a: Double, b: Double) in a + (b - a) * k }
                    ZStack {
                        Circle().fill(look.accent.opacity(0.2)).frame(width: 520, height: 520).blur(radius: 140)
                            .offset(x: at(-220, 280), y: at(140, -180))
                        Circle().fill(look.accent2.opacity(0.17)).frame(width: 560, height: 560).blur(radius: 150)
                            .offset(x: at(240, -260), y: at(-150, 200))
                        Circle().fill(look.accent3.opacity(0.08)).frame(width: 420, height: 420).blur(radius: 140)
                            .offset(x: at(-60, 80), y: at(-260, 260))
                    }
                    .frame(maxWidth: .infinity, maxHeight: .infinity)
                    .drawingGroup()
                }
                Scanlines().opacity(look.dark ? 0.06 : 0.025)
            }
        }
        .ignoresSafeArea()
        .allowsHitTesting(false)
        .onChange(of: still, initial: true) {
            if still {
                if stoppedAt == nil { stoppedAt = Date() }
            } else if let s = stoppedAt {
                skipped += Date().timeIntervalSince(s)
                stoppedAt = nil
            }
        }
    }
}

struct BusyToast: View {
    let text: String
    var body: some View {
        let look = Skin.shared
        HStack(spacing: 10) {
            ProgressView().controlSize(.small).tint(look.accent)
            Text(text)
                .font(.ui(size: 12.5, weight: .semibold, design: .rounded))
                .foregroundStyle(Ink.text.opacity(0.85))
        }
        .padding(.horizontal, 16)
        .frame(height: 36)
        .background(RoundedRectangle(cornerRadius: 11, style: .continuous).fill(Ink.void.opacity(0.6)))
        .background(RoundedRectangle(cornerRadius: 11, style: .continuous).fill(.ultraThinMaterial))
        .halo(look.accent, 14)
        .allowsHitTesting(false)
    }
}

// One slider for brightness, interface scale and font size. The pointer is mapped against the track as it stood when the
// press began, so a scale or font change mid-drag (which moves the track itself) never makes the knob jump.
struct ValueSlider: View {
    let icon: String
    let title: String
    let value: Double
    let range: ClosedRange<Double>
    let step: Double
    let text: String
    var enabled = true
    var note: String?
    let set: (Double) -> Void
    @State private var dragging = false
    @State private var hover = false
    @State private var track = CGRect.zero
    @State private var grab: (minX: CGFloat, width: CGFloat)?

    var body: some View {
        let look = Skin.shared
        let span = range.upperBound - range.lowerBound
        let f = CGFloat(min(1, max(0, (value - range.lowerBound) / span)))
        HStack(spacing: 8) {
            Image(systemName: icon)
                .font(.ui(size: 13, weight: .semibold))
                .foregroundStyle(look.accent3)
                .halo(look.accent3, 6)
                .scaleEffect(hover && enabled && !Neon.calm ? 1.12 : 1)
            Text(title)
                .font(.ui(size: 13, weight: .semibold, design: .rounded))
                .lineLimit(1)
                .fixedSize()
            GeometryReader { g in
                let w = max(1, g.size.width - 16)
                let knob: CGFloat = dragging ? 16 : (hover ? 14 : 12)
                let bar: CGFloat = hover || dragging ? 6 : 4
                ZStack(alignment: .leading) {
                    RoundedRectangle(cornerRadius: bar / 2, style: .continuous)
                        .fill(Ink.text.opacity(0.14))
                        .frame(width: w, height: bar)
                    RoundedRectangle(cornerRadius: bar / 2, style: .continuous)
                        .fill(LinearGradient(colors: [look.accent2, look.accent], startPoint: .leading, endPoint: .trailing))
                        .frame(width: max(bar, w * f), height: bar)
                        .glow(look.accent, dragging ? 14 : 8)
                    SliderKnob(size: knob, tint: look.accent, active: dragging)
                        .offset(x: w * f - knob / 2)
                }
                .padding(.horizontal, 8)
                .frame(width: g.size.width, height: g.size.height)
                .scaleEffect(x: look.rtl ? -1 : 1)
                .contentShape(Rectangle())
                .onGeometryChange(for: CGRect.self) { $0.frame(in: .global) } action: { track = $0 }
                .gesture(DragGesture(minimumDistance: 0, coordinateSpace: .global)
                    .onChanged { v in
                        guard enabled else { return }
                        if grab == nil {
                            let k = track.width / max(1, g.size.width)
                            grab = (track.minX + 8 * k, max(1, track.width - 16 * k))
                            dragging = true
                        }
                        guard let grab else { return }
                        var t = Double((v.location.x - grab.minX) / grab.width)
                        if look.rtl { t = 1 - t }
                        let raw = range.lowerBound + min(1, max(0, t)) * span
                        let stepped = min(range.upperBound, max(range.lowerBound, (raw / step).rounded() * step))
                        if abs(stepped - value) > step / 2 { set(stepped) }
                    }
                    .onEnded { _ in
                        grab = nil
                        dragging = false
                    })
            }
            .environment(\.layoutDirection, .leftToRight)
            .frame(height: 30)
            Text(text)
                .font(.ui(size: 12, weight: .heavy, design: .rounded))
                .monospacedDigit()
                .foregroundStyle(Ink.text.opacity(0.7))
                .lineLimit(1)
                .minimumScaleFactor(0.6)
                .frame(width: 46, alignment: .trailing)
        }
        .foregroundStyle(Ink.text)
        .padding(.leading, 12)
        .padding(.trailing, 10)
        .frame(height: 36)
        .background(RoundedRectangle(cornerRadius: hover ? 12 : 10, style: .continuous).fill(Ink.text.opacity(hover && enabled ? 0.12 : 0.06)))
        .saturation(enabled ? 1 : 0)
        .opacity(enabled ? 1 : 0.4)
        .allowsHitTesting(enabled)
        .help(note ?? title)
        .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
        .animation(Neon.pop, value: dragging)
        .accessibilityElement()
        .accessibilityLabel(title)
        .accessibilityValue(text)
        .accessibilityHint(note ?? "")
        .accessibilityAdjustableAction { d in
            guard enabled else { return }
            set(min(range.upperBound, max(range.lowerBound, value + (d == .increment ? step : -step))))
        }
    }
}

// Language, theme, Simplified UI, Reduce animations, style, brightness, interface scale and font size: the top of Settings.
struct SettingsHead: View {
    @Binding var langOpen: Bool
    @Binding var styleOpen: Bool

    var body: some View {
        let look = Skin.shared
        let locale = Locale(identifier: L10n.shared.id)
        VStack(spacing: 8) {
            HStack(spacing: 8) {
                LanguageField(open: $langOpen)
                ThemeSwitch()
            }
            if langOpen {
                LanguageMenu(open: $langOpen)
                    .transition(.menu)
            }
            ToggleLine(title: L("Simplified UI"), detail: L("Plain monochrome look without lighting"), on: look.simplified) {
                look.host?.setLook { $0.simplified.toggle() }
            }
            ToggleLine(title: L("Reduce animations"), detail: L("Calmer interface; working motion stays"), on: look.reduceMotion) {
                look.host?.setLook { $0.reduceMotion.toggle() }
            }
            StyleField(open: $styleOpen)
            if styleOpen {
                StyleMenu(open: $styleOpen)
                    .transition(.menu)
            }
            ValueSlider(icon: "sun.max.fill", title: L("Brightness"), value: look.brightness, range: 0...1, step: 0.01,
                        text: look.brightness.formatted(.percent.precision(.fractionLength(0)).locale(locale)),
                        enabled: !look.simplified, note: look.simplified ? L("Lighting is off in Simplified UI") : nil) { v in
                look.host?.setBrightness(v)
            }
            ValueSlider(icon: "plus.magnifyingglass", title: L("Interface scale"), value: look.scale, range: SkinSettings.scales, step: 0.01,
                        text: look.scale.formatted(.percent.precision(.fractionLength(0)).locale(locale))) { v in
                look.host?.setLook { $0.scale = v }
            }
            ValueSlider(icon: "textformat.size", title: L("Font size"), value: look.values.fontSize, range: SkinSettings.fontSizes, step: 1,
                        text: L("{n} pt", ["n": Int(look.values.fontSize)])) { v in
                look.host?.setLook { $0.fontSize = v }
            }
        }
        .frame(maxWidth: .infinity)
        .onChange(of: langOpen) { if langOpen { withAnimation(Neon.glide) { styleOpen = false } } }
        .onChange(of: styleOpen) { if styleOpen { withAnimation(Neon.glide) { langOpen = false } } }
    }
}

struct LanguageField: View {
    @Binding var open: Bool
    @State private var hover = false

    var body: some View {
        let look = Skin.shared
        let l = Languages.all.first { $0.id == L10n.shared.id } ?? Languages.all[0]
        HStack(spacing: 8) {
            Text(l.flag).font(.ui(size: 14))
            Text(l.name)
                .font(.ui(size: 13, weight: .semibold, design: .rounded))
                .lineLimit(1)
            Spacer(minLength: 0)
            Image(systemName: "chevron.down")
                .font(.ui(size: 10, weight: .bold))
                .rotationEffect(.degrees(open ? 180 : 0))
                .offset(y: hover && !open && !Neon.calm ? 2 : 0)
        }
        .foregroundStyle(hover || open ? look.accent : Ink.text)
        .padding(.horizontal, 12)
        .frame(height: 36)
        .frame(maxWidth: .infinity)
        .background(RoundedRectangle(cornerRadius: 10, style: .continuous).fill(Ink.text.opacity(hover || open ? 0.12 : 0.06)))
        .glow(look.accent, hover ? 12 : (open ? 6 : 0))
        .contentShape(Rectangle())
        .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
        .onTapGesture { withAnimation(Neon.glide) { open.toggle() } }
        .help(L("Language"))
        .accessibilityElement(children: .combine)
        .accessibilityLabel(L("Language"))
        .accessibilityValue(l.name)
        .accessibilityAddTraits(.isButton)
        .accessibilityAction(.default) { withAnimation(Neon.glide) { open.toggle() } }
    }
}

struct LanguageMenu: View {
    @Binding var open: Bool

    var body: some View {
        let look = Skin.shared
        VStack(spacing: 4) {
            ForEach(Array(Languages.all.enumerated()), id: \.element.id) { i, l in
                LanguageRow(flag: l.flag, name: l.name, chosen: l.id == L10n.shared.id) {
                    withAnimation(Neon.glide) { open = false }
                    // The words blend once the menu has folded, so every row stays in place while its text changes.
                    Task {
                        try? await Task.sleep(for: .milliseconds(Neon.calm ? 60 : 320))
                        look.host?.setLanguage(l.id)
                    }
                }
                .environment(\.locale, Locale(identifier: l.id))
                .environment(\.layoutDirection, l.id == "ar" ? .rightToLeft : .leftToRight)
                .cascade(i)
            }
        }
        .padding(6)
        .background(RoundedRectangle(cornerRadius: 14, style: .continuous).fill(Ink.void.opacity(0.6)))
        .background(RoundedRectangle(cornerRadius: 14, style: .continuous).fill(.ultraThinMaterial))
        .shadow(color: look.accent.opacity(0.18 * look.glow), radius: 20)
    }
}

struct LanguageRow: View {
    let flag: String
    let name: String
    let chosen: Bool
    let action: () -> Void
    @State private var hover = false
    @State private var flashes = 0

    var body: some View {
        let look = Skin.shared
        HStack(spacing: 8) {
            Text(flag).font(.ui(size: 15))
            Text(name).font(.ui(size: 13, weight: .medium, design: .rounded)).lineLimit(1)
            Spacer(minLength: 0)
            if chosen {
                Image(systemName: "checkmark").font(.ui(size: 11, weight: .bold)).foregroundStyle(look.mark)
            }
        }
        .foregroundStyle(hover ? Ink.text : Ink.text.opacity(0.85))
        .padding(.horizontal, 10)
        .frame(height: 34)
        .background(RoundedRectangle(cornerRadius: 9, style: .continuous).fill(look.accent.opacity(hover ? 0.14 : 0)))
        .rowMotion(hover, tint: look.accent, flash: flashes)
        .offset(x: hover && !Neon.calm ? 4 : 0)
        .contentShape(Rectangle())
        .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
        .onTapGesture {
            flashes += 1
            action()
        }
        .accessibilityElement(children: .combine)
        .accessibilityAddTraits(chosen ? [.isButton, .isSelected] : .isButton)
        .accessibilityAction(.default, action)
    }
}

struct StyleDots: View {
    let style: Style
    var size: CGFloat = 8
    var glow = false

    var body: some View {
        let a = style.accents
        HStack(spacing: size * 0.6) {
            ForEach(Array([a.0, a.1, a.2].enumerated()), id: \.offset) { _, c in
                RoundedRectangle(cornerRadius: size * 0.3, style: .continuous)
                    .fill(c)
                    .frame(width: size, height: size)
                    .glow(c, glow ? 6 : 0)
            }
        }
    }
}

struct StyleField: View {
    @Binding var open: Bool
    @State private var hover = false

    var body: some View {
        let look = Skin.shared
        let style = look.host?.style ?? .classic
        HStack(spacing: 8) {
            StyleDots(style: style)
            Text(style.label)
                .font(.ui(size: 13, weight: .semibold, design: .rounded))
                .lineLimit(1)
            Spacer(minLength: 0)
            Image(systemName: "chevron.down")
                .font(.ui(size: 10, weight: .bold))
                .rotationEffect(.degrees(open ? 180 : 0))
                .offset(y: hover && !open && !Neon.calm ? 2 : 0)
        }
        .foregroundStyle(hover || open ? look.accent : Ink.text)
        .padding(.horizontal, 12)
        .frame(height: 36)
        .frame(maxWidth: .infinity)
        .background(RoundedRectangle(cornerRadius: 10, style: .continuous).fill(Ink.text.opacity(hover || open ? 0.12 : 0.06)))
        .glow(look.accent, hover ? 12 : (open ? 6 : 0))
        .contentShape(Rectangle())
        .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
        .onTapGesture { withAnimation(Neon.glide) { open.toggle() } }
        .accessibilityElement(children: .combine)
        .accessibilityLabel(L("Style {name}", ["name": style.label]))
        .accessibilityAddTraits(.isButton)
        .accessibilityAction(.default) { withAnimation(Neon.glide) { open.toggle() } }
    }
}

struct StyleMenu: View {
    @Binding var open: Bool
    var body: some View {
        let look = Skin.shared
        let current = look.host?.style ?? .classic
        VStack(spacing: 4) {
            ForEach(Array(Style.allCases.enumerated()), id: \.element) { i, s in
                StyleRow(style: s, chosen: s == current) {
                    withAnimation(Neon.glide) { open = false }
                    look.host?.setStyle(s)
                }
                .cascade(i)
            }
        }
        .padding(6)
        .background(RoundedRectangle(cornerRadius: 14, style: .continuous).fill(Ink.void.opacity(0.6)))
        .background(RoundedRectangle(cornerRadius: 14, style: .continuous).fill(.ultraThinMaterial))
        .shadow(color: look.accent.opacity(0.18 * look.glow), radius: 20)
    }
}

struct StyleRow: View {
    let style: Style
    let chosen: Bool
    var title: String?
    let action: () -> Void
    @State private var hover = false
    @State private var flashes = 0

    var body: some View {
        let accents = style.accents
        HStack(spacing: 8) {
            StyleDots(style: style, size: 9, glow: hover)
            Text(title ?? style.label).font(.ui(size: 13, weight: .medium, design: .rounded)).lineLimit(1)
            Spacer(minLength: 0)
            if chosen {
                Image(systemName: "checkmark").font(.ui(size: 11, weight: .bold)).foregroundStyle(accents.0)
            }
        }
        .foregroundStyle(hover ? Ink.text : Ink.text.opacity(0.85))
        .padding(.horizontal, 10)
        .frame(height: 34)
        .background(RoundedRectangle(cornerRadius: 9, style: .continuous).fill(accents.0.opacity(hover ? 0.14 : 0)))
        .rowMotion(hover, tint: accents.0, flash: flashes)
        .offset(x: hover && !Neon.calm ? 4 : 0)
        .contentShape(Rectangle())
        .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
        .onTapGesture {
            flashes += 1
            action()
        }
        .accessibilityElement(children: .contain)
        .accessibilityLabel(title ?? style.label)
        .accessibilityAddTraits(chosen ? [.isButton, .isSelected] : .isButton)
        .accessibilityAction(.default, action)
    }
}

struct ChoiceField<Option: Identifiable & Equatable>: View {
    let title: String
    let options: [Option]
    @Binding var current: Option
    @Binding var open: Bool
    let label: (Option) -> String
    var icon: String?
    @State private var hover = false

    var body: some View {
        let look = Skin.shared
        VStack(spacing: 6) {
            HStack(spacing: 8) {
                if let icon { Image(systemName: icon).font(.ui(size: 11, weight: .bold)) }
                Text("\(title) · \(label(current))")
                    .font(.ui(size: 13, weight: .semibold, design: .rounded))
                    .lineLimit(1)
                Spacer(minLength: 0)
                Image(systemName: "chevron.down")
                    .font(.ui(size: 10, weight: .bold))
                    .rotationEffect(.degrees(open ? 180 : 0))
                    .offset(y: hover && !open && !Neon.calm ? 2 : 0)
            }
            .foregroundStyle(hover || open ? look.accent : Ink.text)
            .padding(.horizontal, 14)
            .frame(height: 40)
            .background(RoundedRectangle(cornerRadius: 12, style: .continuous).fill(Ink.text.opacity(hover || open ? 0.1 : 0.06)))
            .glow(look.accent, hover ? 10 : (open ? 5 : 0))
            .contentShape(Rectangle())
            .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
            .onTapGesture { withAnimation(Neon.glide) { open.toggle() } }
            .accessibilityElement(children: .combine)
            .accessibilityLabel(title)
            .accessibilityValue(label(current))
            .accessibilityAddTraits(.isButton)
            .accessibilityAction(.default) { withAnimation(Neon.glide) { open.toggle() } }
            if open {
                VStack(spacing: 2) {
                    ForEach(Array(options.enumerated()), id: \.element.id) { i, o in
                        OptionItem(text: label(o), chosen: o == current) {
                            current = o
                            withAnimation(Neon.glide) { open = false }
                        }
                        .cascade(i)
                    }
                }
                .padding(5)
                .background(RoundedRectangle(cornerRadius: 12, style: .continuous).fill(Ink.void.opacity(0.5)))
                .transition(.menu)
            }
        }
    }
}

struct SettingsTitle: View {
    let text: String
    var body: some View {
        Text(text)
            .font(.ui(size: 10.5, weight: .bold, design: .rounded))
            .tracking(1.3)
            .textCase(.uppercase)
            .foregroundStyle(Ink.text.opacity(0.4))
            .padding(.leading, 6)
            .padding(.top, 8)
    }
}

struct Indented<Content: View>: View {
    @ViewBuilder let content: Content
    var body: some View {
        HStack(spacing: 8) {
            RoundedRectangle(cornerRadius: 1, style: .continuous)
                .fill(Skin.shared.accent.opacity(0.35))
                .frame(width: 2)
                .padding(.vertical, 6)
            content
        }
        .padding(.leading, 10)
    }
}

struct SettingLine<Control: View>: View {
    let title: String
    var detail: String?
    @ViewBuilder let control: Control

    var body: some View {
        HStack(spacing: 10) {
            VStack(alignment: .leading, spacing: 2) {
                Text(title).font(.ui(size: 13, weight: .medium, design: .rounded)).foregroundStyle(Ink.text).lineLimit(2)
                if let detail {
                    Text(detail).font(.ui(size: 10.5, design: .rounded)).foregroundStyle(Ink.text.opacity(0.45)).lineLimit(2)
                }
            }
            Spacer(minLength: 4)
            control
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 8)
        .frame(minHeight: 44)
        .background(RoundedRectangle(cornerRadius: 11, style: .continuous).fill(Ink.text.opacity(0.05)))
    }
}

// A shortcut row: the key field plus a switch. Off greys the field out, stops the shortcut and frees its key for others.
struct ShortcutLine<Field: View>: View {
    let title: String
    let on: Bool
    let toggle: () -> Void
    @ViewBuilder let field: Field

    var body: some View {
        SettingLine(title: title) {
            HStack(spacing: 8) {
                field
                    .saturation(on ? 1 : 0)
                    .opacity(on ? 1 : 0.35)
                    .allowsHitTesting(on)
                    .accessibilityHidden(!on)
                NeonToggle(state: on, action: toggle)
                    .accessibilityLabel(L("Use the {action} shortcut", ["action": title]))
            }
        }
    }
}

struct OptionLine<Option: Identifiable & Equatable>: View {
    let title: String
    let options: [Option]
    let current: Option
    let label: (Option) -> String
    let choose: (Option) -> Void
    @State private var open = false
    @State private var hover = false
    @State private var blink = false

    var body: some View {
        let look = Skin.shared
        VStack(spacing: 4) {
            SettingLine(title: title) {
                HStack(spacing: 6) {
                    Text(label(current)).font(.ui(size: 12, weight: .semibold, design: .rounded)).lineLimit(1)
                    Image(systemName: "chevron.down")
                        .font(.ui(size: 9, weight: .bold))
                        .rotationEffect(.degrees(open ? 180 : 0))
                        .offset(y: hover && !open && !Neon.calm ? 2 : 0)
                }
                .foregroundStyle(hover || open ? look.accent : Ink.text)
                .padding(.horizontal, 10)
                .frame(height: 28)
                .background(RoundedRectangle(cornerRadius: 8, style: .continuous).fill(look.accent.opacity(open ? (blink ? 0.28 : 0.1) : (hover ? 0.14 : 0.07))))
                .glow(look.accent, open ? (blink ? 14 : 3) : (hover ? 8 : 0))
                .contentShape(Rectangle())
                .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
                .onTapGesture { withAnimation(Neon.glide) { open.toggle() } }
                .accessibilityElement(children: .combine)
                .accessibilityLabel(title)
                .accessibilityValue(label(current))
                .accessibilityAddTraits(.isButton)
                .accessibilityAction(.default) { withAnimation(Neon.glide) { open.toggle() } }
            }
            if open {
                VStack(spacing: 2) {
                    ForEach(Array(options.enumerated()), id: \.element.id) { i, o in
                        OptionItem(text: label(o), chosen: o == current) {
                            choose(o)
                            withAnimation(Neon.glide) { open = false }
                        }
                        .cascade(i)
                    }
                }
                .padding(5)
                .background(RoundedRectangle(cornerRadius: 12, style: .continuous).fill(Ink.void.opacity(0.55)))
                .background(RoundedRectangle(cornerRadius: 12, style: .continuous).fill(.ultraThinMaterial))
                .transition(.menu)
            }
        }
        .blink(open, $blink)
        .onChange(of: open) { if open { look.host?.endCapture() } }
    }
}

struct OptionItem: View {
    let text: String
    let chosen: Bool
    let action: () -> Void
    @State private var hover = false
    @State private var flashes = 0

    var body: some View {
        let look = Skin.shared
        HStack {
            Text(text).font(.ui(size: 12.5, weight: .medium, design: .rounded)).lineLimit(1)
            Spacer(minLength: 0)
            if chosen { Image(systemName: "checkmark").font(.ui(size: 10, weight: .bold)).foregroundStyle(look.mark) }
        }
        .foregroundStyle(hover ? Ink.text : Ink.text.opacity(0.8))
        .padding(.horizontal, 10)
        .frame(height: 30)
        .background(RoundedRectangle(cornerRadius: 8, style: .continuous).fill(look.accent.opacity(hover ? 0.14 : 0)))
        .rowMotion(hover, tint: look.accent, flash: flashes, radius: 8)
        .offset(x: hover && !Neon.calm ? 3 : 0)
        .contentShape(Rectangle())
        .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
        .onTapGesture {
            flashes += 1
            action()
        }
        .accessibilityElement(children: .combine)
        .accessibilityAddTraits(chosen ? [.isButton, .isSelected] : .isButton)
        .accessibilityAction(.default, action)
    }
}

struct FitHeight: Layout {
    let limit: CGFloat

    func sizeThatFits(proposal: ProposedViewSize, subviews: Subviews, cache: inout ()) -> CGSize {
        guard let s = subviews.first else { return .zero }
        let ideal = s.sizeThatFits(ProposedViewSize(width: proposal.width, height: nil))
        return CGSize(width: proposal.width ?? ideal.width, height: min(ideal.height, limit))
    }

    func placeSubviews(in bounds: CGRect, proposal: ProposedViewSize, subviews: Subviews, cache: inout ()) {
        subviews.first?.place(at: bounds.origin, anchor: .topLeading, proposal: ProposedViewSize(bounds.size))
    }
}

struct CloseX: View {
    let action: () -> Void
    var body: some View {
        Button(action: action) { Image(systemName: "xmark").font(.ui(size: 11, weight: .black)) }
            .buttonStyle(NeonButtonStyle(tint: Neon.red, lit: true, size: 26, spin: true))
            .accessibilityLabel(L("Close"))
            .help(L("Close"))
    }
}

struct CardTitle: View {
    let text: String
    var body: some View {
        Text(text)
            .font(.ui(size: 18, weight: .bold, design: .rounded))
            .foregroundStyle(Ink.text)
            .multilineTextAlignment(.center)
            .lineLimit(3)
            .padding(.horizontal, 22)
            .halo(Skin.shared.accent, 10)
    }
}

struct FieldLook: ViewModifier {
    let focused: Bool
    var alert = false
    @State private var hover = false
    @State private var flare = false

    func body(content: Content) -> some View {
        let look = Skin.shared
        let shape = RoundedRectangle(cornerRadius: 12, style: .continuous)
        content
            .textFieldStyle(.plain)
            .font(.ui(size: 15, weight: .medium, design: .rounded))
            .padding(.horizontal, 14)
            .frame(height: 40)
            .background(shape.fill(alert ? Neon.red.opacity(0.14) : Ink.text.opacity(focused ? 0.1 : (hover ? 0.08 : 0.06))))
            .overlay(shape.strokeBorder(alert ? Neon.red.opacity(0.5) : look.accent.opacity(focused ? (flare ? 0.8 : 0.4) : (hover ? 0.3 : 0)), lineWidth: 1).allowsHitTesting(false))
            .shadow(color: alert ? Neon.red.opacity(0.6) : look.accent.opacity(focused ? (flare ? 0.7 : 0.35) * look.glow : 0), radius: flare ? 22 : 14)
            .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
            .onChange(of: focused) {
                guard focused, !Neon.calm else { return }
                withAnimation(.easeOut(duration: 0.14)) { flare = true }
                Task {
                    try? await Task.sleep(for: .milliseconds(180))
                    withAnimation(.easeInOut(duration: 0.5)) { flare = false }
                }
            }
    }
}

struct Shake: GeometryEffect {
    var amount: CGFloat
    var animatableData: CGFloat {
        get { amount }
        set { amount = newValue }
    }
    func effectValue(size: CGSize) -> ProjectionTransform {
        ProjectionTransform(CGAffineTransform(translationX: 12 * sin(amount * .pi * 6), y: 0))
    }
}

struct ToggleLine: View {
    let title: String
    let detail: String?
    let on: Bool
    let action: () -> Void

    var body: some View {
        HStack(spacing: 10) {
            VStack(alignment: .leading, spacing: 2) {
                Text(title).font(.ui(size: 13, weight: .medium, design: .rounded)).foregroundStyle(Ink.text)
                if let detail {
                    Text(detail).font(.ui(size: 10.5, design: .rounded)).foregroundStyle(Ink.text.opacity(0.45))
                }
            }
            Spacer()
            NeonToggle(state: on, action: action).accessibilityLabel(title)
        }
        .padding(.horizontal, 14)
        .padding(.vertical, 9)
        .background(RoundedRectangle(cornerRadius: 12, style: .continuous).fill(Ink.text.opacity(0.05)))
    }
}

@MainActor
enum Art {
    static func render<V: View>(_ view: V, size: CGSize, scale: CGFloat, to path: String) {
        let r = ImageRenderer(content: view.frame(width: size.width, height: size.height).environment(\.colorScheme, .dark))
        r.scale = scale
        guard let cg = r.cgImage else { return }
        try? NSBitmapImageRep(cgImage: cg).representation(using: .png, properties: [:])?.write(to: URL(fileURLWithPath: path))
    }
}

// MARK: - Shared look · end

// MARK: - Bcad design pieces

extension Keys {
    static let reserved: Set<String> = ["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", "PageUp", "PageDown", "Enter", "Space", "Tab", "KeyA", "KeyX",
                                         "KeyZ", "Backspace", "Delete", "Digit0", "Digit1", "Digit2", "Digit3", "Digit4", "Digit5", "Digit6"]
}

struct KeyField: View {
    @Environment(Workbench.self) private var lib
    let action: Action
    @State private var blink = false
    @State private var hover = false
    @State private var clicks = 0

    var body: some View {
        let look = Skin.shared
        let capturing = lib.capturing == action
        let failed = lib.captureFail == action
        let tint = failed ? Neon.red : look.accent
        Text(capturing ? L("Press a key") : Keys.label(lib.settings.key(action)))
            .font(.ui(size: capturing ? 10.5 : 12.5, weight: .bold, design: .rounded))
            .foregroundStyle(failed ? Neon.red : (capturing || hover ? look.accent : Ink.text))
            .lineLimit(1)
            .padding(.horizontal, 10)
            .frame(minWidth: 56)
            .frame(height: 28)
            .background(RoundedRectangle(cornerRadius: 8, style: .continuous).fill(tint.opacity(failed ? 0.22 : (capturing ? (blink ? 0.3 : 0.1) : (hover ? 0.14 : 0.07)))))
            .glow(tint, failed ? 12 : (capturing ? (blink ? 16 : 3) : (hover ? 8 : 0)))
            .pop(clicks, scale: 1.12)
            .contentShape(Rectangle())
            .onHover { h in withAnimation(Neon.hover(h)) { hover = h } }
            .onTapGesture {
                clicks += 1
                capturing ? lib.endCapture() : lib.beginCapture(action)
            }
            .blink(capturing, $blink)
            .animation(Neon.spring, value: failed)
            .accessibilityElement()
            .accessibilityLabel(L("{action} shortcut", ["action": action.label]))
            .accessibilityValue(capturing ? L("Waiting for a key") : Keys.label(lib.settings.key(action)))
            .accessibilityAddTraits(.isButton)
            .accessibilityAction(.default) { capturing ? lib.endCapture() : lib.beginCapture(action) }
    }
}

struct IconArt: View {
    var body: some View {
        let plate = RoundedRectangle(cornerRadius: 186, style: .continuous)
        ZStack {
            plate
                .fill(Color.black)
                .frame(width: 824, height: 824)
                .shadow(color: .black.opacity(0.5), radius: 24, y: 12)
            ZStack {
                LinearGradient(colors: [Color(red: 0.07, green: 0.05, blue: 0.16), Color(red: 0.02, green: 0.03, blue: 0.07)], startPoint: .top, endPoint: .bottom)
                Circle().fill(Neon.cyan.opacity(0.55)).frame(width: 460, height: 460).blur(radius: 130).offset(x: -170, y: -180)
                Circle().fill(Neon.magenta.opacity(0.6)).frame(width: 460, height: 460).blur(radius: 130).offset(x: 180, y: 190)
                TronGrid(phase: 0.4).opacity(0.35)
                Image(systemName: "cube.transparent")
                    .font(.system(size: 400, weight: .light))
                    .foregroundStyle(LinearGradient(colors: [Neon.cyan, Neon.magenta, Neon.amber], startPoint: .topLeading, endPoint: .bottomTrailing))
                    .shadow(color: Neon.cyan.opacity(0.85), radius: 11)
                    .shadow(color: Neon.cyan.opacity(0.5), radius: 30)
                Scanlines().opacity(0.12)
            }
            .frame(width: 824, height: 824)
            .clipShape(plate)
        }
        .frame(width: 1024, height: 1024)
    }
}
