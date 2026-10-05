// Plans: Free (one document a day), Pro (as many as you like) and Studio (Pro and human figures), each monthly or yearly
// with a free trial; the shop they're bought in (the App Store, or a test store with no real payment for development
// builds and the self-test); and the free plan's file of the day, kept in the Keychain.
import AppKit
import CryptoKit
import Foundation
import Security
import StoreKit

enum Plan: Int, Codable, Comparable, CaseIterable, Sendable {
    case free, pro, studio

    static func < (a: Plan, b: Plan) -> Bool { a.rawValue < b.rawValue }

    // Pro and Studio are names, the same in every language.
    @MainActor var name: String { self == .free ? L("Free") : self == .pro ? "Pro" : "Studio" }
    var unlimitedFiles: Bool { self >= .pro }
    var figures: Bool { self >= .studio }
}

enum Billing: String, Codable, CaseIterable, Sendable {
    case monthly, yearly
}

// The products as the stores know them (an id is for ever: never change one), and the pages the Plans card links to.
enum Catalog {
    static let prefix = "bcad"
    static func id(_ p: Plan, _ b: Billing) -> String { "\(prefix).\(p == .studio ? "studio" : "pro").\(b.rawValue)" }
    static let ids = [Plan.pro, .studio].flatMap { p in Billing.allCases.map { id(p, $0) } }

    static func product(_ id: String) -> (plan: Plan, billing: Billing)? {
        for p in [Plan.pro, .studio] {
            for b in Billing.allCases where Self.id(p, b) == id { return (p, b) }
        }
        return nil
    }

    static let manage = URL(string: "https://apps.apple.com/account/subscriptions")!
    static let terms = URL(string: "https://www.apple.com/legal/internet-services/itunes/dev/stdeula/")!
    static let privacy = URL(string: "https://github.com/4dbbfjv49t-svg/Bcad/blob/main/docs/privacy.md")!
}

// The time plans go by: the real one, moved on in the test store ("Next day"), fixed in the self-test.
final class PlanClock: @unchecked Sendable {
    var fixed: Date?
    var offset: TimeInterval = 0

    func now() -> Date { (fixed ?? Date()).addingTimeInterval(offset) }
}

struct ShopProduct: Identifiable, Equatable, Sendable {
    let id: String
    let plan: Plan
    let billing: Billing
    let price: Decimal
    let displayPrice: String
    let currency: String
    let locale: Locale
    // Days of the free trial a first subscription starts with (none: no trial).
    let trialDays: Int?

    // A price as the store writes its own.
    func format(_ v: Decimal) -> String { v.formatted(.currency(code: currency).locale(locale)) }
}

// The subscription in force.
struct ActivePlan: Equatable, Sendable {
    let product: String
    let plan: Plan
    let billing: Billing
    // When the period (or the trial) ends.
    let expires: Date?
    let trial: Bool
    let renews: Bool
    // The product it changes to at renewal, if any.
    let next: String?
}

enum ShopOutcome: Equatable, Sendable {
    case done, pending, cancelled, nothing, failed(String)
}

@MainActor protocol Shop: AnyObject {
    var isTest: Bool { get }
    // Changes made elsewhere (a renewal, a purchase on another Mac, a refund) are told through `changed`.
    func start(_ changed: @escaping @MainActor () -> Void)
    func products() async throws -> [ShopProduct]
    func active() async -> ActivePlan?
    func trialEligible() async -> Bool
    func buy(_ id: String) async -> ShopOutcome
    func restore() async -> ShopOutcome
}

// MARK: - Test store

// Subscriptions with no real payment, for development builds and the self-test: the products and prices of
// Bcad.storekit, a free trial once, an upgrade at once and a downgrade or another period at renewal; kept in a file of
// its own (or only in memory).
@MainActor final class TestShop: Shop {
    struct Sub: Codable, Equatable {
        var product: String
        var expires: Date
        var trial: Bool
        var renews: Bool
        var next: String?
    }

    struct Ledger: Codable, Equatable {
        var trialUsed = false
        var sub: Sub?
        // How far "Next day" has moved the clock on.
        var offset: TimeInterval = 0
    }

    let isTest = true
    let catalog: [ShopProduct]
    let clock: PlanClock
    private let file: URL?
    private(set) var ledger = Ledger()
    private var changed: (@MainActor () -> Void)?

    init(catalog: [ShopProduct], clock: PlanClock, file: URL?) {
        self.catalog = catalog
        self.clock = clock
        self.file = file
        if let file, let data = try? Data(contentsOf: file), let l = try? JSONDecoder().decode(Ledger.self, from: data) { ledger = l }
        clock.offset = ledger.offset
    }

    // The products of Bcad.storekit (in the app, or where APP_STOREKIT says); none when it can't be read.
    static func load() -> [ShopProduct] {
        let url = Bundle.main.url(forResource: "Bcad", withExtension: "storekit")
            ?? ProcessInfo.processInfo.environment["APP_STOREKIT"].map { URL(fileURLWithPath: $0) }
        return url.flatMap { try? catalog(from: $0) } ?? []
    }

    // Read leniently: only what the products need.
    static func catalog(from url: URL) throws -> [ShopProduct] {
        struct Intro: Decodable {
            let paymentMode: String
            let subscriptionPeriod: String
            let numberOfPeriods: Int?
        }
        struct Item: Decodable {
            let productID: String
            let displayPrice: String
            let recurringSubscriptionPeriod: String
            let introductoryOffer: Intro?
        }
        struct Group: Decodable { let subscriptions: [Item] }
        struct Config: Decodable { let subscriptionGroups: [Group] }
        let config = try JSONDecoder().decode(Config.self, from: Data(contentsOf: url))
        let usd = Locale(identifier: "en_US")
        return config.subscriptionGroups.flatMap(\.subscriptions).compactMap { s in
            guard let c = Catalog.product(s.productID), let price = Decimal(string: s.displayPrice, locale: usd) else { return nil }
            var trial: Int?
            if let o = s.introductoryOffer, o.paymentMode == "free" { trial = days(o.subscriptionPeriod) * (o.numberOfPeriods ?? 1) }
            return ShopProduct(id: s.productID, plan: c.plan, billing: c.billing, price: price,
                               displayPrice: price.formatted(.currency(code: "USD").locale(usd)), currency: "USD", locale: usd, trialDays: trial)
        }
    }

    // An ISO 8601 period (P3D, P1W, P1M, P1Y) in days.
    static func days(_ period: String) -> Int {
        guard period.count >= 3, let n = Int(period.dropFirst().dropLast()) else { return 0 }
        switch period.last {
        case "D": return n
        case "W": return 7 * n
        case "M": return 30 * n
        case "Y": return 365 * n
        default: return 0
        }
    }

    static func period(_ b: Billing, after d: Date) -> Date {
        Calendar(identifier: .gregorian).date(byAdding: b == .monthly ? .month : .year, value: 1, to: d) ?? d
    }

    func product(_ id: String) -> ShopProduct? { catalog.first { $0.id == id } }

    private func save() {
        guard let file else { return }
        try? FileManager.default.createDirectory(at: file.deletingLastPathComponent(), withIntermediateDirectories: true)
        if let data = try? JSONEncoder().encode(ledger) { try? data.write(to: file, options: .atomic) }
    }

    private func told() {
        save()
        changed?()
    }

    // Periods that have run out: renewed (into the product chosen for renewal), or the subscription ends.
    private func roll() {
        guard var s = ledger.sub else { return }
        let now = clock.now()
        while s.expires <= now {
            guard s.renews, let p = product(s.next ?? s.product) else {
                ledger.sub = nil
                save()
                return
            }
            s.product = p.id
            s.next = nil
            s.trial = false
            s.expires = Self.period(p.billing, after: s.expires)
        }
        if s != ledger.sub {
            ledger.sub = s
            save()
        }
    }

    func activeNow() -> ActivePlan? {
        roll()
        guard let s = ledger.sub, let p = product(s.product) else { return nil }
        return ActivePlan(product: p.id, plan: p.plan, billing: p.billing, expires: s.expires, trial: s.trial, renews: s.renews, next: s.next)
    }

    func buyNow(_ id: String) -> ShopOutcome {
        roll()
        guard let p = product(id) else { return .failed(id) }
        let now = clock.now()
        if var s = ledger.sub, let current = product(s.product) {
            if s.product == id {
                guard !s.renews || s.next != nil else { return .nothing }
                s.renews = true
                s.next = nil
                ledger.sub = s
            } else if p.plan > current.plan {
                // An upgrade starts at once (a trial ends with it).
                ledger.sub = Sub(product: id, expires: Self.period(p.billing, after: now), trial: false, renews: true, next: nil)
            } else {
                // A downgrade or another period, at the next renewal.
                s.next = id
                s.renews = true
                ledger.sub = s
            }
        } else {
            let trial = !ledger.trialUsed ? p.trialDays : nil
            let end = trial.flatMap { Calendar(identifier: .gregorian).date(byAdding: .day, value: $0, to: now) } ?? Self.period(p.billing, after: now)
            ledger.sub = Sub(product: id, expires: end, trial: trial != nil, renews: true, next: nil)
            ledger.trialUsed = true
        }
        told()
        return .done
    }

    // MARK: the test store's own controls

    func cancelRenewal() {
        roll()
        ledger.sub?.renews = false
        ledger.sub?.next = nil
        told()
    }

    func expireNow() {
        ledger.sub = nil
        told()
    }

    func nextDay() {
        ledger.offset += 86_400
        clock.offset = ledger.offset
        roll()
        told()
    }

    func reset() {
        ledger = Ledger()
        clock.offset = 0
        told()
    }

    // A subscription as if bought a while ago (its trial not used): for the self-test.
    func give(_ id: String) {
        guard let p = product(id) else { return }
        ledger.sub = Sub(product: id, expires: Self.period(p.billing, after: clock.now()), trial: false, renews: true, next: nil)
        told()
    }

    // MARK: Shop

    func start(_ changed: @escaping @MainActor () -> Void) { self.changed = changed }
    func products() async throws -> [ShopProduct] { catalog }
    func active() async -> ActivePlan? { activeNow() }
    func trialEligible() async -> Bool { !ledger.trialUsed }
    func buy(_ id: String) async -> ShopOutcome { buyNow(id) }
    func restore() async -> ShopOutcome { restoreNow() }

    func restoreNow() -> ShopOutcome {
        if let file, let data = try? Data(contentsOf: file), let l = try? JSONDecoder().decode(Ledger.self, from: data) { ledger = l }
        return activeNow() == nil ? .nothing : .done
    }
}

// MARK: - Personal build

// Everything, for good: the personal build has no plans to buy.
@MainActor final class UnlockedShop: Shop {
    static let studio = ActivePlan(product: Catalog.id(.studio, .yearly), plan: .studio, billing: .yearly, expires: nil, trial: false, renews: true,
                                   next: nil)
    let isTest = false

    func start(_ changed: @escaping @MainActor () -> Void) {}
    func products() async throws -> [ShopProduct] { [] }
    func active() async -> ActivePlan? { Self.studio }
    func trialEligible() async -> Bool { false }
    func buy(_ id: String) async -> ShopOutcome { .nothing }
    func restore() async -> ShopOutcome { .done }
}

// MARK: - App Store

// StoreKit 2. Works once the app is signed and its products are in App Store Connect; until then it finds no products.
@MainActor final class AppStoreShop: Shop {
    let isTest = false
    private var cache: [String: Product] = [:]
    private var listener: Task<Void, Never>?

    func start(_ changed: @escaping @MainActor () -> Void) {
        // Renewals, refunds, Ask to Buy approvals and purchases on other devices arrive only here.
        listener = Task.detached {
            for await r in StoreKit.Transaction.updates {
                if case .verified(let t) = r { await t.finish() }
                await changed()
            }
        }
        Task.detached {
            for await r in StoreKit.Transaction.unfinished {
                if case .verified(let t) = r { await t.finish() }
            }
        }
    }

    func products() async throws -> [ShopProduct] {
        let found = try await Product.products(for: Catalog.ids)
        for p in found { cache[p.id] = p }
        return found.compactMap { p in
            guard let c = Catalog.product(p.id) else { return nil }
            var trial: Int?
            if let o = p.subscription?.introductoryOffer, o.paymentMode == .freeTrial { trial = Self.days(o.period) * o.periodCount }
            let style = p.priceFormatStyle
            return ShopProduct(id: p.id, plan: c.plan, billing: c.billing, price: p.price, displayPrice: p.displayPrice,
                               currency: style.currencyCode, locale: style.locale, trialDays: trial)
        }
    }

    static func days(_ p: Product.SubscriptionPeriod) -> Int {
        switch p.unit {
        case .day: p.value
        case .week: 7 * p.value
        case .month: 30 * p.value
        case .year: 365 * p.value
        @unknown default: p.value
        }
    }

    private func product(_ id: String) async -> Product? {
        if let p = cache[id] { return p }
        let p = try? await Product.products(for: [id]).first
        if let p { cache[id] = p }
        return p
    }

    // The highest plan among the subscriptions in force (signed, not refunded, not replaced by an upgrade).
    func active() async -> ActivePlan? {
        var best: (plan: Plan, t: StoreKit.Transaction)?
        for await r in StoreKit.Transaction.currentEntitlements {
            guard case .verified(let t) = r, t.productType == .autoRenewable, t.revocationDate == nil, !t.isUpgraded,
                  let c = Catalog.product(t.productID) else { continue }
            if best == nil || c.plan > best!.plan { best = (c.plan, t) }
        }
        guard let best, let c = Catalog.product(best.t.productID) else { return nil }
        var renews = true
        var next: String?
        if let p = await product(best.t.productID), let statuses = try? await p.subscription?.status {
            for s in statuses {
                guard case .verified(let t) = s.transaction, t.originalID == best.t.originalID, case .verified(let info) = s.renewalInfo else { continue }
                renews = info.willAutoRenew
                if let n = info.autoRenewPreference, n != best.t.productID { next = n }
            }
        }
        let trial = best.t.offer?.type == .introductory && best.t.offer?.paymentMode == .freeTrial
        return ActivePlan(product: best.t.productID, plan: best.plan, billing: c.billing, expires: best.t.expirationDate, trial: trial,
                          renews: renews, next: next)
    }

    func trialEligible() async -> Bool {
        guard let p = await product(Catalog.id(.pro, .monthly)), let s = p.subscription else { return false }
        return await s.isEligibleForIntroOffer
    }

    func buy(_ id: String) async -> ShopOutcome {
        guard let p = await product(id) else { return .failed(L("Couldn't reach the App Store")) }
        do {
            switch try await p.purchase() {
            case .success(let v):
                guard case .verified(let t) = v else { return .failed(L("The App Store couldn't confirm this purchase")) }
                await t.finish()
                return .done
            case .pending: return .pending
            case .userCancelled: return .cancelled
            @unknown default: return .cancelled
            }
        } catch {
            return .failed(error.localizedDescription)
        }
    }

    func restore() async -> ShopOutcome {
        do { try await AppStore.sync() } catch { return .failed(error.localizedDescription) }
        return await active() == nil ? .nothing : .done
    }
}

// MARK: - The free plan's file of the day

// Today's file: the document first saved or exported today on the free plan, by its id this session, the files it was
// written to (as hashes of their paths) and its shapes (a hash of their ids), so it's known again when opened later.
struct DayClaim: Codable, Equatable, Sendable {
    var day: String
    var doc: UUID
    var name: String
    var files: [String] = []
    var bodies: String?
}

protocol ClaimStore: AnyObject {
    func load() throws -> DayClaim?
    func save(_ c: DayClaim) throws
    func clear() throws
}

final class MemoryClaimStore: ClaimStore {
    struct Broken: Error {}
    var claim: DayClaim?
    // Every call fails, as a Keychain that can't be used would.
    var broken: Bool

    init(broken: Bool = false) { self.broken = broken }

    func load() throws -> DayClaim? {
        if broken { throw Broken() }
        return claim
    }

    func save(_ c: DayClaim) throws {
        if broken { throw Broken() }
        claim = c
    }

    func clear() throws {
        if broken { throw Broken() }
        claim = nil
    }
}

// In the login keychain: an item per day (named by its date), the claim in its attributes. Reading attributes and adding
// an item never ask for permission, even from a rebuilt app; the Keychain is told never to show a dialog.
final class KeychainClaimStore: ClaimStore {
    struct Failed: Error { let status: OSStatus }
    let service: String

    init(service: String) { self.service = service }

    private var base: [String: Any] { [kSecClass as String: kSecClassGenericPassword, kSecAttrService as String: service] }

    private func quietly<T>(_ f: () throws -> T) rethrows -> T {
        var was: DarwinBoolean = true
        SecKeychainGetUserInteractionAllowed(&was)
        SecKeychainSetUserInteractionAllowed(false)
        defer { SecKeychainSetUserInteractionAllowed(was.boolValue) }
        return try f()
    }

    // The latest day's claim.
    func load() throws -> DayClaim? {
        try quietly {
            var q = base
            q[kSecMatchLimit as String] = kSecMatchLimitAll
            q[kSecReturnAttributes as String] = true
            var out: CFTypeRef?
            let s = SecItemCopyMatching(q as CFDictionary, &out)
            if s == errSecItemNotFound { return nil }
            guard s == errSecSuccess else { throw Failed(status: s) }
            let items = out as? [[String: Any]] ?? []
            let claims = items.compactMap { ($0[kSecAttrGeneric as String] as? Data).flatMap { try? JSONDecoder().decode(DayClaim.self, from: $0) } }
            return claims.max { $0.day < $1.day }
        }
    }

    func save(_ c: DayClaim) throws {
        try quietly {
            let data = try JSONEncoder().encode(c)
            var q = base
            q[kSecAttrAccount as String] = "day " + c.day
            let s = SecItemUpdate(q as CFDictionary, [kSecAttrGeneric as String: data] as CFDictionary)
            if s == errSecSuccess { return }
            if s != errSecItemNotFound { _ = SecItemDelete(q as CFDictionary) }
            var add = q
            add[kSecAttrGeneric as String] = data
            add[kSecAttrLabel as String] = "Bcad: today's free file"
            add[kSecValueData as String] = Data([1])
            let a = SecItemAdd(add as CFDictionary, nil)
            guard a == errSecSuccess else { throw Failed(status: a) }
        }
        prune(before: c.day)
    }

    // Earlier days' items, as far as the Keychain lets them go.
    private func prune(before day: String) {
        quietly {
            var q = base
            q[kSecMatchLimit as String] = kSecMatchLimitAll
            q[kSecReturnAttributes as String] = true
            var out: CFTypeRef?
            guard SecItemCopyMatching(q as CFDictionary, &out) == errSecSuccess else { return }
            for item in out as? [[String: Any]] ?? [] {
                guard let account = item[kSecAttrAccount as String] as? String, account.hasPrefix("day "), String(account.dropFirst(4)) < day else { continue }
                var d = base
                d[kSecAttrAccount as String] = account
                _ = SecItemDelete(d as CFDictionary)
            }
        }
    }

    func clear() throws {
        try quietly {
            let s = SecItemDelete(base as CFDictionary)
            guard s == errSecSuccess || s == errSecItemNotFound else { throw Failed(status: s) }
        }
    }
}

// One document a day: the first saved or exported on a day is that day's; any other waits for the next. A day is only
// ever later than the last one used (the clock set back gives nothing new). When the store can't be used, the day is
// kept for this run only, and saving is never held up by it.
@MainActor final class Allowance {
    enum Verdict: Equatable {
        case open, mine, taken(String)
    }

    let store: ClaimStore
    let clock: PlanClock
    var calendar = Calendar.current
    private(set) var degraded = false
    private var memory: DayClaim?

    init(store: ClaimStore, clock: PlanClock) {
        self.store = store
        self.clock = clock
    }

    var today: String {
        let c = calendar.dateComponents([.year, .month, .day], from: clock.now())
        return String(format: "%04d-%02d-%02d", c.year ?? 0, c.month ?? 0, c.day ?? 0)
    }

    // The latest claim (re-read each time: another Bcad may have made one).
    var claim: DayClaim? {
        guard !degraded else { return memory }
        do {
            return try store.load()
        } catch {
            degraded = true
            return memory
        }
    }

    // Today's claim, if there is one.
    var todays: DayClaim? { claim.flatMap { today <= $0.day ? $0 : nil } }

    func verdict(_ doc: UUID) -> Verdict {
        guard let c = todays else { return .open }
        return c.doc == doc ? .mine : .taken(c.name)
    }

    // The document was written (to `file`, if a file of its own).
    func use(_ doc: UUID, name: String, file: URL?, bodies: String?) {
        var c: DayClaim
        if let old = todays {
            guard old.doc == doc else { return }
            c = old
        } else {
            c = DayClaim(day: today, doc: doc, name: name)
        }
        c.name = name
        if let file {
            let k = Self.key(file)
            c.files.removeAll { $0 == k }
            c.files.append(k)
            if c.files.count > 8 { c.files.removeFirst(c.files.count - 8) }
        }
        if let bodies { c.bodies = bodies }
        memory = c
        guard !degraded else { return }
        do { try store.save(c) } catch { degraded = true }
    }

    // A file opened: today's file, written or renamed earlier today, keeps its claim.
    func adopt(_ file: URL, bodies: String?) -> UUID? {
        guard let c = todays else { return nil }
        if c.files.contains(Self.key(file)) { return c.doc }
        if let bodies, bodies == c.bodies { return c.doc }
        return nil
    }

    // Today's file renamed.
    func moved(_ doc: UUID, to file: URL) {
        guard todays?.doc == doc else { return }
        use(doc, name: file.deletingPathExtension().lastPathComponent, file: file, bodies: nil)
    }

    func reset() {
        memory = nil
        try? store.clear()
    }

    nonisolated static func key(_ url: URL) -> String {
        hash(url.standardizedFileURL.resolvingSymlinksInPath().path)
    }

    // Its shapes (none: an empty document is nobody's).
    nonisolated static func fingerprint(_ d: Document) -> String? {
        d.bodies.isEmpty ? nil : hash(d.bodies.map(\.id.uuidString).sorted().joined(separator: ","))
    }

    private nonisolated static func hash(_ s: String) -> String {
        SHA256.hash(data: Data(s.utf8)).prefix(8).map { String(format: "%02x", $0) }.joined()
    }
}

// MARK: - Plans

// The plan in force and what the Plans card shows. Development builds and the self-test use the test store; the personal
// build (-D UNLOCKED) is always Studio; the App Store build (-D APPSTORE) uses StoreKit.
@Observable @MainActor final class Plans {
    let shop: any Shop
    let allowance: Allowance
    let clock: PlanClock
    private(set) var plan = Plan.free
    private(set) var active: ActivePlan?
    private(set) var products: [ShopProduct] = []
    private(set) var trialEligible = false
    // Whether the plan is known (the App Store's answer can take a moment after launch).
    private(set) var ready = false
    private(set) var storeProblem: String?
    private(set) var working = false
    // The Plans card: shown, the plan it points at, monthly or yearly, and why it opened.
    var showing = false
    var focus = Plan.pro
    var billing = Billing.yearly
    var reason: String?
    // Bumped whenever the plan or today's file may have changed, so views showing them look again.
    var stamp = 0
    @ObservationIgnored var openURL: (URL) -> Void = { _ = NSWorkspace.shared.open($0) }
    @ObservationIgnored private var waiting: [() -> Void] = []
    @ObservationIgnored private var expiry: Task<Void, Never>?
    @ObservationIgnored private var loading = false

    init(shop: any Shop, allowance: Allowance, clock: PlanClock) {
        self.shop = shop
        self.allowance = allowance
        self.clock = clock
        refresh()
    }

    static func make() -> Plans {
        let clock = PlanClock()
        #if SELFTEST
        // At noon today, as Studio, nothing kept anywhere.
        clock.fixed = Calendar.current.date(bySettingHour: 12, minute: 0, second: 0, of: Date())
        let shop = TestShop(catalog: TestShop.load(), clock: clock, file: nil)
        shop.give(Catalog.id(.studio, .yearly))
        return Plans(shop: shop, allowance: Allowance(store: MemoryClaimStore(), clock: clock), clock: clock)
        #elseif UNLOCKED
        return Plans(shop: UnlockedShop(), allowance: Allowance(store: MemoryClaimStore(), clock: clock), clock: clock)
        #elseif APPSTORE
        return Plans(shop: AppStoreShop(), allowance: Allowance(store: KeychainClaimStore(service: "Bcad"), clock: clock), clock: clock)
        #else
        let shop = TestShop(catalog: TestShop.load(), clock: clock, file: Paths.testStore)
        return Plans(shop: shop, allowance: Allowance(store: KeychainClaimStore(service: "Bcad"), clock: clock), clock: clock)
        #endif
    }

    var test: TestShop? { shop as? TestShop }
    // The personal build: always Studio, nothing about plans shown.
    var unlocked: Bool { shop is UnlockedShop }

    // At launch: changes made elsewhere followed, the plan looked at again when Bcad comes to the front. Until the store
    // answers, at most 5 seconds, the plan is taken as free.
    func start() {
        shop.start { [weak self] in self?.refresh() }
        NotificationCenter.default.addObserver(forName: NSApplication.didBecomeActiveNotification, object: nil, queue: .main) { [weak self] _ in
            MainActor.assumeIsolated { self?.refresh() }
        }
        Task { [weak self] in
            try? await Task.sleep(for: .seconds(5))
            self?.markReady()
        }
    }

    func refresh() {
        stamp += 1
        if unlocked {
            apply(UnlockedShop.studio, false)
            return
        }
        if let t = test {
            apply(t.activeNow(), !t.ledger.trialUsed)
            if products.isEmpty {
                products = t.catalog
                storeProblem = products.isEmpty ? L("Prices couldn't be loaded") : nil
            }
            return
        }
        Task { [weak self] in
            guard let self else { return }
            let a = await self.shop.active(), e = await self.shop.trialEligible()
            self.apply(a, e)
        }
    }

    private func apply(_ a: ActivePlan?, _ eligible: Bool) {
        active = a
        plan = a?.plan ?? .free
        trialEligible = eligible
        markReady()
        expiry?.cancel()
        // Looked at again as the period ends (a test store's clock moves only by its controls).
        if test == nil, let end = a?.expires {
            let wait = end.timeIntervalSince(clock.now()) + 1
            if wait > 0 && wait < 40 * 86_400 {
                expiry = Task { [weak self] in
                    try? await Task.sleep(for: .seconds(wait))
                    guard !Task.isCancelled else { return }
                    self?.refresh()
                }
            }
        }
    }

    private func markReady() {
        guard !ready else { return }
        ready = true
        let w = waiting
        waiting = []
        w.forEach { $0() }
    }

    // `f` once the plan is known.
    func whenReady(_ f: @escaping () -> Void) {
        if ready { f() } else { waiting.append(f) }
    }

    func loadProducts() {
        guard test == nil, !loading else { return }
        loading = true
        Task { [weak self] in
            guard let self else { return }
            do {
                let p = try await self.shop.products()
                self.products = p
                self.storeProblem = p.count == Catalog.ids.count ? nil : L("Prices couldn't be loaded")
            } catch {
                self.storeProblem = L("Prices couldn't be loaded")
            }
            self.loading = false
        }
    }

    func product(_ p: Plan, _ b: Billing) -> ShopProduct? { products.first { $0.plan == p && $0.billing == b } }

    // How much less a year costs than twelve months of the plan, in percent.
    func yearlySaving(_ p: Plan) -> Int? {
        guard let m = product(p, .monthly), let y = product(p, .yearly), m.price > 0 else { return nil }
        let s = 1 - NSDecimalNumber(decimal: y.price).doubleValue / (12 * NSDecimalNumber(decimal: m.price).doubleValue)
        return s > 0 ? Int((s * 100).rounded()) : nil
    }

    func buy(_ id: String, done: @escaping (ShopOutcome) -> Void) {
        guard !working else { return }
        working = true
        if let t = test {
            let r = t.buyNow(id)
            working = false
            refresh()
            done(r)
            return
        }
        Task { [weak self] in
            guard let self else { return }
            let r = await self.shop.buy(id)
            self.working = false
            self.refresh()
            done(r)
        }
    }

    func restore(done: @escaping (ShopOutcome) -> Void) {
        guard !working else { return }
        working = true
        if let t = test {
            let r = t.restoreNow()
            working = false
            refresh()
            done(r)
            return
        }
        Task { [weak self] in
            guard let self else { return }
            let r = await self.shop.restore()
            self.working = false
            self.refresh()
            done(r)
        }
    }
}
