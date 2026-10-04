// How many windows the named app has on screen: swiftc test/windows.swift -o windows && ./windows Bcad
import CoreGraphics

let name = CommandLine.arguments.dropFirst().first ?? "Bcad"
let windows = CGWindowListCopyWindowInfo([.optionOnScreenOnly, .excludeDesktopElements], kCGNullWindowID) as? [[String: Any]] ?? []
print(windows.filter { $0[kCGWindowOwnerName as String] as? String == name && $0[kCGWindowLayer as String] as? Int == 0 }.count)
