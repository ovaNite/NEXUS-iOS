// swift-tools-version: 5.9
import PackageDescription
let package = Package(name: "NexusPreparation", platforms: [.iOS(.v16), .macOS(.v13)],
 products: [.library(name: "NexusCore", targets: ["NexusCore"]), .library(name: "NexusControls", targets: ["NexusControls"])],
 targets: [.target(name: "CNexusPlanner", publicHeadersPath: "include", linkerSettings: [.linkedLibrary("m")]),
 .target(name: "NexusCore", dependencies: ["CNexusPlanner"]),
 .target(name: "NexusControls", dependencies: ["NexusCore"]),
 .testTarget(name: "NexusCoreTests", dependencies: ["NexusCore"])])
