// swift-tools-version:5.9
import PackageDescription

let package = Package(
    name: "VardeFM",
    platforms: [
        .iOS(.v16),
        .macOS(.v13),
    ],
    products: [
        .library(name: "CVardeFM", targets: ["CVardeFM"]),
    ],
    targets: [
        .binaryTarget(
            name: "CVardeFM",
            path: "build/VardeFM.xcframework"
        ),
        .testTarget(
            name: "CVardeFMTests",
            dependencies: ["CVardeFM"],
            path: "Tests/CVardeFMTests"
        ),
    ]
)
