# NEXUS iOS source preparation, aligned with Android r93

THIS IS NOT AN INSTALLABLE IPA. iOS availability remains false.

Shared candidate-planner source files were refreshed from the current r93 main source and passed the existing Windows C host smoke test. This does not include Android-native game hooks, full bot equivalence, MSAA/GL logo port, game shader, rewards, Facebook integration or a verified iOS game adapter. A shot proposal is not proof of a goal.

The supplied Swift package remains source preparation. No Swift/Xcode toolchain or connected Mac is available here. No iOS game project, source integration or IPA was found. Apple build, signing, device testing and distribution remain blocked on those actual resources. No Apple credentials or signing keys are included. No website download is marked as available.

On a Mac: swift test in this directory validates the package when the toolchain is available. Embed the package into the authorized game project and implement NexusGameAdapter against the real iOS physics/turn/input interfaces. An app must then be archived, signed and tested using Xcode. The Android signing key cannot sign an iOS app.

Official Apple documentation: https://developer.apple.com/documentation/xcode/preparing-your-app-for-distribution
