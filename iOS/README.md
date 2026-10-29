# NEXUS iOS source package

Portable planner source aligned with Android r93. This is a library package, not an installable IPA.

Verified on a genuine macOS 15.7.9 ARM64 GitHub runner with Xcode 16.4: two Swift tests passed, C planner host smoke test passed, and all three targets compiled for the ARM64 iOS Simulator using the iOS 18.5 SDK. See the root README and cloud-status.json for the validated commit and run URL.

No simulator app was launched. Real game adapter, app target, Apple signing and device gameplay testing remain missing. Android-specific hooks, rewards, Facebook integration, graphics and input are not supplied. Candidate generation does not prove native physics or scoring accuracy.

Implement NexusGameAdapter in an authorized iOS game project against actual game physics, turn and input interfaces, then archive, sign and test that application. The Android key cannot sign an iOS app.
