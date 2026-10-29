# NEXUS iOS source validation

[![Source validation](https://github.com/ovaNite/NEXUS-iOS/actions/workflows/ios-build.yml/badge.svg)](https://github.com/ovaNite/NEXUS-iOS/actions/workflows/ios-build.yml)

Portable planner, automation models and SwiftUI controls. These libraries are preparation for integration into an authorized iOS game project.

## Verified on 29 October 2026

[Successful macOS cloud run](https://github.com/ovaNite/NEXUS-iOS/actions/runs/40940981140), source commit `956fda8a09eaf64b798b3c51658c166d86ec3dc8`:

- Genuine macOS 15.7.9, Apple Silicon, Xcode 16.4.
- Two Swift XCTest tests passed on macOS, zero failures.
- Native C planner host test passed on macOS.
- CNexusPlanner, NexusCore and NexusControls compiled for the ARM64 iOS Simulator with the iOS 18.5 SDK and iOS 16 minimum deployment target.

The simulator libraries were compiled; no simulator app was launched. There is no app executable or installable IPA. Real SoccerStars game integration, device testing and Apple signing remain missing. `UnavailableGameAdapter` intentionally rejects game operations.

## Re-run

Open Actions, select **NEXUS iOS source validation**, then **Run workflow** on `main`. The manual workflow uses the standard public `macos-15` runner, has a 15 minute limit and read-only repository permissions. It skips private repositories. No signing keys or account credentials are included.

Machine-readable details are in [cloud-status.json](cloud-status.json).
