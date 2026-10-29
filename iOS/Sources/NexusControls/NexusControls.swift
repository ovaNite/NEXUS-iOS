import SwiftUI
import NexusCore
/* Embed in the real iOS project's own window. This package is not an app,
 * background controller, distribution profile or installable IPA. */
public struct NexusControls: View {
 @Binding private var settings: AutomationSettings
 private let arenas: [Arena]
 public init(settings: Binding<AutomationSettings>, arenas: [Arena]) { _settings=settings; self.arenas=arenas }
 public var body: some View {
  Form {
   Section("Spiel") {
    Picker("Suchgenauigkeit", selection: $settings.accuracy) { Text("Normal").tag(0);Text("Mittel").tag(1);Text("Stark").tag(2) }
    Picker("Scanspeed", selection: $settings.scanSpeed) { Text("1×").tag(0);Text("2×").tag(1);Text("3×").tag(2) }
   }
   Section("Automatik") {
    Toggle("Automatisch spielen", isOn: $settings.enabled)
    Stepper("Einsatzgrenze: \(settings.coinLimit) Coins", value: $settings.coinLimit, in: 0...100000000, step: 100)
    if arenas.isEmpty { Text("Das Spiel muss zuerst aktuelle Arenen und echte Kosten bereitstellen.").foregroundStyle(.secondary) }
    ForEach(arenas) { arena in
     Toggle(isOn: Binding(get: { settings.selectedArenaIDs.contains(arena.id) }, set: { if $0 { settings.selectedArenaIDs.insert(arena.id) } else { settings.selectedArenaIDs.remove(arena.id) } })) {
      VStack(alignment: .leading) { Text(arena.title);Text(arena.cost.label).font(.caption).foregroundStyle(.secondary) }
     }.disabled(arena.locked)
     if !arena.bets.isEmpty {
      Picker("Einsatz", selection: Binding(get: { settings.preferredBets[arena.id] ?? 0 }, set: { settings.preferredBets[arena.id] = $0 == 0 ? nil : $0 })) {
       Text("Automatisch im Limit").tag(0)
       ForEach(arena.bets, id: \.self) { Text("\($0) Coins").tag($0) }
      }
     }
    }
   }
   Section { Text("Vorbereitung r76 · Spielanbindung und iOS-Gerätetest stehen aus.").foregroundStyle(.secondary) }
  }.navigationTitle("NEXUS")
 }
}
