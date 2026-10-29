import Foundation
public enum EntryCost: Equatable, Sendable {
 case coins(Int), ownedItem(currency: String, amount: Int, available: Int), allIn(balance: Int), unavailable
 public func permitted(coinLimit: Int, wallet: Int) -> Bool {
  guard coinLimit >= 0, wallet >= 0 else { return false }
  switch self {
  case .coins(let amount): return amount >= 0 && amount <= coinLimit && amount <= wallet
  case .ownedItem(let currency, let amount, let available): return !currency.isEmpty && amount >= 0 && amount <= available
  case .allIn(let balance): return balance > 0 && balance == wallet && balance <= coinLimit
  case .unavailable: return false
  }
 }
 public var label: String {
  switch self {
  case .coins(let n): return "\(n) Coins"
  case .ownedItem(let currency, let n, _): return "\(n) \(currency)"
  case .allIn(let balance): return "Gesamtes Guthaben (\(balance) Coins)"
  case .unavailable: return "Kosten noch laden"
  }
 }
}
public struct Arena: Identifiable, Equatable, Sendable {
 public let id: String
 public var title: String
 public var cost: EntryCost
 public var bets: [Int]
 public var locked: Bool
 public init(id: String, title: String, cost: EntryCost, bets: [Int] = [], locked: Bool = false) {
  self.id=id; self.title=title; self.cost=cost; self.bets=bets; self.locked=locked
 }
}
public struct AutomationSettings: Codable, Equatable, Sendable {
 public var enabled = false
 public var coinLimit = 500
 public var selectedArenaIDs: Set<String> = []
 public var preferredBets: [String:Int] = [:]
 public var accuracy = 0
 public var scanSpeed = 0
 public init() {}
 public func canEnter(_ arena: Arena, wallet: Int) -> Bool {
  guard enabled, !arena.locked, selectedArenaIDs.contains(arena.id) else { return false }
  if let amount=preferredBets[arena.id] {
   guard arena.bets.contains(amount) else { return false }
   return EntryCost.coins(amount).permitted(coinLimit: coinLimit, wallet: wallet)
  }
  return arena.cost.permitted(coinLimit: coinLimit, wallet: wallet)
 }
}
public enum AdapterError: Error { case gameIntegrationMissing, expiredSnapshot, noVerifiedShot }
/* An authorized iOS game project supplies this adapter. Android ELF offsets,
 * JNI, root input and APK installation must never be used on iOS. */
public protocol NexusGameAdapter: Sendable {
 func availableArenas() async throws -> [Arena]
 func startSelectedArena(id: String, coins: Int?) async throws
 func verifyAndSubmitShot(puck: Int, angle: Double, power: Double, snapshotID: UUID) async throws
}
public struct UnavailableGameAdapter: NexusGameAdapter {
 public init() {}
 public func availableArenas() async throws -> [Arena] { throw AdapterError.gameIntegrationMissing }
 public func startSelectedArena(id: String, coins: Int?) async throws { throw AdapterError.gameIntegrationMissing }
 public func verifyAndSubmitShot(puck: Int, angle: Double, power: Double, snapshotID: UUID) async throws { throw AdapterError.gameIntegrationMissing }
}
