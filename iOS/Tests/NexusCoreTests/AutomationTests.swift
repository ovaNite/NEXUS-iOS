import XCTest
@testable import NexusCore
final class AutomationTests: XCTestCase {
 func testCostsAndSelections() {
  XCTAssertFalse(EntryCost.unavailable.permitted(coinLimit: 500, wallet: 10000))
  XCTAssertFalse(EntryCost.allIn(balance: 10000).permitted(coinLimit: 500, wallet: 10000))
  XCTAssertTrue(EntryCost.ownedItem(currency: "event_ticket", amount: 1, available: 1).permitted(coinLimit: 0, wallet: 0))
  XCTAssertFalse(EntryCost.ownedItem(currency: "event_ticket", amount: 1, available: 0).permitted(coinLimit: 500, wallet: 10000))
  var settings=AutomationSettings(); settings.enabled=true;settings.selectedArenaIDs=["trickshot"]
  let arena=Arena(id: "trickshot", title: "Trickshot", cost: .coins(100), bets: [100,500,1000])
  settings.preferredBets[arena.id]=500;XCTAssertTrue(settings.canEnter(arena, wallet: 10000))
  settings.preferredBets[arena.id]=1000;XCTAssertFalse(settings.canEnter(arena, wallet: 10000))
  settings.preferredBets[arena.id]=750;XCTAssertFalse(settings.canEnter(arena, wallet: 10000))
 }
 func testUnboundAdapterFailsClosed() async {
  do { _ = try await UnavailableGameAdapter().availableArenas();XCTFail("Must require actual integration") } catch { }
 }
}
