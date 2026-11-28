// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Behavioural tests of the fault state machine. Time is a number the test
// advances by hand, so "two seconds later" needs no sleeping.

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

#include "fault_core/fault_manager.hpp"

using namespace fault_core;  // NOLINT(build/namespaces)

namespace
{

TimePoint at(int milliseconds)
{
  return TimePoint{} + Duration{milliseconds};
}

FaultDefinition definition(
  const std::string & code, Severity severity, int activate_ms = 0, int clear_ms = 0,
  bool latched = false, int timeout_ms = 0)
{
  FaultDefinition result;
  result.code = code;
  result.severity = severity;
  result.description = "description of " + code;
  result.activate_after = Duration{activate_ms};
  result.clear_after = Duration{clear_ms};
  result.latched = latched;
  result.timeout = Duration{timeout_ms};
  return result;
}

}  // namespace

class FaultManagerTest : public ::testing::Test
{
protected:
  FaultCatalog catalog()
  {
    return FaultCatalog(
    {
      definition("INSTANT", Severity::Warning),
      definition("DEBOUNCED", Severity::Error, 500, 2000),
      definition("LATCHED", Severity::Critical, 0, 0, true),
      definition("LATCHED-SLOW", Severity::Critical, 0, 1000, true),
      definition("WATCHED", Severity::Error, 0, 0, false, 3000),
      definition("INFO", Severity::Info),
      });
  }

  FaultState state_of(const std::string & code) {return manager.get(code)->state;}

  std::vector<std::string> raised_codes()
  {
    std::vector<std::string> codes;
    for (const FaultRecord & record : manager.raised()) {
      codes.push_back(record.code);
    }
    return codes;
  }

  // A wall clock that advances one second per event, so stamps are distinct
  // and predictable.
  WallTime wall{std::chrono::seconds{1795722600}};
  FaultManager manager{catalog(), {}, [this] {
      const WallTime now = wall;
      wall += std::chrono::seconds{1};
      return now;
    }};
};

// ----------------------------------------------------------- basic cycle ----

TEST_F(FaultManagerTest, StartsWithNothingRaised)
{
  EXPECT_TRUE(manager.raised().empty());
  EXPECT_EQ(manager.summary().raised, 0u);
  EXPECT_FALSE(manager.summary().highest.has_value());
  EXPECT_TRUE(manager.history().empty());
  EXPECT_TRUE(state_of("INSTANT") == FaultState::Inactive);  // catalogued faults exist
  EXPECT_FALSE(manager.get("NEVER-SEEN").has_value());
}

TEST_F(FaultManagerTest, ActivatesAndClearsImmediatelyWithoutDebounce)
{
  manager.report("INSTANT", true, at(0), "lidar_driver", "scan rate 3 Hz");

  FaultRecord record = *manager.get("INSTANT");
  EXPECT_TRUE(record.state == FaultState::Active);
  EXPECT_TRUE(record.severity == Severity::Warning);
  EXPECT_EQ(record.description, "description of INSTANT");
  EXPECT_EQ(record.message, "scan rate 3 Hz");
  EXPECT_EQ(record.source, "lidar_driver");
  EXPECT_EQ(record.occurrence_count, 1u);
  EXPECT_TRUE(record.known);

  manager.report("INSTANT", false, at(100));
  EXPECT_TRUE(state_of("INSTANT") == FaultState::Inactive);

  const std::vector<FaultEvent> events = manager.take_new_events();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_TRUE(events[0].type == EventType::Activated);
  EXPECT_EQ(events[0].sequence, 1u);
  EXPECT_EQ(events[0].code, "INSTANT");
  EXPECT_EQ(events[0].message, "scan rate 3 Hz");
  EXPECT_TRUE(events[1].type == EventType::Cleared);
  EXPECT_EQ(events[1].sequence, 2u);
  EXPECT_TRUE(events[1].time > events[0].time);
  EXPECT_TRUE(manager.take_new_events().empty());  // handed over exactly once
}

TEST_F(FaultManagerTest, RepeatedFailingReportsAreOneOccurrence)
{
  for (int t = 0; t < 1000; t += 100) {
    manager.report("INSTANT", true, at(t));
  }
  EXPECT_EQ(manager.get("INSTANT")->occurrence_count, 1u);
  EXPECT_EQ(manager.take_new_events().size(), 1u);
}

TEST_F(FaultManagerTest, HealthyReportsForAnInactiveFaultDoNothing)
{
  manager.report("INSTANT", false, at(0));
  EXPECT_TRUE(state_of("INSTANT") == FaultState::Inactive);
  EXPECT_TRUE(manager.take_new_events().empty());
}

TEST_F(FaultManagerTest, CountsEachSeparateOccurrence)
{
  for (int i = 0; i < 3; ++i) {
    manager.report("INSTANT", true, at(i * 100));
    manager.report("INSTANT", false, at(i * 100 + 50));
  }
  const FaultRecord record = *manager.get("INSTANT");
  EXPECT_EQ(record.occurrence_count, 3u);
  EXPECT_TRUE(record.last_activated > record.first_activated);
  EXPECT_EQ(manager.history().size(), 6u);
}

TEST_F(FaultManagerTest, LatestFailingMessageIsKeptAndEmptyOnesDoNotEraseIt)
{
  manager.report("INSTANT", true, at(0), "a", "first");
  manager.report("INSTANT", true, at(10), "", "");
  EXPECT_EQ(manager.get("INSTANT")->message, "first");
  manager.report("INSTANT", true, at(20), "b", "second");
  EXPECT_EQ(manager.get("INSTANT")->message, "second");
  EXPECT_EQ(manager.get("INSTANT")->source, "b");
}

// -------------------------------------------------------------- debounce ----

TEST_F(FaultManagerTest, GlitchShorterThanDebounceNeverBecomesAFault)
{
  manager.report("DEBOUNCED", true, at(0));
  EXPECT_TRUE(state_of("DEBOUNCED") == FaultState::Pending);
  EXPECT_TRUE(manager.raised().empty());  // pending is not shown to an operator

  manager.report("DEBOUNCED", true, at(300));
  manager.report("DEBOUNCED", false, at(400));

  EXPECT_TRUE(state_of("DEBOUNCED") == FaultState::Inactive);
  EXPECT_TRUE(manager.take_new_events().empty());
  EXPECT_EQ(manager.get("DEBOUNCED")->occurrence_count, 0u);
}

TEST_F(FaultManagerTest, ActivatesOnceFailingForTheDebounceTime)
{
  manager.report("DEBOUNCED", true, at(0));
  manager.report("DEBOUNCED", true, at(499));
  EXPECT_TRUE(state_of("DEBOUNCED") == FaultState::Pending);
  manager.report("DEBOUNCED", true, at(500));
  EXPECT_TRUE(state_of("DEBOUNCED") == FaultState::Active);
}

TEST_F(FaultManagerTest, TickActivatesAPendingFaultWithoutAFurtherReport)
{
  // A monitor that reports only on change sends "failing" once.
  manager.report("DEBOUNCED", true, at(0));
  manager.tick(at(499));
  EXPECT_TRUE(state_of("DEBOUNCED") == FaultState::Pending);
  manager.tick(at(500));
  EXPECT_TRUE(state_of("DEBOUNCED") == FaultState::Active);
}

TEST_F(FaultManagerTest, DebounceRestartsAfterAHealthyReport)
{
  manager.report("DEBOUNCED", true, at(0));
  manager.report("DEBOUNCED", false, at(400));
  manager.report("DEBOUNCED", true, at(450));
  manager.tick(at(900));  // 450 ms into the new period, not 900
  EXPECT_TRUE(state_of("DEBOUNCED") == FaultState::Pending);
  manager.tick(at(950));
  EXPECT_TRUE(state_of("DEBOUNCED") == FaultState::Active);
}

TEST_F(FaultManagerTest, ClearsOnlyAfterBeingHealthyForTheClearTime)
{
  manager.report("DEBOUNCED", true, at(0));
  manager.tick(at(500));
  manager.report("DEBOUNCED", false, at(1000));
  EXPECT_TRUE(state_of("DEBOUNCED") == FaultState::Healing);
  ASSERT_EQ(raised_codes().size(), 1u);  // a healing fault is still shown

  manager.tick(at(2999));
  EXPECT_TRUE(state_of("DEBOUNCED") == FaultState::Healing);
  manager.tick(at(3000));
  EXPECT_TRUE(state_of("DEBOUNCED") == FaultState::Inactive);

  const std::vector<FaultEvent> events = manager.take_new_events();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_TRUE(events[1].type == EventType::Cleared);
}

TEST_F(FaultManagerTest, FlickeringConditionProducesOneEventNotAStorm)
{
  manager.report("DEBOUNCED", true, at(0));
  manager.tick(at(500));
  // Healthy for 1.9 s, failing for 0.1 s, twenty times over.
  int t = 500;
  for (int i = 0; i < 20; ++i) {
    manager.report("DEBOUNCED", false, at(t));
    manager.tick(at(t + 1900));
    manager.report("DEBOUNCED", true, at(t + 1900));
    t += 2000;
  }
  EXPECT_TRUE(state_of("DEBOUNCED") == FaultState::Active);
  EXPECT_EQ(manager.get("DEBOUNCED")->occurrence_count, 1u);
  // Filtered by code: the long run of ticks also (rightly) trips the watchdog
  // of the WATCHED fault, which is not what this test is about.
  HistoryQuery query;
  query.code = "DEBOUNCED";
  EXPECT_EQ(manager.history(query).size(), 1u);
}

// --------------------------------------------------------------- latching ----

TEST_F(FaultManagerTest, LatchedFaultStaysRaisedUntilAcknowledged)
{
  manager.report("LATCHED", true, at(0), "safety", "bumper pressed");
  manager.report("LATCHED", false, at(100));

  EXPECT_TRUE(state_of("LATCHED") == FaultState::AwaitingAck);
  ASSERT_EQ(raised_codes().size(), 1u);
  EXPECT_EQ(manager.take_new_events().size(), 1u);  // only Activated so far

  manager.tick(at(60000));
  EXPECT_TRUE(state_of("LATCHED") == FaultState::AwaitingAck);  // time alone never clears it

  EXPECT_TRUE(manager.acknowledge("LATCHED"));
  EXPECT_TRUE(state_of("LATCHED") == FaultState::Inactive);
  const std::vector<FaultEvent> events = manager.take_new_events();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_TRUE(events[0].type == EventType::Acknowledged);
  EXPECT_EQ(events[0].message, "bumper pressed");
}

TEST_F(FaultManagerTest, CannotAcknowledgeWhileTheCauseIsStillPresent)
{
  manager.report("LATCHED", true, at(0));
  EXPECT_FALSE(manager.acknowledge("LATCHED"));
  EXPECT_TRUE(state_of("LATCHED") == FaultState::Active);
}

TEST_F(FaultManagerTest, AcknowledgeIsRefusedForOtherFaults)
{
  EXPECT_FALSE(manager.acknowledge("NEVER-SEEN"));
  EXPECT_FALSE(manager.acknowledge("INSTANT"));  // inactive
  manager.report("INSTANT", true, at(0));
  EXPECT_FALSE(manager.acknowledge("INSTANT"));  // active, not latched
  EXPECT_TRUE(state_of("INSTANT") == FaultState::Active);
}

TEST_F(FaultManagerTest, CauseReturningBeforeAcknowledgeIsTheSameOccurrence)
{
  manager.report("LATCHED", true, at(0));
  manager.report("LATCHED", false, at(100));
  manager.report("LATCHED", true, at(200));

  EXPECT_TRUE(state_of("LATCHED") == FaultState::Active);
  EXPECT_EQ(manager.get("LATCHED")->occurrence_count, 1u);
  EXPECT_FALSE(manager.acknowledge("LATCHED"));
  EXPECT_EQ(manager.take_new_events().size(), 1u);
}

TEST_F(FaultManagerTest, LatchedFaultWithClearTimeWaitsForBoth)
{
  manager.report("LATCHED-SLOW", true, at(0));
  manager.report("LATCHED-SLOW", false, at(100));
  EXPECT_TRUE(state_of("LATCHED-SLOW") == FaultState::Healing);
  EXPECT_FALSE(manager.acknowledge("LATCHED-SLOW"));  // not yet proven healthy

  manager.tick(at(1100));
  EXPECT_TRUE(state_of("LATCHED-SLOW") == FaultState::AwaitingAck);
  EXPECT_TRUE(manager.acknowledge("LATCHED-SLOW"));
}

TEST_F(FaultManagerTest, NewOccurrenceAfterAcknowledgeCountsAgain)
{
  manager.report("LATCHED", true, at(0));
  manager.report("LATCHED", false, at(100));
  manager.acknowledge("LATCHED");
  manager.report("LATCHED", true, at(200));
  EXPECT_EQ(manager.get("LATCHED")->occurrence_count, 2u);
}

// --------------------------------------------------------------- watchdog ----

TEST_F(FaultManagerTest, WatchdogRaisesAFaultWhenAMonitorGoesSilent)
{
  manager.tick(at(0));
  manager.report("WATCHED", false, at(1000));  // healthy heartbeat
  manager.tick(at(3999));
  EXPECT_TRUE(state_of("WATCHED") == FaultState::Inactive);

  manager.tick(at(4000));  // 3000 ms since the last report
  const FaultRecord record = *manager.get("WATCHED");
  EXPECT_TRUE(record.state == FaultState::Active);
  EXPECT_EQ(record.source, "watchdog");
  EXPECT_EQ(record.message, "no report for 3000 ms");
}

TEST_F(FaultManagerTest, WatchdogCoversAMonitorThatNeverReportsAtAll)
{
  manager.tick(at(10000));  // the first tick starts the count
  manager.tick(at(12999));
  EXPECT_TRUE(state_of("WATCHED") == FaultState::Inactive);
  manager.tick(at(13000));
  EXPECT_TRUE(state_of("WATCHED") == FaultState::Active);
}

TEST_F(FaultManagerTest, WatchdogFaultClearsWhenReportsResume)
{
  manager.tick(at(0));
  manager.tick(at(3000));
  ASSERT_TRUE(state_of("WATCHED") == FaultState::Active);

  manager.report("WATCHED", false, at(3500));
  EXPECT_TRUE(state_of("WATCHED") == FaultState::Inactive);
  manager.tick(at(6499));
  EXPECT_TRUE(state_of("WATCHED") == FaultState::Inactive);
  manager.tick(at(6500));  // silent again for 3000 ms
  EXPECT_TRUE(state_of("WATCHED") == FaultState::Active);
  EXPECT_EQ(manager.get("WATCHED")->occurrence_count, 2u);
}

TEST_F(FaultManagerTest, RegularHeartbeatsKeepTheWatchdogQuiet)
{
  manager.tick(at(0));
  for (int t = 1000; t <= 20000; t += 1000) {
    manager.report("WATCHED", false, at(t));
    manager.tick(at(t + 500));
  }
  EXPECT_TRUE(state_of("WATCHED") == FaultState::Inactive);
  EXPECT_TRUE(manager.take_new_events().empty());
}

TEST_F(FaultManagerTest, FaultsWithoutATimeoutHaveNoWatchdog)
{
  manager.tick(at(0));
  manager.tick(at(1000000));
  EXPECT_TRUE(state_of("INSTANT") == FaultState::Inactive);
  EXPECT_TRUE(state_of("DEBOUNCED") == FaultState::Inactive);
}

// ---------------------------------------------------------- unknown codes ----

TEST_F(FaultManagerTest, UnknownCodeIsTrackedAndMarked)
{
  manager.report("TYPO-001", true, at(0), "somewhere", "what is this");
  const FaultRecord record = *manager.get("TYPO-001");
  EXPECT_TRUE(record.state == FaultState::Active);
  EXPECT_FALSE(record.known);
  EXPECT_TRUE(record.severity == Severity::Error);
  EXPECT_EQ(manager.dropped_reports(), 0u);
}

TEST_F(FaultManagerTest, UnknownCodesCanBeRefused)
{
  FaultManagerConfig config;
  config.accept_unknown_codes = false;
  FaultManager strict(catalog(), config);
  strict.report("TYPO-001", true, at(0));
  strict.report("", true, at(0));
  EXPECT_FALSE(strict.get("TYPO-001").has_value());
  EXPECT_TRUE(strict.raised().empty());
  EXPECT_EQ(strict.dropped_reports(), 2u);
}

TEST_F(FaultManagerTest, EmptyCodeIsAlwaysRefused)
{
  manager.report("", true, at(0));
  EXPECT_TRUE(manager.raised().empty());
  EXPECT_EQ(manager.dropped_reports(), 1u);
}

// ------------------------------------------------------ queries and limits ----

TEST_F(FaultManagerTest, RaisedFaultsAreListedMostSevereFirst)
{
  manager.report("INFO", true, at(0));
  manager.report("INSTANT", true, at(0));   // warning
  manager.report("LATCHED", true, at(0));   // critical
  manager.report("DEBOUNCED", true, at(0));
  manager.tick(at(500));                    // error

  const std::vector<std::string> codes = raised_codes();
  ASSERT_EQ(codes.size(), 4u);
  EXPECT_EQ(codes[0], "LATCHED");
  EXPECT_EQ(codes[1], "DEBOUNCED");
  EXPECT_EQ(codes[2], "INSTANT");
  EXPECT_EQ(codes[3], "INFO");

  const Summary summary = manager.summary();
  EXPECT_EQ(summary.raised, 4u);
  EXPECT_TRUE(*summary.highest == Severity::Critical);
  EXPECT_EQ(summary.by_severity[static_cast<int>(Severity::Info)], 1u);
  EXPECT_EQ(summary.by_severity[static_cast<int>(Severity::Warning)], 1u);
  EXPECT_EQ(summary.by_severity[static_cast<int>(Severity::Error)], 1u);
  EXPECT_EQ(summary.by_severity[static_cast<int>(Severity::Critical)], 1u);
}

TEST_F(FaultManagerTest, HistoryCanBeFiltered)
{
  manager.report("INFO", true, at(0));      // seq 1
  manager.report("INSTANT", true, at(0));   // seq 2
  manager.report("INFO", false, at(10));    // seq 3
  manager.report("LATCHED", true, at(20));  // seq 4

  HistoryQuery by_code;
  by_code.code = "INFO";
  EXPECT_EQ(manager.history(by_code).size(), 2u);

  HistoryQuery severe;
  severe.min_severity = Severity::Warning;
  const std::vector<FaultEvent> events = manager.history(severe);
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].code, "INSTANT");
  EXPECT_EQ(events[1].code, "LATCHED");

  HistoryQuery since;
  since.after_sequence = 2;
  ASSERT_EQ(manager.history(since).size(), 2u);
  EXPECT_EQ(manager.history(since)[0].sequence, 3u);

  HistoryQuery newest;
  newest.limit = 1;
  ASSERT_EQ(manager.history(newest).size(), 1u);
  EXPECT_EQ(manager.history(newest)[0].sequence, 4u);
}

TEST_F(FaultManagerTest, HistoryIsBounded)
{
  FaultManagerConfig config;
  config.history_capacity = 5;
  FaultManager small(catalog(), config);
  for (int i = 0; i < 10; ++i) {
    small.report("INSTANT", true, at(i * 10));
    small.report("INSTANT", false, at(i * 10 + 5));
  }
  const std::vector<FaultEvent> events = small.history();
  ASSERT_EQ(events.size(), 5u);
  EXPECT_EQ(events.back().sequence, 20u);   // newest kept
  EXPECT_EQ(events.front().sequence, 16u);
  EXPECT_EQ(small.take_new_events().size(), 5u);  // the uncollected queue is bounded too
}

TEST_F(FaultManagerTest, RestoredHistoryContinuesSequenceAndCounts)
{
  FaultEvent first;
  first.sequence = 41;
  first.type = EventType::Activated;
  first.code = "INSTANT";
  first.severity = Severity::Warning;
  first.time = WallTime{std::chrono::seconds{1000}};
  first.occurrence_count = 7;
  FaultEvent second = first;
  second.sequence = 42;
  second.type = EventType::Cleared;
  second.time = WallTime{std::chrono::seconds{1010}};

  manager.restore_history({first, second});

  EXPECT_EQ(manager.history().size(), 2u);
  EXPECT_TRUE(manager.take_new_events().empty());  // old events are not re-published
  EXPECT_TRUE(state_of("INSTANT") == FaultState::Inactive);  // live state is not restored
  EXPECT_EQ(manager.get("INSTANT")->occurrence_count, 7u);

  manager.report("INSTANT", true, at(0));
  const std::vector<FaultEvent> events = manager.take_new_events();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].sequence, 43u);
  EXPECT_EQ(events[0].occurrence_count, 8u);
  EXPECT_TRUE(manager.get("INSTANT")->first_activated == first.time);
}
