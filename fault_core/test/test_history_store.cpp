// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "fault_core/fault_manager.hpp"
#include "fault_core/history_store.hpp"

using namespace fault_core;  // NOLINT(build/namespaces)

namespace
{

/// A file path that is removed again when the test ends.
class TempFile
{
public:
  explicit TempFile(const std::string & name)
  : path_((std::filesystem::temp_directory_path() / ("fault_core_" + name + ".jsonl")).string())
  {
    std::remove(path_.c_str());
  }
  ~TempFile() {std::remove(path_.c_str());}
  const std::string & path() const {return path_;}

private:
  std::string path_;
};

FaultEvent event(std::uint64_t sequence, const std::string & code, EventType type)
{
  FaultEvent result;
  result.sequence = sequence;
  result.type = type;
  result.code = code;
  result.severity = Severity::Error;
  result.time = WallTime{std::chrono::seconds{1795722600 + static_cast<int>(sequence)}};
  result.message = "message " + std::to_string(sequence);
  result.occurrence_count = sequence;
  return result;
}

}  // namespace

TEST(HistoryStore, MissingFileIsAnEmptyHistory)
{
  TempFile file("missing");
  const LoadResult result = JsonlHistoryStore(file.path()).load();
  EXPECT_TRUE(result.events.empty());
  EXPECT_EQ(result.skipped_lines, 0u);
}

TEST(HistoryStore, AppendsAndLoadsInOrder)
{
  TempFile file("append");
  JsonlHistoryStore store(file.path());
  EXPECT_TRUE(store.append({event(1, "A", EventType::Activated)}));
  EXPECT_TRUE(store.append({}));  // nothing to write is still a success
  EXPECT_TRUE(
    store.append({event(2, "A", EventType::Cleared), event(3, "B", EventType::Activated)}));

  const LoadResult result = store.load();
  ASSERT_EQ(result.events.size(), 3u);
  EXPECT_EQ(result.events[0].sequence, 1u);
  EXPECT_EQ(result.events[2].code, "B");
  EXPECT_EQ(result.events[2].message, "message 3");
  EXPECT_EQ(result.skipped_lines, 0u);
}

TEST(HistoryStore, LoadCanKeepOnlyTheNewest)
{
  TempFile file("newest");
  JsonlHistoryStore store(file.path());
  std::vector<FaultEvent> events;
  for (std::uint64_t i = 1; i <= 10; ++i) {
    events.push_back(event(i, "A", EventType::Activated));
  }
  store.append(events);

  const LoadResult result = store.load(3);
  ASSERT_EQ(result.events.size(), 3u);
  EXPECT_EQ(result.events.front().sequence, 8u);
  EXPECT_EQ(result.events.back().sequence, 10u);
}

TEST(HistoryStore, SkipsDamagedLinesAndKeepsTheRest)
{
  TempFile file("damaged");
  JsonlHistoryStore store(file.path());
  store.append({event(1, "A", EventType::Activated)});
  {
    // Garbage, an empty line, and a line cut off mid-write by a power failure.
    std::ofstream out(file.path(), std::ios::app);
    out << "this is not json\n\n" << R"({"sequence":2,"type":"ACTIV)" << "\n";
  }
  store.append({event(3, "A", EventType::Cleared)});

  const LoadResult result = store.load();
  ASSERT_EQ(result.events.size(), 2u);
  EXPECT_EQ(result.events[0].sequence, 1u);
  EXPECT_EQ(result.events[1].sequence, 3u);
  EXPECT_EQ(result.skipped_lines, 2u);  // the empty line is not counted as damage
}

TEST(HistoryStore, AppendReportsFailureForAnUnwritablePath)
{
  JsonlHistoryStore store("/nonexistent-directory/for/sure/history.jsonl");
  EXPECT_FALSE(store.append({event(1, "A", EventType::Activated)}));
  EXPECT_TRUE(store.load().events.empty());
  EXPECT_EQ(store.path(), "/nonexistent-directory/for/sure/history.jsonl");
}

TEST(HistoryStore, HistorySurvivesARestartOfTheManager)
{
  TempFile file("restart");
  FaultDefinition definition;
  definition.code = "BAT-001";
  definition.severity = Severity::Warning;
  const FaultCatalog catalog({definition});
  const TimePoint t0{};

  {
    FaultManager first_run(catalog);
    JsonlHistoryStore store(file.path());
    first_run.report("BAT-001", true, t0, "bms", "cell 3 low");
    first_run.report("BAT-001", false, t0 + Duration{10});
    first_run.report("BAT-001", true, t0 + Duration{20});
    EXPECT_TRUE(store.append(first_run.take_new_events()));
  }

  FaultManager second_run(catalog);
  second_run.restore_history(JsonlHistoryStore(file.path()).load().events);

  EXPECT_EQ(second_run.history().size(), 3u);
  EXPECT_EQ(second_run.get("BAT-001")->occurrence_count, 2u);
  EXPECT_TRUE(second_run.raised().empty());  // must be re-detected, not assumed

  second_run.report("BAT-001", true, t0);
  const std::vector<FaultEvent> events = second_run.take_new_events();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].sequence, 4u);
  EXPECT_EQ(events[0].occurrence_count, 3u);
}
