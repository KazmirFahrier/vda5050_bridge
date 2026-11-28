// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <string>

#include "fault_core/codec.hpp"
#include "vda5050_core/json.hpp"

using namespace fault_core;  // NOLINT(build/namespaces)
using vda5050_core::Json;

namespace
{

FaultEvent sample_event()
{
  FaultEvent event;
  event.sequence = 17;
  event.type = EventType::Activated;
  event.code = "NAV-001";
  event.severity = Severity::Critical;
  event.time = WallTime{std::chrono::milliseconds{1795722600123LL}};
  event.message = "path blocked by \"pallet\"";
  event.source = "controller_server";
  event.occurrence_count = 3;
  return event;
}

}  // namespace

TEST(FaultCodec, EnumNames)
{
  EXPECT_EQ(std::string(to_string(Severity::Info)), "INFO");
  EXPECT_EQ(std::string(to_string(Severity::Warning)), "WARNING");
  EXPECT_EQ(std::string(to_string(Severity::Error)), "ERROR");
  EXPECT_EQ(std::string(to_string(Severity::Critical)), "CRITICAL");
  EXPECT_EQ(std::string(to_string(FaultState::Inactive)), "INACTIVE");
  EXPECT_EQ(std::string(to_string(FaultState::Pending)), "PENDING");
  EXPECT_EQ(std::string(to_string(FaultState::Active)), "ACTIVE");
  EXPECT_EQ(std::string(to_string(FaultState::Healing)), "HEALING");
  EXPECT_EQ(std::string(to_string(FaultState::AwaitingAck)), "AWAITING_ACK");
  EXPECT_EQ(std::string(to_string(EventType::Activated)), "ACTIVATED");
  EXPECT_EQ(std::string(to_string(EventType::Cleared)), "CLEARED");
  EXPECT_EQ(std::string(to_string(EventType::Acknowledged)), "ACKNOWLEDGED");

  for (const Severity severity : {Severity::Info, Severity::Warning, Severity::Error,
      Severity::Critical})
  {
    EXPECT_TRUE(*severity_from_string(to_string(severity)) == severity);
  }
  EXPECT_FALSE(severity_from_string("error").has_value());  // case-sensitive
  EXPECT_FALSE(severity_from_string("").has_value());
}

TEST(FaultCodec, TimeRoundTripsToTheMillisecond)
{
  const WallTime time{std::chrono::milliseconds{1795722600123LL}};
  EXPECT_EQ(format_time(time), "2026-11-26T19:50:00.123Z");
  EXPECT_TRUE(*parse_time("2026-11-26T19:50:00.123Z") == time);
  EXPECT_TRUE(*parse_time("1970-01-01T00:00:00.000Z") == WallTime{});
}

TEST(FaultCodec, RejectsMalformedTimes)
{
  EXPECT_FALSE(parse_time("").has_value());
  EXPECT_FALSE(parse_time("2026-11-26 19:50:00.123Z").has_value());
  EXPECT_FALSE(parse_time("2026-11-26T19:50:00.123").has_value());
  EXPECT_FALSE(parse_time("2026-11-26T19:50:00.123+").has_value());
  EXPECT_FALSE(parse_time("2026-13-26T19:50:00.123Z").has_value());
  EXPECT_FALSE(parse_time("2026-11-32T19:50:00.123Z").has_value());
  EXPECT_FALSE(parse_time("2026-11-26T24:50:00.123Z").has_value());
  EXPECT_FALSE(parse_time("2026-11-26T19:60:00.123Z").has_value());
  EXPECT_FALSE(parse_time("not a time at all, sorry").has_value());
}

TEST(FaultCodec, EventRoundTrips)
{
  const FaultEvent event = sample_event();
  const std::string text = encode_event(event);
  EXPECT_EQ(text.find('\n'), std::string::npos);  // one line: required for JSON Lines

  const auto decoded = decode_event(text);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->sequence, 17u);
  EXPECT_TRUE(decoded->type == EventType::Activated);
  EXPECT_EQ(decoded->code, "NAV-001");
  EXPECT_TRUE(decoded->severity == Severity::Critical);
  EXPECT_TRUE(decoded->time == event.time);
  EXPECT_EQ(decoded->message, "path blocked by \"pallet\"");
  EXPECT_EQ(decoded->source, "controller_server");
  EXPECT_EQ(decoded->occurrence_count, 3u);
}

TEST(FaultCodec, DecodeRejectsAnythingThatIsNotACompleteEvent)
{
  EXPECT_FALSE(decode_event("").has_value());
  EXPECT_FALSE(decode_event("{").has_value());
  EXPECT_FALSE(decode_event("[]").has_value());
  EXPECT_FALSE(decode_event("{}").has_value());

  const std::string good = encode_event(sample_event());
  ASSERT_TRUE(decode_event(good).has_value());

  // Damage one field at a time.
  const auto with = [&good](const std::string & from, const std::string & to) {
      std::string text = good;
      text.replace(text.find(from), from.size(), to);
      return decode_event(text);
    };
  EXPECT_FALSE(with("\"ACTIVATED\"", "\"EXPLODED\"").has_value());
  EXPECT_FALSE(with("\"CRITICAL\"", "\"BAD\"").has_value());
  EXPECT_FALSE(with("\"sequence\":17", "\"sequence\":-1").has_value());
  EXPECT_FALSE(with("\"sequence\":17", "\"sequence\":1.5").has_value());
  EXPECT_FALSE(with("\"sequence\":17", "\"sequence\":\"17\"").has_value());
  EXPECT_FALSE(with("\"occurrenceCount\":3", "\"occurrenceCount\":null").has_value());
  EXPECT_FALSE(with("\"code\":\"NAV-001\"", "\"code\":\"\"").has_value());
  EXPECT_FALSE(with("\"code\":\"NAV-001\"", "\"code\":5").has_value());
  EXPECT_FALSE(with("2026-11-26T19:50:00.123Z", "yesterday").has_value());
  EXPECT_FALSE(good.empty());

  // A line cut short by a power failure.
  EXPECT_FALSE(decode_event(good.substr(0, good.size() / 2)).has_value());
}

TEST(FaultCodec, MessageAndSourceAreOptionalWhenDecoding)
{
  const auto decoded = decode_event(
    R"({"sequence":1,"type":"CLEARED","code":"X","severity":"INFO",)"
    R"("time":"2026-11-26T19:50:00.000Z","occurrenceCount":1})");
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->message, "");
  EXPECT_EQ(decoded->source, "");
}

TEST(FaultCodec, EncodesRecordsEventsAndSummary)
{
  FaultRecord active;
  active.code = "NAV-001";
  active.state = FaultState::Active;
  active.severity = Severity::Error;
  active.description = "Path blocked";
  active.message = "obstacle";
  active.source = "nav";
  active.occurrence_count = 2;
  active.first_activated = WallTime{std::chrono::seconds{1795722600}};
  active.last_activated = WallTime{std::chrono::seconds{1795722660}};
  FaultRecord never;
  never.code = "BAT-001";
  never.known = false;

  const Json records = Json::parse(encode_records({active, never}));
  const Json::Array & list = records.find("faults")->as_array();
  ASSERT_EQ(list.size(), 2u);
  EXPECT_EQ(list[0].find("code")->as_string(), "NAV-001");
  EXPECT_EQ(list[0].find("state")->as_string(), "ACTIVE");
  EXPECT_EQ(list[0].find("severity")->as_string(), "ERROR");
  EXPECT_EQ(list[0].find("lastActivated")->as_string(), "2026-11-26T19:51:00.000Z");
  EXPECT_TRUE(list[0].find("known")->as_bool());
  EXPECT_FALSE(list[1].find("known")->as_bool());
  EXPECT_EQ(list[1].find("firstActivated"), nullptr);  // never activated: no timestamps
  EXPECT_TRUE(Json::parse(encode_records({})).find("faults")->as_array().empty());

  const Json events = Json::parse(encode_events({sample_event()}));
  EXPECT_EQ(events.find("events")->as_array()[0].find("type")->as_string(), "ACTIVATED");

  Summary summary;
  summary.raised = 3;
  summary.highest = Severity::Critical;
  summary.by_severity[static_cast<int>(Severity::Warning)] = 2;
  summary.by_severity[static_cast<int>(Severity::Critical)] = 1;
  const Json encoded = Json::parse(encode_summary(summary));
  EXPECT_DOUBLE_EQ(encoded.find("raised")->as_number(), 3.0);
  EXPECT_EQ(encoded.find("highest")->as_string(), "CRITICAL");
  EXPECT_DOUBLE_EQ(encoded.find("bySeverity")->find("WARNING")->as_number(), 2.0);
  EXPECT_DOUBLE_EQ(encoded.find("bySeverity")->find("INFO")->as_number(), 0.0);

  EXPECT_TRUE(Json::parse(encode_summary(Summary{})).find("highest")->is_null());
}
