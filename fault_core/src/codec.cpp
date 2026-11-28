// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "fault_core/codec.hpp"

#include <cmath>
#include <cstdio>
#include <ctime>

#include "vda5050_core/header.hpp"
#include "vda5050_core/json.hpp"

namespace fault_core
{

namespace
{

using vda5050_core::Json;

std::optional<EventType> event_type_from_string(const std::string & text)
{
  if (text == "ACTIVATED") {return EventType::Activated;}
  if (text == "CLEARED") {return EventType::Cleared;}
  if (text == "ACKNOWLEDGED") {return EventType::Acknowledged;}
  return std::nullopt;
}

Json event_to_json(const FaultEvent & event)
{
  Json out;
  out["sequence"] = event.sequence;
  out["type"] = to_string(event.type);
  out["code"] = event.code;
  out["severity"] = to_string(event.severity);
  out["time"] = format_time(event.time);
  out["message"] = event.message;
  out["source"] = event.source;
  out["occurrenceCount"] = event.occurrence_count;
  return out;
}

/// A non-negative whole number that fits in 53 bits, or nullopt.
std::optional<std::uint64_t> as_count(const Json * value)
{
  if (value == nullptr || !value->is_number()) {
    return std::nullopt;
  }
  const double number = value->as_number();
  if (number < 0.0 || number != std::floor(number) || number > 9.0e15) {
    return std::nullopt;
  }
  return static_cast<std::uint64_t>(number);
}

}  // namespace

const char * to_string(Severity value)
{
  switch (value) {
    case Severity::Info: return "INFO";
    case Severity::Warning: return "WARNING";
    case Severity::Error: return "ERROR";
    case Severity::Critical: return "CRITICAL";
  }
  return "ERROR";
}

const char * to_string(FaultState value)
{
  switch (value) {
    case FaultState::Inactive: return "INACTIVE";
    case FaultState::Pending: return "PENDING";
    case FaultState::Active: return "ACTIVE";
    case FaultState::Healing: return "HEALING";
    case FaultState::AwaitingAck: return "AWAITING_ACK";
  }
  return "INACTIVE";
}

const char * to_string(EventType value)
{
  switch (value) {
    case EventType::Activated: return "ACTIVATED";
    case EventType::Cleared: return "CLEARED";
    case EventType::Acknowledged: return "ACKNOWLEDGED";
  }
  return "ACTIVATED";
}

std::optional<Severity> severity_from_string(const std::string & text)
{
  if (text == "INFO") {return Severity::Info;}
  if (text == "WARNING") {return Severity::Warning;}
  if (text == "ERROR") {return Severity::Error;}
  if (text == "CRITICAL") {return Severity::Critical;}
  return std::nullopt;
}

std::string format_time(WallTime time)
{
  return vda5050_core::to_iso8601(time);
}

std::optional<WallTime> parse_time(const std::string & text)
{
  // Exactly the form format_time() writes: 2026-11-26T19:50:00.123Z
  int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0, millis = 0;
  char terminator = '\0';
  if (text.size() != 24 ||
    std::sscanf(
      text.c_str(), "%4d-%2d-%2dT%2d:%2d:%2d.%3d%c",
      &year, &month, &day, &hour, &minute, &second, &millis, &terminator) != 8 ||
    terminator != 'Z')
  {
    return std::nullopt;
  }
  if (month < 1 || month > 12 || day < 1 || day > 31 || hour > 23 || minute > 59 ||
    second > 60 || hour < 0 || minute < 0 || second < 0 || millis < 0)
  {
    return std::nullopt;
  }
  std::tm utc{};
  utc.tm_year = year - 1900;
  utc.tm_mon = month - 1;
  utc.tm_mday = day;
  utc.tm_hour = hour;
  utc.tm_min = minute;
  utc.tm_sec = second;
  const std::time_t seconds = timegm(&utc);
  return std::chrono::system_clock::from_time_t(seconds) + std::chrono::milliseconds(millis);
}

std::string encode_event(const FaultEvent & event)
{
  return event_to_json(event).dump();
}

std::optional<FaultEvent> decode_event(std::string_view text)
{
  Json json;
  try {
    json = Json::parse(text);
  } catch (const vda5050_core::JsonParseError &) {
    return std::nullopt;
  }
  const Json * type = json.find("type");
  const Json * code = json.find("code");
  const Json * severity = json.find("severity");
  const Json * time = json.find("time");
  const auto sequence = as_count(json.find("sequence"));
  const auto occurrences = as_count(json.find("occurrenceCount"));
  if (type == nullptr || !type->is_string() || code == nullptr || !code->is_string() ||
    code->as_string().empty() || severity == nullptr || !severity->is_string() ||
    time == nullptr || !time->is_string() || !sequence || !occurrences)
  {
    return std::nullopt;
  }
  const auto parsed_type = event_type_from_string(type->as_string());
  const auto parsed_severity = severity_from_string(severity->as_string());
  const auto parsed_time = parse_time(time->as_string());
  if (!parsed_type || !parsed_severity || !parsed_time) {
    return std::nullopt;
  }

  FaultEvent event;
  event.sequence = *sequence;
  event.type = *parsed_type;
  event.code = code->as_string();
  event.severity = *parsed_severity;
  event.time = *parsed_time;
  event.occurrence_count = *occurrences;
  if (const Json * message = json.find("message"); message != nullptr && message->is_string()) {
    event.message = message->as_string();
  }
  if (const Json * source = json.find("source"); source != nullptr && source->is_string()) {
    event.source = source->as_string();
  }
  return event;
}

std::string encode_records(const std::vector<FaultRecord> & records)
{
  Json list{Json::Array{}};
  for (const FaultRecord & record : records) {
    Json item;
    item["code"] = record.code;
    item["state"] = to_string(record.state);
    item["severity"] = to_string(record.severity);
    item["description"] = record.description;
    item["message"] = record.message;
    item["source"] = record.source;
    item["known"] = record.known;
    item["occurrenceCount"] = record.occurrence_count;
    if (record.occurrence_count > 0) {
      item["firstActivated"] = format_time(record.first_activated);
      item["lastActivated"] = format_time(record.last_activated);
    }
    list.push_back(std::move(item));
  }
  Json out;
  out["faults"] = std::move(list);
  return out.dump();
}

std::string encode_events(const std::vector<FaultEvent> & events)
{
  Json list{Json::Array{}};
  for (const FaultEvent & event : events) {
    list.push_back(event_to_json(event));
  }
  Json out;
  out["events"] = std::move(list);
  return out.dump();
}

std::string encode_summary(const Summary & summary)
{
  Json counts;
  for (const Severity severity : {Severity::Info, Severity::Warning, Severity::Error,
      Severity::Critical})
  {
    counts[to_string(severity)] = summary.by_severity[static_cast<int>(severity)];
  }
  Json out;
  out["raised"] = summary.raised;
  out["highest"] = summary.highest ? Json(to_string(*summary.highest)) : Json(nullptr);
  out["bySeverity"] = std::move(counts);
  return out.dump();
}

}  // namespace fault_core
