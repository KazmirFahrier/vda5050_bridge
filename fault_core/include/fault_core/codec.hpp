// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// JSON forms of faults and events, used for persistence and for the web API.

#ifndef FAULT_CORE__CODEC_HPP_
#define FAULT_CORE__CODEC_HPP_

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "fault_core/fault_manager.hpp"
#include "fault_core/types.hpp"

namespace fault_core
{

const char * to_string(Severity value);
const char * to_string(FaultState value);
const char * to_string(EventType value);
std::optional<Severity> severity_from_string(const std::string & text);

/// ISO 8601 UTC with milliseconds, and back. An unparsable text gives nullopt.
std::string format_time(WallTime time);
std::optional<WallTime> parse_time(const std::string & text);

/// One event as a single-line JSON object.
std::string encode_event(const FaultEvent & event);

/// Inverse of encode_event. Returns nullopt for anything that is not a
/// complete, well-typed event, so a damaged history file is skipped line by
/// line instead of aborting start-up.
std::optional<FaultEvent> decode_event(std::string_view text);

/// {"faults":[...]} for a list of records.
std::string encode_records(const std::vector<FaultRecord> & records);

/// {"events":[...]} for a list of events.
std::string encode_events(const std::vector<FaultEvent> & events);

/// {"raised":2,"highest":"ERROR","bySeverity":{"INFO":0,...}}
std::string encode_summary(const Summary & summary);

}  // namespace fault_core

#endif  // FAULT_CORE__CODEC_HPP_
