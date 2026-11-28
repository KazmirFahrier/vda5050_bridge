// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "fault_core/vda5050_mapping.hpp"

#include <string>
#include <utility>

#include "fault_core/codec.hpp"

namespace fault_core
{

std::vector<vda5050_core::Error> to_vda5050_errors(
  const std::vector<FaultRecord> & raised, Severity min_severity)
{
  std::vector<vda5050_core::Error> errors;
  for (const FaultRecord & record : raised) {
    if (!is_raised(record.state) ||
      static_cast<int>(record.severity) < static_cast<int>(min_severity))
    {
      continue;
    }
    vda5050_core::Error error;
    error.error_type = record.code;
    error.error_level = record.severity == Severity::Critical ?
      vda5050_core::ErrorLevel::Fatal : vda5050_core::ErrorLevel::Warning;
    error.error_description = record.message.empty() ? record.description : record.message;
    error.error_references.push_back({"faultState", to_string(record.state)});
    if (!record.source.empty()) {
      error.error_references.push_back({"source", record.source});
    }
    error.error_references.push_back(
      {"occurrenceCount", std::to_string(record.occurrence_count)});
    errors.push_back(std::move(error));
  }
  return errors;
}

}  // namespace fault_core
