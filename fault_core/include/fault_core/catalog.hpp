// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// The fault catalogue: the list of faults a robot can raise, with their
// severity and timing rules. Kept as data (JSON) so that operations staff can
// tune debounce times or severities without recompiling.

#ifndef FAULT_CORE__CATALOG_HPP_
#define FAULT_CORE__CATALOG_HPP_

#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "fault_core/types.hpp"

namespace fault_core
{

/// Thrown when a catalogue document is malformed. The message names the entry.
class CatalogError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class FaultCatalog
{
public:
  FaultCatalog() = default;

  /// Build from definitions. Throws CatalogError on an empty or duplicate code
  /// or a negative duration.
  explicit FaultCatalog(const std::vector<FaultDefinition> & definitions);

  /// Parse a catalogue document:
  ///
  ///   {"faults": [
  ///     {"code": "NAV-001", "severity": "ERROR", "description": "Path blocked",
  ///      "activateAfterMs": 500, "clearAfterMs": 2000, "latched": false, "timeoutMs": 0}
  ///   ]}
  ///
  /// Only "code" is required. severity is one of INFO, WARNING, ERROR, CRITICAL
  /// (default ERROR); durations default to 0; latched defaults to false.
  static FaultCatalog from_json(std::string_view text);

  /// The definition for `code`, or nullptr if the catalogue does not list it.
  const FaultDefinition * find(const std::string & code) const;

  const std::map<std::string, FaultDefinition> & definitions() const noexcept
  {
    return definitions_;
  }

private:
  void add(const FaultDefinition & definition);

  std::map<std::string, FaultDefinition> definitions_;
};

}  // namespace fault_core

#endif  // FAULT_CORE__CATALOG_HPP_
