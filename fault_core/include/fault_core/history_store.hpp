// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Persistence of the fault history as a JSON Lines file: one event per line,
// appended as it happens. A line is either complete or (after a power cut)
// truncated, in which case that one line is skipped on load; nothing earlier
// in the file can be damaged by a later write.

#ifndef FAULT_CORE__HISTORY_STORE_HPP_
#define FAULT_CORE__HISTORY_STORE_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "fault_core/types.hpp"

namespace fault_core
{

struct LoadResult
{
  std::vector<FaultEvent> events;  ///< Oldest first.
  std::size_t skipped_lines{0};    ///< Lines that could not be decoded.
};

class JsonlHistoryStore
{
public:
  explicit JsonlHistoryStore(std::string path);

  /// Append events and flush. Returns false if the file could not be written;
  /// the caller decides whether that is itself a fault.
  bool append(const std::vector<FaultEvent> & events);

  /// Read the newest `max_events` events (0 = all). A missing file is an
  /// empty history, not an error.
  LoadResult load(std::size_t max_events = 0) const;

  const std::string & path() const noexcept {return path_;}

private:
  std::string path_;
};

}  // namespace fault_core

#endif  // FAULT_CORE__HISTORY_STORE_HPP_
