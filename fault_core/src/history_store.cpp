// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "fault_core/history_store.hpp"

#include <fstream>
#include <utility>

#include "fault_core/codec.hpp"

namespace fault_core
{

JsonlHistoryStore::JsonlHistoryStore(std::string path)
: path_(std::move(path)) {}

bool JsonlHistoryStore::append(const std::vector<FaultEvent> & events)
{
  if (events.empty()) {
    return true;
  }
  std::ofstream out(path_, std::ios::app);
  if (!out) {
    return false;
  }
  for (const FaultEvent & event : events) {
    out << encode_event(event) << '\n';
  }
  out.flush();
  return static_cast<bool>(out);
}

LoadResult JsonlHistoryStore::load(std::size_t max_events) const
{
  LoadResult result;
  std::ifstream in(path_);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) {
      continue;
    }
    if (auto event = decode_event(line)) {
      result.events.push_back(std::move(*event));
    } else {
      ++result.skipped_lines;
    }
  }
  if (max_events > 0 && result.events.size() > max_events) {
    result.events.erase(
      result.events.begin(), result.events.end() - static_cast<std::ptrdiff_t>(max_events));
  }
  return result;
}

}  // namespace fault_core
