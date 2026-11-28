// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "fault_core/history_store.hpp"

#include <filesystem>
#include <fstream>
#include <system_error>
#include <utility>

#include "fault_core/codec.hpp"

namespace fault_core
{

namespace
{

void read_events(const std::string & path, LoadResult & result)
{
  std::ifstream in(path);
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
}

}  // namespace

JsonlHistoryStore::JsonlHistoryStore(std::string path, std::uintmax_t max_bytes)
: path_(std::move(path)), max_bytes_(max_bytes) {}

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
  if (!out) {
    return false;
  }
  out.close();

  if (max_bytes_ > 0) {
    std::error_code error;
    const std::uintmax_t size = std::filesystem::file_size(path_, error);
    if (!error && size > max_bytes_) {
      // rename() replaces the old ".1" in one step. If it fails, the file
      // simply keeps growing: losing history would be worse.
      std::filesystem::rename(path_, rotated_path(), error);
    }
  }
  return true;
}

LoadResult JsonlHistoryStore::load(std::size_t max_events) const
{
  LoadResult result;
  read_events(rotated_path(), result);
  read_events(path_, result);
  if (max_events > 0 && result.events.size() > max_events) {
    result.events.erase(
      result.events.begin(), result.events.end() - static_cast<std::ptrdiff_t>(max_events));
  }
  return result;
}

}  // namespace fault_core
