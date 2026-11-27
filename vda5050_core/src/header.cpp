// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "vda5050_core/header.hpp"

#include <cstdio>
#include <ctime>
#include <utility>

namespace vda5050_core
{

std::string topic_name(const Identity & identity, const std::string & topic)
{
  // "2.0.0" -> "v2". Only the major version is part of the topic so that
  // minor/patch upgrades do not split the fleet across topic trees.
  const std::string major = identity.version.substr(0, identity.version.find('.'));
  return identity.interface_name + "/v" + major + "/" + identity.manufacturer + "/" +
         identity.serial_number + "/" + topic;
}

std::string to_iso8601(std::chrono::system_clock::time_point time)
{
  using std::chrono::duration_cast;
  using std::chrono::milliseconds;

  const auto since_epoch = duration_cast<milliseconds>(time.time_since_epoch());
  // Floor division so times before 1970 also format correctly.
  auto seconds = since_epoch.count() / 1000;
  auto millis = since_epoch.count() % 1000;
  if (millis < 0) {
    millis += 1000;
    --seconds;
  }
  const std::time_t time_seconds = static_cast<std::time_t>(seconds);
  std::tm utc{};
  gmtime_r(&time_seconds, &utc);  // thread-safe, unlike std::gmtime

  char buffer[96];
  std::snprintf(
    buffer, sizeof(buffer), "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
    utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
    utc.tm_hour, utc.tm_min, utc.tm_sec, static_cast<int>(millis));
  return buffer;
}

HeaderFactory::HeaderFactory(Identity identity, Clock clock)
: identity_(std::move(identity)), clock_(std::move(clock)) {}

Header HeaderFactory::next(const std::string & topic)
{
  Header header;
  header.header_id = counters_[topic]++;  // first message on a topic gets id 0
  header.timestamp = to_iso8601(clock_());
  header.version = identity_.version;
  header.manufacturer = identity_.manufacturer;
  header.serial_number = identity_.serial_number;
  return header;
}

}  // namespace vda5050_core
