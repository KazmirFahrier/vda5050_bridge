// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Vehicle identity, MQTT topic names and message header generation.

#ifndef VDA5050_CORE__HEADER_HPP_
#define VDA5050_CORE__HEADER_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <string>

#include "vda5050_core/types.hpp"

namespace vda5050_core
{

/// Who this vehicle is on the VDA 5050 network.
struct Identity
{
  std::string interface_name{"uagv"};
  std::string version{"2.0.0"};
  std::string manufacturer;
  std::string serial_number;
};

/// Build a topic name: <interfaceName>/<majorVersion>/<manufacturer>/<serialNumber>/<topic>
/// e.g. topic_name(id, "state") -> "uagv/v2/acme/amr-001/state".
std::string topic_name(const Identity & identity, const std::string & topic);

/// Format a time point as ISO 8601 UTC with milliseconds: 2026-11-26T19:50:00.123Z
std::string to_iso8601(std::chrono::system_clock::time_point time);

/// Produces message headers with a separate, monotonically increasing headerId
/// per topic, as the specification requires. The clock is injected so tests are
/// deterministic.
class HeaderFactory
{
public:
  using Clock = std::function<std::chrono::system_clock::time_point()>;

  explicit HeaderFactory(
    Identity identity,
    Clock clock = [] {return std::chrono::system_clock::now();});

  /// Header for the next message on `topic` ("state", "connection", ...).
  Header next(const std::string & topic);

  const Identity & identity() const noexcept {return identity_;}

private:
  Identity identity_;
  Clock clock_;
  std::map<std::string, std::uint32_t> counters_;
};

}  // namespace vda5050_core

#endif  // VDA5050_CORE__HEADER_HPP_
