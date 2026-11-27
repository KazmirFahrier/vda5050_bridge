// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Session: everything that happens on the MQTT side of one vehicle.
//
// It owns the OrderManager and decides which message goes out on which topic
// and when. The actual MQTT client is behind MqttTransport, so this class is
// as testable as the state machine: no sockets, no threads, time passed in.

#ifndef VDA5050_CORE__SESSION_HPP_
#define VDA5050_CORE__SESSION_HPP_

#include <chrono>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "vda5050_core/header.hpp"
#include "vda5050_core/order_manager.hpp"
#include "vda5050_core/ports.hpp"
#include "vda5050_core/types.hpp"

namespace vda5050_core
{

struct MqttMessage
{
  std::string topic;
  std::string payload;
  int qos{0};
  bool retain{false};
};

/// Port for the MQTT client. Implementations must not call back into Session
/// from publish().
class MqttTransport
{
public:
  virtual ~MqttTransport() = default;
  virtual void publish(const MqttMessage & message) = 0;
};

struct SessionConfig
{
  /// Longest gap between two state messages when nothing changes.
  std::chrono::milliseconds state_interval{30000};

  /// Period of the visualization topic. Zero disables it.
  std::chrono::milliseconds visualization_interval{1000};

  OrderManagerConfig orders;
};

/// Not thread-safe; same single-caller contract as OrderManager.
///
/// Typical use by an adapter:
///   1. before connecting: register last_will() and subscribe to subscriptions()
///   2. when the broker accepts the connection: on_connected(now)
///   3. for each incoming message: on_message(topic, payload)
///   4. robot events go to orders() directly
///   5. call tick(now) often (e.g. 20 Hz); it publishes whatever is due
///   6. before a clean disconnect: shutdown()
class Session
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  Session(
    Identity identity, MqttTransport & transport, NavigationPort & navigation,
    ActionPort & actions, SessionConfig config = {},
    HeaderFactory::Clock clock = [] {return std::chrono::system_clock::now();});

  OrderManager & orders() noexcept {return orders_;}
  const Identity & identity() const noexcept {return headers_.identity();}
  bool connected() const noexcept {return connected_;}

  /// Topics the transport must subscribe to (order, instantActions).
  std::vector<std::string> subscriptions() const;

  /// The message the broker should publish for us if the connection dies:
  /// connectionState CONNECTIONBROKEN, QoS 1, retained.
  MqttMessage last_will();

  /// Factsheet content to publish when master control asks for it.
  void set_factsheet(Factsheet factsheet);

  /// The broker accepted the connection: announce ONLINE and send a full state.
  void on_connected(TimePoint now);

  /// The connection dropped. Nothing is published until on_connected() again;
  /// pending changes are kept and go out with the first state after reconnect.
  void on_disconnected();

  /// Announce OFFLINE ahead of a deliberate disconnect.
  void shutdown();

  /// Route an incoming payload. Payloads that cannot be decoded, or that are
  /// addressed to another vehicle or protocol version, are reported to master
  /// control as a validationError; they never throw.
  void on_message(const std::string & topic, std::string_view payload);

  /// Publish what is due: factsheet on request, state on change or when the
  /// interval has elapsed, visualization at its own rate.
  void tick(TimePoint now);

private:
  bool header_acceptable(const Header & header, const std::string & topic);
  void publish_connection(ConnectionState connection_state);
  void publish_state(TimePoint now);

  HeaderFactory headers_;
  MqttTransport & transport_;
  SessionConfig config_;
  OrderManager orders_;
  std::optional<Factsheet> factsheet_;

  bool connected_{false};
  std::optional<TimePoint> last_state_;
  std::optional<TimePoint> last_visualization_;
};

}  // namespace vda5050_core

#endif  // VDA5050_CORE__SESSION_HPP_
