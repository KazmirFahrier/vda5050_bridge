// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// MQTT transport built on libmosquitto.
//
// Threading: libmosquitto runs its network loop on its own thread and invokes
// the callbacks there. The protocol core is single-threaded, so callbacks do
// nothing except append an event to a mutex-protected queue. The node drains
// that queue from its timer, on the executor thread. No protocol code ever
// runs on the MQTT thread.

#ifndef VDA5050_NAV2_ADAPTER__MOSQUITTO_TRANSPORT_HPP_
#define VDA5050_NAV2_ADAPTER__MOSQUITTO_TRANSPORT_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "vda5050_core/session.hpp"

struct mosquitto;
struct mosquitto_message;

namespace vda5050_nav2_adapter
{

struct TransportEvent
{
  enum class Type {Connected, Disconnected, Message};
  Type type;
  std::string topic;    // Message only
  std::string payload;  // Message only
};

class MosquittoTransport : public vda5050_core::MqttTransport
{
public:
  struct Options
  {
    std::string host{"localhost"};
    int port{1883};
    int keepalive_s{15};
    std::string client_id;
    std::string username;
    std::string password;
  };

  explicit MosquittoTransport(Options options);
  ~MosquittoTransport() override;

  MosquittoTransport(const MosquittoTransport &) = delete;
  MosquittoTransport & operator=(const MosquittoTransport &) = delete;

  /// Both must be called before connect().
  void set_will(const vda5050_core::MqttMessage & will);
  void set_subscriptions(std::vector<std::string> topics);

  /// Start connecting in the background. The broker's answer arrives later as
  /// a Connected event; lost connections are retried automatically with backoff.
  /// Returns false if the client could not be started at all.
  bool connect();

  /// Send DISCONNECT and stop the network thread.
  void disconnect();

  /// Thread-safe. Messages published while offline are dropped by design:
  /// VDA 5050 state is a snapshot, and a stale one is worse than none.
  void publish(const vda5050_core::MqttMessage & message) override;

  /// Hand over everything that happened since the last call.
  std::vector<TransportEvent> take_events();

private:
  static void on_connect(struct mosquitto *, void * self, int result_code);
  static void on_disconnect(struct mosquitto *, void * self, int result_code);
  static void on_message(struct mosquitto *, void * self, const struct mosquitto_message * message);
  void push(TransportEvent event);

  Options options_;
  struct mosquitto * client_{nullptr};
  bool loop_running_{false};
  std::vector<std::string> subscriptions_;

  std::mutex mutex_;
  std::vector<TransportEvent> events_;
};

}  // namespace vda5050_nav2_adapter

#endif  // VDA5050_NAV2_ADAPTER__MOSQUITTO_TRANSPORT_HPP_
