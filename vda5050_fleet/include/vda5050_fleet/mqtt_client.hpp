// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// MQTT client built on libmosquitto, for the fleet manager and the simulated
// vehicle. The same design as the adapter's transport, which lives in a ROS
// package and cannot be used from here.
//
// Threading: libmosquitto runs its network loop on its own thread and invokes
// the callbacks there. Everything that uses this class is single-threaded, so
// the callbacks do nothing except append an event to a mutex-protected queue,
// which the main loop drains.

#ifndef VDA5050_FLEET__MQTT_CLIENT_HPP_
#define VDA5050_FLEET__MQTT_CLIENT_HPP_

#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

#include "vda5050_core/session.hpp"

struct mosquitto;
struct mosquitto_message;

namespace vda5050_fleet
{

struct MqttEvent
{
  enum class Type {Connected, Disconnected, Message};
  Type type;
  std::string topic;    // Message only
  std::string payload;  // Message only
};

class MqttClient : public vda5050_core::MqttTransport
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

  explicit MqttClient(Options options);
  ~MqttClient() override;

  MqttClient(const MqttClient &) = delete;
  MqttClient & operator=(const MqttClient &) = delete;

  /// Both must be called before connect().
  void set_will(const vda5050_core::MqttMessage & will);
  void set_subscriptions(std::vector<std::string> topics);

  /// Start connecting in the background. The broker's answer arrives later as
  /// a Connected event; lost connections are retried automatically with backoff.
  /// Returns false if the client could not be started at all.
  bool connect();

  /// Wait briefly for messages already handed to the client to reach the
  /// broker (so a final OFFLINE is not lost), then send DISCONNECT and stop
  /// the network thread.
  void disconnect();

  /// Thread-safe. Messages published while offline are dropped by design: an
  /// order or a state that arrives late is worse than one that does not
  /// arrive; whoever sent it notices and sends what is right by then.
  void publish(const vda5050_core::MqttMessage & message) override;

  /// Hand over everything that happened since the last call.
  std::vector<MqttEvent> take_events();

private:
  static void on_connect(struct mosquitto *, void * self, int result_code);
  static void on_disconnect(struct mosquitto *, void * self, int result_code);
  static void on_message(struct mosquitto *, void * self, const struct mosquitto_message * message);
  static void on_publish(struct mosquitto *, void * self, int message_id);
  void push(MqttEvent event);

  Options options_;
  struct mosquitto * client_{nullptr};
  bool loop_running_{false};
  std::vector<std::string> subscriptions_;

  std::mutex mutex_;
  std::vector<MqttEvent> events_;
  int in_flight_{0};  ///< Publishes accepted by the client but not yet sent/acknowledged.
  std::condition_variable in_flight_changed_;
};

}  // namespace vda5050_fleet

#endif  // VDA5050_FLEET__MQTT_CLIENT_HPP_
