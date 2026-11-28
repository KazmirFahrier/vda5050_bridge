// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "vda5050_fleet/mqtt_client.hpp"

#include <mosquitto.h>

#include <chrono>
#include <stdexcept>
#include <utility>

namespace vda5050_fleet
{

MqttClient::MqttClient(Options options)
: options_(std::move(options))
{
  mosquitto_lib_init();
  // clean_session = true: the broker keeps no queue for us while we are away.
  // What was sent during an outage must not arrive late and be acted upon.
  client_ = mosquitto_new(
    options_.client_id.empty() ? nullptr : options_.client_id.c_str(), true, this);
  if (client_ == nullptr) {
    mosquitto_lib_cleanup();
    throw std::runtime_error("mosquitto_new failed");
  }
  mosquitto_connect_callback_set(client_, &MqttClient::on_connect);
  mosquitto_disconnect_callback_set(client_, &MqttClient::on_disconnect);
  mosquitto_message_callback_set(client_, &MqttClient::on_message);
  mosquitto_publish_callback_set(client_, &MqttClient::on_publish);
  // Reconnect after 1 s, doubling up to 30 s.
  mosquitto_reconnect_delay_set(client_, 1, 30, true);
  if (!options_.username.empty()) {
    mosquitto_username_pw_set(client_, options_.username.c_str(), options_.password.c_str());
  }
}

MqttClient::~MqttClient()
{
  disconnect();
  mosquitto_destroy(client_);
  mosquitto_lib_cleanup();
}

void MqttClient::set_will(const vda5050_core::MqttMessage & will)
{
  mosquitto_will_set(
    client_, will.topic.c_str(), static_cast<int>(will.payload.size()), will.payload.data(),
    will.qos, will.retain);
}

void MqttClient::set_subscriptions(std::vector<std::string> topics)
{
  subscriptions_ = std::move(topics);
}

bool MqttClient::connect()
{
  if (loop_running_) {
    return true;
  }
  // connect_async + loop_start: never blocks the caller, even if the broker
  // is unreachable at start-up.
  if (mosquitto_connect_async(
      client_, options_.host.c_str(), options_.port, options_.keepalive_s) != MOSQ_ERR_SUCCESS)
  {
    return false;
  }
  if (mosquitto_loop_start(client_) != MOSQ_ERR_SUCCESS) {
    return false;
  }
  loop_running_ = true;
  return true;
}

void MqttClient::disconnect()
{
  if (!loop_running_) {
    return;
  }
  {
    // Give queued messages (in practice: the final OFFLINE) up to one second
    // to be sent and, for QoS 1, acknowledged. Bounded, so a dead broker
    // cannot stall shutdown.
    std::unique_lock<std::mutex> lock(mutex_);
    in_flight_changed_.wait_for(
      lock, std::chrono::seconds(1), [this] {return in_flight_ <= 0;});
  }
  mosquitto_disconnect(client_);
  mosquitto_loop_stop(client_, false);  // waits for the network thread to finish
  loop_running_ = false;
}

void MqttClient::publish(const vda5050_core::MqttMessage & message)
{
  // The lock is held across the call so the publish callback, which runs on
  // the network thread, cannot decrement the counter before it is incremented.
  std::lock_guard<std::mutex> lock(mutex_);
  const int result = mosquitto_publish(
    client_, nullptr, message.topic.c_str(), static_cast<int>(message.payload.size()),
    message.payload.data(), message.qos, message.retain);
  if (result == MOSQ_ERR_SUCCESS) {
    ++in_flight_;
  }
}

std::vector<MqttEvent> MqttClient::take_events()
{
  std::vector<MqttEvent> events;
  std::lock_guard<std::mutex> lock(mutex_);
  events.swap(events_);
  return events;
}

void MqttClient::push(MqttEvent event)
{
  std::lock_guard<std::mutex> lock(mutex_);
  events_.push_back(std::move(event));
}

// ---- callbacks: these run on libmosquitto's thread -------------------------

void MqttClient::on_connect(struct mosquitto * client, void * self, int result_code)
{
  auto * transport = static_cast<MqttClient *>(self);
  if (result_code != 0) {
    return;  // refused by the broker; libmosquitto keeps retrying
  }
  // Subscribing here (not once at start-up) restores subscriptions after
  // every reconnect, which a clean session requires.
  for (const std::string & topic : transport->subscriptions_) {
    mosquitto_subscribe(client, nullptr, topic.c_str(), 0);
  }
  transport->push(MqttEvent{MqttEvent::Type::Connected, "", ""});
}

void MqttClient::on_disconnect(struct mosquitto *, void * self, int /*result_code*/)
{
  static_cast<MqttClient *>(self)->push(
    MqttEvent{MqttEvent::Type::Disconnected, "", ""});
}

void MqttClient::on_message(
  struct mosquitto *, void * self, const struct mosquitto_message * message)
{
  static_cast<MqttClient *>(self)->push(
    MqttEvent{
      MqttEvent::Type::Message, message->topic,
      std::string(static_cast<const char *>(message->payload),
      static_cast<std::size_t>(message->payloadlen))});
}

void MqttClient::on_publish(struct mosquitto *, void * self, int /*message_id*/)
{
  auto * transport = static_cast<MqttClient *>(self);
  {
    std::lock_guard<std::mutex> lock(transport->mutex_);
    --transport->in_flight_;
  }
  transport->in_flight_changed_.notify_all();
}

}  // namespace vda5050_fleet
