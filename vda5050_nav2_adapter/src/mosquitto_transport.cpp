// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "vda5050_nav2_adapter/mosquitto_transport.hpp"

#include <mosquitto.h>

#include <stdexcept>
#include <utility>

namespace vda5050_nav2_adapter
{

MosquittoTransport::MosquittoTransport(Options options)
: options_(std::move(options))
{
  mosquitto_lib_init();
  // clean_session = true: the broker keeps no queue for us while we are away.
  // Orders that were sent during an outage must not arrive late and be executed.
  client_ = mosquitto_new(
    options_.client_id.empty() ? nullptr : options_.client_id.c_str(), true, this);
  if (client_ == nullptr) {
    mosquitto_lib_cleanup();
    throw std::runtime_error("mosquitto_new failed");
  }
  mosquitto_connect_callback_set(client_, &MosquittoTransport::on_connect);
  mosquitto_disconnect_callback_set(client_, &MosquittoTransport::on_disconnect);
  mosquitto_message_callback_set(client_, &MosquittoTransport::on_message);
  // Reconnect after 1 s, doubling up to 30 s.
  mosquitto_reconnect_delay_set(client_, 1, 30, true);
  if (!options_.username.empty()) {
    mosquitto_username_pw_set(client_, options_.username.c_str(), options_.password.c_str());
  }
}

MosquittoTransport::~MosquittoTransport()
{
  disconnect();
  mosquitto_destroy(client_);
  mosquitto_lib_cleanup();
}

void MosquittoTransport::set_will(const vda5050_core::MqttMessage & will)
{
  mosquitto_will_set(
    client_, will.topic.c_str(), static_cast<int>(will.payload.size()), will.payload.data(),
    will.qos, will.retain);
}

void MosquittoTransport::set_subscriptions(std::vector<std::string> topics)
{
  subscriptions_ = std::move(topics);
}

bool MosquittoTransport::connect()
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

void MosquittoTransport::disconnect()
{
  if (!loop_running_) {
    return;
  }
  mosquitto_disconnect(client_);
  mosquitto_loop_stop(client_, false);  // waits for the network thread to finish
  loop_running_ = false;
}

void MosquittoTransport::publish(const vda5050_core::MqttMessage & message)
{
  mosquitto_publish(
    client_, nullptr, message.topic.c_str(), static_cast<int>(message.payload.size()),
    message.payload.data(), message.qos, message.retain);
}

std::vector<TransportEvent> MosquittoTransport::take_events()
{
  std::vector<TransportEvent> events;
  std::lock_guard<std::mutex> lock(mutex_);
  events.swap(events_);
  return events;
}

void MosquittoTransport::push(TransportEvent event)
{
  std::lock_guard<std::mutex> lock(mutex_);
  events_.push_back(std::move(event));
}

// ---- callbacks: these run on libmosquitto's thread -------------------------

void MosquittoTransport::on_connect(struct mosquitto * client, void * self, int result_code)
{
  auto * transport = static_cast<MosquittoTransport *>(self);
  if (result_code != 0) {
    return;  // refused by the broker; libmosquitto keeps retrying
  }
  // Subscribing here (not once at start-up) restores subscriptions after
  // every reconnect, which a clean session requires.
  for (const std::string & topic : transport->subscriptions_) {
    mosquitto_subscribe(client, nullptr, topic.c_str(), 0);
  }
  transport->push(TransportEvent{TransportEvent::Type::Connected, "", ""});
}

void MosquittoTransport::on_disconnect(struct mosquitto *, void * self, int /*result_code*/)
{
  static_cast<MosquittoTransport *>(self)->push(
    TransportEvent{TransportEvent::Type::Disconnected, "", ""});
}

void MosquittoTransport::on_message(
  struct mosquitto *, void * self, const struct mosquitto_message * message)
{
  static_cast<MosquittoTransport *>(self)->push(
    TransportEvent{
      TransportEvent::Type::Message, message->topic,
      std::string(static_cast<const char *>(message->payload),
      static_cast<std::size_t>(message->payloadlen))});
}

}  // namespace vda5050_nav2_adapter
