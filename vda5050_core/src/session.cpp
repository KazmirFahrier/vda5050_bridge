// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "vda5050_core/session.hpp"

#include <utility>

#include "vda5050_core/codec.hpp"

namespace vda5050_core
{

namespace
{

// QoS and retain settings follow the specification: everything is QoS 0
// except the connection topic, which is QoS 1 and retained so that a master
// connecting later still learns the vehicle's last known connection state.
constexpr int kDefaultQos = 0;
constexpr int kConnectionQos = 1;

std::string major_version(const std::string & version)
{
  return version.substr(0, version.find('.'));
}

}  // namespace

Session::Session(
  Identity identity, MqttTransport & transport, NavigationPort & navigation,
  ActionPort & actions, SessionConfig config, HeaderFactory::Clock clock)
: headers_(std::move(identity), std::move(clock)),
  transport_(transport),
  config_(config),
  orders_(navigation, actions, config.orders) {}

std::vector<std::string> Session::subscriptions() const
{
  return {topic_name(identity(), "order"), topic_name(identity(), "instantActions")};
}

MqttMessage Session::last_will()
{
  Connection connection;
  connection.header = headers_.next("connection");
  connection.connection_state = ConnectionState::ConnectionBroken;
  return MqttMessage{
    topic_name(identity(), "connection"), encode_connection(connection), kConnectionQos, true};
}

void Session::set_factsheet(Factsheet factsheet)
{
  factsheet_ = std::move(factsheet);
}

void Session::on_connected(TimePoint now)
{
  connected_ = true;
  publish_connection(ConnectionState::Online);
  // A full state straight away: master control may have missed everything
  // that happened while the link was down.
  publish_state(now);
}

void Session::on_disconnected()
{
  connected_ = false;
}

void Session::shutdown()
{
  if (!connected_) {
    return;
  }
  publish_connection(ConnectionState::Offline);
  connected_ = false;
}

void Session::on_message(const std::string & topic, std::string_view payload)
{
  const std::string order_topic = topic_name(identity(), "order");
  const std::string instant_topic = topic_name(identity(), "instantActions");
  try {
    if (topic == order_topic) {
      const Order order = decode_order(payload);
      if (header_acceptable(order.header, "order")) {
        orders_.handle_order(order);
      }
    } else if (topic == instant_topic) {
      const InstantActions actions = decode_instant_actions(payload);
      if (header_acceptable(actions.header, "instantActions")) {
        orders_.handle_instant_actions(actions);
      }
    }
    // Any other topic is not ours to interpret and is ignored.
  } catch (const DecodeError & error) {
    orders_.report_malformed_message(
      topic == order_topic ? "order" : "instantActions", error.what());
  }
}

bool Session::header_acceptable(const Header & header, const std::string & topic)
{
  // The topic already names the vehicle, but a misconfigured master or bridge
  // can still deliver someone else's message here. Acting on it would move
  // the wrong vehicle, so the body must agree with who we are.
  if (header.manufacturer != identity().manufacturer ||
    header.serial_number != identity().serial_number)
  {
    orders_.report_malformed_message(
      topic, "message is addressed to " + header.manufacturer + "/" + header.serial_number);
    return false;
  }
  if (major_version(header.version) != major_version(identity().version)) {
    orders_.report_malformed_message(
      topic, "unsupported protocol version " + header.version);
    return false;
  }
  return true;
}

void Session::tick(TimePoint now)
{
  if (!connected_) {
    return;  // change flags stay set and are honoured after reconnect
  }

  if (orders_.consume_factsheet_request() && factsheet_) {
    Factsheet factsheet = *factsheet_;
    factsheet.header = headers_.next("factsheet");
    transport_.publish(
      MqttMessage{
        topic_name(identity(), "factsheet"), encode_factsheet(factsheet), kDefaultQos, false});
  }

  const bool state_due = !last_state_ || now - *last_state_ >= config_.state_interval;
  if (orders_.consume_state_changed() || state_due) {
    publish_state(now);
  }

  if (config_.visualization_interval.count() > 0) {
    const bool visualization_due =
      !last_visualization_ || now - *last_visualization_ >= config_.visualization_interval;
    const State state = orders_.snapshot();
    if (visualization_due && state.agv_position) {
      Visualization visualization;
      visualization.header = headers_.next("visualization");
      visualization.agv_position = state.agv_position;
      visualization.velocity = state.velocity;
      transport_.publish(
        MqttMessage{
          topic_name(identity(), "visualization"), encode_visualization(visualization),
          kDefaultQos, false});
      last_visualization_ = now;
    }
  }
}

void Session::publish_connection(ConnectionState connection_state)
{
  Connection connection;
  connection.header = headers_.next("connection");
  connection.connection_state = connection_state;
  transport_.publish(
    MqttMessage{
      topic_name(identity(), "connection"), encode_connection(connection), kConnectionQos, true});
}

void Session::publish_state(TimePoint now)
{
  orders_.consume_state_changed();  // this message carries every pending change
  State state = orders_.snapshot();
  state.header = headers_.next("state");
  transport_.publish(
    MqttMessage{topic_name(identity(), "state"), encode_state(state), kDefaultQos, false});
  last_state_ = now;
}

}  // namespace vda5050_core
