// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// A vehicle without a vehicle: the complete VDA 5050 vehicle side of
// vda5050_core (the same code the Nav2 adapter runs), connected to a real
// broker, with a point that moves in a straight line where a robot would
// drive. For trying out a fleet without robots, and for the system test.
//
// Actions "pick", "drop" and "wait" take --action-seconds; "fail" fails.

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "options.hpp"
#include "vda5050_core/session.hpp"
#include "vda5050_fleet/mqtt_client.hpp"

namespace
{

volatile std::sig_atomic_t stop_requested = 0;

void on_signal(int) {stop_requested = 1;}

class Body : public vda5050_core::NavigationPort, public vda5050_core::ActionPort
{
public:
  Body(double x, double y, double speed, double action_seconds)
  : x_(x), y_(y), speed_(speed), action_seconds_(action_seconds) {}

  void navigate_to(const vda5050_core::Node & target, const vda5050_core::Edge &) override
  {
    paused_ = false;
    if (!target.node_position) {
      failed_ = "the node has no position";
      return;
    }
    target_x_ = target.node_position->x;
    target_y_ = target.node_position->y;
    moving_ = true;
  }
  void cancel() override
  {
    moving_ = false;
    paused_ = false;
    failed_ = "cancelled";
  }
  void pause() override {paused_ = true;}
  void resume() override {paused_ = false;}

  bool supports(const std::string & type) const override
  {
    return type == "pick" || type == "drop" || type == "wait" || type == "fail";
  }
  void start(const vda5050_core::Action & action) override
  {
    running_[action.action_id] = {action.action_type, action_seconds_};
    started_.push_back(action.action_id);
  }
  void cancel(const std::string & action_id) override {running_.erase(action_id);}

  /// Move on by `dt` seconds and tell the protocol side what happened.
  void step(double dt, vda5050_core::OrderManager & orders, const std::string & map_id)
  {
    if (!failed_.empty()) {
      const std::string reason = std::exchange(failed_, std::string());
      orders.on_navigation_failed(reason);
    }
    if (moving_ && !paused_) {
      const double remaining = std::hypot(target_x_ - x_, target_y_ - y_);
      const double stride = speed_ * dt;
      if (remaining <= stride) {
        x_ = target_x_;
        y_ = target_y_;
        moving_ = false;
        orders.on_navigation_succeeded();
      } else {
        theta_ = std::atan2(target_y_ - y_, target_x_ - x_);
        x_ += (target_x_ - x_) / remaining * stride;
        y_ += (target_y_ - y_) / remaining * stride;
      }
    }
    for (const std::string & id : started_) {
      orders.on_action_status(id, vda5050_core::ActionStatus::Running);
    }
    started_.clear();
    for (auto it = running_.begin(); it != running_.end(); ) {
      it->second.second -= dt;
      if (it->second.second > 0.0) {
        ++it;
        continue;
      }
      const std::string id = it->first;
      const bool fails = it->second.first == "fail";
      it = running_.erase(it);
      orders.on_action_status(
        id, fails ? vda5050_core::ActionStatus::Failed : vda5050_core::ActionStatus::Finished,
        fails ? "simulated failure" : "");
    }
    vda5050_core::AgvPosition position;
    position.x = x_;
    position.y = y_;
    position.theta = theta_;
    position.map_id = map_id;
    position.position_initialized = true;
    orders.set_position(position);
  }

private:
  double x_;
  double y_;
  double theta_{0.0};
  double speed_;
  double action_seconds_;
  double target_x_{0.0};
  double target_y_{0.0};
  bool moving_{false};
  bool paused_{false};
  std::string failed_;
  std::map<std::string, std::pair<std::string, double>> running_;
  std::vector<std::string> started_;
};

}  // namespace

int main(int argc, char ** argv)
{
  using Clock = std::chrono::steady_clock;
  try {
    const vda5050_fleet::Options options(
      argc, argv,
      {{"serial", ""}, {"manufacturer", "sim"}, {"x", "0"}, {"y", "0"}, {"map-id", "map"},
        {"speed", "1.0"}, {"action-seconds", "2.0"}, {"host", "localhost"}, {"port", "1883"},
        {"interface", "uagv"}, {"version", "2.0.0"}, {"state-interval", "5.0"},
        {"visualization-interval", "0.1"}});
    if (options.text("serial").empty()) {
      std::cerr << "usage: sim_vehicle --serial <name> [options]\n" << options.usage();
      return 2;
    }

    vda5050_core::Identity identity;
    identity.interface_name = options.text("interface");
    identity.version = options.text("version");
    identity.manufacturer = options.text("manufacturer");
    identity.serial_number = options.text("serial");

    vda5050_fleet::MqttClient::Options mqtt;
    mqtt.host = options.text("host");
    mqtt.port = static_cast<int>(options.number("port"));
    mqtt.client_id = "sim-vehicle-" + identity.serial_number;
    vda5050_fleet::MqttClient client(mqtt);

    Body body(
      options.number("x"), options.number("y"), options.number("speed"),
      options.number("action-seconds"));
    vda5050_core::SessionConfig config;
    config.state_interval = options.seconds("state-interval");
    config.visualization_interval = options.seconds("visualization-interval");
    vda5050_core::Session session(identity, client, body, body, config);
    const std::string map_id = options.text("map-id");
    body.step(0.0, session.orders(), map_id);  // so that the first state has a position

    client.set_will(session.last_will());
    client.set_subscriptions(session.subscriptions());
    if (!client.connect()) {
      std::cerr << "cannot start the MQTT client" << std::endl;
      return 1;
    }
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    const double dt = 0.02;
    Clock::time_point wake = Clock::now();
    while (stop_requested == 0) {
      const Clock::time_point now = Clock::now();
      for (const vda5050_fleet::MqttEvent & event : client.take_events()) {
        switch (event.type) {
          case vda5050_fleet::MqttEvent::Type::Connected:
            session.on_connected(now);
            break;
          case vda5050_fleet::MqttEvent::Type::Disconnected:
            session.on_disconnected();
            break;
          case vda5050_fleet::MqttEvent::Type::Message:
            session.on_message(event.topic, event.payload);
            break;
        }
      }
      body.step(dt, session.orders(), map_id);
      session.tick(now);
      wake += std::chrono::milliseconds(20);
      std::this_thread::sleep_until(wake);
    }
    session.shutdown();  // says OFFLINE
    client.disconnect();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "sim_vehicle: " << error.what() << std::endl;
    return 1;
  }
}
