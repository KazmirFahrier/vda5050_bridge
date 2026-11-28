// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// The fleet manager as a program: fleet_core behind an MQTT connection.
//
//   vehicles  <->  uagv/v2/<manufacturer>/<serial>/...     (VDA 5050)
//   clients    ->  <api>/command      one JSON command per message
//              <-  <api>/response     one answer per command
//              <-  <api>/state        vehicles, tasks, counters (retained)
//              <-  <api>/events       what happened, one event per message
//              <-  <api>/connection   ONLINE / OFFLINE (retained, last will)
//
// One thread. The MQTT client's network thread only queues what arrives;
// everything is looked at here, 20 times a second.

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fleet_core/api.hpp"
#include "fleet_core/fleet_manager.hpp"
#include "fleet_core/roadmap.hpp"
#include "options.hpp"
#include "vda5050_fleet/mqtt_client.hpp"

namespace
{

volatile std::sig_atomic_t stop_requested = 0;

void on_signal(int) {stop_requested = 1;}

std::string read_file(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("cannot read '" + path + "'");
  }
  std::ostringstream text;
  text << file.rdbuf();
  return text.str();
}

}  // namespace

int main(int argc, char ** argv)
{
  using Clock = std::chrono::steady_clock;
  try {
    const vda5050_fleet::Options options(
      argc, argv,
      {{"roadmap", ""}, {"host", "localhost"}, {"port", "1883"},
        {"client-id", "fleet-manager"}, {"username", ""},
        {"interface", "uagv"}, {"version", "2.0.0"}, {"api", "fleet/v1"},
        {"vehicle-radius", "0.4"}, {"safety-margin", "0.2"}, {"attach-tolerance", "0.25"},
        {"release-ahead", "3"}, {"horizon", "3"}, {"state-timeout", "60"},
        {"discovery-grace", "10"}, {"default-speed", "1.0"}, {"status-interval", "1.0"}});
    if (options.text("roadmap").empty()) {
      std::cerr << "usage: fleet_manager --roadmap <file.json> [options]\n" << options.usage() <<
        "The MQTT password, if any, is read from the environment variable FLEET_MQTT_PASSWORD.\n";
      return 2;
    }

    fleet_core::FleetConfig config;
    config.interface_name = options.text("interface");
    config.version = options.text("version");
    config.vehicle_radius = options.number("vehicle-radius");
    config.safety_margin = options.number("safety-margin");
    config.attach_tolerance = options.number("attach-tolerance");
    config.release_ahead = static_cast<std::size_t>(options.number("release-ahead"));
    config.horizon = static_cast<std::size_t>(options.number("horizon"));
    config.state_timeout = options.seconds("state-timeout");
    config.discovery_grace = options.seconds("discovery-grace");
    config.planner.default_speed = options.number("default-speed");
    const std::chrono::milliseconds status_interval = options.seconds("status-interval");

    fleet_core::Roadmap roadmap =
      fleet_core::Roadmap::from_json(read_file(options.text("roadmap")));
    std::cout << "roadmap '" << roadmap.map_id() << "': " << roadmap.node_count() <<
      " nodes, " << roadmap.edge_count() << " edges" << std::endl;

    vda5050_fleet::MqttClient::Options mqtt;
    mqtt.host = options.text("host");
    mqtt.port = static_cast<int>(options.number("port"));
    mqtt.client_id = options.text("client-id");
    mqtt.username = options.text("username");
    if (const char * password = std::getenv("FLEET_MQTT_PASSWORD")) {
      mqtt.password = password;
    }
    vda5050_fleet::MqttClient client(mqtt);
    fleet_core::FleetManager fleet(std::move(roadmap), config, client);

    const std::string api = options.text("api");
    const std::string command_topic = api + "/command";
    std::vector<std::string> topics = fleet.subscriptions();
    topics.push_back(command_topic);
    client.set_subscriptions(topics);
    client.set_will({api + "/connection", R"({"connectionState":"OFFLINE"})", 1, true});
    if (!client.connect()) {
      std::cerr << "cannot start the MQTT client" << std::endl;
      return 1;
    }
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    const Clock::time_point started = Clock::now();
    Clock::time_point next_status = started;
    Clock::time_point wake = started;
    while (stop_requested == 0) {
      const Clock::time_point now = Clock::now();
      for (const vda5050_fleet::MqttEvent & event : client.take_events()) {
        switch (event.type) {
          case vda5050_fleet::MqttEvent::Type::Connected:
            std::cout << "connected to the broker" << std::endl;
            client.publish({api + "/connection", R"({"connectionState":"ONLINE"})", 1, true});
            next_status = now;
            break;
          case vda5050_fleet::MqttEvent::Type::Disconnected:
            std::cout << "connection to the broker lost" << std::endl;
            break;
          case vda5050_fleet::MqttEvent::Type::Message:
            if (event.topic == command_topic) {
              client.publish(
                {api + "/response", fleet_core::handle_command(fleet, event.payload, now), 1,
                  false});
              next_status = now;
            } else {
              fleet.on_message(event.topic, event.payload, now);
            }
            break;
        }
      }
      fleet.tick(now);
      for (const fleet_core::FleetEvent & event : fleet.take_events()) {
        const double seconds = std::chrono::duration<double>(event.time - started).count();
        std::printf(
          "[%9.2f] %-24s %-12s %s\n", seconds, event.kind.c_str(), event.vehicle.c_str(),
          event.text.c_str());
        client.publish({api + "/events", fleet_core::encode_event(event, seconds), 0, false});
      }
      std::fflush(stdout);
      if (now >= next_status) {
        client.publish({api + "/state", fleet_core::encode_status(fleet), 0, true});
        next_status = now + status_interval;
      }
      wake += std::chrono::milliseconds(50);
      if (wake < Clock::now()) {
        wake = Clock::now();  // fell behind (a long plan): do not try to catch up
      }
      std::this_thread::sleep_until(wake);
    }

    // Vehicles keep the orders they have; nothing is cancelled on the way out.
    client.publish({api + "/connection", R"({"connectionState":"OFFLINE"})", 1, true});
    client.disconnect();
    std::cout << "stopped" << std::endl;
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "fleet_manager: " << error.what() << std::endl;
    return 1;
  }
}
