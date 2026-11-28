// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// A simulated fleet for testing the fleet manager.
//
// Real:  the FleetManager, and on each vehicle the complete vehicle side of
//        VDA 5050 (vda5050_core::Session: order handling, state reporting,
//        instant actions). The two talk in real VDA 5050 JSON.
// Fake:  the broker (a queue with delay, jitter and loss), the drive (a point
//        that moves along straight lines at a set speed), the actions (a
//        timer), and time itself, which the test advances in small steps.
//
// The world watches three things at every step, independently of the fleet
// manager's own bookkeeping:
//   1. Distance: no two vehicles are closer than two vehicle radii.
//   2. Claims:   no two vehicles hold conflicting resources.
//   3. Presence: every vehicle is on something it holds.
// Any breach is recorded in violations().

#ifndef SIM_HPP_
#define SIM_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "fleet_core/fleet_manager.hpp"
#include "vda5050_core/codec.hpp"
#include "vda5050_core/session.hpp"

namespace sim
{

using fleet_core::TimePoint;

struct WorldOptions
{
  double step{0.05};          ///< Seconds per simulation step.
  double latency{0.02};       ///< Seconds a message takes.
  double jitter{0.0};         ///< Extra random delay, up to this many seconds.
  double loss{0.0};           ///< Probability that a QoS 0 message disappears.
  unsigned seed{1};
  double vehicle_speed{1.0};  ///< m/s.
  double action_seconds{2.0};
  std::chrono::milliseconds state_interval{5000};
};

struct Envelope
{
  double due{0.0};
  std::string topic;
  std::string payload;
};

class World;

/// One vehicle: the real protocol stack on top of a point that moves.
class SimVehicle : public vda5050_core::MqttTransport, public vda5050_core::NavigationPort,
  public vda5050_core::ActionPort
{
public:
  SimVehicle(World & world, const std::string & serial, double x, double y);

  // MqttTransport
  void publish(const vda5050_core::MqttMessage & message) override;
  // NavigationPort
  void navigate_to(const vda5050_core::Node & target, const vda5050_core::Edge &) override
  {
    paused_ = false;  // a pause belongs to the goal it was given for
    if (fail_next_navigation_) {
      fail_next_navigation_ = false;
      failing_ = true;  // reported on the next step, as a real stack would
      return;
    }
    if (target.node_position) {
      target_x_ = target.node_position->x;
      target_y_ = target.node_position->y;
      moving_ = true;
    } else {
      failing_ = true;
    }
  }
  void cancel() override
  {
    moving_ = false;
    paused_ = false;    // a cancelled goal is gone; there is nothing left to resume
    cancelled_ = true;  // stops where it is, like a cancelled Nav2 goal
  }
  void pause() override {paused_ = true;}
  void resume() override {paused_ = false;}
  // ActionPort
  bool supports(const std::string & type) const override
  {
    return type == "pick" || type == "drop" || type == "wait" || type == "fail";
  }
  void start(const vda5050_core::Action & action) override
  {
    running_actions_[action.action_id] = {action.action_type, action_seconds_};
    started_.push_back(action.action_id);
    ++actions_started_;
  }
  void cancel(const std::string & action_id) override {running_actions_.erase(action_id);}

  void step(double dt, TimePoint now);
  void deliver(const std::string & topic, const std::string & payload)
  {
    if (connected_) {
      session_.on_message(topic, payload);
    }
  }
  void connect(TimePoint now)
  {
    connected_ = true;
    session_.on_connected(now);
  }
  /// The connection drops without warning; the broker publishes the last will.
  void drop_connection();

  vda5050_core::Session & session() {return session_;}
  const std::string & serial() const {return serial_;}
  double x() const {return x_;}
  double y() const {return y_;}
  bool connected() const {return connected_;}
  bool moving() const {return moving_;}
  void set_speed(double speed) {speed_ = speed;}
  void set_position(double x, double y) {x_ = x; y_ = y;}
  void fail_next_navigation() {fail_next_navigation_ = true;}
  /// Actions the vehicle has executed, in order (ids as sent by the fleet manager).
  const std::vector<std::string> & finished_actions() const {return finished_;}
  double distance_driven() const {return driven_;}
  std::size_t actions_started() const {return actions_started_;}

private:
  World & world_;
  std::string serial_;
  vda5050_core::Session session_;
  double x_;
  double y_;
  double target_x_{0.0};
  double target_y_{0.0};
  double speed_;
  double action_seconds_;
  double driven_{0.0};
  bool moving_{false};
  bool paused_{false};
  bool cancelled_{false};
  bool failing_{false};
  bool fail_next_navigation_{false};
  bool connected_{false};
  std::map<std::string, std::pair<std::string, double>> running_actions_;
  std::vector<std::string> started_;
  std::size_t actions_started_{0};
  std::vector<std::string> finished_;
};

class World : public vda5050_core::MqttTransport
{
public:
  World(fleet_core::Roadmap roadmap, fleet_core::FleetConfig config, WorldOptions options = {})
  : presence_tolerance_(config.attach_tolerance + 1e-6), options_(options),
    random_(options.seed), radius_(config.vehicle_radius),
    fleet_(std::move(roadmap), std::move(config), *this, [this] {return wall_clock();}) {}

  fleet_core::FleetManager & fleet() {return fleet_;}
  const WorldOptions & options() const {return options_;}
  double time() const {return time_;}
  TimePoint now() const
  {
    return TimePoint{} + std::chrono::duration_cast<TimePoint::duration>(
      std::chrono::duration<double>(time_));
  }
  std::chrono::system_clock::time_point wall_clock() const
  {
    return std::chrono::system_clock::time_point{} +
           std::chrono::duration_cast<std::chrono::system_clock::duration>(
      std::chrono::duration<double>(1795824000.0 + time_));
  }

  /// Put a vehicle on a node and (unless told otherwise) connect it.
  SimVehicle & add_vehicle(
    const std::string & serial, const std::string & node_id, bool connect = true)
  {
    const fleet_core::MapNode & node =
      fleet_.roadmap().node(*fleet_.roadmap().find_node(node_id));
    vehicles_.push_back(std::make_unique<SimVehicle>(*this, serial, node.x, node.y));
    if (connect) {
      vehicles_.back()->connect(now());
    }
    return *vehicles_.back();
  }
  /// Lose the next message to the fleet manager whose topic contains `part`.
  void lose_next_to_fleet(const std::string & part) {lose_next_.push_back(part);}
  /// Lose the next message to a vehicle whose topic contains `part`.
  void lose_next_to_vehicles(const std::string & part) {lose_next_out_.push_back(part);}
  /// Every order the fleet manager has sent, decoded, in order.
  const std::vector<vda5050_core::Order> & orders() const {return orders_;}
  SimVehicle & vehicle(const std::string & serial)
  {
    for (const auto & item : vehicles_) {
      if (item->serial() == serial) {
        return *item;
      }
    }
    throw std::runtime_error("no vehicle " + serial);
  }
  const std::vector<std::unique_ptr<SimVehicle>> & vehicles() const {return vehicles_;}

  // MqttTransport of the fleet manager: towards the vehicles.
  void publish(const vda5050_core::MqttMessage & message) override
  {
    if (message.topic.size() > 6 &&
      message.topic.compare(message.topic.size() - 6, 6, "/order") == 0)
    {
      orders_.push_back(vda5050_core::decode_order(message.payload));
    }
    for (auto it = lose_next_out_.begin(); it != lose_next_out_.end(); ++it) {
      if (message.topic.find(*it) != std::string::npos) {
        lose_next_out_.erase(it);
        return;
      }
    }
    post(to_vehicles_, message.topic, message.payload, message.qos == 0);
    ++messages_to_vehicles_;
  }
  void post_to_fleet(const std::string & topic, const std::string & payload, bool lossy)
  {
    for (auto it = lose_next_.begin(); it != lose_next_.end(); ++it) {
      if (topic.find(*it) != std::string::npos) {
        lose_next_.erase(it);
        return;
      }
    }
    post(to_fleet_, topic, payload, lossy);
  }

  void step()
  {
    time_ += options_.step;
    while (!to_vehicles_.empty() && to_vehicles_.front().due <= time_) {
      const Envelope message = to_vehicles_.front();
      to_vehicles_.pop_front();
      for (const auto & item : vehicles_) {
        if (message.topic.find("/" + item->serial() + "/") != std::string::npos) {
          item->deliver(message.topic, message.payload);
        }
      }
    }
    for (const auto & item : vehicles_) {
      item->step(options_.step, now());
    }
    while (!to_fleet_.empty() && to_fleet_.front().due <= time_) {
      const Envelope message = to_fleet_.front();
      to_fleet_.pop_front();
      fleet_.on_message(message.topic, message.payload, now());
    }
    fleet_.tick(now());
    watch();
  }
  void run_for(double seconds)
  {
    const double until = time_ + seconds;
    while (time_ < until) {
      step();
    }
  }
  /// Step until `done` holds; false if `timeout` seconds pass first.
  bool run_until(const std::function<bool()> & done, double timeout)
  {
    const double until = time_ + timeout;
    while (time_ < until) {
      if (done()) {
        return true;
      }
      step();
    }
    return done();
  }
  bool all_tasks_over()
  {
    for (const fleet_core::TaskView & task : fleet_.tasks()) {
      if (task.state == fleet_core::TaskState::Queued ||
        task.state == fleet_core::TaskState::Assigned)
      {
        return false;
      }
    }
    return true;
  }

  const std::vector<std::string> & violations() const {return violations_;}
  double closest_approach() const {return closest_;}
  std::size_t messages_to_vehicles() const {return messages_to_vehicles_;}

private:
  void post(
    std::deque<Envelope> & queue, const std::string & topic,
    const std::string & payload, bool lossy)
  {
    const auto unit = [this] {
        return static_cast<double>(random_() % 1000000) / 1000000.0;
      };
    if (lossy && options_.loss > 0.0 && unit() < options_.loss) {
      return;
    }
    // MQTT keeps the order of messages; so does this queue.
    double due = time_ + options_.latency + options_.jitter * unit();
    if (!queue.empty()) {
      due = std::max(due, queue.back().due);
    }
    queue.push_back(Envelope{due, topic, payload});
  }

  void watch()
  {
    const fleet_core::Roadmap & map = fleet_.roadmap();
    // 1. Distance, from the true positions.
    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
      for (std::size_t j = i + 1; j < vehicles_.size(); ++j) {
        const double distance = std::hypot(
          vehicles_[i]->x() - vehicles_[j]->x(), vehicles_[i]->y() - vehicles_[j]->y());
        closest_ = std::min(closest_, distance);
        if (distance < 2.0 * radius_ - 1e-6) {
          violate(
            "collision: " + vehicles_[i]->serial() + " and " + vehicles_[j]->serial() +
            " are " + std::to_string(distance) + " m apart");
        }
      }
    }
    // 2. Claims, from the fleet manager's books.
    const std::vector<fleet_core::VehicleView> views = fleet_.vehicles();
    const std::vector<std::vector<fleet_core::ResourceId>> claims = fleet_.claims();
    for (std::size_t i = 0; i < claims.size(); ++i) {
      for (std::size_t j = i + 1; j < claims.size(); ++j) {
        for (const fleet_core::ResourceId a : claims[i]) {
          for (const fleet_core::ResourceId b : claims[j]) {
            if (fleet_.conflicts().conflict(a, b)) {
              violate(
                "conflicting claims: " + views[i].serial_number + " holds " +
                map.resource_name(a) + ", " + views[j].serial_number + " holds " +
                map.resource_name(b));
            }
          }
        }
      }
    }
    // 3. Presence: the books must cover where each vehicle really is.
    for (std::size_t i = 0; i < claims.size(); ++i) {
      if (claims[i].empty()) {
        continue;  // not on the map as far as the fleet manager knows
      }
      const SimVehicle & real = vehicle(views[i].serial_number);
      double nearest = 1e9;
      for (const fleet_core::ResourceId resource : claims[i]) {
        if (map.is_node(resource)) {
          const fleet_core::MapNode & node = map.node(resource);
          nearest = std::min(nearest, std::hypot(real.x() - node.x, real.y() - node.y));
        } else {
          const fleet_core::MapEdge & edge = map.edge(resource - map.node_count());
          nearest = std::min(
            nearest, fleet_core::segment_distance(
              real.x(), real.y(), real.x(), real.y(), map.node(edge.from).x,
              map.node(edge.from).y, map.node(edge.to).x, map.node(edge.to).y));
        }
      }
      if (nearest > presence_tolerance_) {
        violate(
          "presence: " + views[i].serial_number + " is " + std::to_string(nearest) +
          " m away from everything it holds");
      }
    }
  }
  void violate(const std::string & what)
  {
    if (violations_.size() < 20) {
      violations_.push_back("t=" + std::to_string(time_) + " " + what);
    }
  }

  /// How far a vehicle may be from what it holds: as far as the fleet manager
  /// accepts a vehicle as being "at" a node.
  double presence_tolerance_;
  WorldOptions options_;
  std::mt19937 random_;
  double radius_;
  double time_{0.0};
  double closest_{1e9};
  std::size_t messages_to_vehicles_{0};
  std::deque<Envelope> to_vehicles_;
  std::deque<Envelope> to_fleet_;
  std::vector<std::unique_ptr<SimVehicle>> vehicles_;
  std::vector<std::string> violations_;
  std::vector<std::string> lose_next_;
  std::vector<std::string> lose_next_out_;
  std::vector<vda5050_core::Order> orders_;
  fleet_core::FleetManager fleet_;
};

inline vda5050_core::Identity identity_of(const std::string & serial)
{
  vda5050_core::Identity identity;
  identity.manufacturer = "sim";
  identity.serial_number = serial;
  return identity;
}

inline vda5050_core::SessionConfig session_config(const WorldOptions & options)
{
  vda5050_core::SessionConfig config;
  config.state_interval = options.state_interval;
  config.visualization_interval = std::chrono::milliseconds(0);
  return config;
}

inline SimVehicle::SimVehicle(World & world, const std::string & serial, double x, double y)
: world_(world), serial_(serial),
  session_(identity_of(serial), *this, *this, *this, session_config(world.options()),
    [&world] {return world.wall_clock();}),
  x_(x), y_(y), speed_(world.options().vehicle_speed),
  action_seconds_(world.options().action_seconds)
{
  // Localised before it connects, so that its first state says where it is.
  vda5050_core::AgvPosition position;
  position.x = x_;
  position.y = y_;
  position.map_id = world.fleet().roadmap().map_id();
  position.position_initialized = true;
  session_.orders().set_position(position);
}

inline void SimVehicle::publish(const vda5050_core::MqttMessage & message)
{
  if (connected_) {
    world_.post_to_fleet(message.topic, message.payload, message.qos == 0);
  }
}

inline void SimVehicle::drop_connection()
{
  connected_ = false;
  session_.on_disconnected();
  const vda5050_core::MqttMessage will = session_.last_will();
  world_.post_to_fleet(will.topic, will.payload, false);
}

inline void SimVehicle::step(double dt, TimePoint now)
{
  if (failing_) {
    failing_ = false;
    session_.orders().on_navigation_failed("simulated failure");
  }
  if (cancelled_) {
    cancelled_ = false;
    session_.orders().on_navigation_failed("cancelled");
  }
  if (moving_ && !paused_) {
    const double remaining = std::hypot(target_x_ - x_, target_y_ - y_);
    const double stride = speed_ * dt;
    if (remaining <= stride) {
      driven_ += remaining;
      x_ = target_x_;
      y_ = target_y_;
      moving_ = false;
      session_.orders().on_navigation_succeeded();
    } else {
      x_ += (target_x_ - x_) / remaining * stride;
      y_ += (target_y_ - y_) / remaining * stride;
      driven_ += stride;
    }
  }
  for (const std::string & id : started_) {
    session_.orders().on_action_status(id, vda5050_core::ActionStatus::Running);
  }
  started_.clear();
  for (auto it = running_actions_.begin(); it != running_actions_.end(); ) {
    it->second.second -= dt;
    if (it->second.second <= 0.0) {
      const std::string id = it->first;
      const bool fails = it->second.first == "fail";
      it = running_actions_.erase(it);
      session_.orders().on_action_status(
        id, fails ? vda5050_core::ActionStatus::Failed : vda5050_core::ActionStatus::Finished,
        fails ? "simulated failure" : "");
      if (!fails) {
        finished_.push_back(id);
      }
    } else {
      ++it;
    }
  }
  vda5050_core::AgvPosition position;
  position.x = x_;
  position.y = y_;
  position.map_id = world_.fleet().roadmap().map_id();
  position.position_initialized = true;
  session_.orders().set_position(position);
  session_.tick(now);
}

}  // namespace sim

#endif  // SIM_HPP_
