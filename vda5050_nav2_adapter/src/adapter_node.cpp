// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "vda5050_nav2_adapter/adapter_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "fault_core/types.hpp"
#include "fault_core/vda5050_mapping.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace vda5050_nav2_adapter
{

using vda5050_core::ActionStatus;

namespace
{

// Fault codes this node reports about itself. They are listed in the example
// catalogue of the fault_manager package.
constexpr char kFaultMqtt[] = "BRIDGE-MQTT-001";
constexpr char kFaultNav[] = "BRIDGE-NAV-001";
constexpr char kFaultLocalization[] = "BRIDGE-LOC-001";

}  // namespace

AdapterNode::AdapterNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("vda5050_adapter", options)
{
  // Identity on the VDA 5050 network. manufacturer and serial_number are
  // mandatory: they form the MQTT topic and must be unique per vehicle.
  declare_parameter("manufacturer", "");
  declare_parameter("serial_number", "");
  declare_parameter("interface_name", "uagv");
  declare_parameter("protocol_version", "2.0.0");

  declare_parameter("mqtt.host", "localhost");
  declare_parameter("mqtt.port", 1883);
  declare_parameter("mqtt.keepalive_s", 15);
  declare_parameter("mqtt.username", "");
  declare_parameter("mqtt.password", "");

  declare_parameter("map_id", "map");
  declare_parameter("global_frame", "map");
  declare_parameter("base_frame", "base_link");
  declare_parameter("navigate_action", "navigate_to_pose");
  declare_parameter("battery_topic", "battery_state");
  declare_parameter("odometry_topic", "odom");

  declare_parameter("update_rate_hz", 20.0);
  declare_parameter("state_interval_s", 30.0);
  declare_parameter("visualization_interval_s", 1.0);
  declare_parameter("default_deviation_xy", 0.5);
  declare_parameter("supported_actions", std::vector<std::string>{});

  declare_parameter("factsheet.series_name", "generic_amr");
  declare_parameter("factsheet.agv_kinematic", "DIFF");
  declare_parameter("factsheet.agv_class", "CARRIER");
  declare_parameter("factsheet.max_load_mass", 0.0);
  declare_parameter("factsheet.speed_max", 1.0);
  declare_parameter("factsheet.acceleration_max", 1.0);
  declare_parameter("factsheet.deceleration_max", 1.0);
  declare_parameter("factsheet.height_max", 0.5);
  declare_parameter("factsheet.width", 0.5);
  declare_parameter("factsheet.length", 0.7);

  // Fault integration; see the note at the top of adapter_node.hpp.
  declare_parameter("faults.enabled", true);
  // Hold the vehicle (pause driving, start nothing new) while a fault that
  // maps to errorLevel FATAL is raised.
  declare_parameter("faults.hold_on_fatal", true);
  // If a fault manager was heard once and then falls silent for this long,
  // master control is told so with a warning in state.errors.
  declare_parameter("faults.timeout_s", 15.0);

  // When true, main() drives the node to ACTIVE itself. Leave false when a
  // lifecycle manager is in charge of start-up ordering.
  declare_parameter("autostart", true);
}

AdapterNode::CallbackReturn AdapterNode::on_configure(const rclcpp_lifecycle::State &)
{
  vda5050_core::Identity identity;
  identity.manufacturer = get_parameter("manufacturer").as_string();
  identity.serial_number = get_parameter("serial_number").as_string();
  identity.interface_name = get_parameter("interface_name").as_string();
  identity.version = get_parameter("protocol_version").as_string();
  if (identity.manufacturer.empty() || identity.serial_number.empty()) {
    RCLCPP_ERROR(get_logger(), "parameters 'manufacturer' and 'serial_number' must be set");
    return CallbackReturn::FAILURE;
  }

  map_id_ = get_parameter("map_id").as_string();
  global_frame_ = get_parameter("global_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  position_ = vda5050_core::AgvPosition{};
  position_.map_id = map_id_;

  vda5050_core::SessionConfig config;
  config.state_interval = std::chrono::milliseconds(
    static_cast<std::int64_t>(get_parameter("state_interval_s").as_double() * 1000.0));
  config.visualization_interval = std::chrono::milliseconds(
    static_cast<std::int64_t>(get_parameter("visualization_interval_s").as_double() * 1000.0));
  config.orders.default_deviation_xy = get_parameter("default_deviation_xy").as_double();

  // The adapters report back through session_, which is created last. The
  // lambdas only run later, from executor callbacks, when session_ exists.
  navigation_ = std::make_unique<Nav2NavigationAdapter>(
    *this, get_parameter("navigate_action").as_string(), global_frame_,
    [this](bool success, const std::string & reason) {
      if (!session_) {
        return;
      }
      if (success) {
        session_->orders().on_navigation_succeeded();
      } else {
        session_->orders().on_navigation_failed(reason);
      }
    },
    [this]() -> std::optional<std::pair<double, double>> {
      if (!position_.position_initialized) {
        return std::nullopt;
      }
      return std::make_pair(position_.x, position_.y);
    });

  actions_ = std::make_unique<TopicActionHandler>(
    *this, get_parameter("supported_actions").as_string_array(),
    [this](const std::string & action_id, ActionStatus status, const std::string & result) {
      if (session_) {
        session_->orders().on_action_status(action_id, status, result);
      }
    });

  MosquittoTransport::Options mqtt;
  mqtt.host = get_parameter("mqtt.host").as_string();
  mqtt.port = static_cast<int>(get_parameter("mqtt.port").as_int());
  mqtt.keepalive_s = static_cast<int>(get_parameter("mqtt.keepalive_s").as_int());
  mqtt.username = get_parameter("mqtt.username").as_string();
  mqtt.password = get_parameter("mqtt.password").as_string();
  mqtt.client_id = identity.manufacturer + "-" + identity.serial_number;
  try {
    transport_ = std::make_unique<MosquittoTransport>(mqtt);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "could not create MQTT client: %s", error.what());
    release_resources();
    return CallbackReturn::FAILURE;
  }

  session_ = std::make_unique<vda5050_core::Session>(
    identity, *transport_, *navigation_, *actions_, config);
  session_->set_factsheet(build_factsheet(config));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  battery_subscription_ = create_subscription<sensor_msgs::msg::BatteryState>(
    get_parameter("battery_topic").as_string(), rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::BatteryState & message) {
      if (!session_) {
        return;
      }
      vda5050_core::BatteryState battery;
      // ROS reports 0..1 (or NaN when unknown); VDA 5050 wants percent.
      battery.battery_charge = std::isfinite(message.percentage) ?
      static_cast<double>(message.percentage) * 100.0 : 0.0;
      battery.charging =
      message.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
      if (std::isfinite(message.voltage)) {
        battery.battery_voltage = static_cast<double>(message.voltage);
      }
      session_->orders().set_battery(battery);
    });

  odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    get_parameter("odometry_topic").as_string(), rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry & message) {
      if (session_) {
        session_->orders().set_velocity(
          vda5050_core::Velocity{
          message.twist.twist.linear.x, message.twist.twist.linear.y,
          message.twist.twist.angular.z});
      }
    });

  if (get_parameter("faults.enabled").as_bool()) {
    hold_on_fatal_fault_ = get_parameter("faults.hold_on_fatal").as_bool();
    fault_timeout_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(get_parameter("faults.timeout_s").as_double()));
    fault_errors_.clear();
    faults_received_ = false;
    fault_manager_silent_ = false;
    // Transient local: the current list arrives at once, also when this node
    // starts after the fault manager.
    fault_subscription_ = create_subscription<fault_msgs::msg::FaultArray>(
      "faults/active", rclcpp::QoS(1).reliable().transient_local(),
      [this](const fault_msgs::msg::FaultArray & faults) {on_faults(faults);});
    fault_report_publisher_ = create_publisher<fault_msgs::msg::FaultReport>(
      "faults/report", rclcpp::QoS(100).reliable());
  }

  RCLCPP_INFO(
    get_logger(), "configured as %s",
    vda5050_core::topic_name(identity, "").c_str());
  return CallbackReturn::SUCCESS;
}

AdapterNode::CallbackReturn AdapterNode::on_activate(const rclcpp_lifecycle::State & state)
{
  // Activates the lifecycle publishers created in on_configure.
  LifecycleNode::on_activate(state);

  transport_->set_will(session_->last_will());
  transport_->set_subscriptions(session_->subscriptions());
  if (!transport_->connect()) {
    RCLCPP_ERROR(get_logger(), "could not start the MQTT client");
    return CallbackReturn::FAILURE;
  }

  const double rate = get_parameter("update_rate_hz").as_double();
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / (rate > 0.0 ? rate : 20.0)), [this] {on_timer();});
  return CallbackReturn::SUCCESS;
}

AdapterNode::CallbackReturn AdapterNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  go_offline();
  LifecycleNode::on_deactivate(state);
  return CallbackReturn::SUCCESS;
}

AdapterNode::CallbackReturn AdapterNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  release_resources();
  return CallbackReturn::SUCCESS;
}

AdapterNode::CallbackReturn AdapterNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  go_offline();
  release_resources();
  return CallbackReturn::SUCCESS;
}

void AdapterNode::stop()
{
  go_offline();
  release_resources();
}

void AdapterNode::go_offline()
{
  timer_.reset();
  if (session_ && transport_) {
    // Clean exit: say OFFLINE ourselves so the broker does not publish the
    // CONNECTIONBROKEN last will.
    session_->shutdown();
    transport_->disconnect();
    // Drop whatever the MQTT thread queued on the way out (typically the
    // "disconnected" event), so a later activation starts with a clean slate.
    transport_->take_events();
  }
}

void AdapterNode::release_resources()
{
  timer_.reset();
  battery_subscription_.reset();
  odometry_subscription_.reset();
  fault_subscription_.reset();
  fault_report_publisher_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  session_.reset();  // first: it refers to the three below
  actions_.reset();
  navigation_.reset();
  transport_.reset();
}

void AdapterNode::on_timer()
{
  const auto now = std::chrono::steady_clock::now();

  // 1. Whatever arrived from the broker since the last tick.
  for (TransportEvent & event : transport_->take_events()) {
    switch (event.type) {
      case TransportEvent::Type::Connected:
        RCLCPP_INFO(get_logger(), "connected to MQTT broker");
        session_->on_connected(now);
        break;
      case TransportEvent::Type::Disconnected:
        RCLCPP_WARN(get_logger(), "MQTT connection lost; reconnecting");
        session_->on_disconnected();
        break;
      case TransportEvent::Type::Message:
        session_->on_message(event.topic, event.payload);
        break;
    }
  }

  // 2. Fresh pose, then 3. publish whatever is due.
  update_position();
  report_own_health(now);
  session_->tick(now);
}

void AdapterNode::on_faults(const fault_msgs::msg::FaultArray & faults)
{
  if (!session_) {
    return;
  }
  std::vector<fault_core::FaultRecord> records;
  for (const fault_msgs::msg::Fault & fault : faults.faults) {
    fault_core::FaultRecord record;
    record.code = fault.code;
    record.severity = static_cast<fault_core::Severity>(
      std::min<int>(fault.severity, static_cast<int>(fault_core::Severity::Critical)));
    record.state = static_cast<fault_core::FaultState>(
      std::min<int>(fault.state, static_cast<int>(fault_core::FaultState::AwaitingAck)));
    record.description = fault.description;
    record.message = fault.message;
    record.source = fault.source;
    record.known = fault.known;
    record.occurrence_count = fault.occurrence_count;
    records.push_back(std::move(record));
  }
  fault_errors_ = fault_core::to_vda5050_errors(records);
  faults_received_ = true;
  fault_manager_silent_ = false;
  last_faults_time_ = std::chrono::steady_clock::now();
  apply_faults();
}

void AdapterNode::apply_faults()
{
  std::vector<vda5050_core::Error> errors = fault_errors_;
  bool fatal = false;
  for (const vda5050_core::Error & error : errors) {
    fatal = fatal || error.error_level == vda5050_core::ErrorLevel::Fatal;
  }
  if (fault_manager_silent_) {
    // The list above may be out of date. Say so instead of pretending.
    errors.push_back(
      vda5050_core::Error{
        "faultManagerTimeout", {}, "the fault manager has stopped publishing; "
        "the reported faults may be out of date", vda5050_core::ErrorLevel::Warning});
  }
  session_->orders().set_external_errors(std::move(errors));
  if (hold_on_fatal_fault_) {
    session_->orders().set_hold(false);
  }
}

void AdapterNode::report_own_health(std::chrono::steady_clock::time_point now)
{
  if (!fault_report_publisher_) {
    return;
  }
  if (faults_received_ && !fault_manager_silent_ && now - last_faults_time_ > fault_timeout_) {
    RCLCPP_WARN(get_logger(), "no message on faults/active for too long");
    fault_manager_silent_ = true;
    apply_faults();
  }
  if (now - last_health_report_ < std::chrono::seconds(1)) {
    return;
  }
  last_health_report_ = now;

  const auto report = [this](const char * code, bool failing, const char * message) {
      fault_msgs::msg::FaultReport out;
      out.code = code;
      out.failing = failing;
      out.source = get_name();
      out.message = failing ? message : "";
      fault_report_publisher_->publish(out);
    };
  report(kFaultMqtt, !session_->connected(), "not connected to the MQTT broker");
  report(kFaultNav, !navigation_->server_available(), "Nav2 action server is not available");
  report(
    kFaultLocalization, !position_.position_initialized,
    "no transform from the global frame to the robot");
}

void AdapterNode::update_position()
{
  try {
    const geometry_msgs::msg::TransformStamped transform =
      tf_buffer_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
    const auto & q = transform.transform.rotation;
    position_.x = transform.transform.translation.x;
    position_.y = transform.transform.translation.y;
    // Yaw from a quaternion (rotation about z).
    position_.theta = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    position_.position_initialized = true;
  } catch (const tf2::TransformException &) {
    // No localisation (yet, or lost). Keep the last pose but tell master
    // control it is not trustworthy. The core then accepts a new order only
    // if it starts at the node the vehicle last reached.
    position_.position_initialized = false;
  }
  session_->orders().set_position(position_);
}

vda5050_core::Factsheet AdapterNode::build_factsheet(const vda5050_core::SessionConfig & config)
{
  vda5050_core::Factsheet factsheet;
  factsheet.series_name = get_parameter("factsheet.series_name").as_string();
  factsheet.agv_kinematic = get_parameter("factsheet.agv_kinematic").as_string();
  factsheet.agv_class = get_parameter("factsheet.agv_class").as_string();
  factsheet.max_load_mass = get_parameter("factsheet.max_load_mass").as_double();
  factsheet.speed_max = get_parameter("factsheet.speed_max").as_double();
  factsheet.acceleration_max = get_parameter("factsheet.acceleration_max").as_double();
  factsheet.deceleration_max = get_parameter("factsheet.deceleration_max").as_double();
  factsheet.height_max = get_parameter("factsheet.height_max").as_double();
  factsheet.width = get_parameter("factsheet.width").as_double();
  factsheet.length = get_parameter("factsheet.length").as_double();

  const double rate = get_parameter("update_rate_hz").as_double();
  factsheet.min_state_interval = rate > 0.0 ? 1.0 / rate : 0.05;
  factsheet.min_order_interval = factsheet.min_state_interval;
  factsheet.default_state_interval =
    static_cast<double>(config.state_interval.count()) / 1000.0;
  if (config.visualization_interval.count() > 0) {
    factsheet.visualization_interval =
      static_cast<double>(config.visualization_interval.count()) / 1000.0;
  }

  for (const char * builtin : {"cancelOrder", "startPause", "stopPause", "stateRequest",
      "factsheetRequest"})
  {
    factsheet.agv_actions.push_back(
      vda5050_core::FactsheetAction{builtin, std::nullopt, {"INSTANT"}});
  }
  // Copy first: iterating directly over get_parameter(...).as_string_array()
  // would read from a Parameter temporary that is already destroyed.
  const std::vector<std::string> supported = get_parameter("supported_actions").as_string_array();
  for (const std::string & type : supported) {
    factsheet.agv_actions.push_back(
      vda5050_core::FactsheetAction{type, std::nullopt, {"INSTANT", "NODE", "EDGE"}});
  }
  return factsheet;
}

}  // namespace vda5050_nav2_adapter
