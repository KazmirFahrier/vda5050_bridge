// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// AdapterNode: the ROS 2 lifecycle node that hosts a VDA 5050 session.
//
// Lifecycle mapping:
//   configure   read parameters, build the session and its adapters (no I/O yet)
//   activate    connect to the broker, start the update timer
//   deactivate  announce OFFLINE, disconnect, stop the timer
//   cleanup     destroy everything built in configure
//
// Threading: run this node on a single-threaded executor. Every callback
// (timer, action results, subscriptions) then runs one at a time, which is the
// contract vda5050_core::Session requires. MQTT events cross from the
// libmosquitto thread through a queue that the timer drains.
//
// Faults: the node reports its own health (broker connection, Nav2, pose) on
// faults/report, and mirrors what a fault manager publishes on faults/active
// into the `errors` list of the VDA 5050 state. Both are optional: without a
// fault manager running, nothing changes.

#ifndef VDA5050_NAV2_ADAPTER__ADAPTER_NODE_HPP_
#define VDA5050_NAV2_ADAPTER__ADAPTER_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "fault_msgs/msg/fault_array.hpp"
#include "fault_msgs/msg/fault_report.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "vda5050_core/session.hpp"
#include "vda5050_nav2_adapter/mosquitto_transport.hpp"
#include "vda5050_nav2_adapter/nav2_navigation_adapter.hpp"
#include "vda5050_nav2_adapter/topic_action_handler.hpp"

namespace vda5050_nav2_adapter
{

class AdapterNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit AdapterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /// Announce OFFLINE, disconnect and release everything, without going
  /// through a lifecycle transition. For process exit (Ctrl-C), when the ROS
  /// context is already shut down and transitions can no longer run.
  void stop();

private:
  void on_timer();
  void update_position();
  void on_faults(const fault_msgs::msg::FaultArray & faults);
  void apply_faults();
  void report_own_health(std::chrono::steady_clock::time_point now);
  void go_offline();
  void release_resources();
  vda5050_core::Factsheet build_factsheet(const vda5050_core::SessionConfig & config);

  // Built in on_configure, destroyed in on_cleanup. Declaration order matters:
  // the session holds references to the three objects above it.
  std::unique_ptr<MosquittoTransport> transport_;
  std::unique_ptr<Nav2NavigationAdapter> navigation_;
  std::unique_ptr<TopicActionHandler> actions_;
  std::unique_ptr<vda5050_core::Session> session_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Fault integration.
  rclcpp::Subscription<fault_msgs::msg::FaultArray>::SharedPtr fault_subscription_;
  rclcpp::Publisher<fault_msgs::msg::FaultReport>::SharedPtr fault_report_publisher_;
  std::vector<vda5050_core::Error> fault_errors_;  ///< From the last faults/active message.
  bool hold_on_fatal_fault_{true};
  bool faults_received_{false};
  bool fault_manager_silent_{false};
  std::chrono::steady_clock::time_point last_faults_time_{};
  std::chrono::steady_clock::time_point last_health_report_{};
  std::chrono::steady_clock::duration fault_timeout_{std::chrono::seconds(15)};

  std::string map_id_;
  std::string global_frame_;
  std::string base_frame_;
  vda5050_core::AgvPosition position_;
};

}  // namespace vda5050_nav2_adapter

#endif  // VDA5050_NAV2_ADAPTER__ADAPTER_NODE_HPP_
