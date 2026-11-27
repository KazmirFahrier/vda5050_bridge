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

#ifndef VDA5050_NAV2_ADAPTER__ADAPTER_NODE_HPP_
#define VDA5050_NAV2_ADAPTER__ADAPTER_NODE_HPP_

#include <memory>
#include <string>

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

private:
  void on_timer();
  void update_position();
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

  std::string map_id_;
  std::string global_frame_;
  std::string base_frame_;
  vda5050_core::AgvPosition position_;
};

}  // namespace vda5050_nav2_adapter

#endif  // VDA5050_NAV2_ADAPTER__ADAPTER_NODE_HPP_
