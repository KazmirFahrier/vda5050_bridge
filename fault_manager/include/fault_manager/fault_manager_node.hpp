// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// FaultManagerNode: the ROS 2 face of fault_core::FaultManager.
//
// Inputs
//   faults/report     fault_msgs/FaultReport            results from monitors
//   /diagnostics      diagnostic_msgs/DiagnosticArray   mapped to fault codes by parameter
// Outputs
//   faults/active     fault_msgs/FaultArray   latched; what is wrong now
//   faults/events     fault_msgs/FaultEvent   every recorded change
// Services
//   faults/acknowledge   fault_msgs/AcknowledgeFault
//   faults/get_history   fault_msgs/GetFaultHistory
//
// Threading: run on a single-threaded executor. fault_core has no locks; all
// callbacks of this node then run one at a time.

#ifndef FAULT_MANAGER__FAULT_MANAGER_NODE_HPP_
#define FAULT_MANAGER__FAULT_MANAGER_NODE_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "fault_core/fault_manager.hpp"
#include "fault_core/history_store.hpp"
#include "fault_manager/snapshot_recorder.hpp"
#include "fault_msgs/msg/fault_array.hpp"
#include "fault_msgs/msg/fault_event.hpp"
#include "fault_msgs/msg/fault_report.hpp"
#include "fault_msgs/srv/acknowledge_fault.hpp"
#include "fault_msgs/srv/get_fault_history.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fault_manager
{

class FaultManagerNode : public rclcpp::Node
{
public:
  /// Throws std::runtime_error if the catalogue cannot be loaded. A robot
  /// should not start with a broken fault catalogue.
  explicit FaultManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_report(const fault_msgs::msg::FaultReport & report);
  void on_diagnostics(const diagnostic_msgs::msg::DiagnosticArray & diagnostics);
  void on_timer();

  /// Persist and publish whatever changed since the last call.
  void process_changes();
  void publish_active();

  std::unique_ptr<fault_core::FaultManager> manager_;
  std::unique_ptr<fault_core::JsonlHistoryStore> store_;
  std::unique_ptr<SnapshotRecorder> recorder_;
  fault_core::Severity capture_min_severity_{fault_core::Severity::Error};

  /// DiagnosticStatus.name -> fault code.
  std::map<std::string, std::string> diagnostic_codes_;
  int diagnostic_fail_level_{2};

  rclcpp::Subscription<fault_msgs::msg::FaultReport>::SharedPtr report_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_subscription_;
  rclcpp::Publisher<fault_msgs::msg::FaultArray>::SharedPtr active_publisher_;
  rclcpp::Publisher<fault_msgs::msg::FaultEvent>::SharedPtr event_publisher_;
  rclcpp::Service<fault_msgs::srv::AcknowledgeFault>::SharedPtr acknowledge_service_;
  rclcpp::Service<fault_msgs::srv::GetFaultHistory>::SharedPtr history_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string last_published_;  ///< Encoded form of the last faults/active message.
  std::chrono::steady_clock::time_point last_publish_time_{};
  std::chrono::steady_clock::duration publish_interval_{std::chrono::seconds(5)};

  /// Watchdogs start counting only this long after start-up; see the
  /// parameter 'startup_grace_s'.
  std::chrono::steady_clock::time_point start_time_{};
  std::chrono::steady_clock::duration startup_grace_{std::chrono::seconds(5)};
};

}  // namespace fault_manager

#endif  // FAULT_MANAGER__FAULT_MANAGER_NODE_HPP_
