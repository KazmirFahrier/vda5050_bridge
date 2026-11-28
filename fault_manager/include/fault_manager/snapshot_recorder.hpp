// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// SnapshotRecorder: a "flight recorder" for faults.
//
// It keeps the last few seconds of the configured topics in memory. When a
// fault activates, it keeps recording for a short time and then writes
// everything to a rosbag2 bag, so the moments before and after the fault can
// be replayed later with `ros2 bag play`.
//
// Messages are held in serialized form and never decoded, so any message type
// can be captured without this package depending on it.

#ifndef FAULT_MANAGER__SNAPSHOT_RECORDER_HPP_
#define FAULT_MANAGER__SNAPSHOT_RECORDER_HPP_

#include <chrono>
#include <cstddef>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "fault_core/types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"

namespace fault_manager
{

class SnapshotRecorder
{
public:
  struct Options
  {
    std::string directory;            ///< Where bags are written; one subdirectory per fault.
    std::vector<std::string> topics;  ///< Topics to keep in the buffer.
    double pre_seconds{10.0};         ///< How much history before the fault is kept.
    double post_seconds{2.0};         ///< How long recording continues after the fault.
    std::size_t max_buffer_bytes{64u * 1024u * 1024u};  ///< Memory bound of the buffer.
    std::size_t max_bags{20};         ///< Oldest bags beyond this number are deleted. 0: keep all.
  };

  SnapshotRecorder(rclcpp::Node & node, Options options);

  /// A fault activated. Starts a capture unless one is already running; in
  /// that case the fault is noted in the running capture's summary file.
  void trigger(const fault_core::FaultEvent & event);

  /// Call regularly (from the node's timer). Subscribes to configured topics
  /// once they exist, drops old messages, and writes a finished capture.
  void spin_once();

  /// Directory of the most recently written bag, empty if none yet.
  const std::string & last_bag() const noexcept {return last_bag_;}

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  struct Entry
  {
    std::string topic;
    std::string type;
    std::shared_ptr<rclcpp::SerializedMessage> message;
    rclcpp::Time stamp;   ///< Reception time on the node's clock; goes into the bag.
    SteadyTime received;  ///< For pruning; unaffected by clock jumps.
  };

  struct Pending
  {
    fault_core::FaultEvent event;
    std::vector<std::string> also_activated;
    SteadyTime deadline;
  };

  void subscribe_to_new_topics();
  void on_message(
    const std::string & topic, const std::string & type,
    std::shared_ptr<rclcpp::SerializedMessage> message);
  void prune(SteadyTime now);
  void write(const Pending & pending);
  void delete_old_bags();

  rclcpp::Node & node_;
  Options options_;
  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::deque<Entry> buffer_;
  std::size_t buffer_bytes_{0};
  std::optional<Pending> pending_;
  SteadyTime last_graph_check_{};
  std::string last_bag_;
};

}  // namespace fault_manager

#endif  // FAULT_MANAGER__SNAPSHOT_RECORDER_HPP_
