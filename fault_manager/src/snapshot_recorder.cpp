// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "fault_manager/snapshot_recorder.hpp"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <system_error>
#include <utility>

#include "fault_core/codec.hpp"
#include "rosbag2_cpp/writer.hpp"

namespace fault_manager
{

namespace
{

/// A fault code reduced to characters that are safe in a directory name.
std::string sanitized(const std::string & code)
{
  std::string result;
  for (const char c : code) {
    const bool safe = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
      (c >= '0' && c <= '9') || c == '-' || c == '_';
    result.push_back(safe ? c : '_');
  }
  return result.empty() ? "fault" : result.substr(0, 48);
}

/// 2026-11-27T19:33:00.123Z -> 20261127T193300Z
std::string compact_time(fault_core::WallTime time)
{
  std::string text = fault_core::format_time(time);
  text.erase(std::remove(text.begin(), text.end(), '-'), text.end());
  text.erase(std::remove(text.begin(), text.end(), ':'), text.end());
  const std::size_t dot = text.find('.');
  return dot == std::string::npos ? text : text.substr(0, dot) + "Z";
}

std::chrono::steady_clock::duration seconds(double value)
{
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(std::max(0.0, value)));
}

}  // namespace

SnapshotRecorder::SnapshotRecorder(rclcpp::Node & node, Options options)
: node_(node), options_(std::move(options))
{
  // Resolve the names once (namespace, relative names), so that they can be
  // compared with what the ROS graph reports.
  for (std::string & topic : options_.topics) {
    topic = node_.get_node_topics_interface()->resolve_topic_name(topic);
  }
}

void SnapshotRecorder::trigger(const fault_core::FaultEvent & event)
{
  if (pending_) {
    pending_->also_activated.push_back(event.code);
    return;
  }
  Pending pending;
  pending.event = event;
  pending.deadline = std::chrono::steady_clock::now() + seconds(options_.post_seconds);
  pending_ = std::move(pending);
}

void SnapshotRecorder::spin_once()
{
  const SteadyTime now = std::chrono::steady_clock::now();
  if (now - last_graph_check_ >= std::chrono::seconds(1)) {
    last_graph_check_ = now;
    subscribe_to_new_topics();
  }
  if (pending_ && now >= pending_->deadline) {
    const Pending pending = std::move(*pending_);
    pending_.reset();
    write(pending);
  }
  prune(now);
}

void SnapshotRecorder::subscribe_to_new_topics()
{
  if (subscriptions_.size() == options_.topics.size()) {
    return;
  }
  const auto graph = node_.get_topic_names_and_types();
  for (const std::string & topic : options_.topics) {
    if (subscriptions_.count(topic) > 0) {
      continue;
    }
    const auto found = graph.find(topic);
    if (found == graph.end() || found->second.empty()) {
      continue;  // nobody publishes it yet; look again later
    }
    const std::string type = found->second.front();
    try {
      // Best effort matches both reliable and best-effort publishers.
      subscriptions_[topic] = node_.create_generic_subscription(
        topic, type, rclcpp::QoS(rclcpp::KeepLast(100)).best_effort(),
        [this, topic, type](std::shared_ptr<rclcpp::SerializedMessage> message) {
          on_message(topic, type, std::move(message));
        });
      RCLCPP_INFO(
        node_.get_logger(), "capture: buffering %s [%s]", topic.c_str(), type.c_str());
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        node_.get_logger(), "capture: cannot subscribe to %s [%s]: %s",
        topic.c_str(), type.c_str(), error.what());
    }
  }
}

void SnapshotRecorder::on_message(
  const std::string & topic, const std::string & type,
  std::shared_ptr<rclcpp::SerializedMessage> message)
{
  buffer_bytes_ += message->size();
  buffer_.push_back(
    Entry{topic, type, std::move(message), node_.now(), std::chrono::steady_clock::now()});
  while (buffer_bytes_ > options_.max_buffer_bytes && !buffer_.empty()) {
    buffer_bytes_ -= buffer_.front().message->size();
    buffer_.pop_front();
  }
}

void SnapshotRecorder::prune(SteadyTime now)
{
  if (pending_) {
    return;  // everything in the buffer belongs to the capture in progress
  }
  const auto keep = seconds(options_.pre_seconds);
  while (!buffer_.empty() && now - buffer_.front().received > keep) {
    buffer_bytes_ -= buffer_.front().message->size();
    buffer_.pop_front();
  }
}

void SnapshotRecorder::write(const Pending & pending)
{
  char sequence[32];
  std::snprintf(
    sequence, sizeof(sequence), "%06llu",
    static_cast<unsigned long long>(pending.event.sequence));  // NOLINT(runtime/int)
  const std::filesystem::path directory = std::filesystem::path(options_.directory) /
    ("fault_" + std::string(sequence) + "_" + sanitized(pending.event.code) + "_" +
    compact_time(pending.event.time));

  std::size_t written = 0;
  try {
    std::filesystem::create_directories(options_.directory);
    {
      rosbag2_cpp::Writer writer;
      writer.open(directory.string());  // creates the directory; fails if it exists
      for (const Entry & entry : buffer_) {
        writer.write(*entry.message, entry.topic, entry.type, entry.stamp);
        ++written;
      }
    }  // closing the writer finishes the bag and writes metadata.yaml

    // A small summary next to the bag, readable without ROS tools.
    std::ofstream summary(directory / "fault.json");
    summary << "{\"event\":" << fault_core::encode_event(pending.event) <<
      ",\"messages\":" << written << ",\"preSeconds\":" << options_.pre_seconds <<
      ",\"postSeconds\":" << options_.post_seconds << ",\"alsoActivated\":[";
    for (std::size_t i = 0; i < pending.also_activated.size(); ++i) {
      summary << (i == 0 ? "" : ",") << '"' << sanitized(pending.also_activated[i]) << '"';
    }
    summary << "]}\n";
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      node_.get_logger(), "capture: could not write %s: %s",
      directory.string().c_str(), error.what());
    return;
  }

  last_bag_ = directory.string();
  RCLCPP_INFO(
    node_.get_logger(), "capture: wrote %zu messages for fault %s to %s",
    written, pending.event.code.c_str(), last_bag_.c_str());
  delete_old_bags();
}

void SnapshotRecorder::delete_old_bags()
{
  if (options_.max_bags == 0) {
    return;
  }
  std::error_code error;
  std::vector<std::filesystem::path> bags;
  for (const auto & item : std::filesystem::directory_iterator(options_.directory, error)) {
    // Only directories this class created; never anything else in the folder.
    if (item.is_directory() && item.path().filename().string().rfind("fault_", 0) == 0) {
      bags.push_back(item.path());
    }
  }
  if (bags.size() <= options_.max_bags) {
    return;
  }
  // The name starts with the zero-padded sequence number, so name order is age order.
  std::sort(bags.begin(), bags.end());
  for (std::size_t i = 0; i + options_.max_bags < bags.size(); ++i) {
    std::filesystem::remove_all(bags[i], error);
  }
}

}  // namespace fault_manager
