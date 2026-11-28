// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "fault_manager/fault_manager_node.hpp"

#include <algorithm>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "fault_core/catalog.hpp"
#include "fault_core/codec.hpp"
#include "fault_manager/conversions.hpp"

namespace fault_manager
{

namespace
{

fault_core::FaultCatalog load_catalog(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("cannot open fault catalogue '" + path + "'");
  }
  std::stringstream text;
  text << file.rdbuf();
  try {
    return fault_core::FaultCatalog::from_json(text.str());
  } catch (const fault_core::CatalogError & error) {
    throw std::runtime_error("fault catalogue '" + path + "': " + error.what());
  }
}

}  // namespace

FaultManagerNode::FaultManagerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("fault_manager", options)
{
  declare_parameter("catalog_file", "");
  declare_parameter("history_file", "");
  declare_parameter("history_max_bytes", 1048576);
  declare_parameter("history_capacity", 1000);
  declare_parameter("accept_unknown_codes", true);
  declare_parameter("tick_rate_hz", 10.0);
  declare_parameter("publish_interval_s", 5.0);

  declare_parameter("diagnostics.enabled", true);
  declare_parameter("diagnostics.topic", "/diagnostics");
  // Two lists of equal length: a DiagnosticStatus name and the fault code it maps to.
  declare_parameter("diagnostics.names", std::vector<std::string>{});
  declare_parameter("diagnostics.codes", std::vector<std::string>{});
  // DiagnosticStatus level from which the condition counts as failing
  // (0 OK, 1 WARN, 2 ERROR, 3 STALE).
  declare_parameter("diagnostics.fail_level", 2);

  declare_parameter("capture.enabled", false);
  declare_parameter("capture.directory", "fault_captures");
  declare_parameter("capture.topics", std::vector<std::string>{});
  declare_parameter("capture.pre_seconds", 10.0);
  declare_parameter("capture.post_seconds", 2.0);
  declare_parameter("capture.min_severity", "ERROR");
  declare_parameter("capture.max_buffer_mb", 64);
  declare_parameter("capture.max_bags", 20);

  // ---- catalogue and manager ------------------------------------------------
  fault_core::FaultCatalog catalog;
  const std::string catalog_file = get_parameter("catalog_file").as_string();
  if (catalog_file.empty()) {
    RCLCPP_WARN(
      get_logger(), "parameter 'catalog_file' is empty: every fault code will be 'unknown'");
  } else {
    catalog = load_catalog(catalog_file);  // throws
    RCLCPP_INFO(
      get_logger(), "loaded %zu fault definitions from %s",
      catalog.definitions().size(), catalog_file.c_str());
  }

  fault_core::FaultManagerConfig config;
  config.history_capacity =
    static_cast<std::size_t>(std::max<std::int64_t>(1, get_parameter("history_capacity").as_int()));
  config.accept_unknown_codes = get_parameter("accept_unknown_codes").as_bool();
  manager_ = std::make_unique<fault_core::FaultManager>(std::move(catalog), config);

  // ---- history file -----------------------------------------------------------
  const std::string history_file = get_parameter("history_file").as_string();
  if (!history_file.empty()) {
    store_ = std::make_unique<fault_core::JsonlHistoryStore>(
      history_file,
      static_cast<std::uintmax_t>(
        std::max<std::int64_t>(0, get_parameter("history_max_bytes").as_int())));
    const fault_core::LoadResult loaded = store_->load(config.history_capacity);
    manager_->restore_history(loaded.events);
    RCLCPP_INFO(
      get_logger(), "restored %zu events from %s", loaded.events.size(), history_file.c_str());
    if (loaded.skipped_lines > 0) {
      RCLCPP_WARN(
        get_logger(), "%zu unreadable lines in %s were skipped",
        loaded.skipped_lines, history_file.c_str());
    }
  }

  // ---- data capture -----------------------------------------------------------
  if (get_parameter("capture.enabled").as_bool()) {
    SnapshotRecorder::Options capture;
    capture.directory = get_parameter("capture.directory").as_string();
    capture.topics = get_parameter("capture.topics").as_string_array();
    capture.pre_seconds = get_parameter("capture.pre_seconds").as_double();
    capture.post_seconds = get_parameter("capture.post_seconds").as_double();
    capture.max_buffer_bytes = static_cast<std::size_t>(
      std::max<std::int64_t>(1, get_parameter("capture.max_buffer_mb").as_int())) * 1024u * 1024u;
    capture.max_bags =
      static_cast<std::size_t>(std::max<std::int64_t>(0, get_parameter("capture.max_bags").as_int()));
    const std::string min_severity = get_parameter("capture.min_severity").as_string();
    const std::optional<fault_core::Severity> parsed =
      fault_core::severity_from_string(min_severity);
    if (!parsed) {
      throw std::runtime_error(
              "parameter 'capture.min_severity' must be INFO, WARNING, ERROR or CRITICAL, not '" +
              min_severity + "'");
    }
    capture_min_severity_ = *parsed;
    recorder_ = std::make_unique<SnapshotRecorder>(*this, std::move(capture));
  }

  // ---- diagnostics mapping ----------------------------------------------------
  const std::vector<std::string> names = get_parameter("diagnostics.names").as_string_array();
  const std::vector<std::string> codes = get_parameter("diagnostics.codes").as_string_array();
  if (names.size() != codes.size()) {
    throw std::runtime_error(
            "parameters 'diagnostics.names' and 'diagnostics.codes' must have the same length");
  }
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (!names[i].empty() && !codes[i].empty()) {
      diagnostic_codes_[names[i]] = codes[i];
    }
  }
  diagnostic_fail_level_ = static_cast<int>(get_parameter("diagnostics.fail_level").as_int());

  // ---- interfaces -------------------------------------------------------------
  active_publisher_ = create_publisher<fault_msgs::msg::FaultArray>(
    "faults/active", rclcpp::QoS(1).reliable().transient_local());
  event_publisher_ = create_publisher<fault_msgs::msg::FaultEvent>(
    "faults/events", rclcpp::QoS(100).reliable());

  report_subscription_ = create_subscription<fault_msgs::msg::FaultReport>(
    "faults/report", rclcpp::QoS(100).reliable(),
    [this](const fault_msgs::msg::FaultReport & report) {on_report(report);});

  if (get_parameter("diagnostics.enabled").as_bool() && !diagnostic_codes_.empty()) {
    diagnostics_subscription_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      get_parameter("diagnostics.topic").as_string(), rclcpp::QoS(100),
      [this](const diagnostic_msgs::msg::DiagnosticArray & message) {on_diagnostics(message);});
  }

  acknowledge_service_ = create_service<fault_msgs::srv::AcknowledgeFault>(
    "faults/acknowledge",
    [this](
      const std::shared_ptr<fault_msgs::srv::AcknowledgeFault::Request> request,
      std::shared_ptr<fault_msgs::srv::AcknowledgeFault::Response> response) {
      response->success = manager_->acknowledge(request->code);
      if (response->success) {
        RCLCPP_INFO(get_logger(), "fault %s acknowledged", request->code.c_str());
        process_changes();
      } else {
        const std::optional<fault_core::FaultRecord> record = manager_->get(request->code);
        response->message = !record ? "unknown fault code" :
        std::string("fault is ") + fault_core::to_string(record->state) +
        ", not AWAITING_ACK";
      }
    });

  history_service_ = create_service<fault_msgs::srv::GetFaultHistory>(
    "faults/get_history",
    [this](
      const std::shared_ptr<fault_msgs::srv::GetFaultHistory::Request> request,
      std::shared_ptr<fault_msgs::srv::GetFaultHistory::Response> response) {
      fault_core::HistoryQuery query;
      if (!request->code.empty()) {
        query.code = request->code;
      }
      if (request->min_severity > 0) {
        query.min_severity = static_cast<fault_core::Severity>(
          std::min<int>(request->min_severity, static_cast<int>(fault_core::Severity::Critical)));
      }
      query.after_sequence = request->after_sequence;
      query.limit = request->limit;
      for (const fault_core::FaultEvent & event : manager_->history(query)) {
        response->events.push_back(to_msg(event));
      }
    });

  publish_interval_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(std::max(0.1, get_parameter("publish_interval_s").as_double())));
  const double rate = get_parameter("tick_rate_hz").as_double();
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / (rate > 0.0 ? rate : 10.0)), [this] {on_timer();});

  publish_active();  // latched: late subscribers learn at once that nothing is wrong
}

void FaultManagerNode::on_report(const fault_msgs::msg::FaultReport & report)
{
  manager_->report(
    report.code, report.failing, std::chrono::steady_clock::now(), report.source,
    report.message);
  process_changes();
}

void FaultManagerNode::on_diagnostics(const diagnostic_msgs::msg::DiagnosticArray & diagnostics)
{
  const auto now = std::chrono::steady_clock::now();
  for (const diagnostic_msgs::msg::DiagnosticStatus & status : diagnostics.status) {
    const auto mapping = diagnostic_codes_.find(status.name);
    if (mapping == diagnostic_codes_.end()) {
      continue;
    }
    manager_->report(
      mapping->second, static_cast<int>(status.level) >= diagnostic_fail_level_, now,
      status.hardware_id.empty() ? "diagnostics" : status.hardware_id, status.message);
  }
  process_changes();
}

void FaultManagerNode::on_timer()
{
  manager_->tick(std::chrono::steady_clock::now());
  process_changes();
  if (recorder_) {
    recorder_->spin_once();
  }
  if (std::chrono::steady_clock::now() - last_publish_time_ >= publish_interval_) {
    last_published_.clear();  // forces the periodic repeat
    publish_active();
  }
}

void FaultManagerNode::process_changes()
{
  const std::vector<fault_core::FaultEvent> events = manager_->take_new_events();
  if (!events.empty()) {
    if (store_ && !store_->append(events)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 10000, "cannot write fault history to %s",
        store_->path().c_str());
    }
    for (const fault_core::FaultEvent & event : events) {
      RCLCPP_INFO(
        get_logger(), "fault %s %s (#%llu, %s)%s%s", event.code.c_str(),
        fault_core::to_string(event.type),
        static_cast<unsigned long long>(event.sequence),  // NOLINT(runtime/int)
        fault_core::to_string(event.severity), event.message.empty() ? "" : ": ",
        event.message.c_str());
      event_publisher_->publish(to_msg(event));
      if (recorder_ && event.type == fault_core::EventType::Activated &&
        static_cast<int>(event.severity) >= static_cast<int>(capture_min_severity_))
      {
        recorder_->trigger(event);
      }
    }
  }
  publish_active();
}

void FaultManagerNode::publish_active()
{
  const std::vector<fault_core::FaultRecord> raised = manager_->raised();
  // Publish on change only (plus the periodic repeat forced by the timer).
  std::string encoded = fault_core::encode_records(raised);
  if (encoded == last_published_) {
    return;
  }
  fault_msgs::msg::FaultArray message;
  message.stamp = now();
  for (const fault_core::FaultRecord & record : raised) {
    message.faults.push_back(to_msg(record));
  }
  active_publisher_->publish(message);
  last_published_ = std::move(encoded);
  last_publish_time_ = std::chrono::steady_clock::now();
}

}  // namespace fault_manager
