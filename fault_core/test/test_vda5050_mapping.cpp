// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "fault_core/fault_manager.hpp"
#include "fault_core/vda5050_mapping.hpp"
#include "vda5050_core/codec.hpp"
#include "vda5050_core/json.hpp"
#include "vda5050_core/order_manager.hpp"

using namespace fault_core;  // NOLINT(build/namespaces)

namespace
{

FaultRecord record(const std::string & code, Severity severity, FaultState state)
{
  FaultRecord result;
  result.code = code;
  result.severity = severity;
  result.state = state;
  result.description = "catalogue text for " + code;
  result.occurrence_count = 2;
  return result;
}

}  // namespace

TEST(Vda5050Mapping, MapsSeverityToTheTwoVda5050Levels)
{
  const auto errors = to_vda5050_errors(
  {
    record("C", Severity::Critical, FaultState::Active),
    record("E", Severity::Error, FaultState::Active),
    record("W", Severity::Warning, FaultState::Healing),
    record("I", Severity::Info, FaultState::Active),
    });
  ASSERT_EQ(errors.size(), 3u);  // INFO is not sent to master control by default
  EXPECT_EQ(errors[0].error_type, "C");
  EXPECT_TRUE(errors[0].error_level == vda5050_core::ErrorLevel::Fatal);
  EXPECT_TRUE(errors[1].error_level == vda5050_core::ErrorLevel::Warning);
  EXPECT_TRUE(errors[2].error_level == vda5050_core::ErrorLevel::Warning);

  EXPECT_EQ(to_vda5050_errors({record("I", Severity::Info, FaultState::Active)},
    Severity::Info).size(), 1u);
  EXPECT_TRUE(to_vda5050_errors({record("E", Severity::Error, FaultState::Active)},
    Severity::Critical).empty());
}

TEST(Vda5050Mapping, CarriesDescriptionAndReferences)
{
  FaultRecord with_message = record("NAV-001", Severity::Error, FaultState::AwaitingAck);
  with_message.message = "blocked at dock 3";
  with_message.source = "controller_server";
  const auto errors = to_vda5050_errors(
    {with_message, record("BAT-001", Severity::Warning, FaultState::Active)});

  ASSERT_EQ(errors.size(), 2u);
  EXPECT_EQ(errors[0].error_description, "blocked at dock 3");
  ASSERT_EQ(errors[0].error_references.size(), 3u);
  EXPECT_EQ(errors[0].error_references[0].reference_key, "faultState");
  EXPECT_EQ(errors[0].error_references[0].reference_value, "AWAITING_ACK");
  EXPECT_EQ(errors[0].error_references[1].reference_value, "controller_server");
  EXPECT_EQ(errors[0].error_references[2].reference_key, "occurrenceCount");
  EXPECT_EQ(errors[0].error_references[2].reference_value, "2");

  EXPECT_EQ(errors[1].error_description, "catalogue text for BAT-001");  // no message yet
  EXPECT_EQ(errors[1].error_references.size(), 2u);  // no source
}

TEST(Vda5050Mapping, SkipsFaultsThatAreNotRaised)
{
  EXPECT_TRUE(
    to_vda5050_errors(
      {record("A", Severity::Critical, FaultState::Inactive),
        record("B", Severity::Critical, FaultState::Pending)}).empty());
}

// The whole path: a fault is reported, and it appears in the JSON state
// message that the bridge would publish to master control.
TEST(Vda5050Mapping, AFaultReachesTheEncodedStateMessage)
{
  FaultDefinition definition;
  definition.code = "LIDAR-001";
  definition.severity = Severity::Critical;
  definition.description = "Lidar is not delivering scans";
  FaultManager faults{FaultCatalog({definition})};

  // The order manager needs its ports only when it executes an order; none
  // are called here.
  struct NoNavigation : vda5050_core::NavigationPort
  {
    void navigate_to(const vda5050_core::Node &, const vda5050_core::Edge &) override {}
    void cancel() override {}
    void pause() override {}
    void resume() override {}
  } navigation;
  struct NoActions : vda5050_core::ActionPort
  {
    bool supports(const std::string &) const override {return false;}
    void start(const vda5050_core::Action &) override {}
    void cancel(const std::string &) override {}
  } actions;
  vda5050_core::OrderManager orders(navigation, actions);
  orders.consume_state_changed();

  faults.report("LIDAR-001", true, TimePoint{}, "lidar_driver", "no scan for 2.0 s");
  orders.set_external_errors(to_vda5050_errors(faults.raised()));
  EXPECT_TRUE(orders.consume_state_changed());

  const vda5050_core::Json state =
    vda5050_core::Json::parse(vda5050_core::encode_state(orders.snapshot()));
  const auto & errors = state.find("errors")->as_array();
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0].find("errorType")->as_string(), "LIDAR-001");
  EXPECT_EQ(errors[0].find("errorLevel")->as_string(), "FATAL");
  EXPECT_EQ(errors[0].find("errorDescription")->as_string(), "no scan for 2.0 s");

  faults.report("LIDAR-001", false, TimePoint{} + Duration{100});
  orders.set_external_errors(to_vda5050_errors(faults.raised()));
  EXPECT_TRUE(orders.snapshot().errors.empty());
}
