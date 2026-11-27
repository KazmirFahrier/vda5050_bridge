// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>

#include "vda5050_core/header.hpp"

using namespace vda5050_core;  // NOLINT(build/namespaces)

namespace
{

Identity test_identity()
{
  Identity identity;
  identity.manufacturer = "acme";
  identity.serial_number = "amr-001";
  return identity;
}

}  // namespace

TEST(Header, BuildsTopicNameFromIdentityAndMajorVersion)
{
  Identity identity = test_identity();
  EXPECT_EQ(topic_name(identity, "state"), "uagv/v2/acme/amr-001/state");
  identity.version = "3.1.4";
  identity.interface_name = "fleet";
  EXPECT_EQ(topic_name(identity, "order"), "fleet/v3/acme/amr-001/order");
}

TEST(Header, FormatsTimestampAsIso8601Utc)
{
  using std::chrono::milliseconds;
  using std::chrono::system_clock;
  EXPECT_EQ(to_iso8601(system_clock::time_point{}), "1970-01-01T00:00:00.000Z");
  // 2026-11-26T19:50:00.123Z is 1795722600 s after the epoch.
  EXPECT_EQ(
    to_iso8601(system_clock::time_point{milliseconds{1795722600123LL}}),
    "2026-11-26T19:50:00.123Z");
  EXPECT_EQ(
    to_iso8601(system_clock::time_point{milliseconds{-1}}), "1969-12-31T23:59:59.999Z");
}

TEST(Header, CountsHeaderIdsPerTopic)
{
  const auto fixed_clock = [] {return std::chrono::system_clock::time_point{};};
  HeaderFactory factory(test_identity(), fixed_clock);

  EXPECT_EQ(factory.next("state").header_id, 0u);
  EXPECT_EQ(factory.next("state").header_id, 1u);
  EXPECT_EQ(factory.next("connection").header_id, 0u);  // independent counter
  const Header header = factory.next("state");
  EXPECT_EQ(header.header_id, 2u);
  EXPECT_EQ(header.timestamp, "1970-01-01T00:00:00.000Z");
  EXPECT_EQ(header.version, "2.0.0");
  EXPECT_EQ(header.manufacturer, "acme");
  EXPECT_EQ(header.serial_number, "amr-001");
}
