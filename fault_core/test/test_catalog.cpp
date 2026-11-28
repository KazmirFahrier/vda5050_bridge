// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "fault_core/catalog.hpp"

using namespace fault_core;  // NOLINT(build/namespaces)

namespace
{

/// The CatalogError message for `text`, or "" if it parses.
std::string catalog_error(const std::string & text)
{
  try {
    FaultCatalog::from_json(text);
  } catch (const CatalogError & error) {
    return error.what();
  }
  return "";
}

bool contains(const std::string & text, const std::string & part)
{
  return text.find(part) != std::string::npos;
}

}  // namespace

TEST(Catalog, ParsesAFullDefinition)
{
  const FaultCatalog catalog = FaultCatalog::from_json(
    R"({"faults":[{"code":"NAV-001","severity":"CRITICAL","description":"Path blocked",)"
    R"("activateAfterMs":500,"clearAfterMs":2000,"latched":true,"timeoutMs":3000}]})");

  const FaultDefinition * definition = catalog.find("NAV-001");
  ASSERT_NE(definition, nullptr);
  EXPECT_TRUE(definition->severity == Severity::Critical);
  EXPECT_EQ(definition->description, "Path blocked");
  EXPECT_EQ(definition->activate_after.count(), 500);
  EXPECT_EQ(definition->clear_after.count(), 2000);
  EXPECT_EQ(definition->timeout.count(), 3000);
  EXPECT_TRUE(definition->latched);
  EXPECT_EQ(catalog.find("NAV-999"), nullptr);
}

TEST(Catalog, AppliesDefaultsWhenOnlyTheCodeIsGiven)
{
  const FaultCatalog catalog = FaultCatalog::from_json(R"({"faults":[{"code":"X"}]})");
  const FaultDefinition * definition = catalog.find("X");
  ASSERT_NE(definition, nullptr);
  EXPECT_TRUE(definition->severity == Severity::Error);
  EXPECT_EQ(definition->activate_after.count(), 0);
  EXPECT_EQ(definition->clear_after.count(), 0);
  EXPECT_EQ(definition->timeout.count(), 0);
  EXPECT_FALSE(definition->latched);
  EXPECT_EQ(definition->description, "");
}

TEST(Catalog, AcceptsAnEmptyList)
{
  EXPECT_TRUE(FaultCatalog::from_json(R"({"faults":[]})").definitions().empty());
}

TEST(Catalog, ErrorsNameTheOffendingEntry)
{
  EXPECT_TRUE(contains(catalog_error("not json"), "catalog: invalid JSON"));
  EXPECT_TRUE(contains(catalog_error("[]"), "expected an object with a \"faults\" array"));
  EXPECT_TRUE(contains(catalog_error(R"({"faults":{}})"), "\"faults\" array"));
  EXPECT_TRUE(contains(catalog_error(R"({"faults":[5]})"), "faults[0]: expected object"));
  EXPECT_TRUE(contains(catalog_error(R"({"faults":[{}]})"), "faults[0].code"));
  EXPECT_TRUE(contains(catalog_error(R"({"faults":[{"code":""}]})"), "faults[0].code"));
  EXPECT_TRUE(
    contains(catalog_error(R"({"faults":[{"code":"A","severity":"BAD"}]})"), "A.severity"));
  EXPECT_TRUE(
    contains(catalog_error(R"({"faults":[{"code":"A","severity":3}]})"), "A.severity"));
  EXPECT_TRUE(
    contains(catalog_error(R"({"faults":[{"code":"A","description":1}]})"), "A.description"));
  EXPECT_TRUE(
    contains(catalog_error(R"({"faults":[{"code":"A","latched":"yes"}]})"), "A.latched"));
  EXPECT_TRUE(
    contains(catalog_error(R"({"faults":[{"code":"A","clearAfterMs":-1}]})"), "A.clearAfterMs"));
  EXPECT_TRUE(
    contains(catalog_error(R"({"faults":[{"code":"A","timeoutMs":1.5}]})"), "A.timeoutMs"));
  EXPECT_TRUE(
    contains(
      catalog_error(R"({"faults":[{"code":"A","activateAfterMs":"1"}]})"), "A.activateAfterMs"));
  EXPECT_TRUE(
    contains(
      catalog_error(R"({"faults":[{"code":"A"},{"code":"B"},{"code":"A"}]})"),
      "A: duplicate fault code"));
}

TEST(Catalog, BuildsFromDefinitionsAndValidatesThem)
{
  FaultDefinition good;
  good.code = "A";
  EXPECT_NE(FaultCatalog({good}).find("A"), nullptr);

  FaultDefinition unnamed;
  EXPECT_THROW(FaultCatalog({unnamed}), CatalogError);

  FaultDefinition negative;
  negative.code = "B";
  negative.clear_after = Duration{-5};
  EXPECT_THROW(FaultCatalog({negative}), CatalogError);

  EXPECT_THROW(FaultCatalog({good, good}), CatalogError);
}
