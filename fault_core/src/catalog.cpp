// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "fault_core/catalog.hpp"

#include <cmath>

#include "fault_core/codec.hpp"
#include "vda5050_core/json.hpp"

namespace fault_core
{

namespace
{

using vda5050_core::Json;

[[noreturn]] void fail(const std::string & where, const std::string & reason)
{
  throw CatalogError(where + ": " + reason);
}

Duration read_duration(const Json & entry, const char * key, const std::string & where)
{
  const Json * value = entry.find(key);
  if (value == nullptr) {
    return Duration{0};
  }
  if (!value->is_number() || value->as_number() < 0.0 ||
    value->as_number() != std::floor(value->as_number()) || value->as_number() > 8.64e10)
  {
    fail(where + "." + key, "expected a whole number of milliseconds, 0 or more");
  }
  return Duration{static_cast<Duration::rep>(value->as_number())};
}

}  // namespace

FaultCatalog::FaultCatalog(const std::vector<FaultDefinition> & definitions)
{
  for (const FaultDefinition & definition : definitions) {
    add(definition);
  }
}

void FaultCatalog::add(const FaultDefinition & definition)
{
  if (definition.code.empty()) {
    fail("fault", "code must not be empty");
  }
  if (definition.activate_after.count() < 0 || definition.clear_after.count() < 0 ||
    definition.timeout.count() < 0)
  {
    fail(definition.code, "durations must not be negative");
  }
  if (!definitions_.emplace(definition.code, definition).second) {
    fail(definition.code, "duplicate fault code");
  }
}

FaultCatalog FaultCatalog::from_json(std::string_view text)
{
  Json root;
  try {
    root = Json::parse(text);
  } catch (const vda5050_core::JsonParseError & error) {
    fail("catalog", std::string("invalid JSON: ") + error.what());
  }
  const Json * faults = root.find("faults");
  if (faults == nullptr || !faults->is_array()) {
    fail("catalog", "expected an object with a \"faults\" array");
  }

  FaultCatalog catalog;
  std::size_t index = 0;
  for (const Json & entry : faults->as_array()) {
    std::string where = "faults[" + std::to_string(index++) + "]";
    if (!entry.is_object()) {
      fail(where, "expected object");
    }
    const Json * code = entry.find("code");
    if (code == nullptr || !code->is_string() || code->as_string().empty()) {
      fail(where + ".code", "expected a non-empty string");
    }
    where = code->as_string();  // from here on, name the entry by its code

    FaultDefinition definition;
    definition.code = code->as_string();
    if (const Json * severity = entry.find("severity")) {
      const auto parsed =
        severity->is_string() ? severity_from_string(severity->as_string()) : std::nullopt;
      if (!parsed) {
        fail(where + ".severity", "expected one of INFO, WARNING, ERROR, CRITICAL");
      }
      definition.severity = *parsed;
    }
    if (const Json * description = entry.find("description")) {
      if (!description->is_string()) {
        fail(where + ".description", "expected string");
      }
      definition.description = description->as_string();
    }
    if (const Json * latched = entry.find("latched")) {
      if (!latched->is_bool()) {
        fail(where + ".latched", "expected boolean");
      }
      definition.latched = latched->as_bool();
    }
    definition.activate_after = read_duration(entry, "activateAfterMs", where);
    definition.clear_after = read_duration(entry, "clearAfterMs", where);
    definition.timeout = read_duration(entry, "timeoutMs", where);
    catalog.add(definition);
  }
  return catalog;
}

const FaultDefinition * FaultCatalog::find(const std::string & code) const
{
  const auto it = definitions_.find(code);
  return it == definitions_.end() ? nullptr : &it->second;
}

}  // namespace fault_core
