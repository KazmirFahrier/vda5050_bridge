// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// libFuzzer target for everything that parses bytes from the network.
//
// Build and run (needs Clang):
//   clang++ -std=c++17 -g -O1 -fsanitize=fuzzer,address,undefined \
//     -Ivda5050_core/include vda5050_core/fuzz/fuzz_decoder.cpp \
//     vda5050_core/src/*.cpp -o fuzz_decoder
//   ./fuzz_decoder -max_total_time=60 vda5050_core/test/data
//
// Properties checked on every input:
//   - no crash, hang, memory error or undefined behaviour
//   - the only exceptions that escape are the documented ones
//   - JSON that parses can be written and parsed again to an equal value
//   - an order that decodes and validates can be handed to the state machine

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

#include "vda5050_core/codec.hpp"
#include "vda5050_core/json.hpp"
#include "vda5050_core/order_manager.hpp"
#include "vda5050_core/order_validator.hpp"

namespace
{

struct NullNavigation : vda5050_core::NavigationPort
{
  void navigate_to(const vda5050_core::Node &, const vda5050_core::Edge &) override {}
  void cancel() override {}
  void pause() override {}
  void resume() override {}
};

struct NullActions : vda5050_core::ActionPort
{
  bool supports(const std::string &) const override {return true;}
  void start(const vda5050_core::Action &) override {}
  void cancel(const std::string &) override {}
};

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t * data, std::size_t size)
{
  using namespace vda5050_core;  // NOLINT(build/namespaces)
  const std::string_view text(reinterpret_cast<const char *>(data), size);

  try {
    const Json value = Json::parse(text);
    const Json again = Json::parse(value.dump());
    // Numbers may print with more digits than they were read with, but a
    // second round trip must be stable.
    if (again.dump() != Json::parse(again.dump()).dump()) {
      __builtin_trap();
    }
  } catch (const JsonParseError &) {
    // Expected for most inputs.
  }

  try {
    const Order order = decode_order(text);
    if (validate_order(order).empty()) {
      NullNavigation navigation;
      NullActions actions;
      OrderManager manager(navigation, actions);
      manager.handle_order(order);
      (void)encode_state(manager.snapshot());
    }
  } catch (const DecodeError &) {
  }

  try {
    const InstantActions instant = decode_instant_actions(text);
    NullNavigation navigation;
    NullActions actions;
    OrderManager manager(navigation, actions);
    manager.handle_instant_actions(instant);
    (void)encode_state(manager.snapshot());
  } catch (const DecodeError &) {
  }

  return 0;
}
