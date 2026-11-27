// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Conversion between VDA 5050 JSON payloads and the types in types.hpp.
// This is the only place that knows the JSON field names.

#ifndef VDA5050_CORE__CODEC_HPP_
#define VDA5050_CORE__CODEC_HPP_

#include <stdexcept>
#include <string>
#include <string_view>

#include "vda5050_core/types.hpp"

namespace vda5050_core
{

/// Thrown when a payload is not valid JSON or does not match the message layout.
/// The message names the offending field, e.g. "order.nodes[1].sequenceId: ...",
/// so it can be reported to master control verbatim as a validationError.
class DecodeError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Decode an `order` topic payload. Checks structure and types only; the
/// protocol rules (sequence ids, base/horizon) are checked by validate_order().
Order decode_order(std::string_view payload);

/// Decode an `instantActions` topic payload.
InstantActions decode_instant_actions(std::string_view payload);

/// Encode a `state` topic payload.
std::string encode_state(const State & state);

/// Encode a `connection` topic payload.
std::string encode_connection(const Connection & connection);

/// Encode a `visualization` topic payload.
std::string encode_visualization(const Visualization & visualization);

/// Encode a `factsheet` topic payload.
std::string encode_factsheet(const Factsheet & factsheet);

// String forms of the protocol enums (used by the codec and by log messages).
const char * to_string(BlockingType value);
const char * to_string(ActionStatus value);
const char * to_string(ErrorLevel value);
const char * to_string(OperatingMode value);
const char * to_string(EStop value);
const char * to_string(ConnectionState value);

}  // namespace vda5050_core

#endif  // VDA5050_CORE__CODEC_HPP_
