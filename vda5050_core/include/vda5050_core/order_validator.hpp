// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Protocol-level checks on a decoded order: the rules a structurally valid
// JSON document can still break.

#ifndef VDA5050_CORE__ORDER_VALIDATOR_HPP_
#define VDA5050_CORE__ORDER_VALIDATOR_HPP_

#include <string>
#include <vector>

#include "vda5050_core/types.hpp"

namespace vda5050_core
{

/// Check an order against the VDA 5050 graph rules. Returns one human-readable
/// message per violation; an empty vector means the order is well-formed.
///
/// Rules checked:
///  - orderId is not empty and there is at least one node
///  - there is exactly one edge between each pair of consecutive nodes
///  - node sequenceIds are even and increase by 2; each edge's sequenceId is
///    the odd number between its two nodes
///  - each edge's startNodeId/endNodeId name the nodes on either side of it
///  - the first node is released, and the base (released part) is contiguous:
///    nothing released may follow something unreleased
///  - an edge is released exactly when its end node is released
///  - actionIds are unique within the order
///
/// This is a pure function: it does not look at vehicle state. Whether the
/// order can be *accepted right now* is decided by OrderManager.
std::vector<std::string> validate_order(const Order & order);

}  // namespace vda5050_core

#endif  // VDA5050_CORE__ORDER_VALIDATOR_HPP_
