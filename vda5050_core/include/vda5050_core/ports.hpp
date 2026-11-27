// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Ports: the interfaces through which the protocol logic drives a robot.
//
// OrderManager knows *what* the vehicle must do next according to VDA 5050.
// It does not know *how*: that is supplied by an adapter implementing these
// interfaces (Nav2 action clients in the ROS 2 package, fakes in unit tests).
// This is the "ports and adapters" (hexagonal) pattern; see docs/architecture.md.

#ifndef VDA5050_CORE__PORTS_HPP_
#define VDA5050_CORE__PORTS_HPP_

#include <string>

#include "vda5050_core/types.hpp"

namespace vda5050_core
{

/// Moves the vehicle. All calls are requests and must return quickly; the
/// outcome is reported later through OrderManager::on_navigation_succeeded()
/// or on_navigation_failed().
class NavigationPort
{
public:
  virtual ~NavigationPort() = default;

  /// Drive to `target` along `via`. Exactly one result must follow.
  virtual void navigate_to(const Node & target, const Edge & via) = 0;

  /// Abort the current drive. A result (success or failure) must still follow,
  /// which is how OrderManager learns the vehicle has actually stopped.
  virtual void cancel() = 0;

  /// Hold position without abandoning the goal / continue afterwards.
  virtual void pause() = 0;
  virtual void resume() = 0;
};

/// Executes VDA 5050 actions (pick, drop, charge, custom...). Progress is
/// reported through OrderManager::on_action_status().
class ActionPort
{
public:
  virtual ~ActionPort() = default;

  /// Whether this vehicle can execute `action_type`. Orders containing an
  /// unsupported action are rejected up front rather than failing half-way.
  virtual bool supports(const std::string & action_type) const = 0;

  /// Begin executing the action.
  virtual void start(const Action & action) = 0;

  /// Stop a started action (used when an order is cancelled).
  virtual void cancel(const std::string & action_id) = 0;
};

}  // namespace vda5050_core

#endif  // VDA5050_CORE__PORTS_HPP_
