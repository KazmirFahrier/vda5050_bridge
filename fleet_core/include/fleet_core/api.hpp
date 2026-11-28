// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// The fleet manager's interface for whoever gives it work: a warehouse
// management system, an operator's console, a test script. Text in, text
// out (JSON), with no knowledge of how the text travels. The daemon in
// vda5050_fleet carries it over MQTT.
//
// Commands (one JSON object each):
//
//   {"command":"submit","requestId":"r1","priority":0,"vehicle":"",
//    "stops":[{"node":"A","dwell":5,
//              "actions":[{"actionType":"pick","blockingType":"HARD",
//                          "actionParameters":[{"key":"k","value":1}]}]}]}
//   {"command":"cancel","requestId":"r2","taskId":3}
//   {"command":"resetVehicle","requestId":"r3","serialNumber":"v1"}
//
// Every command gets exactly one response, also when it cannot be read:
//
//   {"requestId":"r1","ok":true,"taskId":3}
//   {"requestId":"r2","ok":false,"error":"no such task, or it is already over"}
//
// Nothing a client sends can throw past handle_command().

#ifndef FLEET_CORE__API_HPP_
#define FLEET_CORE__API_HPP_

#include <string>
#include <string_view>

#include "fleet_core/fleet_manager.hpp"

namespace fleet_core
{

/// Carry out one command and return the response.
std::string handle_command(FleetManager & fleet, std::string_view text, TimePoint now);

/// Everything an operator's display needs: vehicles, tasks, counters.
std::string encode_status(const FleetManager & fleet);

/// One event, for a log or a live display. `seconds` is the event's time
/// in whatever clock the caller keeps.
std::string encode_event(const FleetEvent & event, double seconds);

const char * to_string(TaskState state);

}  // namespace fleet_core

#endif  // FLEET_CORE__API_HPP_
