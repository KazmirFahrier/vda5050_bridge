// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Translation of raised faults into VDA 5050 errors, so that master control
// sees the vehicle's faults in the `errors` list of its state message.

#ifndef FAULT_CORE__VDA5050_MAPPING_HPP_
#define FAULT_CORE__VDA5050_MAPPING_HPP_

#include <vector>

#include "fault_core/types.hpp"
#include "vda5050_core/types.hpp"

namespace fault_core
{

/// Convert raised faults to VDA 5050 errors, in the order given.
///
/// VDA 5050 knows two levels only. CRITICAL becomes FATAL ("the vehicle cannot
/// continue without help"); ERROR and WARNING become WARNING. Faults below
/// `min_severity` are left out: INFO is for the maintenance log, not for a
/// fleet manager.
///
/// errorType carries the fault code. errorReferences carry the state, the
/// reporting source and the occurrence count. errorDescription is the latest
/// message of the fault, or its catalogue description when there is none.
std::vector<vda5050_core::Error> to_vda5050_errors(
  const std::vector<FaultRecord> & raised, Severity min_severity = Severity::Warning);

}  // namespace fault_core

#endif  // FAULT_CORE__VDA5050_MAPPING_HPP_
