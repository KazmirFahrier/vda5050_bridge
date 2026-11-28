# Roadmap and known limitations

## Where things stand

| Area | State |
|---|---|
| `vda5050_core` | Implemented; 115 unit tests, 98.8 % line coverage, schema-conformant |
| `fault_core` | Implemented; 54 unit tests |
| `fault_msgs`, `fault_manager`, `fault_gateway` | Implemented; 36 unit tests for the gateway; one end-to-end test of all fault nodes passes in CI |
| `vda5050_nav2_adapter` | Implemented; builds on ROS 2 Jazzy; integration test and Nav2 system test (loopback and Gazebo) pass in CI |
| CI | Eight jobs: two standalone builds, colcon build with tests and linters plus the integration test and the fault system test, schema conformance, coverage, fuzzing, Nav2 in two simulators |

## Fault management

All planned steps are implemented and verified in CI. See
[fault_management.md](fault_management.md) for the design and its limits.

| Step | Content | State |
|---|---|---|
| 1 | `fault_core` library; `OrderManager::set_external_errors()`; mapping to VDA 5050 errors | Done |
| 2 | `fault_msgs` and the `fault_manager` node: report topic, `/diagnostics` input, services to query and acknowledge, history file with rotation | Done |
| 3 | The adapter reports its own health, lists raised faults in `state.errors` and holds the vehicle on a critical fault | Done |
| 4 | Flight recorder: a rosbag2 bag of the seconds before and after a fault | Done |
| 5 | Web API (REST and server-sent events) and a status page | Done |

Possible next steps here:

- Monitors for real hardware (scan rate, battery, motor drivers), or a generic
  "topic rate" monitor configured by parameter
- List and download captures through the web API
- TLS, user accounts and an audit trail for the web API
- Unit tests for the ROS code of `fault_manager` and `fault_gateway`
- Make the fault manager a lifecycle node

## Next steps for the bridge, in order

### 1. Nav2 parameters for deployment
- Ship a recommended Nav2 parameter file, with a goal tolerance tighter than
  the default 0.25 m so the vehicle stops within the fleet manager's
  `allowedDeviationXY`
- Tune the controller and collision monitor for narrow passages and for
  obstacles that appear close ahead: with the defaults Nav2 stalled in the
  pillar gap once, gave up on a partly blocked strip in two runs out of five,
  and in one run brushed a box that was dropped half a metre in front of the
  moving robot

### 2. A real counterpart
- Run against a real fleet manager (for example Open-RMF's VDA 5050 support or
  a vendor's test instance)
- Run on a physical robot

### 3. Tests for the adapter's classes
- Unit tests with a mocked action server for `Nav2NavigationAdapter`
- Coverage measurement for the adapter package

### 4. Protocol completeness
- Decide the `PAUSED` action status question once the official schemas and the
  specification text agree
- `loads`, `information`, `distanceSinceLastNode`, `trajectory`
- v2.1 additions (for example `maps`)
- Standard actions with defined behaviour: `initPosition`, `startCharging`,
  `stopCharging`, `logReport`
- Factsheet geometry and load specification (currently present but empty)

### Later
- VDA 5050 v3.0 codec alongside v2 (the state machine is wire-format neutral)
- TLS and certificate authentication for MQTT
- `NavigateThroughPoses` across several released nodes for smoother motion

## Known limitations

Behavioural simplifications in the core, all deliberate and all testable to change:

| Area | Current behaviour | Specification intent |
|---|---|---|
| Blocking edge actions | Executed at the start of the edge, before moving | May require stopping part-way along the edge |
| Edge action lifetime | An edge action still running when the next node is reached keeps running | Edge actions end when the edge is left |
| Instant actions with SOFT/HARD blocking | Do not stop a vehicle that is already driving | Should take effect on driving too |
| Pause | Navigation is paused; running actions are not told to pause | Actions that can pause should report `PAUSED` |
| Failed action | Warning, order continues | Left to master control; a per-type policy may be wanted |
| Stitching node actions | Actions listed on an update's first node are ignored | Should match what the vehicle already has |
| `allowedDeviationTheta` | Parsed, not used for the first-node check | Orientation should be checked too |
| Factsheet request without a configured factsheet | The instant action reports `FINISHED` but nothing is published | Should fail |

Limitations of the adapter design:

| Area | Current behaviour |
|---|---|
| Pause | Implemented by cancelling the Nav2 goal and re-sending it on resume; Nav2 replans from the current pose |
| Edge properties | `maxSpeed`, `orientation` and other edge fields are not passed to Nav2 |
| Goal orientation without `theta` | Faces along the direction of travel, measured from the robot's current position |
| Offline publishing | Messages published while the broker is unreachable are dropped; a full state is sent on reconnect |
| Security | Username/password only, sent in clear text without TLS |

Not implemented at all: `loads`, `information`, `trajectory`, `zoneSetId`
semantics (stored and echoed only).
