# Roadmap and known limitations

## Where things stand

| Area | State |
|---|---|
| `vda5050_core` | Implemented; 109 unit tests, 98.8 % line coverage, schema-conformant |
| `vda5050_nav2_adapter` | Implemented; builds on ROS 2 Jazzy; integration test and Nav2 system test (loopback and Gazebo) pass in CI |
| CI | Eight jobs: two standalone builds, colcon build with tests and linters plus the integration test, schema conformance, coverage, fuzzing, Nav2 in two simulators |

## Next steps, in order

### 1. Nav2 parameters for deployment
- Ship a recommended Nav2 parameter file, with a goal tolerance tighter than
  the default 0.25 m so the vehicle stops within the fleet manager's
  `allowedDeviationXY`
- Tune the controller and collision monitor for narrow passages and for
  obstacles that appear close ahead: with the defaults Nav2 stalled in the
  pillar gap once, gave up on a partly blocked strip in two runs out of four,
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
- `diagnostic_msgs` output and a fault-history service
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
