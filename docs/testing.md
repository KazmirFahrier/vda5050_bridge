# Testing

## Summary

| Package | Automated tests | Where they run |
|---|---|---|
| `vda5050_core` | 96 unit tests, 1 end-to-end example | GitHub Actions: GCC and Clang with sanitizers (real GoogleTest), and colcon on ROS 2 Jazzy |
| `vda5050_nav2_adapter` | 1 integration test, 1 system test | GitHub Actions, ROS 2 Jazzy container. Integration: real Mosquitto broker, fake Nav2. System: real broker, real Nav2, simulated robot |

## How to run

```bash
# Debug build with AddressSanitizer + UndefinedBehaviorSanitizer
cmake -S vda5050_core -B build -DCMAKE_BUILD_TYPE=Debug -DVDA5050_CORE_SANITIZE=ON
cmake --build build
ctest --test-dir build --output-on-failure

./build/vda5050_core_tests     # every test name
./build/simulated_vehicle      # the end-to-end example
```

CMake prints `GoogleTest not found - using test/support/mini_gtest` when it
falls back to the bundled shim. CI installs GoogleTest, so the shim is only
used on machines without it.

With ROS 2 (what the `ros2` CI job does):

```bash
colcon build && colcon test && colcon test-result --verbose
bash src/vda5050_bridge/vda5050_nav2_adapter/test/integration_test.sh
```

## What exists

96 unit tests in seven suites:

| Suite | Tests | What it pins down |
|---|---|---|
| `Json` | 11 | Scalars, nesting, string escapes incl. surrogate pairs, malformed inputs, numbers that are not JSON, nesting limit, error offset, deterministic output, round trip |
| `Codec` | 10 | Full order decode, both spellings of `allowedDeviation`, error messages naming the bad field, instant actions (current and legacy key), state encoding incl. every schema-required field, empty state, connection, visualization, factsheet required fields |
| `Header` | 3 | Topic names, ISO 8601 formatting incl. pre-1970, per-topic header counters |
| `OrderValidator` | 8 | Each graph rule, plus "all problems reported, not just the first" |
| `OrderManagerTest` | 47 | The state machine (below) |
| `OrderManagerReentrancy` | 1 | Ports that report results synchronously from inside the call |
| `SessionTest` | 16 | MQTT behaviour (below) |

`OrderManagerTest` by area:

- **Traversal**: idle state, drive to next node, visit all nodes then idle,
  single-node order, stale navigation results
- **Acceptance**: wrong position, per-node vs default deviation, wrong map,
  logical position without coordinates, busy vehicle, malformed order,
  unsupported action, non-automatic operating mode, state reset on new order,
  bounded error list
- **Updates**: stop at end of base, release horizon, update while driving,
  replace horizon with another route, duplicate, stale, non-stitching, update
  after completion
- **Actions**: HARD / SOFT / NONE semantics, ordering, actions on later nodes,
  edge actions, horizon actions, failure, late reports, order active while a
  NONE action runs
- **Instant actions**: cancel while driving / standing / idle, new order after
  cancel, redelivery, pause and resume, state request, factsheet request,
  custom and unsupported instant actions
- **Faults**: navigation failure halts until cancelled, malformed message
  report, telemetry mirroring

`SessionTest`:

- Subscriptions; last will (topic, QoS 1, retained, `CONNECTIONBROKEN`)
- Connect publishes `ONLINE` then a full state; shutdown publishes `OFFLINE` once
- Header ids increase per topic
- Order and instant-action payloads are routed; state follows on the next tick
- State is published on change, otherwise only when the interval elapses
- Undecodable payloads become `validationError` and never throw
- Messages for another vehicle or another major protocol version are refused;
  a different minor version is accepted
- Nothing is published while disconnected; progress made offline is reported
  after reconnect
- Visualization rate, disabling it, and not publishing without a position
- Factsheet published once per request

The **end-to-end example** (`simulated_vehicle`) is also registered with CTest.
It sends real JSON through `Session`, simulates the robot, and exits non-zero
unless the vehicle ends idle at the last node with its action `FINISHED`.

## How the tests are built

- **Fakes, not mocks.** `FakeNavigation`, `FakeActions` and `FakeTransport`
  only record calls. Each test then plays the robot or the broker. Assertions
  are on observable outcomes: port calls and published messages.
- **One rule per test**, named as a sentence
  (`RejectsUpdateThatDoesNotStartAtEndOfBase`).
- **No real time.** `Session::tick()` takes the current time as an argument, so
  "30 seconds later" is an addition, not a sleep. The suite runs in milliseconds.

## Evidence the tests and tools can catch bugs

Four deliberate bugs were injected into the code one at a time; each was caught:

| Injected bug | Result |
|---|---|
| HARD action no longer waits for running actions | `HardActionWaitsForRunningActionsAndKeepsOrder` failed |
| Duplicate order not ignored | `DuplicateOrderIsIgnoredSilently` failed |
| Navigation failure does not halt | `NavigationFailureRaisesFatalErrorAndHaltsUntilCancelled` failed |
| JSON nesting limit removed | Test binary crashed (stack overflow), i.e. the run failed |

This is a spot check, not systematic mutation testing.

AddressSanitizer also found one real defect during development, in a test
rather than in the library: a reference was kept into a temporary JSON document
(`last_state().find("errors")->as_array()`), a use-after-free that passed
silently without the sanitizer. It was fixed by keeping the document alive.

## The adapter integration test

`vda5050_nav2_adapter/test/integration_test.sh` runs the real executable.

- **Real:** the adapter node, a Mosquitto broker, tf2, the ROS 2 action protocol.
- **Fake:** Nav2 (`fake_nav2_server.py` succeeds every goal after 0.5 s) and
  the robot pose (a static transform at the origin).

It checks: `ONLINE` on connect; a state message; the tf2 pose in
`visualization`; `stateRequest`; an order N0 -> N1 driven through the action
server to completion with no errors; a `pick` action started, and a report
sent on the ROS status topic returned to master control (the request topic's
content is not inspected); a malformed order reported as
`validationError`; the factsheet on request; and on SIGINT, `OFFLINE`
published without the last will firing.

## The Nav2 system test

`vda5050_nav2_adapter/test/nav2_simulation_test.sh` replaces the fake action
server with the real Nav2 stack, launched from `nav2_bringup` with the loopback
simulator on the TurtleBot3 world map.

- **Real:** the adapter, the broker, and Nav2's map server, planner,
  controller, behaviour-tree navigator and costmaps.
- **Simulated:** the robot. The loopback simulator turns Nav2's velocity
  commands directly into motion. There are no sensors, no localisation error,
  no obstacles and no physics.

Four scenarios along a free strip of the map:

1. **A -> B -> C.** Both legs are driven; the test checks the intermediate
   state at B, completion at C with no errors, and that the reported position
   is within 0.4 m of C.
2. **C -> A with pause and resume.** While driving, `startPause`: state shows
   paused and not driving, and the leg does not complete during three seconds.
   After `stopPause` the order completes at A with no errors.
3. **A -> X, where X is off the map.** Nav2 fails the goal. The test checks a
   `FATAL` `navigationError`, that the vehicle is not driving, still lists X
   ahead and does not retry; that `cancelOrder` clears the order; and that a
   fresh order from the robot's reported position then completes with an empty
   error list. In CI the failure arrived within about 0.2 s, so this is Nav2
   refusing an impossible goal, not giving up after trying.
4. **A -> C, cancelled.** While driving, `cancelOrder`: the cancel action
   reaches `FINISHED`, the vehicle is stopped, the route is cleared, no error is
   raised and C is not reached.

One thing the first run showed: the test passed but the CI step failed, because
the script's cleanup killed every process with "nav2" in its command line,
which included the script itself. A green log with a red job is worth reading
to the end.

Observed stopping distance from the target node was 0.22 to 0.24 m, which is
Nav2's default goal tolerance.

## What the first real build found

The adapter was written without ROS installed. Its first CI runs found three
problems, none of which the earlier stand-in-header syntax check could have:

| Problem | How it was found | Fix |
|---|---|---|
| No rosdep key `libmosquitto-dev` | `rosdep install` failed | Install with apt; documented |
| Range-for over a member of a destroyed `Parameter` temporary | GCC `-Wdangling-reference` | Copy the array first |
| Ctrl-C left a retained `ONLINE` on the broker | Reasoning while writing the integration test, then confirmed by it | `main()` calls `AdapterNode::stop()`; `disconnect()` waits for in-flight publishes |

## Configurations verified in CI

| Job | Result |
|---|---|
| Standalone CMake, GCC, Debug, ASan + UBSan, `-Werror`, real GoogleTest | 96/96 pass, example passes |
| Standalone CMake, Clang, same flags | 96/96 pass, example passes |
| colcon on `ros:jazzy`, both packages | builds with no warnings; 96/96 pass |
| Integration test on `ros:jazzy` | passes |
| Nav2 system test on `ros:jazzy` | passes |

The core was additionally built locally in Release with
`-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror`.

## What has NOT been verified

Read this before claiming more than is true.

- **A path blocked part-way.** The failure test uses a goal Nav2 refuses
  immediately. Failure after the vehicle has started moving, with Nav2's
  recovery behaviours running first, is not exercised.
- **A physics simulator or physical robot.** The Nav2 test uses the loopback
  simulator: no sensors, localisation, obstacles or dynamics.
- **A real fleet manager.** Master control is a shell script.
- **The launch file** has not been executed; the test starts the binary directly.
- **Reconnect behaviour** after a broker outage (implemented in the core and
  tested there with a fake transport; not tested with a real broker).
- **ROS 2 linters** (`ament_cpplint`, `ament_uncrustify`) are not wired in.
- **Coverage measurement.** No line or branch coverage number exists.
- **Schema conformance.** Field names were compared with the published v2.0
  JSON schemas by reading them, not by automated validation. The
  `instantActions` schema could not be retrieved, so the array key (`actions`)
  is from memory; the decoder also accepts the older `instantActions` key.
- **Fuzzing** of the JSON parser and decoder.

## Planned test layers

| Layer | Tooling |
|---|---|
| Schema conformance: validate encoded messages against the official JSON schemas | Python `jsonschema` in CI |
| A path that becomes blocked while driving | needs a simulator with obstacles |
| Lifecycle transitions driven externally, via the launch file | `launch_testing` |
| Bring-up with a physics simulator (sensors, localisation) | Nav2 TurtleBot bring-up with Gazebo |
| Fuzzing the decoder | libFuzzer |
