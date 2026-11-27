# Testing

## Summary

| What | Result | Where it runs |
|---|---|---|
| `vda5050_core` unit tests | 109 tests, 98.8 % line coverage | GitHub Actions: GCC and Clang with sanitizers (real GoogleTest), and colcon on ROS 2 Jazzy |
| Conformance to the official JSON schemas | All published messages and all example inputs conform to versions 2.0.0 and 2.1.0, with one documented deviation | GitHub Actions; schemas cloned from the official repository at fixed tags |
| ROS 2 linters | cpplint, uncrustify, xmllint, lint_cmake, flake8, pep257 pass for both packages | `colcon test` |
| `vda5050_nav2_adapter` integration test | Passes | ROS 2 Jazzy container, real Mosquitto broker, fake Nav2 |
| `vda5050_nav2_adapter` system test | Passes in both modes | Real broker and real Nav2: once with the loopback simulator, once in Gazebo with lidar and AMCL |

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

109 unit tests in seven suites:

| Suite | Tests | What it pins down |
|---|---|---|
| `Json` | 15 | Scalars, nesting, string escapes incl. surrogate pairs, malformed inputs, numbers that are not JSON, nesting limit, error offset, deterministic output, round trip |
| `Codec` | 14 | Full order decode, both spellings of `allowedDeviation`, error messages naming the bad field, instant actions (current and legacy key), state encoding incl. every schema-required field, empty state, connection, visualization, factsheet required fields |
| `Header` | 3 | Topic names, ISO 8601 formatting incl. pre-1970, per-topic header counters |
| `OrderValidator` | 9 | Each graph rule, plus "all problems reported, not just the first" |
| `OrderManagerTest` | 51 | The state machine (below) |
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

## Coverage

Measured in CI with `gcovr` over the unit tests only (not the example program):

| File | Lines | Covered |
|---|---|---|
| `codec.cpp` | 388 | 97 % |
| `header.cpp` | 29 | 96 % |
| `json.cpp` | 274 | 99 % |
| `order_manager.cpp` | 392 | 99 % |
| `order_validator.cpp` | 59 | 100 % |
| `session.cpp` | 102 | 100 % |
| **Total** | 1244 | **98.8 %** |

All 125 functions are executed. Of the 15 uncovered lines, 14 cannot be
reached (closing braces after code that always throws or returns, and
fall-through returns after exhaustive `switch` statements) and one is a
defensive guard against the same action id arriving twice. The job fails
below 95 %.

Branch coverage reads 62 %. That figure is not meaningful for C++ compiled
this way: most of the untaken branches are the compiler's exception paths out
of standard-library calls, which no test can take without the allocator failing.

Coverage was first measured at 94.5 %. The missing lines pointed at real
untested behaviour, and tests were added for it: a cancel that completes
because the leg finished first, a second cancel during a cancel, an
unsupported action on an edge, every JSON escape, and wrong types in messages.

Coverage exists for the core only. The adapter has no unit tests.

## Conformance to the official JSON schemas

`vda5050_core/test/run_schema_conformance.sh` checks both directions against
the schemas published in the official VDA 5050 repository, at tags 2.0.0 and
2.1.0 (CI clones them; they are not copied into this repository):

- **Outbound.** `tools/emit_samples` writes a fully populated example of every
  message the library publishes: state (full and minimal), connection (all
  three values), visualization, factsheet. Each must validate.
- **Inbound.** Example orders and instant actions in `test/data/` must validate
  against the schema *and* be accepted by the library's decoder and order
  validator.

What the first run found:

| Finding | Resolution |
|---|---|
| Factsheet lacked `agvGeometry` and `loadSpecification`, which the schema requires | Now emitted (empty, as their members are optional) |
| The 2.0.0 schema names an instant action's type `actionName`; the text and the 2.1.0 schema say `actionType` | Decoder accepts both |
| The 2.0.0 schema spells `allowedDeviationXy`, the 2.1.0 schema `allowedDeviationXY` | Decoder already accepted both |
| The schemas' `actionStatus` enum has no `PAUSED`, though the specification text defines it | Kept as a documented deviation (ADR-019). The script asserts it is the only one |

The same exercise settled one thing that had been written from memory: the
instant-actions array is named `actions`, in both schema versions.

Checking claims against the specification text (also in the official
repository) found one more defect: the factsheet must be published retained,
and was not. Fixed, with a test. It also confirmed QoS 0 for all topics but
`connection` (QoS 1), the 30-second state interval, and the wording of the
`noOrderToCancel` rule.

## Linters

Both packages run `ament_lint_auto` in `colcon test`: cpplint, uncrustify,
xmllint, lint_cmake, flake8 and pep257. Their first run asked for formatting
changes in five files and reported six cpplint findings (three long lines, a
header guard, a C `long long`, and constructors that are implicit on purpose
and are now marked as such). The GoogleTest stand-in under `test/support/shim`
is excluded. The copyright linter is not enabled: files carry SPDX identifiers
instead of the full licence header it expects.

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

`vda5050_nav2_adapter/test/integration_test.sh` runs the real node, started
through its launch file.

- **Real:** the adapter node, a Mosquitto broker, tf2, the ROS 2 action protocol.
- **Fake:** Nav2 (`fake_nav2_server.py` succeeds every goal after 4 s) and the
  robot pose (a static transform at the origin).

Message flow: `ONLINE` on connect; a state message; the tf2 pose in
`visualization`; `stateRequest`; an order N0 -> N1 driven through the action
server to completion with no errors; a `pick` action started, and a report
sent on the ROS status topic returned to master control (the request topic's
content is not inspected); a malformed order reported as `validationError`;
the factsheet on request.

Connection events:

| Event | What the test does | What it checks |
|---|---|---|
| Broker outage | Kills the broker while a leg is being driven, waits for the leg to finish offline, starts a new broker | The vehicle reconnects by itself, announces `ONLINE`, and its state reports the leg finished at N2 with no errors. The broker's log shows one connect and no `CONNECTIONBROKEN` afterwards |
| Clean stop | SIGINT to the node | `OFFLINE` is published; the last will is not |
| Crash | Starts the node again, then SIGKILL | The broker publishes the last will, `CONNECTIONBROKEN` |

One thing this turned up about the broker, not the adapter: when the Mosquitto
broker itself is terminated, it publishes connected clients' wills to
subscribers that are still attached. An early version of the test recorded
that and reported it as a failure of the clean stop that came later.

## The Nav2 system test

`vda5050_nav2_adapter/test/nav2_simulation_test.sh` replaces the fake action
server with the real Nav2 stack from `nav2_bringup`, on the TurtleBot3 world
map. Real in both modes: the adapter, the broker, and Nav2's map server,
planner, controller, behaviour-tree navigator and costmaps.

| Mode | What simulates the robot | What that adds |
|---|---|---|
| loopback (default) | Nav2's loopback simulator turns velocity commands directly into motion | Nothing: no sensors, localisation error, obstacles or physics. Fast and deterministic |
| `SIMULATOR=gazebo` | A TurtleBot3 in the Gazebo physics simulator | Wheel odometry, a simulated lidar, AMCL localising from the scans, and walls and pillars the lidar really sees |

Six scenarios (the fifth in Gazebo only):

1. **A -> B -> C.** Both legs are driven; the test checks the intermediate
   state at B, completion at C with no errors, and the reported position.
2. **C -> A with pause and resume.** While driving, `startPause`: state shows
   paused and not driving, and the leg does not complete during three seconds.
   After `stopPause` the order completes at A with no errors.
3. **A -> X, where X is off the map.** Nav2 fails the goal. The test checks a
   `FATAL` `navigationError`, that the vehicle is not driving, still lists X
   ahead and does not retry; that `cancelOrder` clears the order; and that a
   fresh order from the robot's reported position then completes with an empty
   error list. The failure arrives within about 0.2 s, so this is Nav2
   refusing an impossible goal, not giving up after trying.
4. **A -> C, cancelled.** While driving, `cancelOrder`: the cancel action
   reaches `FINISHED`, the vehicle is stopped, the route is cleared, no error is
   raised and C is not reached.
5. **A -> C with a box in the way (Gazebo only).** A 0.3 m box that is not on
   the map is placed on the straight line from A to C, so only the lidar can
   reveal it. Two outcomes are acceptable and the test checks the adapter's
   state for whichever happens: the order completes with the robot at C, or
   Nav2 gives up and the vehicle halts with a `FATAL` `navigationError`, after
   which `cancelOrder` must clear the order. The script prints the outcome, the
   time, the number of controller stalls and the closest approach to the box.
6. **To D, a point in the middle of four pillars.** The robot first returns
   to A, then drives about 2.5 m, passing between two pillars. Completes with
   no errors; the script prints the time taken and how often Nav2's controller
   stalled on the way.

### Proving the Gazebo run is real

A simulation test can pass for the wrong reason, so in Gazebo mode the script
asserts, before sending any order:

- the `gz sim` process exists and the `/amcl` node exists
- one message on `/scan` contains real ranges (in CI: 360 of 360 beams finite,
  nearest obstacle about 0.5 m)
- simulation time on `/clock` advances

At the end it prints AMCL's own pose estimate and a summary of errors in the
Nav2 log.

### What these runs found

| Finding | Detail |
|---|---|
| The readiness check was wrong | It looked for the word `active` in the lifecycle state, which also matches `inactive`. The test moved on before Nav2 was up and still passed, because later steps happened to wait long enough. Found only because a Gazebo start-up of four seconds was not believable. Now anchored |
| Cleanup killed the test itself | `pkill -f nav2` matched the script's own name: every check passed and the CI step went red |
| Stopping distance | The robot stops 0.17 to 0.24 m from the target node. That is Nav2's default goal tolerance (0.25 m). A deployment must set it tighter than the fleet manager's `allowedDeviationXY` |
| A blocked strip makes Nav2 give up, and the adapter reports it | With the box placed before the order, Nav2 tried for about three minutes (14 controller stalls), then aborted. The vehicle reported a `FATAL` `navigationError` and halted; `cancelOrder` cleared it; the following order completed. This is the failure-after-driving path, exercised against real Nav2 for the first time. Seen in two runs out of two |
| A box dropped close ahead of a moving robot was brushed | In an earlier variant of the scenario the box was spawned 0.5 m in front of the robot while it was driving. Nav2 got round it after 164 s and 11 stalls, but the closest reported distance to the box centre was 0.26 m, which for this robot means contact or very nearly. Stopping for a suddenly appearing obstacle is the job of Nav2's collision monitor and of a real robot's safety scanner, not of this adapter; with default parameters it was not enough. Seen once |
| Slow first leg (fixed) | The first 0.5 m leg took about 45 s, with Nav2's controller logging "Failed to make progress" four times. Cause, found by logging the goal the adapter sent and tracing the robot's pose: for a first leg without `theta` the adapter asked for heading 0. The robot, facing east, was told to reach a node 0.5 m to its north and end up facing east again; the controller stalled until a spin recovery turned it. The heading is now the direction of travel from the robot's current position. The leg takes 4 s in both simulators, and the test fails if it is slow or if Nav2 logs any progress failure |
| Nav2 stalling in a tight gap (observed once, not an adapter fault) | In one Gazebo run the robot approached the gap between two pillars at an angle, from wherever a cancelled order had left it. Nav2's controller failed its progress check 8 times and ran spin, wait and back-up recoveries before getting through; the leg took 100 s where it normally takes 8. The adapter kept the order in progress and reported completion with no errors, which is the correct behaviour. The scenario now starts from a fixed node, so the approach is the same every run. The gap leaves under 0.2 m clearance each side with Nav2's default parameters |

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
| Standalone CMake, GCC, Debug, ASan + UBSan, `-Werror`, real GoogleTest | 109/109 pass, example passes |
| Standalone CMake, Clang, same flags | 109/109 pass, example passes |
| colcon on `ros:jazzy`, both packages | builds with no warnings; 109/109 pass; all linters pass |
| Schema conformance, 2.0.0 and 2.1.0 | passes |
| Coverage of `vda5050_core` | 98.8 % lines |
| Integration test on `ros:jazzy` | passes |
| Nav2 system test on `ros:jazzy`, loopback simulator | passes |
| Nav2 system test on `ros:jazzy`, Gazebo with software rendering under Xvfb | passes |

The core was additionally built locally in Release with
`-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror`.

## What has NOT been verified

Read this before claiming more than is true.

- **A physical robot.** Everything that moves is simulated.
- **A real fleet manager.** Master control is a shell script. Interoperability
  rests on schema conformance, not on a session with a real product.
- **Unit tests and coverage for the adapter.** It is exercised only end to end.
- **Lifecycle transitions driven from outside** (a lifecycle manager calling
  configure, activate, deactivate, cleanup). The node activates itself in every test.
- **Repeatability of the Gazebo job.** A small number of CI runs. Physics on
  shared CI machines can be flaky; the obstacle scenario takes about three
  minutes and its outcome depends on Nav2.
- **Fuzzing** of the JSON parser and decoder.
- **Systematic mutation testing.** Four hand-injected bugs were caught; that is
  a spot check.

## Possible further test layers

| Layer | Tooling |
|---|---|
| Unit tests for the adapter classes against a mocked action server | GoogleTest with `rclcpp_action` test servers |
| Lifecycle transitions driven externally | `launch_testing` |
| Fuzzing the decoder | libFuzzer |
| A session with a real fleet manager | Open-RMF or a vendor test instance |
