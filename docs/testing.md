# Testing

## Summary

| What | Result | Where it runs |
|---|---|---|
| `vda5050_core` unit tests | 123 tests, 98.8 % line coverage | GitHub Actions: GCC and Clang with sanitizers (real GoogleTest), and colcon on ROS 2 Jazzy |
| `fault_core` unit tests | 54 tests, 98 % line coverage | Same as above |
| `fleet_core` unit tests | 91 tests, 97 % line coverage, among them a simulated fleet with independent monitors and seeded soak tests | Same as above |
| Fleet system test | Passes | GitHub Actions: the fleet manager program, a real Mosquitto broker, four vehicle processes |
| `fault_gateway` unit tests | 37 tests of the HTTP layer and the JSON conversion | `colcon test` (pytest) |
| Fault management system test | Passes | ROS 2 Jazzy container: fault manager, adapter, gateway, real Mosquitto broker, rosbag2, fake Nav2 |
| Conformance to the official JSON schemas | All published messages and all example inputs conform to versions 2.0.0 and 2.1.0, with one documented deviation | GitHub Actions; schemas cloned from the official repository at fixed tags |
| ROS 2 linters | cpplint, uncrustify, xmllint, lint_cmake, flake8, pep257 pass for all eight packages (the Python package runs flake8 and pep257) | `colcon test` |
| Fuzzing of the JSON parser and decoders | 60 s per CI run (about 235,000 inputs), no findings | GitHub Actions, libFuzzer with ASan and UBSan |
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

123 unit tests in eight suites:

| Suite | Tests | What it pins down |
|---|---|---|
| `Json` | 15 | Scalars, nesting, string escapes incl. surrogate pairs, malformed inputs, numbers that are not JSON, nesting limit, error offset, deterministic output, round trip |
| `Codec` | 14 | Full order decode, both spellings of `allowedDeviation`, error messages naming the bad field, instant actions (current and legacy key), state encoding incl. every schema-required field, empty state, connection, visualization, factsheet required fields |
| `CodecMaster` | 8 | The master control side: an order and instant actions encoded here are decoded by the vehicle side unchanged; a state and a connection message from the vehicle side are decoded here unchanged; wrong types are named |
| `Header` | 3 | Topic names, ISO 8601 formatting incl. pre-1970, per-topic header counters |
| `OrderValidator` | 9 | Each graph rule, plus "all problems reported, not just the first" |
| `OrderManagerTest` | 57 | The state machine (below) |
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
| `codec.cpp` | 610 | 97 % |
| `header.cpp` | 29 | 96 % |
| `json.cpp` | 274 | 99 % |
| `order_manager.cpp` | 419 | 99 % |
| `order_validator.cpp` | 59 | 100 % |
| `session.cpp` | 102 | 100 % |
| **Total** | 1493 | **98.8 %** |

Of the uncovered lines, all but one cannot be reached (closing braces after
code that always throws or returns, and fall-through returns after
exhaustive `switch` statements); that one is a defensive guard against the
same action id arriving twice. The job fails
below 95 %.

Branch coverage reads 62 %. That figure is not meaningful for C++ compiled
this way: most of the untaken branches are the compiler's exception paths out
of standard-library calls, which no test can take without the allocator failing.

Coverage was first measured at 94.5 %. The missing lines pointed at real
untested behaviour, and tests were added for it: a cancel that completes
because the leg finished first, a second cancel during a cancel, an
unsupported action on an edge, every JSON escape, and wrong types in messages.

`fault_core` and `fleet_core` are measured the same way, with the same 95 %
threshold: 98.1 % (457 of 466 lines) and 97.4 % (2090 of 2146).

| File of `fleet_core` | Lines | Covered |
|---|---|---|
| `api.cpp` | 188 | 99 % |
| `conflicts.cpp` | 56 | 100 % |
| `fleet_manager.cpp` | 936 | 97 % |
| `planner.cpp` | 814 | 96 % |
| `reservations.cpp` | 26 | 100 % |
| `roadmap.cpp` | 126 | 100 % |

Its uncovered lines are the handling of a plan that fails the check (which
no test can provoke), a few defensive branches in the planner's stages, and
fall-through returns.

The adapter, the ROS nodes and the two programs of `vda5050_fleet` have no
unit tests and no coverage figure.

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

## Fuzzing

`vda5050_core/fuzz/fuzz_decoder.cpp` is a libFuzzer target that feeds arbitrary
bytes to everything that parses network input: `Json::parse`, `decode_order`
and `decode_instant_actions`. Orders that decode and validate are handed to the
state machine, and the resulting state is encoded. It checks that nothing
crashes, leaks, reads out of bounds or triggers undefined behaviour, that only
the documented exceptions escape, and that JSON survives a write-and-parse
round trip.

CI runs it for 60 seconds per build under AddressSanitizer and UBSan, starting
from the example messages: about 235,000 inputs, no findings. Sixty seconds is
a smoke test, not a fuzzing campaign.

## Linters

The C++ packages run `ament_lint_auto` in `colcon test`: cpplint, uncrustify,
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

The same was done for the fault management code, with eleven bugs:

| Injected bug | Result |
|---|---|
| Gateway: acknowledge no longer checks the access token | `test_token_protects_acknowledge_but_not_reading` failed |
| Gateway: replayed events are sent a second time | `test_stream_replays_missed_events` failed |
| Gateway: no size limit on a POST body | `test_oversized_post_body_is_refused` failed |
| Gateway: a client that does not read is never dropped | `test_slow_reader_is_dropped_not_buffered_forever` failed |
| History file is never rotated | `RotatesWhenTheFileGrowsPastItsLimit` failed |
| Rotated history file is not read back | `RotatesWhenTheFileGrowsPastItsLimit` failed |
| `stopPause` releases a vehicle that is held by a fault | `HoldAndMasterPauseAreIndependent` failed |
| A held vehicle starts its next leg | `HeldVehicleDoesNotStartTheNextLeg` and `HoldBeforeAnOrderDelaysItsStart` failed |
| Releasing the hold does not continue the order | The same two tests failed |
| External errors are not put into the state | `ExternalErrorsAreReportedAlongsideOrderErrors` failed |
| Adapter never holds the vehicle (in CI, on a separate branch) | The fault system test failed at "vehicle is held: paused, not driving" |

The last one matters because the fault system test passed on its first CI
run. A test that has never failed proves little, so it was made to fail once.

And for the fleet manager, the two rules that everything rests on were each
switched off once:

| Injected bug | Result |
|---|---|
| The order rule always says yes (moves are granted in any order) | Five tests failed: the head-on meeting at the siding, the corridor case, the test with message loss, and both soak tests. Vehicles locked each other in and tasks never finished |
| The claims rule is skipped (a move is granted whatever others hold) | Nine tests failed, each with the monitors reporting conflicting claims or vehicles too close |

These are spot checks, not systematic mutation testing.

AddressSanitizer also found one real defect during development, in a test
rather than in the library: a reference was kept into a temporary JSON document
(`last_state().find("errors")->as_array()`), a use-after-free that passed
silently without the sanitizer. It was fixed by keeping the document alive.

## The fault management tests

Described in [fault_management.md](fault_management.md#what-is-tested): the
unit tests of `fault_core` and of the gateway's HTTP layer, and a system test
(`fault_manager/test/fault_system_test.py`) in which the fault manager, the
adapter and the gateway run together against a real broker. That test covers
twelve scenarios, among them a critical fault that pauses an order until an
operator acknowledges it over HTTP, the bag written by the flight recorder,
a restart of the fault manager, and a killed adapter detected by watchdogs.

What the work on these tests found:

- **False watchdog fault after a restart** (fixed). In one CI run the
  restarted fault manager needed almost three seconds to be discovered by a
  monitor that was reporting all along. A fault with a 2 s watchdog was raised
  and cleared again. Watchdogs now start only after a start-up grace period,
  and the system test fails if a restart produces an event by itself.
- **An empty bag** (fixed). That same false fault triggered a capture before
  the recorder had found its topics, and a bag with no messages was written.
  The recorder now writes nothing when its buffer is empty.

No other defect was found in the fault management code. Two defects were in tests (a watchdog in the shared test catalogue
fired during an unrelated flicker test; an HTTP test closed a connection but
not its response, so the socket stayed open). The first CI build found one
deprecated rosbag2 call and one line over the length limit.

## Fleet traffic control

What the fleet manager is: [fleet_management.md](fleet_management.md).

**Unit tests of the parts** (`test_roadmap.cpp`, `test_planner.cpp`): the
roadmap and its error messages, the distance between segments, which
resources conflict; the single-vehicle planner (waiting, stops in order,
dwell, resting out of the way); the fleet planner on cases built to need
each of its stages; the plan check itself, which must reject each kind of
wrong plan; 400 random problems.

**The simulated fleet** (`test/sim.hpp`). The class `World` holds one
`FleetManager` and any number of `SimVehicle`s. A `SimVehicle` is a real
`vda5050_core::Session` whose navigation port moves a point. Messages between
them pass through queues with a delay, random extra delay and a loss rate.
Time is a number that `World::step()` advances by 50 ms, so a simulated
minute takes about a millisecond.

After every step three monitors run. They use only the vehicles' true
positions, the roadmap's geometry and the list of what each vehicle holds:

| Monitor | Fails when |
|---|---|
| Distance | Two vehicles are closer than two radii |
| Claims | Two vehicles hold resources that conflict |
| Presence | A vehicle is further than the attach tolerance from everything it holds |

The second and third together imply the first, with margin. They are checked
separately because they fail earlier and say why.

**Tests of what goes wrong** (`test_fleet_robustness.cpp`, 33 tests). One
per failure: a lost first state, a vehicle that never says where it is, a
vehicle appearing on a granted route, vehicles switched on too close, a
vehicle on a foreign order, off the roadmap, on an edge, in manual mode, a
vehicle that forgets its order, lost orders, an offline vehicle, failed
drives, failed and unsupported actions, cancelled tasks, a fault hold. Each
asserts the specific reaction and that the monitors stayed silent.

**Soak tests.** `FleetSoak` generates scenarios from a seed: one of four map
kinds, one to six vehicles with speeds between 0.5 and 1.5 m/s, two to nine
tasks with one or two stops arriving over 30 s, network delay of 20 to
120 ms, and 10 % message loss in every third scenario. The second test adds
a fault every ten seconds on average during the first minute (a dropped
connection, a failing drive, a change of speed, a hold, a fatal error, a
cancelled task) and, afterwards, an operator who puts failed vehicles back
into service.

Pass means the monitors were silent, no plan failed the check, and at the
end no task was open. Without faults, every task must have finished.

The suite runs 40 and 25 scenarios (a second in a Release build, under a
minute with sanitizers). The same binary runs more:

```bash
cmake -S fleet_core -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
FLEET_SOAK_SCENARIOS=20000 ./build/fleet_core_tests
```

Result on the code as merged, one run:

| | Scenarios | Tasks | Finished | Monitor violations | Tasks left open |
|---|---|---|---|---|---|
| Without faults | 20,000 | 109,755 | 109,755 | 0 | 0 |
| With faults | 20,000 | 110,223 | 102,132 | 0 | 0 |

With faults, the tasks not finished were cancelled by the injected operator
or failed because their vehicle was taken out of service or an order was
cut short; none was left hanging.

What this does and does not show: over these scenarios the two properties
held. The scenarios are small, the generator has its habits, and the
vehicle is a point. It is evidence, not proof. The claims rule has an
argument behind it that does not depend on tests (see the design document);
the liveness of the whole system has not.

**What the soak tests found** during development, each fixed and pinned by a
named test:

| Found | Now tested by |
|---|---|
| A task cancelled while its first order was in flight left the fleet manager believing the vehicle idle; the vehicle then drove with nothing reserved | `AnOrderOnItsWayToAVehicleThatIsToBeStoppedIsSeenThrough` |
| A drive failed while an order update was lost; the update was never sent again because the vehicle counted as unusable, so the failure was never handled | `AFailureIsNoticedEvenWhileAnOrderIsUnconfirmed` |
| After a task failed at a stop, the vehicle drove on, and the stops it reached were credited to its next task | `AFailedActionFailsTheTask` |
| The same with an order confirmed late: a counter ran past the end of an array (found by AddressSanitizer) | the soak test itself |
| On a crowded one-way loop vehicles circled for ever: a vehicle passing its stop while it had no complete route was never told to work there | `ACrowdedOneWayLoopKeepsMovingUntilEveryStopIsServed` |
| A vehicle with three granted steps of a detour kept a corridor shut while the plan changed under it | `TwoRobotsWhoseStopsLieBehindEachOtherInACorridor` |
| The one-step stage pushed away a vehicle that stood at its own stop, and the result failed the plan check | `ARobotWorkingAtItsStopIsNotPushedAway` |
| A vehicle appearing inside another's granted route made every plan fail the check until it left | `AVehicleAppearingOnAnotherOnesRouteStopsThatOne` |

One finding was in the test harness, not the product: the simulated vehicle
kept a pause from a goal that had failed, and never moved again.

**System test** (`vda5050_fleet/test/fleet_system_test.py`). The parts the
unit tests cannot reach: the MQTT client, the two programs' loops, real
processes dying. The script starts a Mosquitto broker, `fleet_manager` and
four `sim_vehicle` processes on the demo roadmap, and then:

1. waits until all four vehicles are found at their nodes
2. sends a task to a node that does not exist and expects a refusal
3. submits eight tasks that cross each other and waits until all are finished
4. cancels a task while its vehicle drives; the vehicle must end at a node,
   usable
5. kills a vehicle process in the middle of a task; the fleet manager must
   report it offline and keep what it held; two other vehicles finish new
   tasks meanwhile
6. starts that vehicle again where it stood; it is found on its edge, sent
   on, and finishes its task
7. checks the counters (eleven tasks finished, no plan failed the check)
8. stops the fleet manager and expects its `OFFLINE`

Throughout, it records each vehicle's position from the `visualization`
topic (10 Hz) and fails if two were ever closer than two radii minus 0.25 m
of allowance for the positions not being simultaneous. In the runs so far
the closest approach was 2.0 m.

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
| Lifecycle | Starts the node with `autostart:=false` and drives it with `ros2 lifecycle set`, as a lifecycle manager would | `configure` alone does not connect; `activate` gives `ONLINE`; `deactivate` gives `OFFLINE`; a second activate/deactivate cycle works; after `cleanup`, `configure` and `activate` work again; SIGINT while active gives `OFFLINE` |

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
| An unmapped box in the path: both outcomes seen, both handled | In the first five CI runs with the box placed before the order, Nav2 gave up twice and got round the box three times. When it gave up (after about three minutes and 14 controller stalls) the vehicle reported a `FATAL` `navigationError` and halted, `cancelOrder` cleared it, and the next order completed. When it got round (in 13, 66 and 112 s, with 0, 4 and 8 stalls) the order completed normally. Closest approach to the box centre was 0.33 to 0.35 m, i.e. no contact. The first outcome is the failure-after-driving path, exercised against real Nav2 for the first time. Which outcome occurs is down to Nav2, not the adapter |
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
| Standalone CMake, GCC, Debug, ASan + UBSan, `-Werror`, real GoogleTest | 123/123 pass, example passes; `fault_core` 54/54 pass; `fleet_core` 91/91 pass |
| Standalone CMake, Clang, same flags | 123/123 pass, example passes; `fault_core` 54/54 pass; `fleet_core` 91/91 pass |
| colcon on `ros:jazzy`, all eight packages | builds with no warnings; 123/123, 54/54, 91/91 and the gateway's 37 pass; all linters pass |
| Fleet system test, Release build, real Mosquitto broker | passes |
| Schema conformance, 2.0.0 and 2.1.0 | passes |
| Coverage of `vda5050_core`, `fault_core`, `fleet_core` | 98.8 %, 98 %, 97 % lines |
| Integration test on `ros:jazzy` | passes |
| Nav2 system test on `ros:jazzy`, loopback simulator | passes |
| Nav2 system test on `ros:jazzy`, Gazebo with software rendering under Xvfb | passes |

The core was additionally built locally in Release with
`-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror`.

## What has NOT been verified

Read this before claiming more than is true.

- **A physical robot.** Everything that moves is simulated.
- **A real fleet manager from somebody else.** Against the adapter, master
  control is a shell script. The fleet manager in this repository has only
  met this repository's own vehicle side, so a shared misreading of the
  standard would not show. Interoperability rests on schema conformance, not
  on a session with a real product.
- **The fleet manager with Nav2.** Its vehicles in every test are the
  protocol core with a point for a body. It has not driven the adapter.
- **The fleet manager at scale or over days.** Up to six vehicles, up to 24
  nodes, simulated minutes. Planning time on a large roadmap was not
  measured.
- **Unit tests and coverage for the ROS nodes.** The adapter, the fault manager
  node and the gateway's ROS code are exercised only end to end.
- **The fault hold as a safety measure.** It is tested as software behaviour
  (the order pauses and resumes). It is not a safety function.
- **The web API under load or attack.** It is tested for correct behaviour,
  including bad input and vanished clients, not for throughput or security.
- **Repeatability of the Gazebo job.** A small number of CI runs. Physics on
  shared CI machines can be flaky; the obstacle scenario can take three
  minutes and its outcome depends on Nav2.
- **A long fuzzing campaign.** CI fuzzes for 60 seconds per run.
- **Systematic mutation testing.** Four hand-injected bugs were caught; that is
  a spot check.

## Possible further test layers

| Layer | Tooling |
|---|---|
| Unit tests for the adapter classes against a mocked action server | GoogleTest with `rclcpp_action` test servers |
| Hours-long fuzzing with a saved corpus | libFuzzer, scheduled job |
| A session with a real fleet manager | Open-RMF or a vendor test instance |
