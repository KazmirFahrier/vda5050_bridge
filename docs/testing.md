# Testing

## Summary

| Package | Automated tests | Run where |
|---|---|---|
| `vda5050_core` | 96 unit tests, 1 end-to-end example | Ubuntu 24.04, GCC 13.3 and Clang |
| `vda5050_nav2_adapter` | none | never compiled |

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
falls back to the bundled shim.

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

## Build configurations verified (`vda5050_core`)

| Configuration | Result |
|---|---|
| GCC 13.3, Debug, ASan + UBSan, `-Werror` | 96/96 pass, example passes, no sanitizer reports |
| GCC 13.3, Release, `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror` | builds clean, 96/96 pass |
| Clang, `-Wall -Wextra -Werror` | builds clean, 96/96 pass |

## What has NOT been verified

Read this before claiming more than is true.

- **The whole `vda5050_nav2_adapter` package.** It has never been compiled
  against ROS 2, Nav2 or libmosquitto, and has no tests. Its sources passed a
  compiler syntax check against small hand-written stand-in headers. That
  catches typos and inconsistencies inside the package; it cannot catch a
  wrong assumption about a real API, and it exercises no behaviour.
- **Real GoogleTest.** All runs used the shim.
- **colcon / ament build** of either package.
- **ROS 2 linters** (`ament_cpplint`, `ament_uncrustify`). The code follows the
  ROS 2 style by hand; the linters are not wired in yet.
- **The CI workflow** has never run.
- **Coverage measurement.** No line or branch coverage number exists.
- **Interoperability.** Nothing has been exchanged with a real broker or fleet
  manager. Field names were compared with the published v2.0 JSON schemas by
  reading them, not by automated schema validation. The `instantActions` schema
  could not be retrieved, so the array key (`actions`) is from memory; the
  decoder also accepts the older `instantActions` key.
- **Fuzzing** of the JSON parser and decoder.

## Planned test layers

| Layer | Tooling |
|---|---|
| Schema conformance: validate encoded messages against the official JSON schemas | Python `jsonschema` in CI |
| Node integration: lifecycle transitions, Nav2 client against a fake `NavigateToPose` server | `launch_testing` |
| System: adapter + Mosquitto broker + scripted master control | Docker Compose in CI |
| Fuzzing the decoder | libFuzzer |
