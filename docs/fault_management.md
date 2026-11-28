# Fault management

This document describes how the robot answers one question at any moment:
**what is wrong with me right now, and what has gone wrong before?**

Four packages do this together:

| Package | Role |
|---|---|
| `fault_core` | The logic, as a C++17 library without ROS: when a report becomes a fault, when it clears, what is recorded |
| `fault_msgs` | ROS 2 messages and services |
| `fault_manager` | ROS 2 node that runs `fault_core`, keeps the history file and records a rosbag2 snapshot when a fault activates |
| `fault_gateway` | Web API (REST and a live event stream) and a status page, for operators and other systems |

The VDA 5050 adapter is the fifth participant: it reports its own health and
passes the raised faults on to the fleet manager.

The first half of this document explains the concepts and the library. The
second half, from [The system on a robot](#the-system-on-a-robot), explains
the nodes.

## Why a robot needs this

Every part of a robot can find problems: a lidar driver sees no scans, a
battery monitor sees a low cell, Nav2 cannot find a path. Without a central
place for these findings, each part logs a line of text and the information
is lost. Three groups of people then have no answer:

- an **operator** on the floor: why did the robot stop, and may I restart it?
- a **fleet manager** (master control): can this vehicle still take orders?
- a **service engineer** next week: how often does this happen, since when?

A fault manager collects the findings, decides which ones are real, keeps
them visible for as long as they matter, and records every change.
The car industry calls this a diagnostic event manager (AUTOSAR DEM); PLC
systems call it alarm management. The ideas in this library come from there.

## Terms

| Term | Meaning |
|---|---|
| **Fault** | A named condition that should not be present, e.g. `LIDAR-001` "no scans" |
| **Fault code** | The stable name of a fault. It never changes, so statistics stay comparable |
| **Monitor** | Any code that checks a condition and reports the result. Lives outside this library |
| **Report** | One message from a monitor: "fault X is failing" or "fault X is healthy" |
| **Catalogue** | The list of all known faults with their settings. A JSON file, reviewed like code |
| **Severity** | `INFO`, `WARNING`, `ERROR` or `CRITICAL` |
| **Debounce** | Waiting a set time before believing a report, so that a short glitch is not a fault |
| **Latched** | A fault that stays visible after its cause has gone, until a person acknowledges it |
| **Watchdog** | A check that the monitor itself is still alive |
| **Occurrence** | One period from "became active" to "cleared" |
| **Event** | A recorded change: `ACTIVATED`, `CLEARED` or `ACKNOWLEDGED` |

## The life of a fault

Each fault code is in exactly one of five states:

```
                 failing report
   INACTIVE ───────────────────► PENDING
      ▲  ▲                          │
      │  │  healthy report          │ failing for activateAfter
      │  └──────(no event)──────────┤
      │                             ▼
      │                          ACTIVE ◄──────────────┐
      │                             │                  │ failing report
      │               healthy report│                  │ (no event)
      │                             ▼                  │
      │  healthy for clearAfter  HEALING ──────────────┤
      ├──────(not latched)──────────┤                  │
      │                             │ (latched)        │
      │       acknowledge           ▼                  │
      └──────────────────────── AWAITING_ACK ──────────┘
```

- `PENDING`: a monitor says "failing", but not yet for long enough. Not shown
  to anyone. If the monitor says "healthy" again, nothing happened.
- `ACTIVE`: the fault is real. An `ACTIVATED` event is recorded.
- `HEALING`: the monitor says "healthy", but not yet for long enough. The
  fault is still shown. If it fails again, this is the same occurrence.
- `AWAITING_ACK` (latched faults only): the cause is gone, but a person must
  confirm they have seen the fault. Time alone never clears it.
- `INACTIVE`: nothing is wrong. A `CLEARED` or `ACKNOWLEDGED` event is recorded
  on the way here.

With `activateAfter` and `clearAfter` both zero, the two waiting states are
skipped and the fault follows the reports directly.

### Why two waiting times

A sensor value near its limit crosses the limit many times a second. Without
debounce, one loose connector produces thousands of events, the history fills
up, and the operator's screen flickers. This is called an alarm storm.
`activateAfter` filters short glitches; `clearAfter` joins a flickering
condition into a single occurrence. The test
`FlickeringConditionProducesOneEventNotAStorm` shows twenty flickers
producing one event.

### Why latching

A bumper was pressed and released within half a second. If the fault simply
cleared, nobody would know that the robot hit something. A latched fault
stays on the screen until someone presses "acknowledge". Acknowledging is
refused while the cause is still present: one cannot acknowledge a problem
away.

### Why a watchdog

A monitor that has crashed reports nothing, and "no report" looks exactly
like "no problem". For faults with a `timeoutMs`, silence longer than that
time is treated as a failing report with the source `watchdog`. The fault
clears when reports resume. This also covers a monitor that never starts.

## The catalogue

```json
{
  "faults": [
    {
      "code": "LIDAR-001",
      "severity": "CRITICAL",
      "description": "Lidar is not delivering scans",
      "activateAfterMs": 500,
      "clearAfterMs": 2000,
      "latched": false,
      "timeoutMs": 3000
    },
    { "code": "BUMPER-001", "severity": "CRITICAL", "latched": true }
  ]
}
```

Only `code` is required. Defaults: severity `ERROR`, all times 0, not latched,
no watchdog. Loading fails with a message that names the entry, for example
`LIDAR-001.clearAfterMs: expected a whole number of milliseconds, 0 or more`.
A robot should refuse to start with a broken catalogue; that is better than
running with a fault that can never activate.

A report for a code that is not in the catalogue is, by default, accepted
with severity `ERROR` and marked `known: false`. The reasoning: a misspelt
code is a bug, but hiding a real problem because of a spelling mistake is
worse. Strict behaviour (drop and count) is one setting away.

## Using the library

```cpp
#include "fault_core/fault_manager.hpp"
#include "fault_core/history_store.hpp"

using namespace fault_core;

FaultManager faults(FaultCatalog::from_json(catalogue_text));
JsonlHistoryStore store("/var/lib/robot/fault_history.jsonl");
faults.restore_history(store.load(1000).events);   // after a restart

// from monitors, whenever they have a result
faults.report("LIDAR-001", true, now, "lidar_driver", "no scan for 0.6 s");

// from a timer, e.g. every 100 ms
faults.tick(now);                          // debounce times and watchdogs
store.append(faults.take_new_events());    // persist what changed

faults.raised();        // what is wrong now, most severe first
faults.summary();       // counts per severity, highest severity
faults.history(query);  // filtered: by code, by severity, since a sequence number
faults.acknowledge("BUMPER-001");
```

`now` is passed in rather than read from a clock. That is why the tests can
check "exactly 500 ms later" without sleeping, and run in milliseconds.

## Design decisions

**Level-based reports.** A monitor says "failing" or "healthy" for a code. It
does not say "raise" or "clear". The decision whether that amounts to a fault
is made in one place, with one set of rules, and a monitor cannot get it wrong.
A repeated or lost report does no harm, because the next one carries the same
information.

**Events only on real changes.** Three event types exist. Going in and out of
`PENDING` or `HEALING` is not recorded. The history therefore shows what an
operator saw, not what a sensor did.

**Sequence numbers.** Each event has a number that only grows, also across
restarts. A client that knows "I have everything up to 512" asks for
`after_sequence = 512` and gets exactly what it missed. This is how a web
page or a fleet manager stays in step without polling everything.

**Live state is not restored after a restart.** The history and the
occurrence counters are; whether a fault is active is not. After a restart the
cause may be gone (often the restart was the repair). A fault that is still
there is reported again by its monitor within its normal period. Restoring
"active" from disk risks showing a fault that no longer exists and that no
monitor will ever clear.

**History file in JSON Lines.** One event per line, appended. A power cut can
at worst damage the last line; `load()` skips lines it cannot read and says how
many. The file can be read with `grep` and `jq`. A database would add a
dependency for no benefit at this size. The file is bounded: past a set size
it is renamed to `<file>.1`, replacing the previous one, and a new file is
started. At most about twice that size is kept.

**Bounded memory.** The in-memory history and the queue of uncollected events
have a fixed capacity; the oldest entries are dropped. A robot that runs for
months must not grow.

**Single-threaded, no ROS.** Like `vda5050_core`, the library has no locks,
no threads and no middleware. The caller serialises access (in ROS, one
callback group). This keeps the logic testable and reusable outside ROS.

## Connection to VDA 5050

A fleet manager learns about vehicle faults through the `errors` list of the
`state` message. Two pieces connect the libraries:

- `vda5050_core::OrderManager::set_external_errors()` accepts errors from
  outside the order logic. They appear after the order manager's own errors,
  are not cleared by a new order, and trigger an immediate state message only
  when the set changes.
- `fault_core::to_vda5050_errors()` converts raised faults:

| Fault | VDA 5050 error |
|---|---|
| code | `errorType` |
| severity `CRITICAL` | `errorLevel: FATAL` |
| severity `ERROR`, `WARNING` | `errorLevel: WARNING` |
| severity `INFO` | not sent (configurable) |
| message, or catalogue description if empty | `errorDescription` |
| state, source, occurrence count | `errorReferences` |

VDA 5050 has only two levels. `FATAL` means the vehicle cannot continue
without help, so only `CRITICAL` maps to it.

The test `AFaultReachesTheEncodedStateMessage` follows one fault from the
report to the JSON text that would be published.

## The system on a robot

```
 monitors (any node)                         operator's browser, scripts,
   │  fault_msgs/FaultReport                  other systems
   │  on  faults/report                              ▲
   │                                                 │ HTTP: REST, live events
   ▼                                                 │
 ┌───────────────────────────────┐   faults/active   ┌───────────────┐
 │ fault_manager                 │──────────────────►│ fault_gateway │
 │  fault_core::FaultManager     │   faults/events   │  (Python)     │
 │  history file (JSON Lines)    │──────────────────►│               │
 │  flight recorder ─► rosbag2   │◄──────────────────│               │
 └───────────────────────────────┘  services:        └───────────────┘
   ▲                │               acknowledge, get_history
   │ /diagnostics   │ faults/active
   │                ▼
 diagnostic       ┌───────────────────────┐   MQTT: state.errors, paused
 publishers       │ vda5050_nav2_adapter  │──────────────────────────────► fleet
                  │  reports its own      │                                manager
                  │  health on            │
                  │  faults/report        │
                  └───────────────────────┘
```

### fault_manager

| Kind | Name | Type | Purpose |
|---|---|---|---|
| Subscription | `faults/report` | `fault_msgs/FaultReport` | Results from monitors |
| Subscription | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Statuses mapped to fault codes by parameter |
| Publisher | `faults/active` | `fault_msgs/FaultArray` | What is raised now. Latched (transient local), sent on change and every few seconds |
| Publisher | `faults/events` | `fault_msgs/FaultEvent` | Every recorded change |
| Service | `faults/acknowledge` | `fault_msgs/AcknowledgeFault` | Acknowledge a latched fault |
| Service | `faults/get_history` | `fault_msgs/GetFaultHistory` | Query recorded events |

Parameters are listed with comments in `fault_manager/config/fault_manager.yaml`.
The node refuses to start if the catalogue cannot be read; the error names the
entry at fault.

**Writing a monitor** takes one publisher:

```cpp
auto reports = node->create_publisher<fault_msgs::msg::FaultReport>("faults/report", 100);
fault_msgs::msg::FaultReport report;
report.code = "LIDAR-001";
report.failing = seconds_since_last_scan > 0.5;
report.source = node->get_name();
report.message = "no scan for " + std::to_string(seconds_since_last_scan) + " s";
reports->publish(report);   // regularly, or at least on every change
```

**Existing diagnostics** need no code. Two parameter lists map a
`DiagnosticStatus` name to a fault code; a status at or above `fail_level`
(default `ERROR`) counts as failing, and its `message` and `hardware_id` are
carried over.

### Data capture: the flight recorder

A list of faults says *what* happened. To find out *why*, an engineer needs
the sensor data from the moments before. Recording everything all the time
fills the disk; recording after the fault is too late.

The fault manager therefore keeps the last seconds of the configured topics in
memory. When a fault of sufficient severity activates, it keeps recording for
a short time and then writes everything to a rosbag2 bag:

```
/var/lib/robot/fault_captures/
└── fault_000042_LIDAR-001_20261127T193300Z/
    ├── metadata.yaml       rosbag2
    ├── *.mcap              the messages: 10 s before, 2 s after (configurable)
    └── fault.json          the event, readable without ROS tools
```

`ros2 bag play` replays it; `ros2 bag info` lists it. Details:

- Messages are kept serialized and never decoded, so any message type can be
  captured and the node does not depend on those types.
- Memory is bounded by a size limit; the number of bags on disk is bounded
  too, and the oldest are deleted.
- A fault that activates while a capture is already running does not start a
  second one. Its code is added to `alsoActivated` in `fault.json`: faults
  that arrive together usually have one cause.
- Topics are subscribed best-effort and without history. A latched topic such
  as `/map` is therefore not captured unless it is published again.

### The VDA 5050 adapter

The adapter does three things with faults (parameters `faults.*`):

1. **Reports its own health** once a second: `BRIDGE-MQTT-001` (no broker
   connection), `BRIDGE-NAV-001` (Nav2 action server not available) and
   `BRIDGE-LOC-001` (no transform from map to robot). These have watchdogs in
   the example catalogue, so an adapter that has crashed raises all three.
2. **Mirrors the raised faults into the VDA 5050 state**, using the mapping in
   [Connection to VDA 5050](#connection-to-vda-5050).
3. **Holds the vehicle** while any fault that maps to `FATAL` is raised: a
   running drive is paused, no new leg or action starts, and `paused` is true
   in the state. The order is kept. When the fault is gone (for a latched
   fault: when it has been acknowledged), the vehicle continues. This hold is
   independent of `startPause` from the fleet manager; the vehicle moves only
   when neither applies.

If the fault manager stops publishing, the adapter keeps the last list but
adds a warning `faultManagerTimeout` to the state, so the fleet manager knows
the list may be out of date. Without a fault manager running at all, the
adapter behaves exactly as before.

The hold is a software reaction through Nav2. It is not a safety function and
does not replace a safety controller.

### fault_gateway: the web API

| Method and path | Answer |
|---|---|
| `GET /` | Status page: raised faults, recent events, acknowledge button; updates live |
| `GET /api/v1/health` | `{"status":"ok","faultManager":true}` |
| `GET /api/v1/faults` | Raised faults, most severe first, and a summary by severity |
| `GET /api/v1/faults/<code>` | One raised fault, or 404 |
| `POST /api/v1/faults/<code>/acknowledge` | 200, or 409 with the reason |
| `GET /api/v1/history?code=&min_severity=&after=&limit=` | Recorded events, oldest first |
| `GET /api/v1/events` | Server-sent events: `faults` (the full list, on change) and `fault_event` |

```bash
curl http://localhost:8080/api/v1/faults
curl 'http://localhost:8080/api/v1/history?min_severity=ERROR&limit=20'
curl -X POST -H 'Authorization: Bearer <token>' \
     http://localhost:8080/api/v1/faults/BUMPER-001/acknowledge
curl -N 'http://localhost:8080/api/v1/events?after=512'     # live, replaying from 513
```

- **No lost events.** Each `fault_event` carries its sequence number as the
  event id. A client that reconnects sends the last number it saw (browsers do
  this by themselves with `Last-Event-ID`) and receives what it missed first.
- **Honest when the fault manager is down.** `faults` and `history` answer 503
  and `health` reports `faultManager: false`; the gateway does not keep
  serving the last list it heard.
- **Slow clients cannot hurt.** Each stream has a bounded queue; a client that
  does not read is disconnected.
- **Security.** The server listens on `127.0.0.1` by default. Reading needs no
  credentials. Acknowledging needs a bearer token when `api_token` is set, and
  the node warns at start-up if it is reachable from the network without one.
  There is no TLS and there are no user accounts; on an untrusted network, put
  a reverse proxy with TLS in front.
- **No web framework.** The server is Python's standard `http.server`, so the
  package adds no dependencies. The HTTP layer knows nothing about ROS and is
  tested against a stand-in backend.

### Running it

```bash
ros2 launch fault_manager fault_manager.launch.py      # uses the example catalogue
ros2 launch fault_gateway fault_gateway.launch.py      # http://localhost:8080/
ros2 launch vda5050_nav2_adapter adapter.launch.py

ros2 topic pub --once /faults/report fault_msgs/msg/FaultReport \
  '{code: BUMPER-001, failing: true, source: demo, message: "contact front left"}'
ros2 topic echo /faults/active
ros2 service call /faults/acknowledge fault_msgs/srv/AcknowledgeFault '{code: BUMPER-001}'
```

## What is tested

| Level | What | Where |
|---|---|---|
| `fault_core` unit tests | 54 tests in five suites: each transition of the state machine, debounce at the exact millisecond, flicker, latching, watchdog, unknown codes, queries, limits, restart, file damage, rotation, mapping to VDA 5050 | GCC and Clang with sanitizers; colcon |
| `vda5050_core` unit tests | External errors and the fault hold in the order state machine (6 of its 115 tests) | Same |
| `fault_gateway` unit tests | 36 tests: 7 of the JSON conversion, 29 of the HTTP layer over real sockets (routes, validation, token, event stream, replay, slow and vanished clients) | colcon (`pytest`) |
| System test | All nodes together with a real broker; see below | colcon job in CI |

The system test (`fault_manager/test/fault_system_test.py`) starts the fault
manager, the adapter, the gateway, a Mosquitto broker and a fake Nav2. The
script plays the monitors, the operator and the fleet manager, and checks:

1. A broken catalogue stops the fault manager with a message naming the entry.
2. A reported fault appears in the web API, in the live event stream and in the
   VDA 5050 state on the broker, with the right level and text.
3. A bag is written with the messages from before and after the fault, and
   `ros2 bag info` can read it. A low-severity fault writes none and is not
   sent to the fleet manager.
4. A 0.3 s glitch does not become a fault; a persistent condition does, not
   before its debounce time, and shows `HEALING` before it clears.
5. A critical latched fault during an order: the state reports `FATAL`, the
   vehicle is paused and stays paused after the cause is gone. Acknowledging
   is refused without the token (401) and for a fault that is not waiting
   (409). After a valid acknowledgement the vehicle finishes the order.
6. A `/diagnostics` status raises and clears its mapped fault; a status below
   the configured level and an unmapped status do nothing.
7. A monitor that falls silent raises its fault through the watchdog.
8. An unknown fault code is listed and marked as unknown.
9. The history has no gaps and the filters work.
10. Old bags are deleted beyond the configured number.
11. The fault manager is stopped: the web API answers 503 and the adapter adds
    `faultManagerTimeout` to the state. After the restart the history is back,
    no fault is restored as active, and sequence numbers and occurrence counts
    continue.
12. The adapter is killed: its three faults are raised by their watchdogs, and
    clear when it is started again.

## Limits

- **Never run on a physical robot.** As for the rest of the repository, all of
  this was verified in CI with a simulated robot.
- **The hold is not a safety function.** It pauses navigation through
  software. A real vehicle needs a certified safety controller for anything
  that protects people.
- **Only three built-in monitors**, all about the bridge itself. Monitors for
  sensors, battery or motors are robot-specific and must be written, or fed in
  through `/diagnostics`.
- **Debounce and watchdog times use the steady system clock**, not ROS time.
  In a simulation that runs slower than real time they are measured in real
  seconds.
- **Captures are not linked from the API.** The bag directory name contains
  the event's sequence number and fault code; the web API does not list or
  serve bags.
- **The web API has one optional shared token**, no TLS, no user accounts and
  no audit trail of who acknowledged.
- **The gateway and the fault manager have no unit tests of their ROS code.**
  Their logic is unit-tested in `fault_core` and in the HTTP layer; the ROS
  wiring is covered by the system test only, and has no coverage figure.
- **One capture at a time**, and capture subscriptions are best-effort.
- **The fault manager is a plain node**, not a lifecycle node.
