# Fault management

This document describes `fault_core`, the second library in this repository.
It answers one question a robot must be able to answer at any moment:
**what is wrong with me right now, and what has gone wrong before?**

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
dependency for no benefit at this size. Known limit: the file grows without
bound; rotation is not implemented.

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

## What is tested

52 unit tests in five suites, run with GCC and Clang under AddressSanitizer
and UBSan, and under colcon on ROS 2 Jazzy with the ROS linters:

| Suite | Tests | Covers |
|---|---|---|
| `FaultManagerTest` | 30 | Each transition of the state machine, debounce at the exact millisecond, flicker, latching, watchdog, unknown codes, queries, limits, restart |
| `Catalog` | 5 | Parsing, defaults, every validation message |
| `FaultCodec` | 7 | Round trips, timestamps, rejection of damaged input one field at a time |
| `HistoryStore` | 6 | Append, load, damaged lines, unwritable path, restart with a real file |
| `Vda5050Mapping` | 4 | Severity mapping, references, end-to-end into the state message |

## What does not exist yet

This is a library. Nothing on a running robot calls it so far.

- No ROS node. The adapter does not yet create a `FaultManager`, and no
  topic or service carries reports. `set_external_errors()` is called only
  by tests.
- No monitors. Nothing turns `/diagnostics`, a missing topic or a Nav2
  failure into a report.
- No data capture on a fault (rosbag snapshot).
- No web interface.
- No rotation of the history file.
- No stop reaction. A `CRITICAL` fault is reported; it does not halt the robot.

These are the next milestones; see [roadmap.md](roadmap.md).
