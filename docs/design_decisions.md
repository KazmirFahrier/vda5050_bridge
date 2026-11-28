# Design decisions

Each record states the decision, why it was made, what it costs, and what was
rejected. They are the answers to "why did you do it this way?".

---

## ADR-001: Protocol logic lives in a ROS-independent library

**Decision.** `vda5050_core` uses only the C++17 standard library. ROS 2, Nav2
and MQTT are confined to a separate adapter package that depends on the core.

**Why.**
- The hard part of VDA 5050 is the rules (when to accept an update, when an
  action may start). Those rules are pure logic and should be testable in
  milliseconds, without launching nodes.
- ROS 2 distributions change APIs between releases. Keeping the logic out of
  reach of that churn limits what must be re-verified on an upgrade.
- The same core can serve a non-ROS vehicle controller.

**Cost.** One extra layer of types: ROS messages are converted to core structs
at the boundary instead of being used directly.

**Rejected.** Writing the logic inside the node's callbacks. Faster to start,
but tests would need a running ROS graph and timing-dependent assertions.

---

## ADR-002: Ports as abstract interfaces, injected by reference

**Decision.** `OrderManager` takes `NavigationPort&` and `ActionPort&` in its
constructor and calls them through virtual functions.

**Why.** Dependency inversion: the core declares what it needs, the adapter
supplies it. Tests pass in fakes that record calls.

**Cost.** A virtual call per command. Irrelevant here: commands are issued a
few times per minute, not in a control loop.

**Rejected.** Templates (static polymorphism): no runtime cost, but it would
push `OrderManager` into headers and make error messages worse, for no
measurable gain. `std::function` callbacks: workable, but an interface groups
related operations and documents the contract in one place.

---

## ADR-003: A small built-in JSON module instead of a third-party library

**Decision.** `json.hpp`/`json.cpp` implement a strict RFC 8259 parser, a value
type and a writer (about 550 lines), with their own tests.

**Why.**
- The core has zero external dependencies, so it builds anywhere a compiler exists.
- The input comes from the network. Owning the parser means the limits are
  explicit: maximum nesting depth, no trailing data, no non-standard numbers.
- Honest context: the first build environment had no JSON library and no way to
  install one. A dependency-free core was already a goal (ADR-001), and this
  made it the practical choice too.

**Cost.** Code to maintain that a library would provide. No JSON Schema
validation, so structural checks are hand-written in `codec.cpp`.

**Rejected / revisit.** `nlohmann/json` is the usual choice in ROS 2 projects
and is available through rosdep. All JSON use is confined to `json.*` and
`codec.cpp`, so replacing the module is a contained change. If schema
validation against the official VDA 5050 schemas becomes a requirement, switch
then.

---

## ADR-004: Data-driven `advance()` rather than an explicit state enum

**Decision.** Event handlers mutate a few fields, then call `advance()`, which
decides the next step from the data. There is no `enum class State`.

**Why.**
- An explicit state machine needs a transition for every (state, event) pair.
  Missing one is the classic bug: the vehicle sits still because nothing
  handled "action finished while waiting for base".
- Several conditions are independent (paused, cancelling, base exhausted,
  action blocking). As a single enum they multiply; as flags they stay flat.
- `advance()` is idempotent: calling it when nothing changed does nothing. So
  "when in doubt, call advance()" is always safe.

**Cost.** The states are implicit; the diagram in `architecture.md` is a
description, not something the compiler enforces.

**Rejected.** A state-machine library or behaviour tree for the order logic:
heavier than the problem. A behaviour tree is the right tool one level down,
inside Nav2.

---

## ADR-005: Single-threaded, non-blocking core

**Decision.** `OrderManager` has no mutex, no thread and no blocking call. The
owner serialises access.

**Why.** Concurrency bugs are the hardest to reproduce on a robot. ROS 2
already offers serialisation through mutually-exclusive callback groups, so
locking inside the core would be redundant and would invite deadlocks when a
port calls back.

**Cost.** The contract must be respected by the adapter. It is stated in the
class documentation.

---

## ADR-006: Reject new orders while busy; never pre-empt

**Decision.** An order with a different `orderId` is rejected with `orderError`
unless the vehicle is idle. Master control must `cancelOrder` first.

**Why.** It is the behaviour the specification's order-handling flow describes,
and it is the safe default: a vehicle carrying a load does not silently
abandon its task because of one message.

---

## ADR-007: Navigation failure halts; it is never retried automatically

**Decision.** If the navigation port reports failure, the vehicle stops, raises
a `FATAL` `navigationError`, and waits for `cancelOrder`.

**Why.** Master control owns the route and the traffic reservations. A local
retry might drive into a stretch that has since been given to another vehicle.
Local recovery (replanning around an obstacle on the same edge) belongs inside
the navigation adapter, where Nav2's own recovery behaviours live.

---

## ADR-008: Failed actions do not abort the order

**Decision.** A `FAILED` action raises a `WARNING` and the vehicle continues.

**Why.** The specification leaves the reaction to master control. Aborting
locally would remove that choice.

**Cost / revisit.** For some actions (a failed `pick` followed by a `drop`)
continuing is pointless. A per-action-type policy is on the roadmap.

---

## ADR-009: Tolerant reader, strict writer

**Decision.** Unknown JSON fields are ignored, and three known variants are
accepted on input: `allowedDeviationXY` / `allowedDeviationXy`, an instant
action's `actionType` / `actionName` (the 2.0.0 schema uses the latter), and
the instant-actions array under `actions` / `instantActions` (the v1.1 name).
Output always uses the specification spelling, emits required arrays and
objects even when empty, and omits unset optional fields instead of writing
`null`. Output is validated in CI against the official 2.0.0 and 2.1.0 schemas.

**Why.** Interoperability with fleet managers written by other people is the
whole purpose. Minor-version additions must not break decoding, and our output
must be as conservative as possible.

---

## ADR-010: Idempotent message handling

**Decision.** A repeated order (same `orderId` and `orderUpdateId`) and a
repeated instant action (same `actionId`) are ignored without error.

**Why.** MQTT delivery above QoS 0 can duplicate messages, especially around
reconnects. Processing must be safe to repeat.

---

## ADR-011: Bounded error list

**Decision.** `State.errors` holds at most `max_errors` entries (default 32);
the oldest is dropped first.

**Why.** A faulty master sending bad orders in a loop would otherwise grow
every state message without limit, on a link that is often Wi-Fi.

---

## ADR-012: Tests written for GoogleTest, with a fallback shim

**Decision.** Tests include `<gtest/gtest.h>`. CMake uses real GoogleTest when
available (always under ament) and otherwise a shim of about 230 lines that implements
the macros the tests use.

**Why.** The tests could be executed on a machine with no GoogleTest, while
remaining ordinary GoogleTest tests for ROS 2 CI.

**Cost.** The shim prints less on failure. CI runs the suite under real
GoogleTest, which is what guards against a construct the shim accepts and
GoogleTest does not.

---

## ADR-013: MQTT behaviour lives in the core, behind a transport port

**Decision.** `Session` (in `vda5050_core`) decides topics, QoS, retain flags,
the last will, and when each message is sent. The MQTT client is a one-method
port, `MqttTransport::publish()`.

**Why.** "Publish ONLINE retained on connect", "send a full state after a
reconnect" and "at most 30 s between states" are protocol rules. Putting them
in the core makes them unit-testable with a fake transport and leaves the
libmosquitto wrapper with nothing to get wrong but plumbing.

**Cost.** The core now has an opinion about time, handled by passing `now`
into `tick()` instead of reading a clock.

---

## ADR-014: One publishing point, driven by a tick

**Decision.** Handlers set a "changed" flag. Only `Session::tick(now)` publishes.

**Why.** Coalesces bursts of changes into one consistent state message, and
makes periodic behaviour deterministic in tests.

**Cost.** Up to one tick period (50 ms at 20 Hz) of latency before a change is
reported. Acceptable against a protocol whose own timing unit is seconds.

**Rejected.** Publishing from inside each handler: lower latency, but several
messages per event burst, and intermediate states visible to master control.

---

## ADR-015: MQTT callbacks only enqueue

**Decision.** libmosquitto callbacks (which run on its network thread) push an
event into a mutex-protected queue. The node's timer drains it.

**Why.** Keeps the single-threaded contract of the core (ADR-005) with one
small, obviously correct critical section, instead of locking the state machine.

**Cost.** Incoming orders wait for the next tick.

---

## ADR-016: Clean MQTT session, no offline queueing

**Decision.** The client connects with a clean session, and messages published
while offline are dropped. On reconnect the session sends `ONLINE` and a full state.

**Why.** An order that was queued by the broker during an outage would be
executed late, when the situation in the warehouse may have changed. State is a
snapshot, so a backlog of old snapshots has no value; the newest one replaces them.

---

## ADR-017: Vehicle actions are delegated over ROS topics

**Decision.** `TopicActionHandler` publishes each action as JSON on
`~/action_request` and takes results from `~/action_status`.

**Why.** How a robot picks, drops or charges is specific to that robot. Topics
let any node implement an action without changing or recompiling the bridge.

**Cost.** JSON in `std_msgs/String` is untyped. A dedicated message package or a
ROS 2 action per VDA 5050 action type would give type checking, feedback and
cancellation semantics for free.

**Revisit.** Replace with a `vda5050_msgs` package and a generic ROS 2 action
once the set of actions for a real robot is known.

---

## ADR-018: Refuse messages whose header names another vehicle

**Decision.** `Session` compares `manufacturer`/`serialNumber` in the message
body with its own identity, and the major protocol version with its own. A
mismatch is reported as a `validationError` and the message is not acted on.

**Why.** The topic already identifies the vehicle, but a misconfigured bridge or
master can deliver a message to the wrong topic. Acting on it would move the
wrong vehicle. A different minor version is accepted because minor versions are
backwards compatible.

---

## ADR-019: Follow the specification text where it disagrees with the schema

**Decision.** An action that reports `PAUSED` is published as `PAUSED`.

**Context.** The specification text lists `PAUSED` among the action statuses.
The official JSON schemas for 2.0.0 and 2.1.0 leave it out of the
`actionStatus` enum. Schema validation in CI found this.

**Why.** The text is the normative description of behaviour, and a master that
tracks actions needs to tell a paused action from a running one. Mapping it to
`RUNNING` would make the state untrue.

**Cost.** A master that validates states strictly against the schema would
reject one that contains a paused action. The adapter only emits `PAUSED` when
the robot's own action executor reports it.

**Guard.** `run_schema_conformance.sh` asserts that this is the only message
rejected by the schemas, and that it is rejected for this reason alone. If the
schemas are corrected the script says so.

---

## ADR-020: Publish the factsheet retained

**Decision.** The factsheet is published with the MQTT retain flag, QoS 0.

**Why.** The specification's factsheet section says all messages on that topic
shall be sent retained, so a master that connects later still receives it. An
earlier version of this code published it non-retained, on an assumption; it
was corrected when the claim was checked against the specification text.

