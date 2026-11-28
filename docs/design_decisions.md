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

## ADR-021: Faults from outside the order logic enter through one setter

**Context.** `state.errors` so far held only errors the order manager itself
produced (a rejected order, a failed navigation). Hardware and software faults
must appear there too.

**Decision.** `OrderManager::set_external_errors()` replaces a separate list
that `snapshot()` appends to its own errors. The caller passes the complete
current set each time.

**Why.** Passing the whole set makes the call idempotent: a lost or repeated
call cannot leave a stale error behind, which add/remove calls could. The two
lists have different lifetimes (order errors are cleared by the next order; a
broken lidar is not), so they are stored separately. The order manager stays
unaware of what a fault manager is.

**Cost.** The caller must compute the full set on every change. A state
message is requested only when the set differs from the previous one.

## ADR-022: `fault_core` depends on `vda5050_core`

**Context.** The fault library needs JSON reading and writing, and a function
that maps faults to VDA 5050 errors.

**Decision.** `fault_core` links `vda5050_core` and uses its `Json` class.

**Why.** A second JSON implementation, or an external one, would add code or
a dependency for no gain. The dependency points one way: the bridge core knows
nothing about faults.

**Cost.** `fault_core` cannot be used without `vda5050_core`. If it is ever
needed alone, the JSON module should move to a small package of its own.

## ADR-023: A critical fault holds the vehicle; the hold is separate from pause

**Context.** VDA 5050 says a `FATAL` error means the vehicle cannot continue
without help. Reporting `FATAL` while driving on would contradict that.

**Decision.** `OrderManager::set_hold(bool)`. While held, a running drive is
paused and nothing new starts; the order is kept. The adapter sets the hold
while any raised fault maps to `FATAL`. The hold is a second flag next to the
pause flag set by `startPause`; the vehicle moves only when both are false, and
`state.paused` is true for either.

**Why.** One shared flag would let `stopPause` from the fleet manager release
a vehicle with a broken lidar, or let a cleared fault resume a vehicle the
fleet manager had paused. Keeping the order (instead of cancelling) lets work
continue after a short fault, which is what an operator expects after
acknowledging.

**Cost.** `state.paused` does not say which of the two reasons applies; the
`errors` list does. This is a software reaction through Nav2 and must not be
presented as a safety function. Parameter `faults.hold_on_fatal` turns it off.

## ADR-024: The flight recorder is a ring buffer inside the fault manager

**Context.** Data from before a fault is needed. rosbag2 offers a snapshot
mode (`ros2 bag record --snapshot-mode`) that keeps a buffer and writes it when
a service is called.

**Decision.** The fault manager holds its own ring buffer of serialized
messages (generic subscriptions) and writes a new bag per fault with
`rosbag2_cpp::Writer`.

**Why.** One bag per fault, named after the fault and stored with a summary,
needs control over when a bag starts and ends; the snapshot mode appends to one
recording of a separately started process. An in-process buffer also needs no
second process to supervise and lets the capture continue for a defined time
after the fault.

**Cost.** More code than calling a service, and message data passes through
this node, so very high-rate topics (cameras, point clouds) cost memory and
CPU here. The buffer is bounded by size for that reason. For heavy sensors the
rosbag2 snapshot mode, triggered from this node, would be the better tool.

## ADR-025: The web API uses Python's standard HTTP server

**Context.** Operators and other systems need the fault data over HTTP.
FastAPI or Flask are the usual choices.

**Decision.** `http.server.ThreadingHTTPServer` from the standard library, in a
module that does not import ROS, with the ROS node as one implementation of a
three-method backend interface.

**Why.** Seven routes do not need a framework, and every dependency on a robot
is something to package, update and audit. The separation makes the HTTP layer
testable with real sockets and a fake backend, without ROS.

**Cost.** No automatic request validation, no OpenAPI document, one thread per
connection (fine for a handful of clients, not for hundreds), and
authentication is a single shared token. A framework becomes the right choice
when any of these starts to matter.

## ADR-026: Debounce and watchdog times use the steady clock

**Context.** ROS nodes usually use ROS time, which follows `/clock` in
simulation.

**Decision.** `fault_manager` passes `std::chrono::steady_clock` time to
`fault_core`.

**Why.** A watchdog must fire when a monitor is silent, also when the silence
is caused by the simulator or the clock source having stopped. With ROS time, a
stopped clock would stop the watchdogs too. The steady clock is also immune to
wall-clock jumps (NTP).

**Cost.** In a simulation running slower than real time, a 500 ms debounce is
500 ms of real time, not of simulated time.


## ADR-027: Fleet control is a library without ROS and without MQTT

**Context.** The fleet manager needs MQTT to talk to vehicles and nothing
from ROS. Its hard part is logic: who may drive where, and when.

**Decision.** `fleet_core` has neither. It gets messages as text through
`on_message()`, sends through the `MqttTransport` interface that
`vda5050_core` already defines, and is driven by `tick(now)` with the time
passed in. The program `fleet_manager` in `vda5050_fleet` adds the broker
connection and a loop.

**Why.** The same reasons as for `vda5050_core` (ADR-001), and one more: with
time and transport in the caller's hands, a test can run a fleet of real
vehicle state machines against the real fleet manager, lose and delay
messages on purpose, and simulate ten minutes in a few milliseconds. The
soak tests run tens of thousands of such scenarios. Through a broker and
real clocks that would take weeks.

**Cost.** One more package and an interface to keep.

## ADR-028: Safety comes from claims checked at grant time, not from the plan

**Context.** The planner is the largest and most intricate part of the fleet
manager. It will have bugs.

**Decision.** A move is handed to a vehicle only if what it needs conflicts
with nothing another vehicle holds. That check is a few lines, uses no plan
and no time, and is applied to every move whatever produced it. In addition
every plan is checked by a separate validator before use.

**Why.** Two properties with very different costs of failure should not rest
on the same code. A planner bug now costs waiting, or a plan thrown away. It
cannot send two vehicles into the same place.

**Cost.** The plan and the claims can disagree. Then a vehicle waits although
the plan said go. That is the intended direction of the error.

## ADR-029: Resources conflict by geometry

**Context.** The usual first version reserves nodes and edges by identity:
two vehicles may not have the same node or edge.

**Decision.** A table says for every pair of resources whether vehicles on
them could touch, computed from the distance between the two shapes and the
vehicle size.

**Why.** Identity misses the cases that actually happen on a floor: the two
directions of one aisle are two edges; an edge passes within a vehicle's
width of a node it does not end at; two edges cross without a node. With
the table, all of these are the same case as "same node".

**Cost.** Quadratic in the number of resources, computed at start. The
roadmap's geometry must be right: the table believes it.

## ADR-030: What is granted is never taken back; three edges are granted ahead

**Context.** A vehicle that is told its route one edge at a time stops at
every node. A vehicle that is told all of it cannot be rerouted.

**Decision.** Up to three edges are granted ahead and sent as the base of
the order; a further three are shown as horizon. A grant is final. Changing
a vehicle's mind means cancelling its order and waiting until it stands.
Moves that only make room for others are granted one at a time.

**Why.** Revoking a grant is a race against a moving vehicle: by the time the
message arrives it may have entered the edge. "Final" removes the race. The
price is that everything granted is closed to others, so the window is kept
short, and shortest for moves the next plan may not want any more. That
last rule came from a test run in which a vehicle with three granted steps
of a detour held a corridor shut indefinitely.

**Cost.** Vehicles slow down or stop when the next grant is late. Three is a
default, not a measured optimum.

## ADR-031: The planner is fast and incomplete, with a searched second stage

**Context.** Finding optimal routes for several vehicles is NP-hard.
Complete algorithms exist (conflict-based search is one) but their running
time explodes in exactly the crowded cases where they are needed.

**Decision.** First plan one vehicle after another (prioritised planning
with safe-interval path planning), trying several orderings. If a vehicle is
left without a route, run a conflict-based search with a fixed budget. If
that fails too, move vehicles one step and plan again.

**Why.** The fleet manager replans often and must answer in milliseconds;
most replans are easy. The search is there for the cases an ordering cannot
solve, of which a vehicle that must overshoot and come back is the smallest.
The budget turns "may run for minutes" into "may not find it", and for that
case the third stage keeps things moving.

**Cost.** No optimality and no completeness. A hard case can take many small
steps where one good plan existed. The orderings and repair rules of the
first stage are heuristics that grew from failing cases; they are the
least principled part of the code.

## ADR-032: A vehicle out of step is stopped and started afresh

**Context.** The fleet manager's picture of a vehicle can become wrong: an
order was rejected, a drive failed, the vehicle restarted, a task was
cancelled. One could try to repair the order in place for each case.

**Decision.** One procedure for all of them. Send `cancelOrder`, wait until
the vehicle reports that it stands with nothing to do, find out where it is,
and start a new order from there. A vehicle standing on an edge keeps the
edge and both its ends and is sent to the end. An order that was still on
its way when the decision fell is first seen through to the vehicle and
cancelled then.

**Why.** Many causes, one path, and that path is tested heavily. During the
procedure the vehicle keeps everything it was granted, so the claims rule
covers it wherever it comes to rest.

**Cost.** A stop that a cleverer update could sometimes avoid.

## ADR-033: After a waiting time the fleet works without a vehicle it cannot place

**Context.** A vehicle is known to be online, but not where it is: its state
message was lost, or it is not localised. It could be on anybody's route.

**Decision.** While such a vehicle exists, nothing is granted to anybody, and
the vehicle is asked for its state. After ten seconds (`discovery_grace`) the
fleet carries on without it and reports `vehicle_unlocated`.

**Why.** Holding the whole fleet for ever because one vehicle does not report
is not acceptable in operation; never waiting would send vehicles past one
that is about to say where it is. A lost first message is resolved within a
second by the request.

**Cost.** After the waiting time a vehicle that is physically on the roadmap
but unknown to the fleet manager is not protected by it. This is the
largest hole in the design and is stated as a limit.

## ADR-034: The fleet is tested against the real vehicle side, with monitors and seeded faults

**Context.** The properties that matter (never too close, never stuck) are
about many vehicles over time. Example-based tests only cover the cases
somebody thought of.

**Decision.** A test harness runs the fleet manager and one real
`vda5050_core::Session` per vehicle over a message bus that delays, reorders
across topics and loses messages. Three monitors that share no code with the
fleet manager check distance, claims and presence after every step. Soak
tests generate scenarios from a seed, with and without injected faults; a
failing seed reproduces exactly.

**Why.** Every serious defect of the fleet manager was found this way, none
by a hand-written example. The monitors define the properties once, instead
of each test asserting its own idea of them.

**Cost.** The vehicle side is this project's own; both sides could share a
misreading of the standard. The simulated vehicle is a point: nothing about
real motion is tested.

## ADR-035: The command interface is text in, text out, inside the library

**Context.** Something must submit tasks: a warehouse system, an operator, a
script. Over what?

**Decision.** `fleet_core/api.hpp` offers three functions: carry out a JSON
command and return a JSON response, encode the status, encode an event. The
program publishes and subscribes them on MQTT topics next to the VDA 5050
ones.

**Why.** MQTT is already there, so no second server and no second port. And
the interface can be tested without any transport, including every
malformed command.

**Cost.** MQTT has no request and response; the client matches by
`requestId`. Access control is left to the broker.
