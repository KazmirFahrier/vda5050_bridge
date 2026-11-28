# Interview guide

How to present this project, and the questions it is likely to draw. Answers
point at the code so you can open the file and show it.

A rule for using this document: only claim what you can explain with the code
open. Read `order_manager.cpp` and `test_order_manager.cpp` end to end, change
something, break a test, fix it. The "be honest about" section is as important
as the rest; interviewers probe exactly there.

## 30-second version

> I built the vehicle side of VDA 5050, the standard that lets one fleet manager
> control mobile robots from different vendors. The core is a C++17 library with
> no ROS dependency: it decodes orders, validates them, runs a state machine
> that walks the route node by node, schedules actions by their blocking type,
> handles cancel and pause, and decides what goes out over MQTT and when. It has
> 123 unit tests that run in milliseconds, with 98.8 % line coverage, and its
> messages validate against the official VDA 5050 JSON schemas. A ROS 2 lifecycle node attaches it to
> Nav2 and an MQTT broker.

The whole thing builds in CI on ROS 2 Jazzy. One test runs the node against a
real MQTT broker; another drives orders through the real Nav2 stack, including
in Gazebo with a simulated lidar and AMCL: multi-leg orders, pause, resume,
cancel, a navigation failure with recovery, an unmapped obstacle, and a broker
outage. The parsers are fuzzed and the code passes the ROS 2 linters.

## 2-minute version: add these three points

1. **Why the split.** The difficult part is the protocol rules, not the
   plumbing. Keeping them free of ROS and MQTT means every rule is a fast,
   deterministic unit test (ADR-001).
2. **One design idea.** No explicit state enum. Handlers update data and call
   one idempotent `advance()` that works out the next step. That removes the
   "missing transition" class of bugs (ADR-004).
3. **One production concern.** Input comes from the network, so the JSON parser
   is strict and depth-limited, duplicate deliveries are harmless, and the
   error list is bounded (ADR-003, 010, 011).

## Questions about the protocol

**What is the difference between base and horizon?**
Base is the released part of the route, which the vehicle may drive. Horizon is
an unreleased preview. Master control extends the base with order updates as it
reserves more of the route; that is its traffic-control mechanism. The vehicle
stops at the end of the base. See `advance_once()` step 2.

**How do you tell a new order from an update?**
By `orderId`. Different id: new order, accepted only if the vehicle is idle and
standing on the first node. Same id: compare `orderUpdateId`. Lower is stale and
rejected, equal is a duplicate and ignored, higher is an update, which must
start at the last node of the current base. See `handle_order()`.

**Why must an update start at the end of the current base?**
So the pieces join without a gap or overlap. The check uses both `nodeId` and
`sequenceId` because a node can occur twice in a route.

**Explain blocking types.**
NONE runs alongside driving and other actions. SOFT runs alongside other actions
but the vehicle must stand still. HARD runs alone and standing still. See
`start_ready_actions()`: a HARD action waits until nothing is running; anything
else waits while a HARD one runs; `break` rather than `continue` preserves the
listed order.

**What happens on cancelOrder?**
Waiting actions fail, running ones are stopped, and navigation is cancelled. The
cancel action stays RUNNING until the navigation port confirms the vehicle has
stopped, then becomes FINISHED and the remaining nodes are cleared. `orderId` and
`lastNodeId` are kept. With nothing to cancel it fails with `noOrderToCancel`.

**How does master control know a vehicle dropped off the network?**
MQTT last will: the vehicle registers a `CONNECTIONBROKEN` message that the
broker publishes if the connection dies. `Session::last_will()` builds it; a
clean shutdown publishes `OFFLINE` first so the will is not triggered.

## Questions about design

**Why not just use the ROS 2 node for everything?**
Testability and lifetime. With logic in callbacks, tests need a ROS graph and
become timing-dependent. The core will also outlive several ROS distributions.

**Why virtual interfaces and not templates?**
Calls happen a few times a minute, so virtual dispatch costs nothing that
matters, and the implementation stays out of headers (ADR-002).

**Why write a JSON parser? Isn't that reinventing the wheel?**
Fair challenge; say so. Reasons: zero dependencies for the core, and explicit
limits on untrusted input. It is about 550 lines with its own tests, and all
JSON use is confined to two files, so swapping in `nlohmann/json` is a contained
change. I would switch if I needed JSON Schema validation (ADR-003).

**Is OrderManager thread-safe?**
No, deliberately. It never blocks and expects its owner to serialise calls. In
ROS 2 that is a mutually-exclusive callback group. Locks inside would add
deadlock risk when ports call back (ADR-005).

**What if a port calls back synchronously?**
`advance()` has a re-entrancy guard: a nested call sets a flag and the outer
call loops again. Also, `advance_once()` copies the target node before calling
`navigate_to()`, because a synchronous success would pop that deque entry while
the port still holds a reference to it.

**Why does a navigation failure not retry?**
Master control owns the route and reservations. A blind retry could enter a
stretch now assigned to another vehicle. Local recovery belongs inside Nav2
(ADR-007).

**How do you avoid unbounded growth or abuse from a bad master?**
Depth-limited parser, structural checks with field-path errors, rejection that
never disturbs the running order, bounded error list, idempotent handling.

## Questions about testing

**How is it tested?**
Fakes for the two ports record calls; each test drives a scenario and asserts on
port calls and the published state. No sleeps, no threads. See `testing.md`.

**How do you know the tests are any good?**
I injected four bugs and confirmed each made a test fail. That is a spot check;
systematic mutation testing and coverage measurement are not done yet.

**Tell me about a bug you found.**
Three good ones, all in `docs/testing.md`. AddressSanitizer caught a
use-after-free in a test that passed silently without it. The first real ROS
build flagged a range-for over a member of a destroyed temporary
(`-Wdangling-reference`). And writing the integration test exposed that Ctrl-C
left a retained `ONLINE` on the broker, because the process exited without
announcing `OFFLINE`. The lesson in each: the bug was found by a tool or a test
at a boundary, not by reading the code.

**Walk me through debugging something.**
In simulation the first leg of the first order took 45 seconds and every later
leg took about 4. Nav2 logged "Failed to make progress" four times. I did not
guess: I made the adapter log each goal it sent and had the test record the
robot's pose once a second. The trace showed the robot sitting still for 35
seconds, and the log showed the goal heading was 0 while the node was straight
to the robot's side. For a leg with no `theta` and no previous node, my code
fell back to heading 0, so Nav2 was asked to move sideways and finish facing
the way it started. It stalled until a spin recovery turned it. The fix was to
aim the heading along the direction of travel from the robot's actual position.
The leg now takes 4 seconds, and the test fails if it is slow or if Nav2 logs
any progress failure, so it cannot come back unnoticed.

**How do you know it really speaks VDA 5050?**
CI clones the official repository at tags 2.0.0 and 2.1.0 and validates every
message the library publishes against the JSON schemas, and checks that
schema-valid orders and instant actions are accepted by my decoder. The first
run found three mismatches I had not caught by reading: the factsheet lacked
two required objects, the 2.0.0 schema calls an instant action's type
`actionName` where the text says `actionType`, and the factsheet must be
published retained. It also showed the schema and the specification text
disagree about `PAUSED`; I follow the text and the test asserts that this is
the only deviation.

**What happens when the broker goes down?**
libmosquitto reconnects with backoff. Nothing is queued while offline. On
reconnect the session publishes `ONLINE` and a complete state, so progress made
offline is reported in one message. The integration test kills the broker while
a leg is being driven, lets the leg finish offline, restarts the broker and
checks exactly that. A separate step kills the adapter with SIGKILL and checks
the broker publishes the `CONNECTIONBROKEN` last will.

**Have you had a test that passed for the wrong reason?**
Yes. The Nav2 test waited for the navigator to be "active" by searching the
lifecycle state for that word, which also matches "inactive". It passed for
several runs. I caught it when the Gazebo job claimed Nav2 was up four seconds
after launch, which is not credible for a physics simulator. Since then the
Gazebo mode asserts that the simulator, the lidar and the localiser are really
running before it sends an order.

**Fakes vs mocks?**
A fake is a simple working stand-in whose state you inspect afterwards. A mock
has expectations set up front. Fakes keep tests about outcomes, not about the
exact sequence of calls, so refactoring does not break them.

**What would you add next for testing?**
Schema conformance against the official JSON schemas, `launch_testing` for the
lifecycle node, a system test with a real broker, and fuzzing the decoder.

## Questions about the ROS 2 adapter

**How does the single-threaded core coexist with the MQTT thread?**
libmosquitto's callbacks only push events into a mutex-protected queue. The
node's timer drains the queue on the executor thread, so protocol code never
runs on the MQTT thread (ADR-015).

**Why a lifecycle node?**
Start-up order matters: the bridge must not announce `ONLINE` before Nav2 can
accept goals. `configure` builds everything without I/O, `activate` connects.
A lifecycle manager can then sequence it after Nav2.

**Nav2 has no pause. How do you pause?**
Cancel the goal, keep the target, re-send on resume. A generation counter on
goals makes the resulting CANCELED result harmless: callbacks from a goal that
is no longer current are ignored.

**Why not queue messages while offline?**
State is a snapshot; an old one is worse than none. And an order queued by the
broker during an outage would execute late. Clean session, full state on
reconnect (ADR-016).

**Why is state published from a timer and not straight from the callbacks?**
One publishing point coalesces bursts into one consistent message and makes the
periodic rule testable with a fake clock (ADR-014).

## ROS 2 topics to be able to explain

- **Lifecycle (managed) nodes**: unconfigured -> inactive -> active; why a
  bridge should connect in `on_configure` and only publish when active.
- **Actions vs services vs topics**: why navigation is an action (long-running,
  feedback, cancellable).
- **QoS**: reliable vs best-effort, transient-local for latched data; how that
  maps to MQTT QoS 0/1 and retained messages.
- **Executors and callback groups**: mutually exclusive vs reentrant.
- **Nav2**: `NavigateToPose`, behaviour trees, costmaps, recovery behaviours.
- **tf2**: `map -> odom -> base_link`, and where `agvPosition` comes from.
- **colcon / ament**: `package.xml` dependency types, `ament_export_targets`.

## Questions about fault management

The repository has a second library, `fault_core`. Full text:
[fault_management.md](fault_management.md). Short answers:

**What does it do?**
Monitors report "failing" or "healthy" for a fault code. The library decides
when that is a real fault, keeps it visible as long as needed, and records
every change with a sequence number.

**What is debouncing and why both directions?**
A fault becomes active only after failing for a set time, and clears only
after being healthy for a set time. The first filters glitches. The second
joins a flickering condition into one occurrence, so a loose connector gives
one event and not thousands (an alarm storm).

**What is a latched fault?**
One that stays visible after its cause has gone, until a person acknowledges
it. Example: a bumper contact that lasted half a second. Acknowledging is
refused while the cause is present.

**How do you notice a dead monitor?**
A watchdog per fault: no report for longer than the timeout counts as
failing. Otherwise a crashed monitor looks the same as a healthy system.

**Why is the active state not restored after a restart?**
The cause may be gone, and nothing would then clear the restored fault. Real
faults are reported again by their monitors within one period. History and
occurrence counters are restored.

**Why a JSON Lines file and not a database?**
Appending a line is cheap and a power cut can damage at most the last line,
which the loader skips and counts. No dependency. The cost: no rotation yet,
and filtering happens in memory.

**How do faults reach the fleet manager?**
Raised faults are converted to VDA 5050 errors and given to the order
manager, which lists them in `state.errors`. `CRITICAL` maps to `FATAL`,
`ERROR` and `WARNING` to `WARNING`.

**What happens on the robot when a critical fault appears?**
The fault manager publishes it. The adapter lists it in the VDA 5050 state as
`FATAL` and holds the vehicle: the drive is paused, the order is kept. When
the fault is gone, or for a latched fault acknowledged, the vehicle continues.
Add at once: this is a software reaction through Nav2, not a safety function.

**How do you find out why a fault happened?**
A flight recorder. The fault manager keeps the last seconds of chosen topics
in memory, serialized, and writes them to a rosbag2 bag when a fault
activates, together with a second or two afterwards. Recording everything
always would fill the disk; recording after the fault is too late.

**Why keep messages serialized?**
A generic subscription delivers bytes. The node never decodes them, so it can
capture any message type without depending on it, and buffering is cheap.

**How does the web page stay up to date without polling?**
Server-sent events: one long HTTP response on which the server writes each
event. Each carries its sequence number as id. After a disconnect the browser
sends the last id it saw, and the server replays what was missed from the
history before continuing live.

**Why no web framework?**
The API is seven routes. Python's standard `http.server` handles that with no
dependency to install on the robot. The HTTP layer is separate from ROS, which
is why it has 30 tests that need no ROS. With more routes, authentication or
many clients I would move to a framework.

**How do ROS and the HTTP threads work together?**
rclpy spins in the main thread; each HTTP request has its own thread. They
share three things: the latest fault list behind a lock, a thread-safe event
broker, and service calls, where the HTTP thread sends the request and waits
with a timeout for the ROS thread to deliver the answer.

**What if the fault manager crashes?**
The gateway answers 503 and says so in `health`; it does not keep serving the
old list. The adapter adds a warning to the VDA 5050 state that its fault
list may be out of date. After the restart, history and counters come back
from the file; active faults are detected afresh.

**What if the adapter crashes?**
Its three health faults have watchdogs, so its silence raises them.

**What is the state of it?**
All of it runs in CI in one end-to-end test with a real broker: reports,
debounce, latching, the hold during an order, acknowledge over HTTP, the bag,
restarts. None of it has run on a physical robot.

## Questions about fleet traffic control

The repository also has the other side of the protocol: a fleet manager.
Full text: [fleet_management.md](fleet_management.md). This is the part with
the most algorithmic content, so expect the deepest questions here.

**30 seconds.**

> Then I built the other side: a fleet manager for several vehicles on one
> roadmap. It plans timed routes for all of them and hands them out a few
> edges at a time as VDA 5050 orders. Safety and progress are separate
> mechanisms. A vehicle is only given an edge if nothing it needs conflicts
> with what another vehicle holds; that rule alone prevents collisions,
> whatever the planner does. And edges are given in the order of a plan, which
> prevents deadlock. I test it by running the real vehicle state machines
> against it over a lossy simulated network, with monitors, over tens of
> thousands of random scenarios with injected faults.

**How do you guarantee two vehicles never collide?**
Every node and edge is a resource. A table computed from the geometry says
which pairs are too close for two vehicles. A vehicle holds the node it
stands on; to drive an edge it must first hold the edge and the node at its
end, and it keeps the node it leaves until it has arrived. A move is granted
only if what it needs conflicts with nothing anybody else holds
(`FleetManager::grant_and_send()`). No time and no plan are involved, so a
slow vehicle or a wrong plan cannot break it. The assumption is that each
vehicle is physically within what it holds.

**Why a conflict table and not just "same node"?**
Because the two directions of an aisle are two edges, an edge can pass close
to a node it does not touch, and edges can cross. All of these are conflicts
between different resources (ADR-029).

**What is a deadlock here and how do you prevent it?**
Vehicles waiting for each other in a cycle: two head-on in an aisle, or
four around a block. The claims rule alone allows that. So moves are granted
in the order of a plan: a move waits until every conflicting use that the
plan has earlier is done. In a valid plan those uses do not overlap in time,
so "earlier" is an order without cycles, and a cycle of waiting needs a
cycle. The plan's times only order things; nobody has to be on time
(`grantable()`).

**So can it never get stuck?**
It cannot deadlock on a valid plan. It can fail to find a plan: the planner
is not complete. Then it moves vehicles one step at a time and replans. And
a task whose stop is permanently occupied waits for ever. I say "no deadlock
under a valid plan", not "always finishes".

**How does the planner work?**
Three stages, each only if the one before left someone without a route.
First one vehicle after another by priority, each planning around the
others with safe-interval path planning, trying several orderings. Second a
conflict-based search with a budget, for cases where each of two vehicles
must give way to the other somewhere. Third, one step for everybody who is
still stuck. Then every plan goes through a separate check before use.

**What is safe-interval path planning?**
Planning in space and time, but grouping time: a node is free during a few
intervals between other vehicles' reservations, and the search state is
(node, interval) with the earliest arrival, not (node, time). Far fewer
states, same result.

**Why not just use conflict-based search for everything? It is complete.**
Its running time grows exponentially with the number of conflicts, which is
largest exactly when the floor is crowded. The fleet manager replans many
times a minute and most replans are trivial. So: cheap method first, search
with a budget second (ADR-031). I give up optimality and completeness for a
bounded answer time.

**What is base and horizon and how do you use it?**
The released and the announced part of a VDA 5050 order. I release three
edges ahead and announce three more. When the vehicle passes a node I send
an update with the same `orderId` and the next `orderUpdateId`, starting at
the last released node. Why three: a release cannot be taken back, so
everything released is closed to the others; one edge would make vehicles
stop at every node (ADR-030).

**A message gets lost. What happens?**
QoS 0, so it happens. An order counts as delivered when the vehicle's state
shows its ids. Until then it is sent again every two seconds and nothing new
is granted to that vehicle. A lost state costs nothing; the next one has the
same content.

**A vehicle fails or goes offline in the middle of an aisle?**
It keeps everything it was granted, because it may be anywhere in there. It
is planned around. For failures there is one procedure whatever the cause
(ADR-032): cancel the order, wait until the vehicle stands, find out where
it is. If it stands on an edge it keeps the edge and both ends and gets an
order to the end of the edge whose first node allows for the distance
already driven.

**What was the hardest bug?**
A task was cancelled in the instant its first order had been sent. The
vehicle's last state still said "idle", so the fleet manager considered it
stopped and freed its reservations. A moment later the order arrived and
the vehicle drove off with nothing reserved. A cancellation would not have
helped: it could overtake the order and find nothing to cancel. The fix is
to see the order through, sending it until the vehicle has it, and cancel
then. No hand-written test had it; a random scenario with a claims monitor
did. It is now `AnOrderOnItsWayToAVehicleThatIsToBeStoppedIsSeenThrough`.

**How do you test something like this?**
Three layers. Unit tests for the geometry and the planner stages. A
simulated fleet in which the fleet manager talks to real instances of the
vehicle-side state machine through queues that delay and lose messages;
only motion is simulated, and time is a variable, so a minute takes a
millisecond. And monitors that check distance, claims and presence after
every step without using the fleet manager's logic. On top, seeded random
scenarios with injected faults: 20,000 without faults and 20,000 with, no
violation and no task left hanging. And one system test with the real
program, a real broker and separate vehicle processes.

**What does that prove?**
That the properties held in those scenarios: small maps, up to six
vehicles, point-shaped vehicles. Not that they always hold. For the claims
rule there is an argument independent of the tests. For liveness of the
whole system there is not.

**What is its biggest weakness?**
A vehicle that is online but cannot be placed. It holds nothing, and after
ten seconds the fleet works past it (ADR-033). Second: it has never driven
a real robot, not even the Nav2 adapter next to it in the repository. Third:
no memory across a restart.

**Is this a safety system?**
No. It is traffic control in software, based on positions the vehicles
report. Each vehicle needs its own certified obstacle detection.

**How does this compare to Open-RMF or a commercial fleet manager?**
Much smaller. Open-RMF handles several fleets from different vendors with
its own traffic negotiation, doors, lifts and task bidding. This is one
fleet on one roadmap, with the mechanism (reservation plus plan order)
visible in a few hundred lines. I would use it to explain how such systems
work, not to replace one.

## Be honest about

State these before you are asked.

- It has never run on a physical robot or against a real fleet manager. A
  script plays master control.
- The fleet manager has only ever met this repository's own vehicle side,
  with a point for a robot. It has not driven the Nav2 adapter. It is not a
  safety function, forgets queued tasks on restart, assumes one vehicle size
  and straight edges, and its planner can miss routes that exist. Its
  first-stage heuristics grew from failing cases and are the least principled
  code in the repository.
- Fault management has only three built-in monitors, all about the bridge
  itself. Holding the vehicle on a critical fault is not a safety function.
  The web API has one optional shared token and no TLS.
- Coverage is measured for the three libraries only; the adapter, the fault
  manager node and the gateway's ROS code have end-to-end tests but no unit
  tests.
- With default Nav2 parameters the robot stops up to 0.24 m from a node and
  Nav2 can give up in a narrow passage. I report that correctly; I have not
  tuned Nav2.
- One deliberate deviation from the official JSON schema: action status
  `PAUSED`, which the specification text defines and the schema omits.
- Simplifications: blocking edge actions run at the start of the edge; edge
  actions are not stopped when the edge is left; `trajectory`, `loads` and
  `information` are not implemented; action pause/resume is not propagated to
  running actions; MQTT has no TLS.
- It targets v2.0. v2.1 additions and v3.0 are not supported.
- If you used an AI assistant to write it, say so, and be ready to modify the
  code live. That is a normal way to work; pretending otherwise is the risk.

## Exercises that make it yours

Each is small and forces you through the important code.

1. Add the `distanceSinceLastNode` state field: type, snapshot, encoder, test.
2. Make a failed HARD action halt the order (change ADR-008), with a config flag.
3. Add `validate_order` rule: `nodePosition.theta` must be within [-pi, pi].
4. Add `decode_factsheet()` for the master control side, with a round-trip
   test like those in `test_codec_master.cpp`.
5. Read `PortsMayReportResultsSynchronously`, then remove the re-entrancy guard
   in `advance()` and the copies in `advance_once()` one at a time, and explain
   what breaks under AddressSanitizer and why.
6. In `fleet_core`: set `release_ahead` to 1 and then to 10, run
   `FleetSim` and the soak tests, and explain what changes and why.
7. Remove the order rule from `grantable()` (return true) and find the
   smallest test that now hangs. Then remove the claims check instead and see
   which monitor fires first.
