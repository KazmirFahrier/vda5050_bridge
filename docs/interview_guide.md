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
> 96 unit tests that run in milliseconds. A ROS 2 lifecycle node attaches it to
> Nav2 and an MQTT broker.

The whole thing builds in CI on ROS 2 Jazzy. One test runs the node against a
real MQTT broker; another drives orders through the real Nav2 stack, including
in Gazebo with a simulated lidar and AMCL: multi-leg orders, pause, resume,
cancel, and a navigation failure with recovery.

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

## Be honest about

State these before you are asked.

- It has never run on a physical robot or against a real fleet manager. A
  script plays master control.
- In simulation the obstacles are static. Nothing moves into the path, and
  navigation failure is tested only for a goal Nav2 refuses outright.
- The first leg after start-up is slow (about 46 s, with Nav2 progress
  failures) and I have not diagnosed why.
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
4. Implement `encode_order()` and a decode/encode round-trip test.
5. Read `PortsMayReportResultsSynchronously`, then remove the re-entrancy guard
   in `advance()` and the copies in `advance_once()` one at a time, and explain
   what breaks under AddressSanitizer and why.
