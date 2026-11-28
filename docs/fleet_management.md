# Fleet traffic control

The rest of this repository is the vehicle side of VDA 5050. This part is the
other side: a master control that gives work to several vehicles on one
roadmap and keeps them from colliding and from blocking each other.

It is two packages:

| Package | What it is |
|---|---|
| `fleet_core` | C++17 library without ROS and without MQTT: roadmap, conflict detection, multi-robot planner, fleet manager, command and status interface |
| `vda5050_fleet` | Two programs: `fleet_manager` (the library behind an MQTT connection) and `sim_vehicle` (a VDA 5050 vehicle without a robot, for trying it out) |

What it is not: a warehouse management system (it carries out tasks, it does
not decide which goods go where), a safety system (see [Limits](#limits)), or
a product that has run a real fleet. Everything that moves is simulated.

Contents: [The problem](#the-problem) ·
[Roadmap and conflicts](#roadmap-and-conflicts) ·
[The two rules](#the-two-rules) · [The planner](#the-planner) ·
[From plan to orders](#from-plan-to-orders) ·
[When things go wrong](#when-things-go-wrong) ·
[Running it](#running-it) · [Interface](#interface) · [Limits](#limits) ·
[Evidence](#evidence)

## The problem

A vehicle that follows a VDA 5050 order drives where it is told. It does not
know about other vehicles. Whoever writes the orders must make sure that:

1. **No two vehicles are ever sent into the same space.** This must hold
   whatever happens: a vehicle is slower than planned, a message is lost, a
   vehicle fails half-way, somebody switches a vehicle on in the middle of an
   aisle.
2. **Vehicles do not wait for each other for ever.** Two vehicles facing each
   other in an aisle, each waiting for the other to leave, is a deadlock. So
   is a ring of four, each waiting for the one in front.
3. **Work gets done**, and reasonably quickly.

The first is a safety property, the second a liveness property, the third is
quality. This design keeps them apart: each has its own mechanism, and the
first does not depend on the other two being right.

## Roadmap and conflicts

The roadmap is a directed graph: **nodes** (places where a vehicle can stand)
and **edges** (straight connections it can drive). An edge may be one-way. It
is loaded from JSON:

```json
{"mapId": "warehouse",
 "nodes": [{"id": "A1", "x": 0.0, "y": 4.0}, {"id": "P1", "x": 0.0, "y": -2.0, "parking": true}],
 "edges": [{"id": "A1-A2", "from": "A1", "to": "A2", "bidirectional": true, "maxSpeed": 1.5}]}
```

Every node and every edge is a **resource**. Two resources **conflict** if a
vehicle on one could touch a vehicle on the other: if the distance between
them (point to point, point to segment, or segment to segment) is less than
the **clearance**

```
clearance = 2 × vehicle_radius + max(safety_margin, 2 × attach_tolerance)
```

`attach_tolerance` is how far from a node a vehicle may stand and still count
as being at it. Two vehicles may each be that far off, which is why it enters
twice. With the defaults (radius 0.4 m, margin 0.2 m, tolerance 0.25 m) the
clearance is 1.3 m.

The conflict table is computed once from the geometry. This matters: asking
only "do two vehicles want the same node?" misses an edge that passes close
by a node, two edges that cross, and the two directions of the same aisle,
which are different edges in the graph but the same strip of floor.

**What a vehicle holds.** A vehicle standing at a node holds that node. To
drive an edge from A to B it must hold the edge and B before it starts, and
it keeps A until it has arrived. So a moving vehicle holds where it was,
where it is and where it is going.

## The two rules

Vehicles are given their route in small pieces (see
[From plan to orders](#from-plan-to-orders)). Giving a vehicle the next edge
is called a **grant**. A grant cannot be taken back: the vehicle may already
be driving. Two rules decide whether a move is granted.

**The claims rule (safety).** A move is granted only if the edge and the node
it leads to conflict with nothing that any other vehicle holds.

This rule alone makes collisions impossible, as long as every vehicle is
within what it holds. It needs no plan, no clock and no assumption about
speed. A vehicle that is late simply holds its resources longer; the others
are not granted theirs. Whatever the planner computes, nothing reaches a
vehicle without passing this check. Even a wrong plan cannot cause a
collision. It can only cause waiting.

**The order rule (liveness).** The claims rule alone deadlocks easily: two
vehicles enter an aisle from both ends, each holding its half. So moves are
granted in the order of a plan. The planner gives every move a time. A move
is granted only when every use of a conflicting resource that another
vehicle's plan has *earlier* has been completed and released.

Why this prevents deadlock: in a valid plan, the uses of any two conflicting
resources do not overlap in time, so "must happen before" is an ordering
without cycles. Vehicles may be late by any amount; the order in which they
pass each contested place stays as planned. A deadlock needs a cycle of
vehicles waiting for each other, and an order without cycles has none.

The plan's times are therefore used only to order things. Nothing is
scheduled by the clock, and a vehicle that is slow delays others without
invalidating the plan.

## The planner

`plan_fleet()` gets, for every vehicle, where it is (or will be when it has
driven what it was granted), what it holds, and the stops of its task, and
returns a timed route for each. It works in stages. Each stage is tried only
if the one before left a vehicle without a route.

**One vehicle: safe-interval path planning.** Given the times at which other
vehicles hold each resource, a node is free during certain intervals. The
search runs over (node, free interval) pairs instead of (node, time), which
keeps it small. It supports several stops in order, waiting at a stop
(dwell), and "rest anywhere" for vehicles without a task that only have to
get out of the way. A route ends where the vehicle can stay for good.

**Stage 1: one vehicle after another.** Vehicles are planned in order of
priority, each around the routes already fixed. Refinements that turned out
to be needed:

- A vehicle not planned yet reserves the node it stands on for as long as it
  needs at least to leave it. Without this, others plan straight through it.
- A vehicle without a task rests where it is, but softly: a vehicle that
  needs the place **pushes** it on, and the pushed one may push the next (up
  to four deep).
- When a vehicle finds no route, the order is repaired: the vehicle is moved
  up, or its own task is set aside for this round, or the least urgent task
  in its way is.
- Several orderings are tried (the natural one, one that first clears idle
  vehicles out of the way and moves vehicles as a convoy, reversed ones and a
  few fixed shuffles). The result with the fewest vehicles left without a
  route wins; among equals, the lowest cost with urgent tasks weighted
  higher.

**Stage 2: search.** Planning one after another fails when two vehicles must
each give way to the other at different places, for example a vehicle that
has to overshoot its stop, let another one through and come back. The second
stage is a conflict-based search: plan everybody alone, find the first
conflict, and branch on who gives way there. It has a budget (300 search
nodes and a bound on total work). Within it, it finds routes that no
ordering can. Beyond it, the search stops and stage 1's result stands.

**Stage 3: one step.** If vehicles are still without a route, they are moved
one node towards making room, pushing along whoever rests in front (a chain
of up to eight). This guarantees nothing by itself, but together with
replanning after every step it gets a crowded one-way loop moving.

**Two details of the fleet manager** belong here because they are what makes
crowded cases finish:

- A task with several stops that cannot be routed as a whole is routed to
  its next stop only. Being there is progress, and the stop's work is done
  while the rest of the way is still closed.
- A vehicle whose task has no route is granted moves one at a time. Three
  granted steps of a detour that the next plan no longer wants would occupy
  exactly the nodes somebody is waiting for.

**The check.** `validate_fleet_plan()` is written separately from the
planner and checks every plan before it is used: routes are connected, times
only go forward, stops are visited in order, and no two vehicles hold
conflicting resources at the same time. A plan that fails is thrown away,
counted (`invalid_plans`) and reported. In all test runs this counter is
zero. It exists because a planner of this size will have bugs, and a wrong
plan should cost a planning round and not a search for the cause on the
floor.

**What the planner does not promise.** Routes are not optimal. With many
vehicles in little space it may find no complete set of routes although one
exists; then vehicles are moved step by step, which can take long. See
[Limits](#limits).

## From plan to orders

**Tasks.** A task is a list of stops (nodes), each with optional VDA 5050
actions, an optional vehicle and a priority. Tasks wait in a queue. A free
vehicle gets the most urgent task; among equally urgent ones the oldest, and
of several free vehicles the nearest (by travel time on the roadmap).

**Orders.** VDA 5050 lets master control tell a vehicle more than it may
drive yet: the released part of an order is the **base**, the announced part
the **horizon**. The fleet manager uses exactly this:

- Up to three edges ahead are granted and sent as base (`release_ahead`).
- Up to three more planned edges are sent as horizon. The vehicle shows it
  but does not drive it.
- When the vehicle has passed a node, the next edge is granted if both rules
  allow, and an **order update** goes out: same `orderId`, `orderUpdateId`
  one higher, starting at the last base node with the same `sequenceId`.
- Actions of a stop are attached to the node of that stop.
- A vehicle that has come to rest and has nothing announced gets a new
  `orderId` for its next journey.

Three edges ahead is a balance. More would let vehicles drive without
stopping at each node, but a grant cannot be taken back, so everything
granted is closed to the others even if the plan changes. One would be the
most flexible and make every vehicle stop at every node.

**Confirmation.** The transport is MQTT with QoS 0, as the standard
prescribes, so an order can be lost. The fleet manager considers an order
received when the vehicle's state shows its `orderId` and `orderUpdateId`.
Until then it sends the same order again every two seconds (the vehicle
ignores an order it already has) and grants nothing new to that vehicle.

**Replanning** happens when something changes: a task arrives or ends, a
vehicle appears or becomes unusable, a stop is done. And every two seconds
while a task has no route. What is granted is never changed by replanning;
the new plan starts where the granted moves end.

## When things go wrong

| What happens | What the fleet manager does |
|---|---|
| An order is lost | Sends it again until the vehicle's state confirms it |
| A state message is lost | Nothing; the next one carries the same information |
| A vehicle is slower than planned | Others wait. No replanning is needed for safety |
| A vehicle's connection drops (last will), or it is silent for 60 s | The vehicle is unusable. It keeps everything it holds: it may be anywhere in there. Others are planned around it |
| A vehicle reports a navigation failure | Counts it, cancels the order, waits until the vehicle stands, replans. After three in a row the vehicle is taken out of service and its task fails; an operator puts it back with `resetVehicle` |
| An action fails | The task fails. The vehicle is stopped and is free for other work |
| The vehicle rejects an order | Reported; the vehicle is stopped and brought back into step |
| A task is cancelled while the vehicle drives | `cancelOrder`. The vehicle stops where it is, possibly on an edge; it keeps that edge and both its ends and gets an order to the end of the edge, whose first node allows for the distance already covered |
| A task ends while its order is still on the way to the vehicle | The order cannot be called back, and a cancellation overtaking it would do nothing. It is sent until the vehicle has it and cancelled then. Meanwhile the vehicle keeps what it was granted |
| A vehicle is paused, in manual mode, or reports a fatal error | Unusable until that ends; its task waits |
| A vehicle reports an order the fleet manager did not send, or none where it should have one | It is stopped and brought back into step |
| The fleet manager hears of a vehicle but not where it is | For ten seconds nobody is granted anything and the vehicle is asked for its state (`stateRequest`). Then the fleet carries on without it; see [Limits](#limits) |
| A vehicle turns up where another one has been granted | The other one is stopped at once. Both stay where they are until what they hold no longer conflicts |
| Two vehicles are switched on closer together than the clearance | Neither is moved; the rest of the fleet is planned around both. Somebody has to move one by hand |
| A vehicle is found standing on an edge (for example after a restart of the fleet manager) | If nobody else holds the edge or its ends, it gets them and an order to the end of the edge. Otherwise it waits |
| A vehicle stands away from every node and edge | Unusable until it is at a node |
| The planner returns a plan that fails the check | The plan is dropped, counted and reported. Everybody keeps what they have |
| The fleet manager is restarted | It starts with no memory. Vehicles in the middle of an order are stopped and brought to a node, then work continues. Tasks that were queued are gone; whoever submitted them must do so again |

Every one of these is an event with a name (`navigation_failed`,
`claim_conflict`, `vehicle_recovering`, ...), published and printed.

## Running it

Without ROS. Needs CMake, a C++17 compiler, `libmosquitto-dev` and a broker
(`mosquitto`).

```bash
cmake -S vda5050_fleet -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build

mosquitto -p 1883 &
./build/fleet_manager --roadmap vda5050_fleet/config/demo_roadmap.json &
./build/sim_vehicle --serial v1 --x 0 --y -2 --map-id warehouse &
./build/sim_vehicle --serial v2 --x 4 --y -2 --map-id warehouse &
./build/sim_vehicle --serial v3 --x 8 --y -2 --map-id warehouse &

mosquitto_sub -t 'fleet/v1/events' &
mosquitto_pub -t fleet/v1/command -m \
  '{"command":"submit","stops":[{"node":"A1","actions":[{"actionType":"pick"}]},{"node":"C5","actions":[{"actionType":"drop"}]}]}'
mosquitto_pub -t fleet/v1/command -m '{"command":"submit","stops":[{"node":"A5"},{"node":"C1"}]}'
```

The demo roadmap is two aisles joined by three cross lanes, with three
parking places:

```
 A1 ── A2 ── A3 ── A4 ── A5      y = 4
 │           │           │
 B1          B3          B5      y = 2
 │           │           │
 C1 ── C2 ── C3 ── C4 ── C5      y = 0
 │           │           │
 P1          P3          P5      y = -2
```

`sim_vehicle` is the complete vehicle side of `vda5050_core`, the same code
the Nav2 adapter runs, with a point moving in a straight line in place of a
robot. A real robot running `vda5050_nav2_adapter` connects in the same way;
this combination has not been tested (see [Limits](#limits)).

Under colcon both programs are installed in `lib/vda5050_fleet`.

Options of `fleet_manager` (`--name value`): `roadmap` (required), `host`,
`port`, `client-id`, `username` (the password is read from the environment
variable `FLEET_MQTT_PASSWORD`), `interface`, `version`, `api`,
`vehicle-radius`, `safety-margin`, `attach-tolerance`, `release-ahead`,
`horizon`, `state-timeout`, `discovery-grace`, `default-speed`,
`status-interval`.

## Interface

All JSON over MQTT, under a prefix (default `fleet/v1`).

| Topic | Direction | Content |
|---|---|---|
| `fleet/v1/command` | to the fleet manager | One command per message |
| `fleet/v1/response` | from | One response per command, matched by `requestId` |
| `fleet/v1/state` | from, retained | Vehicles, tasks, counters; once a second and after every command |
| `fleet/v1/events` | from | One event per message |
| `fleet/v1/connection` | from, retained | `ONLINE`, or `OFFLINE` (also as last will) |

Commands:

```json
{"command": "submit", "requestId": "r1", "priority": 0, "vehicle": "",
 "stops": [{"node": "A1", "dwell": 5,
            "actions": [{"actionType": "pick", "blockingType": "HARD",
                         "actionParameters": [{"key": "station", "value": "S3"}]}]}]}
{"command": "cancel", "requestId": "r2", "taskId": 3}
{"command": "resetVehicle", "requestId": "r3", "serialNumber": "v1"}
```

Responses: `{"requestId":"r1","ok":true,"taskId":3}` or
`{"requestId":"r2","ok":false,"error":"no such task, or it is already over"}`.
Every command gets one, also one that is not JSON.

A task goes through `QUEUED`, `ASSIGNED`, and ends as `FINISHED`, `FAILED`
or `CANCELLED`. The last 200 ended tasks are kept for display.

The interface is implemented in `fleet_core` (`api.hpp`) as text in, text
out. The program only carries it. Another transport (HTTP, a ROS service)
would call the same three functions.

There is no access control in the fleet manager. Whoever may publish on the
command topic may give orders to the fleet. Restricting that is the broker's
job (users and topic permissions).

## Limits

These are the things to know before trusting this with real vehicles.

- **Not a safety function.** Keeping vehicles apart here is software that
  relies on positions the vehicles report. Every vehicle needs its own
  certified obstacle detection; this system reduces how often that has to
  act.
- **A vehicle the fleet manager cannot place is invisible to it.** A vehicle
  that is online but reports no usable position, or stands away from the
  roadmap, holds nothing. After the ten seconds of waiting, other vehicles
  are sent past it. There is no good automatic answer; the event
  `vehicle_unlocated` is what an operator has to act on.
- **Vehicles are assumed to stay on the roadmap**, within `attach_tolerance`
  of the node or edge they hold. Nav2 with default parameters stops up to
  0.25 m from a goal, which is why the default tolerance is 0.25 m. A vehicle
  that leaves the path to go round an obstacle by more than that is outside
  what it holds.
- **All vehicles have one size.** The clearance is one number for the fleet.
- **Edges are straight lines.** No curves, no turning space, no orientation.
  A long vehicle turning at a node sweeps more than a point does.
- **No state is kept across a restart.** Queued tasks are lost; vehicles are
  stopped and re-adopted.
- **One fleet manager.** No redundancy. If it stops, vehicles finish what
  they were granted and stand.
- **The planner is not complete.** It can fail to find routes that exist,
  and falls back to moving vehicles step by step. On a crowded one-way loop
  this works but is slow. Tasks whose stops can never all be free (two tasks
  ending at the same node with nowhere else to rest) wait for ever; nothing
  detects that a task is hopeless.
- **No battery or charging logic**, no opening hours, no zones, no lifts or
  doors.
- **Assignment is greedy**: nearest free vehicle for the most urgent task,
  one at a time. It does not look ahead.
- **Scale.** Tested with up to six vehicles on up to 24 nodes. Planning
  takes milliseconds there (400 random problems with up to eight vehicles on
  up to 30 nodes: 1.4 ms on average, 84 ms at worst, on one core). The conflict table is quadratic in the number of
  resources and the planner was not measured on large maps.
- **Never run against a real fleet manager's vehicles or real robots.** The
  vehicle side it was tested against is this repository's own. An error in
  how both sides read the standard would not show.

## Evidence

**Unit tests** (`fleet_core`, 91 tests, 97 % line coverage, run under
AddressSanitizer and UBSan with GCC and Clang):

- roadmap and conflict geometry; the single-vehicle planner; each stage of
  the fleet planner on cases built to need it (head-on meeting with a
  siding, a convoy on a one-way loop, pushing, a case only the search
  solves); 400 random planning problems that must all pass the check
- the orders on the wire: base, horizon, stitching, ids
- the command interface, including 28 malformed commands that must each be
  refused with a reason

**A simulated fleet** (`test/sim.hpp`). The fleet manager and, for every
vehicle, the real vehicle side of `vda5050_core`, exchange their real
messages over a message bus with delay, jitter and loss. Only the motion is
simulated. Three monitors watch every step, independent of the code under
test:

- distance: no two vehicles closer than two radii
- claims: no two vehicles holding conflicting resources
- presence: every vehicle within what it holds

Nearly every row of the table in
[When things go wrong](#when-things-go-wrong) has a named test here. Two do
not: a vehicle that is merely slow (covered by the soak tests, where every
vehicle has its own speed), and a plan that fails the check (no test can make
the planner produce one).

**Soak tests.** Random maps (grids, a corridor with a siding, one-way
loops), one to six vehicles of different speeds, tasks arriving over time,
network delay and, in every third scenario, 10 % message loss. A second
variant injects a fault every ten seconds on average for the first minute:
connections dropped, drives failing, vehicles slowed or held, fatal errors,
tasks cancelled. Pass means: the monitors stayed silent, no plan failed the
check, and no task was left unfinished (without faults) or hanging (with
faults). The test suite runs 40 and 25 scenarios. The same tests run any
number:

```bash
FLEET_SOAK_SCENARIOS=20000 ./build/fleet_core_tests
```

That run, on the code as merged: 20,000 scenarios without faults, 109,755
tasks, all finished; 20,000 with faults, 110,223 tasks, 102,132 finished and
the rest cancelled or failed by the injected faults; no monitor violation,
no plan that failed the check, no task left hanging. Details in
[testing.md](testing.md#fleet-traffic-control).

These runs found what the hand-written tests had not: an order still on its
way when its task was cancelled, a failure hidden behind a lost message,
credit for a stop given to the wrong task, a crowded loop circling for ever
without serving a stop, a detour granted three steps deep that locked a
corridor. Each is now a named test.

**System test** (`vda5050_fleet/test/fleet_system_test.py`, in CI): the
`fleet_manager` program, a real Mosquitto broker and four `sim_vehicle`
processes. Eleven tasks; one is cancelled under way; one vehicle process is
killed in the middle of a task and started again where it stood. The script
records every vehicle's position ten times a second and fails if two come
closer than they may.
