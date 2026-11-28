# vda5050_bridge

Connect a ROS 2 mobile robot to any VDA 5050 fleet manager.

[VDA 5050](https://github.com/VDA5050/VDA5050) is the open standard that lets
one fleet manager ("master control") command mobile robots from different
manufacturers. It defines JSON messages exchanged over MQTT. This project is the
**vehicle side** of that interface: it receives orders, drives the robot through
them node by node with Nav2, runs the attached actions, and reports state back.

## Status

[![CI](https://github.com/KazmirFahrier/vda5050_bridge/actions/workflows/ci.yml/badge.svg)](https://github.com/KazmirFahrier/vda5050_bridge/actions/workflows/ci.yml)

| Package | What it is | Verification |
|---|---|---|
| `vda5050_core` | ROS-independent C++17 library: JSON, message types, codec, order validation, order state machine, MQTT session logic | 115 unit tests and one end-to-end example, 98.8 % line coverage. Runs in CI with GCC and Clang under AddressSanitizer/UBSan and under colcon on ROS 2 Jazzy. Its messages are validated against the official VDA 5050 JSON schemas, versions 2.0.0 and 2.1.0, and its parsers are fuzzed for 60 s per build |
| `fault_core` | ROS-independent C++17 library for fault management: fault catalogue, debounce / latch / watchdog state machine, event history with persistence and rotation, mapping of faults to VDA 5050 errors | 54 unit tests, 98 % line coverage. Run in CI with GCC and Clang under AddressSanitizer/UBSan and under colcon |
| `fault_msgs` | ROS 2 messages and services for reporting, listing and acknowledging faults | Built in CI |
| `fault_manager` | ROS 2 node around `fault_core`: fault reports and `/diagnostics` in, active faults and events out, history file, and a flight recorder that writes a rosbag2 bag of the seconds before and after a fault | End-to-end test in CI with all nodes and a real broker. No unit tests of its ROS code |
| `fault_gateway` | Web API in Python: REST, a live event stream and a status page | 37 unit tests of the HTTP layer and the conversion, plus the end-to-end test |
| `vda5050_nav2_adapter` | ROS 2 lifecycle node: Nav2 action client, libmosquitto transport, action forwarding, parameters, launch file | Builds on ROS 2 Jazzy with no warnings. It also reports its own health as faults, lists raised faults in the VDA 5050 state and holds the vehicle while a critical fault is raised. Tested in CI against a real Mosquitto broker (including a broker outage, a crash, and lifecycle transitions driven from outside) and against the **real Nav2 stack**, both with Nav2's loopback simulator and in **Gazebo** with a simulated lidar and AMCL |

All packages pass the ROS 2 linters (cpplint, uncrustify, xmllint, lint_cmake,
flake8, pep257) as part of `colcon test`.

What has **not** been done, and cannot be from a CI machine:

- No physical robot. Everything that moves is simulated.
- No real fleet manager. Master control is played by a script; the evidence of
  interoperability is schema conformance, not a session with a commercial product.

Known gaps and deviations:

- Line coverage is measured for the two libraries only. The adapter, the fault
  manager node and the gateway's ROS code are exercised by end-to-end tests but
  have no unit tests and no coverage figure.
- Holding the vehicle on a critical fault is a software reaction through Nav2.
  It is not a safety function.
- Action status `PAUSED` is in the specification text but not in the official
  schemas' enum. The library follows the text; a master that validates strictly
  against the schema would reject a state containing a paused action.
- With Nav2's default parameters the robot stops 0.17 to 0.24 m from a node,
  and in a narrow passage Nav2 can stall or give up. See
  [docs/testing.md](docs/testing.md); tuning Nav2 is deployment work.
- Protocol features not implemented are listed in
  [docs/roadmap.md](docs/roadmap.md).

Details: [docs/testing.md](docs/testing.md).

## Repository layout

```
vda5050_bridge/
├── README.md
├── LICENSE                         Apache-2.0
├── .github/workflows/ci.yml        CI: 8 jobs (builds, tests, linters, schemas, coverage, fuzzing, Nav2, Gazebo)
├── docs/
│   ├── vda5050_primer.md           The protocol, explained from scratch
│   ├── architecture.md             How the code is structured and why
│   ├── design_decisions.md         Decision records with trade-offs
│   ├── testing.md                  What is tested, how, and what is not
│   ├── fault_management.md         Fault management: concepts, nodes, web API
│   ├── interview_guide.md          How to present and defend this project
│   └── roadmap.md                  Next steps and known limitations
├── vda5050_core/                   ament_cmake package, no ROS dependencies
│   ├── include/vda5050_core/
│   │   ├── json.hpp                JSON value, strict parser, writer
│   │   ├── types.hpp               Order, State, Factsheet ... as plain structs
│   │   ├── codec.hpp               JSON <-> types
│   │   ├── header.hpp              Identity, topic names, header ids, timestamps
│   │   ├── order_validator.hpp     Graph rules for an order
│   │   ├── ports.hpp               NavigationPort / ActionPort interfaces
│   │   ├── order_manager.hpp       The order state machine
│   │   └── session.hpp             What is published on which topic, and when
│   ├── src/
│   ├── test/                       Unit tests, schema-conformance scripts, example messages
│   ├── tools/emit_samples.cpp      Writes example messages for schema validation
│   ├── fuzz/fuzz_decoder.cpp       libFuzzer target for the parsers
│   └── examples/simulated_vehicle.cpp   Runnable demo: no ROS, no broker
├── fault_core/                     ament_cmake package, no ROS dependencies
│   ├── include/fault_core/
│   │   ├── types.hpp               Fault definition, record, event
│   │   ├── catalog.hpp             The list of known faults, loaded from JSON
│   │   ├── fault_manager.hpp       Debounce / latch / watchdog state machine, history
│   │   ├── codec.hpp               JSON for events, records and summaries
│   │   ├── history_store.hpp       History file (JSON Lines) with rotation
│   │   └── vda5050_mapping.hpp     Raised faults -> VDA 5050 errors
│   ├── src/
│   └── test/
├── fault_msgs/                     Messages and services
├── fault_manager/                  ament_cmake package: the fault manager node
│   ├── include/fault_manager/
│   │   ├── fault_manager_node.hpp  Topics, services, history file
│   │   ├── snapshot_recorder.hpp   Flight recorder: ring buffer -> rosbag2
│   │   └── conversions.hpp         fault_core types <-> messages
│   ├── config/                     Example parameters and fault catalogue
│   ├── launch/fault_manager.launch.py
│   └── test/fault_system_test.py   End-to-end test of all fault nodes
├── fault_gateway/                  ament_python package: the web API
│   ├── fault_gateway/
│   │   ├── http_api.py             REST, event stream; no ROS in here
│   │   ├── gateway_node.py         The ROS side
│   │   ├── convert.py              Messages -> JSON
│   │   └── static/index.html       Status page
│   └── test/
└── vda5050_nav2_adapter/           ament_cmake package
    ├── README.md                   Interfaces, parameters, how to run
    ├── include/vda5050_nav2_adapter/
    │   ├── adapter_node.hpp        Lifecycle node
    │   ├── nav2_navigation_adapter.hpp   NavigationPort -> NavigateToPose
    │   ├── mosquitto_transport.hpp       MqttTransport -> libmosquitto
    │   └── topic_action_handler.hpp      ActionPort -> ROS topics
    ├── src/
    ├── config/adapter.yaml
    ├── launch/adapter.launch.py
    └── test/                       Integration test (fake Nav2) and system test (real Nav2)
```

## Try it without ROS

Any machine with CMake >= 3.16 and a C++17 compiler:

```bash
cmake -S vda5050_core -B build -DCMAKE_BUILD_TYPE=Debug -DVDA5050_CORE_SANITIZE=ON
cmake --build build
ctest --test-dir build --output-on-failure
./build/simulated_vehicle
```

`simulated_vehicle` plays a scripted master control against a simulated robot
and prints the MQTT conversation:

```
master -> order o1/0: N0 -> N1 [pick] released, N2 on horizon
  vehicle -> state  order=o1/0 lastNode=N0 driving=yes nodesAhead=2 newBaseRequest=yes [pick:WAITING]
  (robot arrived)
  vehicle -> state  order=o1/0 lastNode=N1 driving=no nodesAhead=1 newBaseRequest=yes [pick:INITIALIZING]
  (robot finished action)
  vehicle -> state  order=o1/0 lastNode=N1 driving=no nodesAhead=1 newBaseRequest=yes [pick:FINISHED]
master -> order o1/1: releases N1 -> N2
  vehicle -> state  order=o1/1 lastNode=N1 driving=yes nodesAhead=1 newBaseRequest=no [pick:FINISHED]
  (robot arrived)
  vehicle -> state  order=o1/1 lastNode=N2 driving=no nodesAhead=0 newBaseRequest=no [pick:FINISHED]
```

## Build with ROS 2

Target: ROS 2 Jazzy on Ubuntu 24.04. These are the commands CI runs.

```bash
cd ~/ros2_ws/src && git clone https://github.com/KazmirFahrier/vda5050_bridge.git
cd ~/ros2_ws
sudo apt-get install -y libmosquitto-dev      # no rosdep key exists for it
rosdep install --from-paths src --ignore-src -y
colcon build
colcon test && colcon test-result --verbose
source install/setup.bash
ros2 launch vda5050_nav2_adapter adapter.launch.py
ros2 launch fault_manager fault_manager.launch.py      # optional: fault management
ros2 launch fault_gateway fault_gateway.launch.py      # optional: http://localhost:8080/
```

To run the tests locally (needs `mosquitto` and `mosquitto-clients`; the second
also needs `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup` and
`ros-jazzy-nav2-loopback-sim`):

```bash
bash src/vda5050_bridge/vda5050_nav2_adapter/test/integration_test.sh
bash src/vda5050_bridge/vda5050_nav2_adapter/test/nav2_simulation_test.sh
SIMULATOR=gazebo bash src/vda5050_bridge/vda5050_nav2_adapter/test/nav2_simulation_test.sh
python3 src/vda5050_bridge/fault_manager/test/fault_system_test.py
```

The Gazebo mode also needs `ros-jazzy-nav2-minimal-tb3-sim` and, on a machine
without a display, `xvfb`.

## Where to start reading

1. [docs/vda5050_primer.md](docs/vda5050_primer.md) if the protocol is new to you.
2. [docs/architecture.md](docs/architecture.md) for the structure.
3. `vda5050_core/test/test_order_manager.cpp`: each test is one protocol rule,
   written as a short scenario.
4. `vda5050_core/src/order_manager.cpp`, then `session.cpp`.
5. `vda5050_nav2_adapter/src/adapter_node.cpp` for how ROS 2 is attached.
6. [docs/fault_management.md](docs/fault_management.md), then
   `fault_core/test/test_fault_manager.cpp`, for fault management. The
   end-to-end behaviour is easiest to read in
   `fault_manager/test/fault_system_test.py`.

## License

Apache-2.0. See [LICENSE](LICENSE).
