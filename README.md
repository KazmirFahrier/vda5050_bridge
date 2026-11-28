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
| `vda5050_core` | ROS-independent C++17 library: JSON, message types, codec, order validation, order state machine, MQTT session logic | 109 unit tests and one end-to-end example, 98.8 % line coverage. Runs in CI with GCC and Clang under AddressSanitizer/UBSan and under colcon on ROS 2 Jazzy. Its messages are validated against the official VDA 5050 JSON schemas, versions 2.0.0 and 2.1.0, and its parsers are fuzzed for 60 s per build |
| `vda5050_nav2_adapter` | ROS 2 lifecycle node: Nav2 action client, libmosquitto transport, action forwarding, parameters, launch file | Builds on ROS 2 Jazzy with no warnings. Tested in CI against a real Mosquitto broker (including a broker outage, a crash, and lifecycle transitions driven from outside) and against the **real Nav2 stack**, both with Nav2's loopback simulator and in **Gazebo** with a simulated lidar and AMCL |

Both packages pass the ROS 2 linters (cpplint, uncrustify, xmllint, lint_cmake,
flake8, pep257) as part of `colcon test`.

What has **not** been done, and cannot be from a CI machine:

- No physical robot. Everything that moves is simulated.
- No real fleet manager. Master control is played by a script; the evidence of
  interoperability is schema conformance, not a session with a commercial product.

Known gaps and deviations:

- Line coverage is measured for the core only. The adapter is exercised by
  three end-to-end tests but has no unit tests and no coverage figure.
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
```

To run the tests locally (needs `mosquitto` and `mosquitto-clients`; the second
also needs `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup` and
`ros-jazzy-nav2-loopback-sim`):

```bash
bash src/vda5050_bridge/vda5050_nav2_adapter/test/integration_test.sh
bash src/vda5050_bridge/vda5050_nav2_adapter/test/nav2_simulation_test.sh
SIMULATOR=gazebo bash src/vda5050_bridge/vda5050_nav2_adapter/test/nav2_simulation_test.sh
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

## License

Apache-2.0. See [LICENSE](LICENSE).
