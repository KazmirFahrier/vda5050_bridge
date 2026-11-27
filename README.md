# vda5050_bridge

Connect a ROS 2 mobile robot to any VDA 5050 fleet manager.

[VDA 5050](https://github.com/VDA5050/VDA5050) is the open standard that lets
one fleet manager ("master control") command mobile robots from different
manufacturers. It defines JSON messages exchanged over MQTT. This project is the
**vehicle side** of that interface: it receives orders, drives the robot through
them node by node with Nav2, runs the attached actions, and reports state back.

## Status

Two packages, at very different levels of verification. Read this table before
relying on either.

| Package | What it is | Verification |
|---|---|---|
| `vda5050_core` | ROS-independent C++17 library: JSON, message types, codec, order validation, order state machine, MQTT session logic | **Built and tested.** 96 unit tests and one end-to-end example pass with GCC and Clang, including under AddressSanitizer/UBSan |
| `vda5050_nav2_adapter` | ROS 2 lifecycle node: Nav2 action client, libmosquitto transport, action forwarding, parameters, launch file | **Written, never compiled.** No ROS 2 installation was available. It was only syntax-checked against hand-written stand-in headers. Expect build errors on the first `colcon build` |

Also not yet done, for either package:

- The tests ran with the bundled GoogleTest-compatible shim, not real GoogleTest.
- Nothing has been built with colcon; the CI workflow has never run.
- Nothing has been exchanged with a real MQTT broker, fleet manager, simulator
  or robot. JSON field names were checked by reading the published v2.0 schemas.

Details: [docs/testing.md](docs/testing.md).

## Repository layout

```
vda5050_bridge/
├── README.md
├── LICENSE                         Apache-2.0
├── .github/workflows/ci.yml        CI definition (never run)
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
│   ├── test/
│   └── examples/simulated_vehicle.cpp   Runnable demo: no ROS, no broker
└── vda5050_nav2_adapter/           ament_cmake package (UNBUILT)
    ├── README.md                   Interfaces, parameters, first-build checklist
    ├── include/vda5050_nav2_adapter/
    │   ├── adapter_node.hpp        Lifecycle node
    │   ├── nav2_navigation_adapter.hpp   NavigationPort -> NavigateToPose
    │   ├── mosquitto_transport.hpp       MqttTransport -> libmosquitto
    │   └── topic_action_handler.hpp      ActionPort -> ROS topics
    ├── src/
    ├── config/adapter.yaml
    └── launch/adapter.launch.py
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

## Build with ROS 2 (intended; not yet verified)

Target: ROS 2 Jazzy on Ubuntu 24.04, with Nav2 and an MQTT broker such as Mosquitto.

```bash
cd ~/ros2_ws/src && git clone <this repository>
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build
colcon test && colcon test-result --verbose
source install/setup.bash
ros2 launch vda5050_nav2_adapter adapter.launch.py
```

See [vda5050_nav2_adapter/README.md](vda5050_nav2_adapter/README.md) for the
first-build checklist.

## Where to start reading

1. [docs/vda5050_primer.md](docs/vda5050_primer.md) if the protocol is new to you.
2. [docs/architecture.md](docs/architecture.md) for the structure.
3. `vda5050_core/test/test_order_manager.cpp`: each test is one protocol rule,
   written as a short scenario.
4. `vda5050_core/src/order_manager.cpp`, then `session.cpp`.
5. `vda5050_nav2_adapter/src/adapter_node.cpp` for how ROS 2 is attached.

## License

Apache-2.0. See [LICENSE](LICENSE).
