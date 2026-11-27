# vda5050_nav2_adapter

ROS 2 lifecycle node that makes a Nav2 robot a VDA 5050 vehicle.

> **Status: written, never compiled.** This package was authored on a machine
> without ROS 2. Its sources were syntax-checked only against hand-written
> stand-in headers, which proves the code is internally consistent but says
> nothing about whether it matches the real rclcpp, Nav2 or libmosquitto APIs.
> Treat the first `colcon build` as part of the development work.

## What it does

```
            MQTT broker
                 |
        MosquittoTransport  ---- events queue ---->  AdapterNode timer (20 Hz)
                 ^                                         |
                 | publish                                 v
        vda5050_core::Session  <--- on_message / tick ------
            |            |
   NavigationPort     ActionPort
            |            |
 Nav2NavigationAdapter  TopicActionHandler
            |            |
   NavigateToPose     ~/action_request, ~/action_cancel, ~/action_status
```

## Interfaces

| Kind | Name | Type | Direction |
|---|---|---|---|
| Action client | `navigate_to_pose` (parameter `navigate_action`) | `nav2_msgs/action/NavigateToPose` | out |
| Subscription | `battery_state` (parameter `battery_topic`) | `sensor_msgs/msg/BatteryState` | in |
| Subscription | `odom` (parameter `odometry_topic`) | `nav_msgs/msg/Odometry` | in |
| TF | `global_frame` -> `base_frame` | | in |
| Publisher | `~/action_request` | `std_msgs/msg/String` (JSON) | out |
| Publisher | `~/action_cancel` | `std_msgs/msg/String` (actionId) | out |
| Subscription | `~/action_status` | `std_msgs/msg/String` (JSON) | in |

MQTT topics, under `<interface_name>/v<major>/<manufacturer>/<serial_number>/`:
subscribes to `order` and `instantActions`; publishes `state`, `connection`,
`visualization` and `factsheet`.

### Executing actions

The adapter does not know how your robot picks or charges. For every action in
`supported_actions` it publishes a request and waits for another node to report:

```
~/action_request   {"actionId":"a1","actionType":"pick","actionParameters":{"height":0.2}}
~/action_status    {"actionId":"a1","status":"RUNNING"}
~/action_status    {"actionId":"a1","status":"FINISHED","resultDescription":"ok"}
```

`status` is one of `RUNNING`, `PAUSED`, `FINISHED`, `FAILED`.

## Parameters

See `config/adapter.yaml`, which lists every parameter with a comment.
`manufacturer` and `serial_number` are mandatory; configuration fails without them.

## Lifecycle

| Transition | Effect |
|---|---|
| configure | Read parameters; create session, Nav2 client, MQTT client, subscriptions. No network traffic |
| activate | Register the last will, connect to the broker, start the update timer |
| deactivate | Publish `OFFLINE`, disconnect, stop the timer |
| cleanup | Destroy everything created in configure |

With `autostart: true` (the default) the executable configures and activates
itself. Set it to `false` when a lifecycle manager controls start-up order.

## First-build checklist

Points most likely to need attention, in the order you will meet them:

1. **rosdep key for libmosquitto.** `package.xml` uses `libmosquitto-dev`. If
   rosdep does not resolve it, install the system package by hand
   (`sudo apt install libmosquitto-dev`) and adjust the key.
2. **`vda5050_core` as a dependency.** The adapter links
   `vda5050_core::vda5050_core`, which relies on the core's
   `ament_export_targets` call. That path has never been exercised.
3. **rclcpp API details** written from memory of ROS 2 Jazzy:
   `rclcpp_action::create_client` with node interfaces, the
   `SendGoalOptions` callback signatures, `LifecycleNode::on_activate()`
   activating lifecycle publishers, `declare_parameter` with an empty
   string-array default.
4. **Run on a single-threaded executor.** The core is not thread-safe.
   `main.cpp` does this; do not load the node into a multi-threaded container.
5. **OFFLINE on shutdown.** `deactivate` publishes `OFFLINE` and disconnects
   immediately after. Whether the broker receives the message before the
   DISCONNECT has not been tested; if it does not, wait for the publish
   acknowledgement before disconnecting.

## Not implemented

- TLS for the MQTT connection
- `diagnostic_msgs` output
- Automated tests for this package (`launch_testing` against a fake
  `NavigateToPose` server and a real broker). All current tests are in
  `vda5050_core`.
- `NavigateThroughPoses` for smoother motion across several released nodes
