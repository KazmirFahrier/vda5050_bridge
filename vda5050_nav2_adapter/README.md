# vda5050_nav2_adapter

ROS 2 lifecycle node that makes a Nav2 robot a VDA 5050 vehicle.

**Status.** Builds on ROS 2 Jazzy without warnings. CI runs it against a real
Mosquitto broker and against the real Nav2 stack with Nav2's loopback
simulator as the robot. It has not been run with a physics simulator or on a
physical robot.

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

## Running it

Install the one dependency rosdep cannot resolve, then build as usual:

```bash
sudo apt-get install -y libmosquitto-dev
rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to vda5050_nav2_adapter
ros2 launch vda5050_nav2_adapter adapter.launch.py params_file:=/path/to/your.yaml
```

Run it on a single-threaded executor, as `main.cpp` does. The protocol core is
not thread-safe, so do not load the node into a multi-threaded container.

On Ctrl-C the node publishes `OFFLINE`, waits up to one second for the broker
to receive it, and disconnects. If the process is killed instead, the broker
publishes the `CONNECTIONBROKEN` last will.

## Integration test

`test/integration_test.sh` starts a Mosquitto broker, a static `map ->
base_link` transform, `test/fake_nav2_server.py` and the adapter, then plays
master control with `mosquitto_pub` and checks what the vehicle publishes:

| Step | Checked |
|---|---|
| Start-up | `ONLINE` on `connection`, a state message, pose from tf2 in `visualization` |
| `stateRequest` | Answered with a state |
| Order N0 -> N1 | Goal reaches the action server; state shows driving, then completed at N1 with no errors |
| Instant action `pick` | Accepted and started (`INITIALIZING`); a `FINISHED` report sent on `~/action_status` reaches master control. The content of `~/action_request` is not inspected |
| Malformed order | Reported as `validationError` |
| `factsheetRequest` | Factsheet published |
| SIGINT | `OFFLINE` published; last will not triggered |

## System test with real Nav2

`test/nav2_simulation_test.sh` launches `nav2_bringup`'s loopback simulation
(map server, planner, controller, behaviour-tree navigator and costmaps are the
real ones; only the robot is simulated) on the TurtleBot3 world map, then:

| Order | Checked |
|---|---|
| o1: A -> B -> C | Two legs driven; state reports B, then completion at C with no errors; reported position within 0.4 m of C |
| o2: C -> A, `startPause` then `stopPause` while driving | State shows `paused` and not driving; the leg does not finish during the pause; completes at A after resume with no errors |
| ofail: A -> X, with X off the map | Nav2 fails the goal; state shows a `FATAL` `navigationError`, not driving, X still listed ahead; no retry. `cancelOrder` clears it; a fresh order from the robot's position completes and the error list is empty again |
| o3: A -> C, `cancelOrder` while driving | Cancel reaches `FINISHED`; vehicle stopped, route cleared, no error raised, C not reached |

In the recorded CI runs the robot stopped 0.22 to 0.24 m from each target node.
That is Nav2's default goal tolerance (0.25 m), not an adapter error, but a
deployment needs to set it tighter than the fleet manager's `allowedDeviationXY`.

What the first real build found, for the record: a missing rosdep key for
libmosquitto, a dangling reference to a destroyed parameter temporary (caught by
`-Wdangling-reference`), and a retained `ONLINE` left behind on Ctrl-C.

## Not implemented

- TLS for the MQTT connection
- `diagnostic_msgs` output
- Unit tests for the individual adapter classes, and a test of a path that
  becomes blocked after the vehicle has started driving.
- `NavigateThroughPoses` for smoother motion across several released nodes
