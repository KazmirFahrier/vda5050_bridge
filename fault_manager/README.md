# fault_manager

ROS 2 node that decides which reported conditions are real faults, publishes
what is wrong now, keeps a history across restarts, and records a rosbag2 bag
of the seconds before and after a fault.

The logic is in `fault_core`; this package adds the ROS interfaces and the
flight recorder. Concepts, design and limits:
[docs/fault_management.md](../docs/fault_management.md).

## Interfaces

| Kind | Name | Type |
|---|---|---|
| Subscription | `faults/report` | `fault_msgs/msg/FaultReport` |
| Subscription | `/diagnostics` (parameter) | `diagnostic_msgs/msg/DiagnosticArray` |
| Publisher | `faults/active` (latched) | `fault_msgs/msg/FaultArray` |
| Publisher | `faults/events` | `fault_msgs/msg/FaultEvent` |
| Service | `faults/acknowledge` | `fault_msgs/srv/AcknowledgeFault` |
| Service | `faults/get_history` | `fault_msgs/srv/GetFaultHistory` |

## Files

- `config/fault_catalog.json`: example catalogue. Every fault a robot can have
  is listed here with its severity, debounce times, latching and watchdog.
- `config/fault_manager.yaml`: every parameter, with a comment.
- `test/fault_system_test.py`: end-to-end test with the adapter, the gateway
  and a real broker.

## Run

```bash
ros2 launch fault_manager fault_manager.launch.py \
  catalog_file:=/path/to/catalog.json params_file:=/path/to/params.yaml
```

The node exits with an error if the catalogue cannot be read.
