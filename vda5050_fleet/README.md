# vda5050_fleet

Two programs, no ROS at run time:

- `fleet_manager`: the fleet manager of `fleet_core` behind an MQTT
  connection. It talks VDA 5050 to the vehicles and takes tasks as JSON
  commands.
- `sim_vehicle`: a VDA 5050 vehicle without a robot. The protocol side is the
  complete `vda5050_core` (the code the Nav2 adapter runs); the body is a
  point that moves in a straight line. It supports the actions `pick`, `drop`
  and `wait` (they take `--action-seconds`) and `fail` (which fails).

What the fleet manager does, how, and where its limits are:
[docs/fleet_management.md](../docs/fleet_management.md).

## Build and run

```bash
sudo apt-get install -y libmosquitto-dev mosquitto mosquitto-clients
cmake -S vda5050_fleet -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build

mosquitto -p 1883 &
./build/fleet_manager --roadmap vda5050_fleet/config/demo_roadmap.json &
./build/sim_vehicle --serial v1 --x 0 --y -2 --map-id warehouse &
./build/sim_vehicle --serial v2 --x 4 --y -2 --map-id warehouse &
./build/sim_vehicle --serial v3 --x 8 --y -2 --map-id warehouse &

mosquitto_sub -v -t 'fleet/v1/events' -t 'fleet/v1/response' &
mosquitto_pub -t fleet/v1/command -m \
  '{"command":"submit","requestId":"1","stops":[{"node":"A1","actions":[{"actionType":"pick"}]},{"node":"C5","actions":[{"actionType":"drop"}]}]}'
```

Under colcon, both are installed in `lib/vda5050_fleet`
(`ros2 run vda5050_fleet fleet_manager --roadmap ...`).

## Topics

| Topic | Direction | Content |
|---|---|---|
| `uagv/v2/<manufacturer>/<serial>/order`, `.../instantActions` | to vehicles | VDA 5050 |
| `uagv/v2/+/+/state`, `.../connection` | from vehicles | VDA 5050 |
| `fleet/v1/command` | to the fleet manager | `submit`, `cancel`, `resetVehicle` |
| `fleet/v1/response` | from | One answer per command, matched by `requestId` |
| `fleet/v1/state` | from, retained | Vehicles, tasks, counters |
| `fleet/v1/events` | from | One event per message |
| `fleet/v1/connection` | from, retained | `ONLINE` / `OFFLINE` (last will) |

## Options

`fleet_manager --roadmap <file> [--name value ...]`

| Option | Default | Meaning |
|---|---|---|
| `host`, `port` | `localhost`, `1883` | The broker |
| `client-id` | `fleet-manager` | MQTT client id |
| `username` | none | The password is read from the environment variable `FLEET_MQTT_PASSWORD` |
| `interface`, `version` | `uagv`, `2.0.0` | First parts of the VDA 5050 topic names, and the version in headers |
| `api` | `fleet/v1` | Prefix of the command and status topics |
| `vehicle-radius` | 0.4 | Metres. All vehicles have one size |
| `safety-margin` | 0.2 | Metres between two vehicles' outlines |
| `attach-tolerance` | 0.25 | How far from a node a vehicle may stand and count as being there |
| `release-ahead` | 3 | Edges granted ahead of a vehicle |
| `horizon` | 3 | Further edges shown to the vehicle but not released |
| `state-timeout` | 60 | Seconds of silence after which a vehicle counts as lost |
| `discovery-grace` | 10 | Seconds the fleet waits for a vehicle that is online but not located |
| `default-speed` | 1.0 | m/s assumed on edges without `maxSpeed` (for planning only) |
| `status-interval` | 1.0 | Seconds between state messages |

`sim_vehicle --serial <name>`: `manufacturer` (`sim`), `x`, `y`, `map-id`
(`map`; must be the roadmap's `mapId`), `speed` (1.0), `action-seconds` (2.0),
`host`, `port`, `interface`, `version`, `state-interval` (5.0),
`visualization-interval` (0.1).

## Test

```bash
python3 vda5050_fleet/test/fleet_system_test.py build vda5050_fleet/config/demo_roadmap.json
```

Starts its own broker on port 18831, the fleet manager and four vehicles.
Eleven tasks, one cancelled under way, one vehicle killed in the middle of a
task and started again. Fails if two vehicles ever come closer than they
may. Runs in CI.

## What is not here

No TLS, no access control (use the broker's), no persistence across a
restart, one fleet manager only. See the limits in
[docs/fleet_management.md](../docs/fleet_management.md#limits).
