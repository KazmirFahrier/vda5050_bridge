#!/usr/bin/env bash
# Copyright 2026 vda5050_bridge contributors
# SPDX-License-Identifier: Apache-2.0
#
# System test against the real Nav2 stack. Runs in two modes.
#
# Default (loopback):
#   Real: the adapter, a Mosquitto broker, and Nav2 itself (map server,
#         planner, controller, behaviour-tree navigator, costmaps).
#   Simulated: the robot. Nav2's "loopback simulator" turns velocity commands
#         straight into motion (tf), with no physics or sensors.
#
# SIMULATOR=gazebo:
#   Adds the Gazebo physics simulator with a TurtleBot3: a simulated lidar,
#   wheel odometry, and AMCL localising the robot on the map from the scans.
#   The pillars and walls of the world are real obstacles the lidar sees.
#
# Scenario, on the TurtleBot3 world map:
#   1. order o1: A -> B -> C (two legs)                       must complete
#   2. order o2: C -> A, paused and resumed while driving     must complete
#   3. order ofail: A -> X, where X is off the map            Nav2 must fail;
#        the vehicle must halt and report a FATAL navigationError
#      cancelOrder, then order orecover from where the robot
#        now stands back to A                                 must complete
#   4. order o3: A -> C, cancelled while driving              must stop cleanly
#   5. (Gazebo only) order oblock: A -> C, with a box placed in
#        the path just before the order is sent. The box is not
#        on the map; only the lidar can see it.                either Nav2 gets
#        round it and the order completes, or Nav2 gives up and
#        the vehicle reports a navigationError. Both are valid;
#        the test checks the adapter's state is right for
#        whichever happened, and prints which.
#   6. order o4: back to A, then to D, which lies beyond a
#        row of pillars                                       must complete

set -uo pipefail

PREFIX="uagv/v2/acme/amr-001"
PKG_PREFIX="$(ros2 pkg prefix vda5050_nav2_adapter)"
PARAMS="${PKG_PREFIX}/share/vda5050_nav2_adapter/config/adapter.yaml"
ADAPTER_BIN="${PKG_PREFIX}/lib/vda5050_nav2_adapter/vda5050_adapter"
SIMULATOR="${SIMULATOR:-loopback}"
NAV2_LAUNCH_DIR="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch"
if [ "$SIMULATOR" = "gazebo" ]; then
  NAV2_LAUNCH="tb3_simulation_launch.py"
  NAV2_ARGS="headless:=True use_rviz:=False"
  # Gazebo renders the lidar with OpenGL. Without a GPU that needs a virtual
  # display and software rendering.
  LAUNCH_WRAPPER="xvfb-run -a"
  export LIBGL_ALWAYS_SOFTWARE=1
  SCALE=4              # physics on a CI machine runs slower than real time
  NAV2_START_TIMEOUT=420
  TOLERANCE=0.5        # Nav2 goal tolerance (0.25 m) plus localisation error
else
  # The loopback launch file is named differently across Nav2 releases; find it.
  NAV2_LAUNCH="$(basename "$(ls "$NAV2_LAUNCH_DIR"/tb3_loopback* 2>/dev/null | head -n 1)")"
  NAV2_ARGS="use_rviz:=False"
  LAUNCH_WRAPPER=""
  SCALE=1
  NAV2_START_TIMEOUT=180
  TOLERANCE=0.4
fi
if [ -z "$NAV2_LAUNCH" ] || [ ! -f "$NAV2_LAUNCH_DIR/$NAV2_LAUNCH" ]; then
  echo "FAIL: launch file not found in $NAV2_LAUNCH_DIR"; ls "$NAV2_LAUNCH_DIR"; exit 1
fi
LOG="$(mktemp)"
NAV2_LOG="$(mktemp)"
HEADER='"headerId":1,"timestamp":"2026-01-01T00:00:00.000Z","version":"2.0.0","manufacturer":"acme","serialNumber":"amr-001"'

# Waypoints (metres, map frame).
AX=-2.0; AY=-0.5
BX=-2.0; BY=0.0
CX=-2.0; CY=0.5
DX=0.55; DY=-0.55   # centre of four pillars; reaching it means passing between two

PIDS=()
cleanup() {
  # ros2 launch forwards SIGINT to everything it started.
  [ -n "${LAUNCH_PID:-}" ] && kill -INT "$LAUNCH_PID" 2>/dev/null
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null; done
  sleep 2
  # Stragglers. The patterns must not match this script's own name.
  pkill -f component_container 2>/dev/null
  pkill -f loopback_simulator 2>/dev/null
  pkill -f "gz sim" 2>/dev/null
  pkill -f parameter_bridge 2>/dev/null
  pkill -f Xvfb 2>/dev/null
  return 0
}
trap cleanup EXIT

fail() {
  echo "FAIL: $1"
  echo "---- last messages on ${PREFIX}/state ----"
  grep "^${PREFIX}/state " "$LOG" | tail -n 4 | cut -c1-900
  echo "---- last Nav2 log lines ----"
  tail -n 120 "$NAV2_LOG" | cut -c1-300
  exit 1
}

wait_for() {  # wait_for <description> <timeout s> <extended regex>
  local description="$1" timeout="$(($2 * SCALE))" pattern="$3"
  for _ in $(seq 1 $((timeout * 5))); do
    if grep -Eq "$pattern" "$LOG"; then
      echo "ok: $description"
      return 0
    fi
    sleep 0.2
  done
  fail "$description (no message matched: $pattern)"
}

send() { mosquitto_pub -t "${PREFIX}/$1" -m "$2"; }

node() {  # node <id> <sequenceId> <x> <y>
  echo "{\"nodeId\":\"$1\",\"sequenceId\":$2,\"released\":true,\"actions\":[],\"nodePosition\":{\"x\":$3,\"y\":$4,\"mapId\":\"map\"}}"
}
edge() {  # edge <id> <sequenceId> <from> <to>
  echo "{\"edgeId\":\"$1\",\"sequenceId\":$2,\"released\":true,\"startNodeId\":\"$3\",\"endNodeId\":\"$4\",\"actions\":[]}"
}
instant() {  # instant <actionId> <actionType>
  send instantActions "{${HEADER},\"actions\":[{\"actionType\":\"$2\",\"actionId\":\"$1\",\"blockingType\":\"HARD\"}]}"
}

# Position reported in the newest visualization message must be near (x, y).
expect_position() {  # expect_position <description> <x> <y> <tolerance>
  if grep "^${PREFIX}/visualization " "$LOG" | tail -n 1 | python3 -c "
import json, math, sys
line = sys.stdin.readline()
position = json.loads(line.split(' ', 1)[1])['agvPosition']
distance = math.hypot(position['x'] - ($2), position['y'] - ($3))
print(f\"   reported position ({position['x']:.2f}, {position['y']:.2f}), {distance:.2f} m from target\")
sys.exit(0 if distance <= $4 else 1)
"; then
    echo "ok: $1"
  else
    fail "$1"
  fi
}

current_position() {  # prints "x y" from the newest visualization message
  grep "^${PREFIX}/visualization " "$LOG" | tail -n 1 | python3 -c "
import json, sys
position = json.loads(sys.stdin.readline().split(' ', 1)[1])['agvPosition']
print(f\"{position['x']:.3f} {position['y']:.3f}\")"
}

echo "== starting broker and recorder =="
mosquitto -p 1883 >/dev/null 2>&1 & PIDS+=($!)
sleep 1
mosquitto_sub -v -t "${PREFIX}/#" >"$LOG" & PIDS+=($!)

set_initial_pose() {
  # Loopback: places the robot. Gazebo: tells AMCL where the robot (which is
  # spawned at A) is, so localisation can start.
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: ${AX}, y: ${AY}}, orientation: {w: 1.0}}}}" >/dev/null 2>&1
}

echo "== starting Nav2, simulator: ${SIMULATOR} (${NAV2_LAUNCH}) =="
# shellcheck disable=SC2086
$LAUNCH_WRAPPER ros2 launch nav2_bringup "$NAV2_LAUNCH" $NAV2_ARGS >"$NAV2_LOG" 2>&1 &
LAUNCH_PID=$!

echo "waiting for Nav2 to become active..."
ACTIVE=0
for i in $(seq 1 $((NAV2_START_TIMEOUT / 2))); do
  # Anchored on purpose: a plain "active" would also match "inactive".
  if ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active"; then
    ACTIVE=1
    break
  fi
  # With AMCL, Nav2 cannot finish activating until it knows where the robot is.
  if [ $((i % 5)) = 0 ]; then
    set_initial_pose
  fi
  sleep 2
done
[ "$ACTIVE" = 1 ] || fail "Nav2 did not become active within ${NAV2_START_TIMEOUT} s"
echo "ok: Nav2 is active"

if [ "$SIMULATOR" = "gazebo" ]; then
  echo "== checking that the simulated sensors and localisation are really running =="
  pgrep -f "gz sim" >/dev/null || fail "Gazebo (gz sim) is not running"
  echo "ok: Gazebo process is running"

  ros2 node list 2>/dev/null | grep -q "^/amcl$" || fail "AMCL node is not running"
  echo "ok: AMCL node is running"

  # One lidar scan: it must contain real range readings, i.e. the simulated
  # lidar is being rendered and actually sees the walls and pillars.
  timeout 60 ros2 topic echo /scan --once --field ranges 2>/dev/null | python3 -c "
import math, re, sys
values = [float(v) for v in re.findall(r'-?[0-9]+\.[0-9]+(?:e-?[0-9]+)?|inf|nan', sys.stdin.read())]
hits = [v for v in values if math.isfinite(v) and v > 0.0]
print(f'   lidar scan: {len(values)} beams, {len(hits)} with a finite range'
      + (f', nearest obstacle {min(hits):.2f} m' if hits else ''))
sys.exit(0 if len(hits) >= 20 else 1)" || fail "no usable lidar scan on /scan"
  echo "ok: lidar is publishing real ranges"

  # Simulation time must be advancing.
  sim_seconds() {  # ros2 topic echo can print notices too; keep only the number
    timeout 20 ros2 topic echo /clock --once --field clock.sec 2>/dev/null | grep -E '^[0-9]+$' | head -n 1
  }
  T1="$(sim_seconds)"
  sleep 3
  T2="$(sim_seconds)"
  echo "   simulation clock: ${T1} s -> ${T2} s"
  [ -n "$T1" ] && [ -n "$T2" ] && [ "$T2" -gt "$T1" ] || fail "simulation time is not advancing"
  echo "ok: simulation time is advancing"
fi

echo "== starting adapter =="
"$ADAPTER_BIN" --ros-args --params-file "$PARAMS" & ADAPTER_PID=$!
PIDS+=("$ADAPTER_PID")
wait_for "vehicle announces ONLINE" 20 "^${PREFIX}/connection .*\"connectionState\":\"ONLINE\""

echo "== initial pose: A (${AX}, ${AY}) =="
PLACED=0
for _ in 1 2 3 4 5; do
  set_initial_pose
  sleep 3
  if grep -Eq "^${PREFIX}/visualization .*\"positionInitialized\":true" "$LOG"; then
    PLACED=1
    break
  fi
done
[ "$PLACED" = 1 ] || fail "no map -> base_link transform after publishing the initial pose"
sleep 1.5
expect_position "adapter reports the robot at A" "$AX" "$AY" "$TOLERANCE"

echo "== 1. order o1: A -> B -> C =="
LEG1_T0=$(date +%s)
LEG1_STALLS_BEFORE=$(grep -c "Failed to make progress" "$NAV2_LOG")
send order "{${HEADER},\"orderId\":\"o1\",\"orderUpdateId\":0,\"nodes\":[$(node A 0 $AX $AY),$(node B 2 $BX $BY),$(node C 4 $CX $CY)],\"edges\":[$(edge AB 1 A B),$(edge BC 3 B C)]}"
wait_for "order accepted, vehicle driving" 15 \
  "^${PREFIX}/state .*\"driving\":true.*\"orderId\":\"o1\""
wait_for "first leg done: reached B, C still ahead" 60 \
  "^${PREFIX}/state .*\"lastNodeId\":\"B\".*\"orderId\":\"o1\""
LEG1_SECONDS=$(( $(date +%s) - LEG1_T0 ))
echo "   first leg took ${LEG1_SECONDS} s"
# Regression guard: this leg used to take about 45 s (see docs/testing.md).
[ "$LEG1_SECONDS" -le $((20 * SCALE)) ] || fail "first leg took ${LEG1_SECONDS} s"
LEG1_STALLS=$(( $(grep -c "Failed to make progress" "$NAV2_LOG") - LEG1_STALLS_BEFORE ))
[ "$LEG1_STALLS" = 0 ] || fail "Nav2's controller stalled ${LEG1_STALLS} time(s) on the first leg"
echo "ok: first leg completed promptly, controller never stalled"
wait_for "order completed at C with no errors" 60 \
  "^${PREFIX}/state .*\"driving\":false.*\"errors\":\\[\\].*\"lastNodeId\":\"C\".*\"nodeStates\":\\[\\].*\"orderId\":\"o1\""
sleep 1.5
expect_position "robot really is at C" "$CX" "$CY" "$TOLERANCE"

echo "== 2. order o2: C -> A with pause and resume =="
send order "{${HEADER},\"orderId\":\"o2\",\"orderUpdateId\":0,\"nodes\":[$(node C 0 $CX $CY),$(node A 2 $AX $AY)],\"edges\":[$(edge CA 1 C A)]}"
wait_for "vehicle driving" 15 "^${PREFIX}/state .*\"driving\":true.*\"orderId\":\"o2\""
instant p1 startPause
wait_for "state shows paused, not driving" 10 \
  "^${PREFIX}/state .*\"driving\":false.*\"orderId\":\"o2\".*\"paused\":true"
sleep 3
if grep -Eq "^${PREFIX}/state .*\"lastNodeId\":\"A\".*\"orderId\":\"o2\"" "$LOG"; then
  fail "vehicle reached A while paused"
fi
echo "ok: vehicle did not finish the leg while paused"
instant p2 stopPause
wait_for "order completed at A after resume, no errors" 90 \
  "^${PREFIX}/state .*\"driving\":false.*\"errors\":\\[\\].*\"lastNodeId\":\"A\".*\"nodeStates\":\\[\\].*\"orderId\":\"o2\".*\"paused\":false"
sleep 1.5
expect_position "robot really is at A" "$AX" "$AY" "$TOLERANCE"

echo "== 3. order ofail: A -> X, a node Nav2 cannot reach =="
# X lies far outside the map, so Nav2 can never produce a path and gives up.
send order "{${HEADER},\"orderId\":\"ofail\",\"orderUpdateId\":0,\"nodes\":[$(node A 0 $AX $AY),$(node X 2 50.0 50.0)],\"edges\":[$(edge AX 1 A X)]}"
wait_for "Nav2 gave up: FATAL navigationError, halted, X still listed ahead" 240 \
  "^${PREFIX}/state .*\"driving\":false.*\"errorLevel\":\"FATAL\".*\"errorType\":\"navigationError\".*\"lastNodeId\":\"A\".*\"nodeStates\":\[\{.*\"orderId\":\"ofail\""
grep -E "^${PREFIX}/state .*\"errorType\":\"navigationError\"" "$LOG" | tail -n 1 | python3 -c "
import json, sys
state = json.loads(sys.stdin.readline().split(' ', 1)[1])
print('   reported error:', state['errors'][0]['errorDescription'])"
sleep 3
if grep -E "^${PREFIX}/state .*\"orderId\":\"ofail\"" "$LOG" | tail -n 1 | grep -q '"driving":true'; then
  fail "vehicle retried the failed leg on its own"
fi
echo "ok: no automatic retry after the failure"

instant c0 cancelOrder
wait_for "cancel clears the failed order" 15 \
  "^${PREFIX}/state .*\"actionId\":\"c0\",\"actionStatus\":\"FINISHED\".*\"driving\":false.*\"nodeStates\":\[\].*\"orderId\":\"ofail\""

# Nav2's recovery behaviours (spin, back up) may have moved the robot, so the
# next order starts from wherever it reports being now.
read -r HX HY < <(current_position)
echo "   robot is now at (${HX}, ${HY})"
send order "{${HEADER},\"orderId\":\"orecover\",\"orderUpdateId\":0,\"nodes\":[$(node H 0 $HX $HY),$(node A 2 $AX $AY)],\"edges\":[$(edge HA 1 H A)]}"
wait_for "vehicle recovers: new order completes at A, error list cleared" 90 \
  "^${PREFIX}/state .*\"driving\":false.*\"errors\":\[\].*\"lastNodeId\":\"A\".*\"nodeStates\":\[\].*\"orderId\":\"orecover\""
sleep 1.5
expect_position "robot really is at A" "$AX" "$AY" "$TOLERANCE"

echo "== 4. order o3: A -> C, cancelled while driving =="
send order "{${HEADER},\"orderId\":\"o3\",\"orderUpdateId\":0,\"nodes\":[$(node A 0 $AX $AY),$(node C 2 $CX $CY)],\"edges\":[$(edge AC 1 A C)]}"
wait_for "vehicle driving" 15 "^${PREFIX}/state .*\"driving\":true.*\"orderId\":\"o3\""
sleep 1
instant c1 cancelOrder
wait_for "cancel finished: stopped, route cleared, no error raised" 20 \
  "^${PREFIX}/state .*\"actionId\":\"c1\",\"actionStatus\":\"FINISHED\".*\"driving\":false.*\"errors\":\\[\\].*\"nodeStates\":\\[\\].*\"orderId\":\"o3\""
if grep -Eq "^${PREFIX}/state .*\"lastNodeId\":\"C\".*\"orderId\":\"o3\"" "$LOG"; then
  fail "vehicle reached C although the order was cancelled"
fi
echo "ok: vehicle did not reach C"

return_to_a() {  # return_to_a <orderId>
  sleep 2
  read -r HX HY < <(current_position)
  send order "{${HEADER},\"orderId\":\"$1\",\"orderUpdateId\":0,\"nodes\":[$(node H 0 $HX $HY),$(node A 2 $AX $AY)],\"edges\":[$(edge HA 1 H A)]}"
  wait_for "vehicle back at A" 90 \
    "^${PREFIX}/state .*\"driving\":false.*\"errors\":\[\].*\"lastNodeId\":\"A\".*\"nodeStates\":\[\].*\"orderId\":\"$1\""
  sleep 1.5
}

if [ "$SIMULATOR" = "gazebo" ]; then
  echo "== 5. order oblock: A -> C with an unmapped box in the path =="
  BOX_X=-2.0; BOX_Y=0.05
  return_to_a oreturn1
  WORLD="$(gz topic -l 2>/dev/null | grep -o '^/world/[^/]*' | head -n 1 | cut -d/ -f3)"
  [ -n "$WORLD" ] || fail "could not determine the Gazebo world name"
  ros2 run ros_gz_sim create -world "$WORLD" -name obstacle_box -x "$BOX_X" -y "$BOX_Y" -z 0.15 -string \
    '<sdf version="1.8"><model name="obstacle_box"><static>true</static><link name="link"><collision name="c"><geometry><box><size>0.3 0.3 0.3</size></box></geometry></collision><visual name="v"><geometry><box><size>0.3 0.3 0.3</size></box></geometry></visual></link></model></sdf>' \
    >/dev/null 2>&1 || fail "could not spawn the obstacle in Gazebo world '${WORLD}'"
  echo "   box (0.3 m cube) placed at (${BOX_X}, ${BOX_Y}) in world '${WORLD}', on the straight line from A to C"
  sleep 3   # let the lidar pick it up before the order arrives
  VIS_MARK=$(wc -l <"$LOG")
  STALLS_BEFORE=$(grep -c "Failed to make progress" "$NAV2_LOG")
  BLOCK_T0=$(date +%s)
  send order "{${HEADER},\"orderId\":\"oblock\",\"orderUpdateId\":0,\"nodes\":[$(node A 0 $AX $AY),$(node C 2 $CX $CY)],\"edges\":[$(edge AC 1 A C)]}"
  wait_for "vehicle driving" 15 "^${PREFIX}/state .*\"driving\":true.*\"orderId\":\"oblock\""

  DONE="^${PREFIX}/state .*\"driving\":false.*\"errors\":\[\].*\"lastNodeId\":\"C\".*\"nodeStates\":\[\].*\"orderId\":\"oblock\""
  GAVE_UP="^${PREFIX}/state .*\"driving\":false.*\"errorLevel\":\"FATAL\".*\"errorType\":\"navigationError\".*\"orderId\":\"oblock\""
  OUTCOME=""
  for _ in $(seq 1 $((300 * 5))); do
    if grep -Eq "$DONE" "$LOG"; then OUTCOME="completed"; break; fi
    if grep -Eq "$GAVE_UP" "$LOG"; then OUTCOME="gave up"; break; fi
    sleep 0.2
  done
  [ -n "$OUTCOME" ] || fail "order oblock neither completed nor failed within 300 s"
  sleep 1.5
  # How close did the robot's reported position come to the centre of the box?
  tail -n +"$((VIS_MARK + 1))" "$LOG" | grep "^${PREFIX}/visualization " | python3 -c "
import json, math, sys
closest = min(math.hypot(p['x'] - ($BOX_X), p['y'] - ($BOX_Y))
              for p in (json.loads(line.split(' ', 1)[1])['agvPosition'] for line in sys.stdin))
print(f'   closest approach to the centre of the box: {closest:.2f} m (box half-width 0.15 m)')"
  echo "   outcome: Nav2 ${OUTCOME} after $(( $(date +%s) - BLOCK_T0 )) s, with $(( $(grep -c "Failed to make progress" "$NAV2_LOG") - STALLS_BEFORE )) controller stall(s)"
  if [ "$OUTCOME" = "completed" ]; then
    expect_position "order completed: robot got round the box and is at C" "$CX" "$CY" "$TOLERANCE"
  else
    echo "ok: vehicle halted and reported a FATAL navigationError"
    instant cblock cancelOrder
    wait_for "cancel clears the blocked order" 15 \
      "^${PREFIX}/state .*\"actionId\":\"cblock\",\"actionStatus\":\"FINISHED\".*\"nodeStates\":\[\].*\"orderId\":\"oblock\""
  fi
  # Take the box away again so the rest of the test runs in the original world.
  gz service -s "/world/${WORLD}/remove" --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
    --timeout 3000 --req 'name: "obstacle_box" type: MODEL' >/dev/null 2>&1
  sleep 3
fi

echo "== 6. order o4: to D (${DX}, ${DY}), beyond a row of pillars =="
# Start from A so the approach to the gap between the pillars is the same on
# every run, wherever the earlier scenarios left the robot.
return_to_a oreturn2
read -r HX HY < <(current_position)
echo "   starting from (${HX}, ${HY})"
STALLS_BEFORE=$(grep -c "Failed to make progress" "$NAV2_LOG")
D_T0=$(date +%s)
send order "{${HEADER},\"orderId\":\"o4\",\"orderUpdateId\":0,\"nodes\":[$(node H 0 $HX $HY),$(node D 2 $DX $DY)],\"edges\":[$(edge HD 1 H D)]}"
wait_for "vehicle driving" 15 "^${PREFIX}/state .*\"driving\":true.*\"orderId\":\"o4\""
# Generous timeout: in a tight gap Nav2 may need its recovery behaviours
# (spin, wait, back up) before it gets through. That is Nav2 doing its job; the
# adapter's part is to keep waiting and report the order as still in progress.
wait_for "order completed at D with no errors" 120 \
  "^${PREFIX}/state .*\"driving\":false.*\"errors\":\[\].*\"lastNodeId\":\"D\".*\"nodeStates\":\[\].*\"orderId\":\"o4\""
echo "   reached D in $(( $(date +%s) - D_T0 )) s, with $(( $(grep -c "Failed to make progress" "$NAV2_LOG") - STALLS_BEFORE )) controller stall(s) on the way"
sleep 1.5
expect_position "robot really is at D, between the pillars" "$DX" "$DY" "$TOLERANCE"

if [ "$SIMULATOR" = "gazebo" ]; then
  # AMCL's own estimate, to show the reported position came from localisation.
  timeout 20 ros2 topic echo /amcl_pose --once --qos-durability transient_local \
    --qos-reliability reliable --field pose.pose.position 2>/dev/null | tr '\n' ' ' | \
    sed 's/^/   AMCL pose estimate: /; s/ ---.*//'
  echo
fi

echo "== Nav2 log summary =="
echo "   errors: $(grep -c "\[ERROR\]" "$NAV2_LOG")   warnings: $(grep -c "\[WARN\]" "$NAV2_LOG")   controller stalls: $(grep -c "Failed to make progress" "$NAV2_LOG")"
grep "\[ERROR\]" "$NAV2_LOG" | sed 's/^\[[^]]*\] //' | cut -c1-200 | sort | uniq -c | sort -rn | head -n 12

echo "== clean shutdown =="
kill -INT "$ADAPTER_PID"
wait_for "vehicle announces OFFLINE" 10 "^${PREFIX}/connection .*\"connectionState\":\"OFFLINE\""

echo "PASS: Nav2 simulation test (${SIMULATOR})"
exit 0
