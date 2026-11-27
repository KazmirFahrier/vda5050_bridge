#!/usr/bin/env bash
# Copyright 2026 vda5050_bridge contributors
# SPDX-License-Identifier: Apache-2.0
#
# System test against the real Nav2 stack.
#
# Real: the adapter, a Mosquitto broker, and Nav2 itself (map server, planner,
#       controller, behaviour-tree navigator, costmaps) from nav2_bringup.
# Simulated: the robot. Nav2's "loopback simulator" turns the velocity commands
#       Nav2 produces straight into motion (tf), with no physics or sensors.
#       So this proves the adapter drives Nav2 correctly; it says nothing about
#       localisation, obstacle avoidance or wheel slip.
#
# Scenario, on the TurtleBot3 world map, along the free strip at x = -2:
#   1. order o1: A -> B -> C (two legs)                       must complete
#   2. order o2: C -> A, paused and resumed while driving     must complete
#   3. order ofail: A -> X, where X is off the map            Nav2 must fail;
#        the vehicle must halt and report a FATAL navigationError
#      cancelOrder, then order orecover from where the robot
#        now stands back to A                                 must complete
#   4. order o3: A -> C, cancelled while driving              must stop cleanly

set -uo pipefail

PREFIX="uagv/v2/acme/amr-001"
PKG_PREFIX="$(ros2 pkg prefix vda5050_nav2_adapter)"
PARAMS="${PKG_PREFIX}/share/vda5050_nav2_adapter/config/adapter.yaml"
ADAPTER_BIN="${PKG_PREFIX}/lib/vda5050_nav2_adapter/vda5050_adapter"
# The loopback launch file is named differently across Nav2 releases; find it.
NAV2_LAUNCH_DIR="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch"
NAV2_LAUNCH="${NAV2_LAUNCH:-$(basename "$(ls "$NAV2_LAUNCH_DIR"/tb3_loopback* 2>/dev/null | head -n 1)")}"
if [ -z "$NAV2_LAUNCH" ]; then
  echo "FAIL: no tb3_loopback launch file in $NAV2_LAUNCH_DIR"; ls "$NAV2_LAUNCH_DIR"; exit 1
fi
LOG="$(mktemp)"
NAV2_LOG="$(mktemp)"
HEADER='"headerId":1,"timestamp":"2026-01-01T00:00:00.000Z","version":"2.0.0","manufacturer":"acme","serialNumber":"amr-001"'

# Waypoints (metres, map frame).
AX=-2.0; AY=-0.5
BX=-2.0; BY=0.0
CX=-2.0; CY=0.5

PIDS=()
cleanup() {
  # ros2 launch forwards SIGINT to everything it started.
  [ -n "${LAUNCH_PID:-}" ] && kill -INT "$LAUNCH_PID" 2>/dev/null
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null; done
  sleep 2
  # Stragglers. The patterns must not match this script's own name.
  pkill -f component_container 2>/dev/null
  pkill -f loopback_simulator 2>/dev/null
  return 0
}
trap cleanup EXIT

fail() {
  echo "FAIL: $1"
  echo "---- last messages on ${PREFIX}/state ----"
  grep "^${PREFIX}/state " "$LOG" | tail -n 4 | cut -c1-900
  echo "---- last Nav2 log lines ----"
  tail -n 60 "$NAV2_LOG" | cut -c1-300
  exit 1
}

wait_for() {  # wait_for <description> <timeout s> <extended regex>
  local description="$1" timeout="$2" pattern="$3"
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

echo "== starting broker and recorder =="
mosquitto -p 1883 >/dev/null 2>&1 & PIDS+=($!)
sleep 1
mosquitto_sub -v -t "${PREFIX}/#" >"$LOG" & PIDS+=($!)

echo "== starting Nav2 with the loopback simulator (${NAV2_LAUNCH}) =="
ros2 launch nav2_bringup "$NAV2_LAUNCH" use_rviz:=False >"$NAV2_LOG" 2>&1 &
LAUNCH_PID=$!

echo "waiting for Nav2 to become active..."
ACTIVE=0
for _ in $(seq 1 90); do
  if ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "active"; then
    ACTIVE=1
    break
  fi
  sleep 2
done
[ "$ACTIVE" = 1 ] || fail "Nav2 did not become active within 180 s"
echo "ok: Nav2 is active"

echo "== starting adapter =="
"$ADAPTER_BIN" --ros-args --params-file "$PARAMS" & ADAPTER_PID=$!
PIDS+=("$ADAPTER_PID")
wait_for "vehicle announces ONLINE" 20 "^${PREFIX}/connection .*\"connectionState\":\"ONLINE\""

echo "== placing the simulated robot at A (${AX}, ${AY}) =="
PLACED=0
for _ in 1 2 3 4 5; do
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: ${AX}, y: ${AY}}, orientation: {w: 1.0}}}}" >/dev/null 2>&1
  sleep 3
  if grep -Eq "^${PREFIX}/visualization .*\"positionInitialized\":true" "$LOG"; then
    PLACED=1
    break
  fi
done
[ "$PLACED" = 1 ] || fail "no map -> base_link transform after publishing the initial pose"
sleep 1.5
expect_position "adapter reports the robot at A" "$AX" "$AY" 0.3

echo "== 1. order o1: A -> B -> C =="
send order "{${HEADER},\"orderId\":\"o1\",\"orderUpdateId\":0,\"nodes\":[$(node A 0 $AX $AY),$(node B 2 $BX $BY),$(node C 4 $CX $CY)],\"edges\":[$(edge AB 1 A B),$(edge BC 3 B C)]}"
wait_for "order accepted, vehicle driving" 15 \
  "^${PREFIX}/state .*\"driving\":true.*\"orderId\":\"o1\""
wait_for "first leg done: reached B, C still ahead" 60 \
  "^${PREFIX}/state .*\"lastNodeId\":\"B\".*\"orderId\":\"o1\""
wait_for "order completed at C with no errors" 60 \
  "^${PREFIX}/state .*\"driving\":false.*\"errors\":\\[\\].*\"lastNodeId\":\"C\".*\"nodeStates\":\\[\\].*\"orderId\":\"o1\""
sleep 1.5
expect_position "robot really is at C" "$CX" "$CY" 0.4

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
expect_position "robot really is at A" "$AX" "$AY" 0.4

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
read -r HX HY < <(grep "^${PREFIX}/visualization " "$LOG" | tail -n 1 | python3 -c "
import json, sys
position = json.loads(sys.stdin.readline().split(' ', 1)[1])['agvPosition']
print(f\"{position['x']:.3f} {position['y']:.3f}\")")
echo "   robot is now at (${HX}, ${HY})"
send order "{${HEADER},\"orderId\":\"orecover\",\"orderUpdateId\":0,\"nodes\":[$(node H 0 $HX $HY),$(node A 2 $AX $AY)],\"edges\":[$(edge HA 1 H A)]}"
wait_for "vehicle recovers: new order completes at A, error list cleared" 90 \
  "^${PREFIX}/state .*\"driving\":false.*\"errors\":\[\].*\"lastNodeId\":\"A\".*\"nodeStates\":\[\].*\"orderId\":\"orecover\""
sleep 1.5
expect_position "robot really is at A" "$AX" "$AY" 0.4

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

echo "== clean shutdown =="
kill -INT "$ADAPTER_PID"
wait_for "vehicle announces OFFLINE" 10 "^${PREFIX}/connection .*\"connectionState\":\"OFFLINE\""

echo "PASS: Nav2 simulation test"
exit 0
