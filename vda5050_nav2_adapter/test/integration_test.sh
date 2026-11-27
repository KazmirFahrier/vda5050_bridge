#!/usr/bin/env bash
# Copyright 2026 vda5050_bridge contributors
# SPDX-License-Identifier: Apache-2.0
#
# End-to-end test of the adapter executable.
#
# Real: the adapter node (started through its launch file), a Mosquitto
#       broker, tf2, the ROS 2 action protocol.
# Fake: Nav2 (fake_nav2_server.py succeeds every goal after a few seconds) and
#       the robot pose (a static transform map -> base_link at the origin).
#
# Besides the normal message flow it covers three connection events:
#   - the broker dies and comes back while an order is being driven
#   - the adapter is stopped cleanly (must announce OFFLINE)
#   - the adapter is killed (the broker must publish the last will)
#
# This script plays master control with mosquitto_pub and checks what the
# vehicle publishes. Run it with ROS 2 and the workspace sourced.

set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PREFIX="uagv/v2/acme/amr-001"
PKG_PREFIX="$(ros2 pkg prefix vda5050_nav2_adapter)"
PARAMS="${PKG_PREFIX}/share/vda5050_nav2_adapter/config/adapter.yaml"
ADAPTER_BIN="${PKG_PREFIX}/lib/vda5050_nav2_adapter/vda5050_adapter"
LOG="$(mktemp)"
HEADER='"headerId":1,"timestamp":"2026-01-01T00:00:00.000Z","version":"2.0.0","manufacturer":"acme","serialNumber":"amr-001"'

PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null; done
}
trap cleanup EXIT

fail() {
  echo "FAIL: $1"
  echo "---- last messages seen on ${PREFIX}/# ----"
  tail -n 15 "$LOG" | cut -c1-600
  exit 1
}

# wait_for <description> <timeout seconds> <extended regex matched against "topic payload" lines>
wait_for() {
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

# Like wait_for, but only looks at messages recorded after `mark` was called.
MARK=0
mark() { MARK=$(wc -l <"$LOG"); }
wait_for_new() {
  local description="$1" timeout="$2" pattern="$3"
  for _ in $(seq 1 $((timeout * 5))); do
    if tail -n +"$((MARK + 1))" "$LOG" | grep -Eq "$pattern"; then
      echo "ok: $description"
      return 0
    fi
    sleep 0.2
  done
  fail "$description (no new message matched: $pattern)"
}

BROKER_LOG="$(mktemp)"
start_broker() { mosquitto -v -p 1883 >>"$BROKER_LOG" 2>&1 & BROKER_PID=$!; PIDS+=("$BROKER_PID"); sleep 1; }
start_recorder() { mosquitto_sub -v -t "${PREFIX}/#" >>"$LOG" & RECORDER_PID=$!; PIDS+=("$RECORDER_PID"); }

send() {  # send <topic> <payload>
  mosquitto_pub -t "${PREFIX}/$1" -m "$2"
}

echo "== starting broker, recorder, fake robot =="
start_broker
start_recorder
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id base_link \
  >/dev/null 2>&1 & PIDS+=($!)
# Each "drive" takes 4 s, long enough to take the broker away in the middle.
FAKE_NAV_DELAY=4 python3 "${HERE}/fake_nav2_server.py" & PIDS+=($!)

echo "== starting adapter through its launch file =="
ros2 launch vda5050_nav2_adapter adapter.launch.py params_file:="$PARAMS" & ADAPTER_PID=$!
PIDS+=("$ADAPTER_PID")

wait_for "vehicle announces ONLINE" 20 \
  "^${PREFIX}/connection .*\"connectionState\":\"ONLINE\""
wait_for "a state message follows the connection" 10 \
  "^${PREFIX}/state "
wait_for "pose from tf2 appears in visualization" 20 \
  "^${PREFIX}/visualization .*\"positionInitialized\":true"

echo "== instant action: stateRequest =="
send instantActions "{${HEADER},\"actions\":[{\"actionType\":\"stateRequest\",\"actionId\":\"s1\",\"blockingType\":\"NONE\"}]}"
wait_for "stateRequest is answered with a state" 10 \
  "^${PREFIX}/state .*\"actionId\":\"s1\",\"actionStatus\":\"FINISHED\""

echo "== order: N0 -> N1, driven through the NavigateToPose action =="
send order "{${HEADER},\"orderId\":\"o1\",\"orderUpdateId\":0,\"nodes\":[\
{\"nodeId\":\"N0\",\"sequenceId\":0,\"released\":true,\"actions\":[],\"nodePosition\":{\"x\":0,\"y\":0,\"mapId\":\"map\"}},\
{\"nodeId\":\"N1\",\"sequenceId\":2,\"released\":true,\"actions\":[],\"nodePosition\":{\"x\":1,\"y\":0,\"mapId\":\"map\"}}],\
\"edges\":[{\"edgeId\":\"E0\",\"sequenceId\":1,\"released\":true,\"startNodeId\":\"N0\",\"endNodeId\":\"N1\",\"actions\":[]}]}"
wait_for "order accepted and vehicle driving" 10 \
  "^${PREFIX}/state .*\"driving\":true.*\"orderId\":\"o1\""
wait_for "order completed at N1 with no errors" 15 \
  "^${PREFIX}/state .*\"driving\":false.*\"errors\":\\[\\].*\"lastNodeId\":\"N1\".*\"nodeStates\":\\[\\].*\"orderId\":\"o1\""

echo "== vehicle action forwarded over ROS topics =="
send instantActions "{${HEADER},\"actions\":[{\"actionType\":\"pick\",\"actionId\":\"a1\",\"blockingType\":\"HARD\"}]}"
wait_for "pick is handed to the robot" 10 \
  "^${PREFIX}/state .*\"actionId\":\"a1\",\"actionStatus\":\"INITIALIZING\""
ros2 topic pub --once /vda5050_adapter/action_status std_msgs/msg/String \
  '{data: "{\"actionId\":\"a1\",\"status\":\"FINISHED\",\"resultDescription\":\"ok\"}"}' >/dev/null
wait_for "robot's report reaches master control" 10 \
  "^${PREFIX}/state .*\"actionId\":\"a1\",\"actionStatus\":\"FINISHED\""

echo "== malformed order =="
send order "this is not json"
wait_for "malformed payload reported as validationError" 10 \
  "^${PREFIX}/state .*\"errorType\":\"validationError\""

echo "== factsheet on request =="
send instantActions "{${HEADER},\"actions\":[{\"actionType\":\"factsheetRequest\",\"actionId\":\"f1\",\"blockingType\":\"NONE\"}]}"
wait_for "factsheet published" 10 \
  "^${PREFIX}/factsheet .*\"typeSpecification\""

echo "== broker outage while an order is being driven =="
# N1 has no coordinates here: the vehicle accepts it because it is the node it
# last reached (the fake robot pose never moves from the origin).
send order "{${HEADER},\"orderId\":\"o2\",\"orderUpdateId\":0,\"nodes\":[\
{\"nodeId\":\"N1\",\"sequenceId\":0,\"released\":true,\"actions\":[]},\
{\"nodeId\":\"N2\",\"sequenceId\":2,\"released\":true,\"actions\":[],\"nodePosition\":{\"x\":2,\"y\":0,\"mapId\":\"map\"}}],\
\"edges\":[{\"edgeId\":\"E1\",\"sequenceId\":1,\"released\":true,\"startNodeId\":\"N1\",\"endNodeId\":\"N2\",\"actions\":[]}]}"
wait_for "order o2 accepted and vehicle driving" 10 \
  "^${PREFIX}/state .*\"driving\":true.*\"orderId\":\"o2\""
kill "$BROKER_PID"
echo "   broker stopped; the drive finishes while the vehicle is offline"
sleep 7
if grep -Eq "^${PREFIX}/state .*\"lastNodeId\":\"N2\"" "$LOG"; then
  fail "arrival at N2 was reported before the broker went away; the outage tested nothing"
fi
mark
BROKER_MARK=$(wc -l <"$BROKER_LOG")
start_broker
# The recorder (mosquitto_sub) reconnects and resubscribes by itself.
echo "   broker restarted"
wait_for_new "vehicle reconnects by itself and announces ONLINE" 60 \
  "^${PREFIX}/connection .*\"connectionState\":\"ONLINE\""
send instantActions "{${HEADER},\"actions\":[{\"actionType\":\"stateRequest\",\"actionId\":\"s2\",\"blockingType\":\"NONE\"}]}"
wait_for_new "progress made offline is reported: o2 finished at N2, no errors" 15 \
  "^${PREFIX}/state .*\"actionId\":\"s2\",\"actionStatus\":\"FINISHED\".*\"driving\":false.*\"errors\":\[\].*\"lastNodeId\":\"N2\".*\"nodeStates\":\[\].*\"orderId\":\"o2\""

sleep 3
echo "   ---- what the restarted broker saw of the vehicle's client ----"
tail -n +"$((BROKER_MARK + 1))" "$BROKER_LOG" | grep -E "acme-amr-001" | grep -vE "PUBLISH|PINGREQ|PINGRESP|PUBACK|SUBACK" | cut -c1-200 | head -n 30
echo "   ---- connection topic since the restart ----"
tail -n +"$((MARK + 1))" "$LOG" | grep "^${PREFIX}/connection " | cut -c1-200
if tail -n +"$((MARK + 1))" "$LOG" | grep -q "CONNECTIONBROKEN"; then
  fail "the vehicle's connection broke again after it had reconnected"
fi
echo "ok: one clean reconnect, no CONNECTIONBROKEN"

echo "== clean shutdown (SIGINT) =="
mark
# Signal the node itself, as Ctrl-C in its terminal would. (A background
# `ros2 launch` in a non-interactive shell does not react to SIGINT.)
NODE_PID="$(pgrep -f "lib/vda5050_nav2_adapter/vda5050_adapter" | head -n 1)"
[ -n "$NODE_PID" ] || fail "adapter process not found"
kill -INT "$NODE_PID"
wait_for_new "vehicle announces OFFLINE before leaving" 10 \
  "^${PREFIX}/connection .*\"connectionState\":\"OFFLINE\""
sleep 2
if tail -n +"$((MARK + 1))" "$LOG" | grep -q "CONNECTIONBROKEN"; then
  fail "last will was published on a clean shutdown"
fi
echo "ok: last will not triggered"

echo "== crash: the adapter is killed without warning =="
mark
"$ADAPTER_BIN" --ros-args --params-file "$PARAMS" & CRASH_PID=$!
PIDS+=("$CRASH_PID")
wait_for_new "vehicle is ONLINE again" 20 \
  "^${PREFIX}/connection .*\"connectionState\":\"ONLINE\""
kill -9 "$CRASH_PID"
wait_for_new "broker publishes the last will: CONNECTIONBROKEN" 30 \
  "^${PREFIX}/connection .*\"connectionState\":\"CONNECTIONBROKEN\""

echo "PASS: integration test"
exit 0
