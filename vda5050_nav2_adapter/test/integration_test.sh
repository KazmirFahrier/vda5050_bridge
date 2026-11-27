#!/usr/bin/env bash
# Copyright 2026 vda5050_bridge contributors
# SPDX-License-Identifier: Apache-2.0
#
# End-to-end test of the adapter executable.
#
# Real: the adapter node, a Mosquitto broker, tf2, the ROS 2 action protocol.
# Fake: Nav2 (fake_nav2_server.py succeeds every goal after 0.5 s) and the
#       robot pose (a static transform map -> base_link at the origin).
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

send() {  # send <topic> <payload>
  mosquitto_pub -t "${PREFIX}/$1" -m "$2"
}

echo "== starting broker, recorder, fake robot =="
mosquitto -p 1883 >/dev/null 2>&1 & PIDS+=($!)
sleep 1
mosquitto_sub -v -t "${PREFIX}/#" >"$LOG" & PIDS+=($!)
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id base_link \
  >/dev/null 2>&1 & PIDS+=($!)
python3 "${HERE}/fake_nav2_server.py" & PIDS+=($!)

echo "== starting adapter =="
"$ADAPTER_BIN" --ros-args --params-file "$PARAMS" & ADAPTER_PID=$!
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

echo "== clean shutdown (SIGINT) =="
kill -INT "$ADAPTER_PID"
wait_for "vehicle announces OFFLINE before leaving" 10 \
  "^${PREFIX}/connection .*\"connectionState\":\"OFFLINE\""
if grep -q "CONNECTIONBROKEN" "$LOG"; then
  fail "last will was published on a clean shutdown"
fi
echo "ok: last will not triggered"

echo "PASS: integration test"
