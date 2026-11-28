#!/usr/bin/env bash
# Copyright 2026 vda5050_bridge contributors
# SPDX-License-Identifier: Apache-2.0
#
# Check this library against the official VDA 5050 JSON schemas.
#
#   run_schema_conformance.sh <emit_samples binary> <schemas 2.0.0 dir> <schemas 2.1.0 dir>
#
# Outbound: every message the library publishes must validate against both
#           schema versions.
# Inbound:  example orders and instant actions must validate against the
#           schemas AND be accepted by the library's decoder.
# One documented deviation is checked the other way round: it must FAIL
# validation, so that this script notices if the schemas are ever corrected.

set -uo pipefail

EMIT="$1"; SCHEMAS_20="$2"; SCHEMAS_21="$3"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA="${HERE}/data"
OUT="$(mktemp -d)"
STATUS=0

echo "== decoding the inbound examples with the library =="
"$EMIT" "$OUT" "$DATA"/*.json || { echo "FAIL: decoder rejected an inbound example"; exit 1; }

OUTBOUND=("$OUT"/state.json "$OUT"/state_minimal.json "$OUT"/connection_*.json
          "$OUT"/visualization.json "$OUT"/factsheet.json)

for VERSION in 2.0.0 2.1.0; do
  [ "$VERSION" = 2.0.0 ] && SCHEMAS="$SCHEMAS_20" || SCHEMAS="$SCHEMAS_21"
  echo "== outbound messages against the ${VERSION} schemas =="
  python3 "${HERE}/schema_conformance.py" "$SCHEMAS" "${OUTBOUND[@]}" || STATUS=1
  echo "== inbound examples against the ${VERSION} schemas =="
  # The two schema versions disagree on one field of an instant action
  # (actionName in 2.0.0, actionType in 2.1.0), so each gets its own example.
  [ "$VERSION" = 2.0.0 ] && INSTANT="$DATA/instantActions_v2_0.json" || INSTANT="$DATA/instantActions_v2_1.json"
  python3 "${HERE}/schema_conformance.py" "$SCHEMAS" "$DATA"/order_*.json "$INSTANT" || STATUS=1
done

echo "== documented deviation: action status PAUSED =="
# The specification text lists PAUSED as an action status; the schemas' enum
# does not include it. The library follows the text.
for SCHEMAS in "$SCHEMAS_20" "$SCHEMAS_21"; do
  if python3 "${HERE}/schema_conformance.py" "$SCHEMAS" "$OUT"/deviation_state_paused_action.json \
       >"$OUT/deviation.log" 2>&1; then
    echo "NOTE: the schema in $SCHEMAS now accepts PAUSED; the deviation note in the docs can go"
  else
    grep -q "'PAUSED' is not one of" "$OUT/deviation.log" && [ "$(grep -c '^     ' "$OUT/deviation.log")" = 1 ] \
      || { echo "FAIL: unexpected validation errors:"; cat "$OUT/deviation.log"; STATUS=1; }
    echo "as expected: rejected only for actionStatus PAUSED"
  fi
done

[ "$STATUS" = 0 ] && echo "PASS: schema conformance" || echo "FAIL: schema conformance"
exit "$STATUS"
