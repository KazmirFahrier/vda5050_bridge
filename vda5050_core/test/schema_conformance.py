#!/usr/bin/env python3
# Copyright 2026 vda5050_bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""
Validate JSON messages against the official VDA 5050 JSON schemas.

  schema_conformance.py <schema-dir> <file.json> [<file.json> ...]

The schema is chosen from the file name: a file whose name contains "state"
is checked against the state schema, "order" against the order schema, and so
on. Exits non-zero if
any file does not conform. Requires the Python package `jsonschema`.

The schemas are not stored in this repository. Fetch them from the official
one at a fixed version, for example:

  git clone --depth 1 --branch 2.0.0 https://github.com/VDA5050/VDA5050
  schema_conformance.py VDA5050/json_schemas out/*.json
"""

import json
import os
import sys

import jsonschema

TOPICS = ['instantActions', 'visualization', 'connection', 'factsheet', 'order', 'state']


def find_schema(schema_dir, topic):
    """Return the parsed schema for a topic; file extensions differ between releases."""
    for name in sorted(os.listdir(schema_dir)):
        stem = name.split('.')[0]
        if stem.lower() == topic.lower():
            with open(os.path.join(schema_dir, name), encoding='utf-8') as handle:
                return name, json.load(handle)
    raise FileNotFoundError(f'no schema for topic {topic!r} in {schema_dir}')


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    schema_dir, files = argv[1], argv[2:]
    failures = 0
    for path in files:
        base = os.path.basename(path)
        topic = next((t for t in TOPICS if t.lower() in base.lower()), None)
        if topic is None:
            print(f'SKIP {base}: file name does not contain a topic name')
            failures += 1
            continue
        schema_name, schema = find_schema(schema_dir, topic)
        with open(path, encoding='utf-8') as handle:
            message = json.load(handle)
        validator_class = jsonschema.validators.validator_for(schema)
        errors = sorted(validator_class(schema).iter_errors(message), key=lambda e: list(e.path))
        if errors:
            failures += 1
            print(f'FAIL {base} against {schema_name}')
            for error in errors[:20]:
                location = '/'.join(str(part) for part in error.path) or '(root)'
                print(f'     {location}: {error.message[:200]}')
        else:
            print(f'ok   {base} conforms to {schema_name}')
    print(f'{len(files) - failures} of {len(files)} file(s) conform')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
