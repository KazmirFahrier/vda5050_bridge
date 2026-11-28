# Copyright 2026 vda5050_bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Tests of the message-to-JSON conversion."""

from types import SimpleNamespace

from fault_gateway.convert import event_to_dict, fault_to_dict, summarize, time_to_iso


def stamp(sec, nanosec=0):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def fault(**overrides):
    values = {
        'code': 'LIDAR-001', 'severity': 3, 'state': 2, 'description': 'Lidar silent',
        'message': 'no scan for 0.6 s', 'source': 'lidar_driver', 'known': True,
        'occurrence_count': 2, 'first_activated': stamp(1795722600),
        'last_activated': stamp(1795722660, 123000000),
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_time_is_iso_utc_with_milliseconds():
    assert time_to_iso(stamp(1795722600, 123456789)) == '2026-11-26T19:50:00.123Z'
    assert time_to_iso(stamp(0)) == '1970-01-01T00:00:00.000Z'


def test_fault_fields():
    assert fault_to_dict(fault()) == {
        'code': 'LIDAR-001', 'state': 'ACTIVE', 'severity': 'CRITICAL',
        'description': 'Lidar silent', 'message': 'no scan for 0.6 s',
        'source': 'lidar_driver', 'known': True, 'occurrenceCount': 2,
        'firstActivated': '2026-11-26T19:50:00.000Z',
        'lastActivated': '2026-11-26T19:51:00.123Z',
    }


def test_fault_that_never_activated_has_no_timestamps():
    result = fault_to_dict(fault(occurrence_count=0, state=0))
    assert 'firstActivated' not in result
    assert result['state'] == 'INACTIVE'


def test_all_names():
    states = [fault_to_dict(fault(state=value))['state'] for value in range(5)]
    assert states == ['INACTIVE', 'PENDING', 'ACTIVE', 'HEALING', 'AWAITING_ACK']
    severities = [fault_to_dict(fault(severity=value))['severity'] for value in range(4)]
    assert severities == ['INFO', 'WARNING', 'ERROR', 'CRITICAL']


def test_unknown_constants_do_not_crash():
    result = fault_to_dict(fault(state=17, severity=9))
    assert result['state'] == 'UNKNOWN'
    assert result['severity'] == 'UNKNOWN'


def test_event_fields():
    event = SimpleNamespace(
        sequence=42, type=2, code='BUMPER-001', severity=3, stamp=stamp(1795722600),
        message='contact', source='safety', occurrence_count=1)
    assert event_to_dict(event) == {
        'sequence': 42, 'type': 'ACKNOWLEDGED', 'code': 'BUMPER-001',
        'severity': 'CRITICAL', 'time': '2026-11-26T19:50:00.000Z', 'message': 'contact',
        'source': 'safety', 'occurrenceCount': 1,
    }


def test_summary():
    faults = [{'severity': 'WARNING'}, {'severity': 'CRITICAL'}, {'severity': 'WARNING'},
              {'severity': 'UNKNOWN'}]
    assert summarize(faults) == {
        'raised': 4, 'highest': 'CRITICAL',
        'bySeverity': {'INFO': 0, 'WARNING': 2, 'ERROR': 0, 'CRITICAL': 1},
    }
    assert summarize([]) == {
        'raised': 0, 'highest': None,
        'bySeverity': {'INFO': 0, 'WARNING': 0, 'ERROR': 0, 'CRITICAL': 0},
    }
