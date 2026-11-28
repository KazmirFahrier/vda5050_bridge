# Copyright 2026 vda5050_bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""
Turn fault messages into the JSON structures the web API serves.

Nothing here imports ROS: the functions only read attributes, so they work on
real messages and on simple stand-ins in tests.
"""

from datetime import datetime, timezone

SEVERITIES = ('INFO', 'WARNING', 'ERROR', 'CRITICAL')
STATES = ('INACTIVE', 'PENDING', 'ACTIVE', 'HEALING', 'AWAITING_ACK')
EVENT_TYPES = ('ACTIVATED', 'CLEARED', 'ACKNOWLEDGED')


def _name(names, index):
    """Return the name for a numeric constant, tolerating values from a newer peer."""
    return names[index] if 0 <= index < len(names) else 'UNKNOWN'


def time_to_iso(stamp):
    """Convert a builtin_interfaces/Time to ISO 8601 in UTC with milliseconds."""
    moment = datetime.fromtimestamp(stamp.sec + stamp.nanosec / 1e9, tz=timezone.utc)
    return moment.strftime('%Y-%m-%dT%H:%M:%S.') + f'{moment.microsecond // 1000:03d}Z'


def fault_to_dict(fault):
    """Convert a fault_msgs/Fault."""
    result = {
        'code': fault.code,
        'state': _name(STATES, fault.state),
        'severity': _name(SEVERITIES, fault.severity),
        'description': fault.description,
        'message': fault.message,
        'source': fault.source,
        'known': bool(fault.known),
        'occurrenceCount': int(fault.occurrence_count),
    }
    if fault.occurrence_count > 0:
        result['firstActivated'] = time_to_iso(fault.first_activated)
        result['lastActivated'] = time_to_iso(fault.last_activated)
    return result


def event_to_dict(event):
    """Convert a fault_msgs/FaultEvent."""
    return {
        'sequence': int(event.sequence),
        'type': _name(EVENT_TYPES, event.type),
        'code': event.code,
        'severity': _name(SEVERITIES, event.severity),
        'time': time_to_iso(event.stamp),
        'message': event.message,
        'source': event.source,
        'occurrenceCount': int(event.occurrence_count),
    }


def summarize(faults):
    """Count a list of fault dictionaries by severity."""
    by_severity = dict.fromkeys(SEVERITIES, 0)
    highest = None
    for fault in faults:
        severity = fault['severity']
        if severity not in by_severity:
            continue
        by_severity[severity] += 1
        if highest is None or SEVERITIES.index(severity) > SEVERITIES.index(highest):
            highest = severity
    return {'raised': len(faults), 'highest': highest, 'bySeverity': by_severity}
