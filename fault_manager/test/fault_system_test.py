#!/usr/bin/env python3
# Copyright 2026 vda5050_bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""
End-to-end test of fault management.

Real: the fault manager node, the VDA 5050 adapter, the web gateway, a
      Mosquitto broker, rosbag2, the ROS 2 middleware.
Fake: Nav2 (fake_nav2_server.py) and the robot pose (a static transform).

This script plays three roles: the monitors (it publishes fault reports and
diagnostics), the operator (it uses the web API), and master control (it reads
the VDA 5050 state from the broker and sends one order).

Run it with ROS 2 and the workspace sourced. It exits with 0 on success.
"""

import http.client
import json
import os
import re
import signal
import subprocess
import sys
import tempfile
import threading
import time

from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from fault_msgs.msg import FaultReport
import rclpy
from std_msgs.msg import String
import yaml

PREFIX = 'uagv/v2/acme/amr-001'
PORT = 18080
TOKEN = 'test-token'
WORK = tempfile.mkdtemp(prefix='fault_system_test_')
HISTORY = os.path.join(WORK, 'history.jsonl')
CAPTURES = os.path.join(WORK, 'captures')
MQTT_LOG = os.path.join(WORK, 'mqtt.log')

CATALOG = {'faults': [
    {'code': 'TEST-INSTANT', 'severity': 'ERROR', 'description': 'Instant test fault'},
    {'code': 'TEST-DEBOUNCED', 'severity': 'WARNING', 'activateAfterMs': 1000,
     'clearAfterMs': 1000},
    {'code': 'TEST-LATCHED', 'severity': 'CRITICAL', 'latched': True,
     'description': 'Latched test fault'},
    {'code': 'TEST-WATCHDOG', 'severity': 'ERROR', 'timeoutMs': 2000},
    {'code': 'TEST-DIAG', 'severity': 'ERROR'},
    {'code': 'TEST-QUIET', 'severity': 'INFO'},
    {'code': 'BRIDGE-MQTT-001', 'severity': 'ERROR', 'activateAfterMs': 2000,
     'clearAfterMs': 500, 'timeoutMs': 4000},
    {'code': 'BRIDGE-NAV-001', 'severity': 'CRITICAL', 'activateAfterMs': 3000,
     'clearAfterMs': 500, 'timeoutMs': 4000},
    {'code': 'BRIDGE-LOC-001', 'severity': 'ERROR', 'activateAfterMs': 3000,
     'clearAfterMs': 500, 'timeoutMs': 4000},
]}

FAULT_MANAGER_PARAMS = {'fault_manager': {'ros__parameters': {
    'catalog_file': os.path.join(WORK, 'catalog.json'),
    'history_file': HISTORY,
    'tick_rate_hz': 20.0,
    'publish_interval_s': 1.0,
    'diagnostics': {
        'names': ['test_sensor: rate'], 'codes': ['TEST-DIAG'], 'fail_level': 2},
    'capture': {
        'enabled': True, 'directory': CAPTURES, 'topics': ['/test/heartbeat'],
        'pre_seconds': 3.0, 'post_seconds': 1.0, 'min_severity': 'ERROR', 'max_bags': 3},
}}}

HEADER = ('"headerId":1,"timestamp":"2026-01-01T00:00:00.000Z","version":"2.0.0",'
          '"manufacturer":"acme","serialNumber":"amr-001"')

processes = {}
logs = {}


# ---- small helpers -----------------------------------------------------------------


def fail(message):
    print(f'FAIL: {message}', flush=True)
    for name, path in logs.items():
        print(f'---- last lines of {name} ----')
        try:
            with open(path, errors='replace') as log:
                print(''.join(log.readlines()[-25:])[-4000:])
        except OSError:
            pass
    stop_all()
    sys.exit(1)


def ok(message):
    print(f'ok: {message}', flush=True)


def wait_until(description, condition, timeout=15.0, interval=0.1):
    """Poll `condition` until it returns something true; fail the test otherwise."""
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        try:
            result = condition()
            if result:
                ok(description)
                return result
        except (OSError, ValueError, KeyError, TypeError, IndexError,
                http.client.HTTPException) as error:
            last_error = error
        time.sleep(interval)
    fail(f'{description} (not within {timeout:.0f} s; last error: {last_error})')


def stays_false(description, condition, duration):
    """Check that `condition` never becomes true for `duration` seconds."""
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        if condition():
            fail(description)
        time.sleep(0.1)
    ok(description)


def start(name, command, env=None):
    logs[name] = os.path.join(WORK, f'{name}.log')
    with open(logs[name], 'ab') as log:
        processes[name] = subprocess.Popen(
            command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            env={**os.environ, **(env or {})})
    return processes[name]


def stop(name, sig=signal.SIGINT, timeout=10):
    process = processes.pop(name, None)
    if process is None:
        return None
    process.send_signal(sig)
    try:
        return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.kill()
        return process.wait()


def stop_all():
    for name in list(processes):
        stop(name, signal.SIGTERM, timeout=3)


def executable(package, name):
    return os.path.join(get_package_prefix(package), 'lib', package, name)


def http_request(method, path, headers=None):
    connection = http.client.HTTPConnection('127.0.0.1', PORT, timeout=5)
    try:
        connection.request(method, path, headers=headers or {})
        response = connection.getresponse()
        body = response.read()
        return response.status, (json.loads(body) if body else None)
    finally:
        connection.close()


def api_faults():
    status, body = http_request('GET', '/api/v1/faults')
    if status != 200:
        raise ValueError(f'/api/v1/faults answered {status}')
    return {fault['code']: fault for fault in body['faults']}


def api_history(query=''):
    status, body = http_request('GET', '/api/v1/history?limit=10000' + query)
    if status != 200:
        raise ValueError(f'/api/v1/history answered {status}')
    return body['events']


def fault_state(code):
    fault = api_faults().get(code)
    return fault['state'] if fault else None


class MqttLog:
    """What the vehicle published on the broker, recorded by mosquitto_sub."""

    def __init__(self):
        self.mark_position = 0

    def lines(self):
        with open(MQTT_LOG, errors='replace') as log:
            return log.readlines()

    def mark(self):
        self.mark_position = len(self.lines())

    def seen(self, pattern, since_mark=True):
        lines = self.lines()[self.mark_position if since_mark else 0:]
        return any(re.search(pattern, line) for line in lines)

    def last_state(self):
        for line in reversed(self.lines()):
            if line.startswith(f'{PREFIX}/state '):
                return json.loads(line.split(' ', 1)[1])
        return None

    def state_errors(self):
        state = self.last_state()
        return {error['errorType']: error for error in state['errors']} if state else {}


class EventStream(threading.Thread):
    """Reads the gateway's server-sent events in the background."""

    def __init__(self):
        super().__init__(daemon=True)
        self.events = []

    def run(self):
        try:
            connection = http.client.HTTPConnection('127.0.0.1', PORT, timeout=120)
            connection.request('GET', '/api/v1/events')
            response = connection.getresponse()
            name = None
            while True:
                line = response.fp.readline()
                if not line:
                    return
                line = line.decode('utf-8').rstrip('\n')
                if line.startswith('event: '):
                    name = line[len('event: '):]
                elif line.startswith('data: ') and name:
                    self.events.append((name, json.loads(line[len('data: '):])))
        except (OSError, http.client.HTTPException):
            return

    def saw(self, event_type, code):
        return any(
            name == 'fault_event' and data['type'] == event_type and data['code'] == code
            for name, data in list(self.events))


class Monitors:
    """Publishes what monitors on a robot would publish."""

    def __init__(self):
        self.node = rclpy.create_node('test_monitors')
        self.reports = self.node.create_publisher(FaultReport, 'faults/report', 100)
        self.diagnostics = self.node.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.heartbeat = self.node.create_publisher(String, '/test/heartbeat', 10)
        self.watchdog_alive = True
        self.running = True
        self.thread = threading.Thread(target=self._background, daemon=True)
        self.thread.start()

    def _background(self):
        count = 0
        while self.running:
            self.heartbeat.publish(String(data=f'beat {count}'))
            if self.watchdog_alive and count % 2 == 0:
                self.report('TEST-WATCHDOG', False)
            count += 1
            time.sleep(0.1)

    def report(self, code, failing, message='', source='system_test'):
        self.reports.publish(
            FaultReport(code=code, failing=failing, source=source, message=message))

    def diagnostic(self, level, message):
        status = DiagnosticStatus(
            level=level, name='test_sensor: rate', message=message, hardware_id='sensor-7')
        other = DiagnosticStatus(
            level=DiagnosticStatus.ERROR, name='unmapped status', message='ignored')
        self.diagnostics.publish(DiagnosticArray(status=[other, status]))


def start_fault_manager():
    return start('fault_manager', [
        executable('fault_manager', 'fault_manager_node'), '--ros-args',
        '--params-file', os.path.join(WORK, 'fault_manager.yaml')])


def start_adapter():
    params = os.path.join(
        get_package_prefix('vda5050_nav2_adapter'), 'share', 'vda5050_nav2_adapter',
        'config', 'adapter.yaml')
    return start('adapter', [
        executable('vda5050_nav2_adapter', 'vda5050_adapter'), '--ros-args',
        '--params-file', params, '-p', 'faults.timeout_s:=3.0'])


def bags():
    if not os.path.isdir(CAPTURES):
        return []
    return sorted(
        name for name in os.listdir(CAPTURES)
        if os.path.isfile(os.path.join(CAPTURES, name, 'fault.json')))


# ---- the test ----------------------------------------------------------------------


def main():
    with open(os.path.join(WORK, 'catalog.json'), 'w') as catalog:
        json.dump(CATALOG, catalog)
    with open(os.path.join(WORK, 'fault_manager.yaml'), 'w') as params:
        yaml.safe_dump(FAULT_MANAGER_PARAMS, params)
    open(MQTT_LOG, 'w').close()

    print('== a broken catalogue stops the fault manager from starting ==', flush=True)
    with open(os.path.join(WORK, 'broken.json'), 'w') as broken:
        broken.write('{"faults":[{"code":"X","severity":"LOUD"}]}')
    result = subprocess.run(
        [executable('fault_manager', 'fault_manager_node'), '--ros-args', '-p',
         'catalog_file:=' + os.path.join(WORK, 'broken.json')],
        capture_output=True, text=True, timeout=20)
    if result.returncode == 0 or 'X.severity' not in result.stdout + result.stderr:
        fail('fault manager accepted a broken catalogue:\n' + result.stdout + result.stderr)
    ok('exit code %d and a message naming the entry' % result.returncode)

    print('== starting broker, fake robot, adapter, fault manager, gateway ==', flush=True)
    start('broker', ['mosquitto', '-p', '1883'])
    time.sleep(1)
    with open(MQTT_LOG, 'ab') as log:
        processes['recorder'] = subprocess.Popen(
            ['mosquitto_sub', '-v', '-t', f'{PREFIX}/#'], stdout=log,
            start_new_session=True)
    start('tf', ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '--frame-id', 'map', '--child-frame-id', 'base_link'])
    here = os.path.dirname(os.path.abspath(__file__))
    fake_nav = os.path.join(
        here, '..', '..', 'vda5050_nav2_adapter', 'test', 'fake_nav2_server.py')
    start('fake_nav2', [sys.executable, fake_nav], env={'FAKE_NAV_DELAY': '3'})

    rclpy.init()
    monitors = Monitors()
    mqtt = MqttLog()

    start_fault_manager()
    start_adapter()
    start('gateway', [
        executable('fault_gateway', 'fault_gateway'), '--ros-args',
        '-p', f'port:={PORT}', '-p', f'api_token:={TOKEN}'])

    wait_until('vehicle is ONLINE on the broker',
               lambda: mqtt.seen(r'/connection .*"connectionState":"ONLINE"'), 30)
    wait_until('gateway answers and has heard the fault manager',
               lambda: http_request('GET', '/api/v1/health')[1]['faultManager'], 30)
    wait_until('monitors are connected to the fault manager',
               lambda: monitors.reports.get_subscription_count() > 0, 20)
    # The adapter reports its own health; once it is up, nothing is wrong.
    wait_until('no faults are raised at the start', lambda: api_faults() == {}, 30)
    stream = EventStream()
    stream.start()
    time.sleep(3.5)  # let the flight recorder fill its 3 s buffer

    print('== a fault appears everywhere ==', flush=True)
    monitors.report('TEST-INSTANT', True, 'something broke')
    fault = wait_until('web API lists the fault as ACTIVE',
                       lambda: api_faults().get('TEST-INSTANT'))
    expected = {
        'state': 'ACTIVE', 'severity': 'ERROR', 'message': 'something broke',
        'source': 'system_test', 'description': 'Instant test fault', 'known': True,
        'occurrenceCount': 1}
    if {key: fault[key] for key in expected} != expected:
        fail(f'unexpected fault content: {fault}')
    ok('severity, message, source and description are as reported')
    error = wait_until('VDA 5050 state lists it in errors',
                       lambda: mqtt.state_errors().get('TEST-INSTANT'))
    if error['errorLevel'] != 'WARNING' or error['errorDescription'] != 'something broke':
        fail(f'unexpected VDA 5050 error: {error}')
    ok('errorLevel WARNING and the message as errorDescription')
    wait_until('event stream delivered ACTIVATED',
               lambda: stream.saw('ACTIVATED', 'TEST-INSTANT'))
    if mqtt.last_state()['paused']:
        fail('a non-critical fault must not hold the vehicle')

    print('== data capture ==', flush=True)
    name = wait_until('a bag was written for the fault',
                      lambda: [bag for bag in bags() if 'TEST-INSTANT' in bag], 15)[0]
    bag = os.path.join(CAPTURES, name)
    with open(os.path.join(bag, 'fault.json')) as summary_file:
        summary = json.load(summary_file)
    with open(os.path.join(bag, 'metadata.yaml')) as metadata_file:
        metadata = yaml.safe_load(metadata_file)['rosbag2_bagfile_information']
    counts = {topic['topic_metadata']['name']: topic['message_count']
              for topic in metadata['topics_with_message_count']}
    beats = counts.get('/test/heartbeat', 0)
    # 3 s before and 1 s after at 10 Hz is 40 messages; allow for start-up jitter.
    if not 25 <= beats <= 60 or summary['messages'] != beats:
        fail(f'unexpected bag content: {counts}, summary {summary}')
    if summary['event']['code'] != 'TEST-INSTANT' or summary['event']['type'] != 'ACTIVATED':
        fail(f'unexpected capture summary: {summary}')
    ok(f'bag holds {beats} messages of /test/heartbeat from before and after the fault')
    info = subprocess.run(['ros2', 'bag', 'info', bag], capture_output=True, text=True)
    if info.returncode != 0 or '/test/heartbeat' not in info.stdout:
        fail('ros2 bag info cannot read the bag:\n' + info.stdout + info.stderr)
    ok('ros2 bag info reads the bag')

    print('== the fault clears ==', flush=True)
    monitors.report('TEST-INSTANT', False)
    wait_until('web API no longer lists it', lambda: 'TEST-INSTANT' not in api_faults())
    wait_until('VDA 5050 state no longer lists it',
               lambda: 'TEST-INSTANT' not in mqtt.state_errors())
    wait_until('event stream delivered CLEARED', lambda: stream.saw('CLEARED', 'TEST-INSTANT'))

    print('== a low-severity fault: listed, but no capture and not sent to master ==',
          flush=True)
    bags_before = bags()
    monitors.report('TEST-QUIET', True)
    wait_until('INFO fault is listed by the web API', lambda: 'TEST-QUIET' in api_faults())
    time.sleep(2.5)
    if bags() != bags_before:
        fail('an INFO fault triggered a capture')
    if 'TEST-QUIET' in mqtt.state_errors():
        fail('an INFO fault was sent to master control')
    ok('no bag and no VDA 5050 error for an INFO fault')
    monitors.report('TEST-QUIET', False)

    print('== debounce ==', flush=True)
    monitors.report('TEST-DEBOUNCED', True)
    time.sleep(0.3)
    monitors.report('TEST-DEBOUNCED', False)
    stays_false('a 0.3 s glitch does not become a fault',
                lambda: 'TEST-DEBOUNCED' in api_faults(), 2.0)
    if any(event['code'] == 'TEST-DEBOUNCED' for event in api_history()):
        fail('the glitch left an event in the history')
    monitors.report('TEST-DEBOUNCED', True)
    started = time.monotonic()
    wait_until('a persistent condition becomes a fault',
               lambda: fault_state('TEST-DEBOUNCED') == 'ACTIVE', 5)
    if time.monotonic() - started < 0.8:
        fail('the fault activated before its debounce time')
    monitors.report('TEST-DEBOUNCED', False)
    wait_until('it shows HEALING before it clears',
               lambda: fault_state('TEST-DEBOUNCED') == 'HEALING', 3, 0.05)
    wait_until('and clears after the clear time',
               lambda: 'TEST-DEBOUNCED' not in api_faults(), 5)

    print('== a critical latched fault holds the vehicle until acknowledged ==',
          flush=True)
    mqtt.mark()
    order = ('{' + HEADER + ',"orderId":"o1","orderUpdateId":0,"nodes":['
             '{"nodeId":"N0","sequenceId":0,"released":true,"actions":[],'
             '"nodePosition":{"x":0,"y":0,"mapId":"map"}},'
             '{"nodeId":"N1","sequenceId":2,"released":true,"actions":[],'
             '"nodePosition":{"x":1,"y":0,"mapId":"map"}}],'
             '"edges":[{"edgeId":"E0","sequenceId":1,"released":true,'
             '"startNodeId":"N0","endNodeId":"N1","actions":[]}]}')
    subprocess.run(['mosquitto_pub', '-t', f'{PREFIX}/order', '-m', order], check=True)
    wait_until('vehicle is driving the order',
               lambda: mqtt.seen(r'/state .*"driving":true.*"orderId":"o1"'))
    monitors.report('TEST-LATCHED', True, 'bumper pressed', 'safety_controller')
    wait_until('VDA 5050 state reports the fault as FATAL',
               lambda: mqtt.state_errors().get('TEST-LATCHED', {}).get('errorLevel') == 'FATAL')
    wait_until('vehicle is held: paused, not driving',
               lambda: mqtt.last_state()['paused'] and not mqtt.last_state()['driving'])
    monitors.report('TEST-LATCHED', False)
    wait_until('cause gone: fault waits for acknowledgement',
               lambda: fault_state('TEST-LATCHED') == 'AWAITING_ACK')
    stays_false('vehicle stays held and the order is not finished meanwhile',
                lambda: not mqtt.last_state()['paused'] or
                mqtt.last_state()['lastNodeId'] == 'N1', 4.0)

    path = '/api/v1/faults/TEST-LATCHED/acknowledge'
    if http_request('POST', path)[0] != 401:
        fail('acknowledge without the access token was not refused')
    if http_request('POST', path, {'Authorization': 'Bearer wrong'})[0] != 401:
        fail('acknowledge with a wrong token was not refused')
    if fault_state('TEST-LATCHED') != 'AWAITING_ACK':
        fail('a refused acknowledge changed the fault')
    ok('acknowledge without the right token is refused (401)')
    authorized = {'Authorization': f'Bearer {TOKEN}'}
    status, body = http_request('POST', '/api/v1/faults/BRIDGE-NAV-001/acknowledge', authorized)
    if status != 409 or 'INACTIVE' not in body['reason']:
        fail(f'acknowledging a fault that is not waiting: {status} {body}')
    ok('acknowledging a fault that is not waiting is refused (409) with the reason')
    status, body = http_request('POST', path, authorized)
    if status != 200 or body != {'acknowledged': True, 'code': 'TEST-LATCHED'}:
        fail(f'acknowledge failed: {status} {body}')
    wait_until('acknowledged: fault is gone', lambda: 'TEST-LATCHED' not in api_faults())
    wait_until('vehicle is released', lambda: not mqtt.last_state()['paused'])
    wait_until('and finishes the order at N1',
               lambda: mqtt.last_state()['lastNodeId'] == 'N1' and
               not mqtt.last_state()['driving'] and not mqtt.last_state()['nodeStates'], 20)
    wait_until('event stream delivered ACKNOWLEDGED',
               lambda: stream.saw('ACKNOWLEDGED', 'TEST-LATCHED'))

    print('== /diagnostics is mapped to a fault code ==', flush=True)
    monitors.diagnostic(DiagnosticStatus.WARN, 'rate a bit low')
    stays_false('a WARN status stays below the configured level',
                lambda: 'TEST-DIAG' in api_faults(), 1.0)
    monitors.diagnostic(DiagnosticStatus.ERROR, 'rate 2 Hz, expected 10 Hz')
    fault = wait_until('an ERROR status raises the mapped fault',
                       lambda: api_faults().get('TEST-DIAG'))
    if fault['message'] != 'rate 2 Hz, expected 10 Hz' or fault['source'] != 'sensor-7':
        fail(f'diagnostic details were not carried over: {fault}')
    if 'unmapped status' in json.dumps(api_faults()):
        fail('an unmapped diagnostic status raised a fault')
    monitors.diagnostic(DiagnosticStatus.OK, '')
    wait_until('an OK status clears it', lambda: 'TEST-DIAG' not in api_faults())

    print('== watchdog: a monitor falls silent ==', flush=True)
    monitors.watchdog_alive = False
    fault = wait_until('silence raises the fault',
                       lambda: api_faults().get('TEST-WATCHDOG'), 8)
    if fault['source'] != 'watchdog' or 'no report for 2000 ms' not in fault['message']:
        fail(f'unexpected watchdog fault: {fault}')
    monitors.watchdog_alive = True
    wait_until('reports resume: fault clears', lambda: 'TEST-WATCHDOG' not in api_faults())

    print('== an unknown fault code is not swallowed ==', flush=True)
    monitors.report('TYPO-123', True, 'who am I')
    fault = wait_until('it is listed', lambda: api_faults().get('TYPO-123'))
    if fault['known'] or fault['severity'] != 'ERROR':
        fail(f'unexpected handling of an unknown code: {fault}')
    ok('marked as unknown, with severity ERROR')
    monitors.report('TYPO-123', False)
    wait_until('and clears', lambda: 'TYPO-123' not in api_faults())

    print('== history ==', flush=True)
    events = api_history()
    sequences = [event['sequence'] for event in events]
    if sequences != list(range(1, len(events) + 1)):
        fail(f'sequence numbers are not 1..n: {sequences}')
    latched = [event['type'] for event in events if event['code'] == 'TEST-LATCHED']
    if latched != ['ACTIVATED', 'ACKNOWLEDGED']:
        fail(f'history of the latched fault: {latched}')
    ok(f'{len(events)} events, numbered without gaps; latched fault: {latched}')
    by_code = api_history('&code=TEST-INSTANT')
    if [event['type'] for event in by_code] != ['ACTIVATED', 'CLEARED']:
        fail(f'filter by code: {by_code}')
    newer = api_history(f'&after={sequences[-3]}')
    if [event['sequence'] for event in newer] != sequences[-2:]:
        fail(f'filter by sequence: {newer}')
    critical = api_history('&min_severity=CRITICAL')
    if not any(event['code'] == 'TEST-LATCHED' for event in critical) or any(
            event['severity'] != 'CRITICAL' for event in critical):
        fail(f'filter by severity: {critical}')
    ok('filters by code, by sequence number and by severity')
    if http_request('GET', '/api/v1/history?limit=nonsense')[0] != 400:
        fail('a malformed query was not refused')

    print('== old captures are deleted ==', flush=True)
    # At least four faults of severity ERROR or higher have activated by now;
    # the recorder is configured to keep three bags.
    names = wait_until(
        'only the 3 newest bags are kept',
        lambda: len(bags()) == 3 and not any('TEST-INSTANT' in bag for bag in bags())
        and bags(), 10)
    ok(f'kept: {names}')

    print('== the fault manager is restarted ==', flush=True)
    last_sequence = sequences[-1]
    mqtt.mark()
    code = stop('fault_manager')
    if code != 0:
        fail(f'fault manager exited with {code} on SIGINT')
    wait_until('web API reports the fault manager as unavailable (503)',
               lambda: http_request('GET', '/api/v1/history')[0] == 503 and
               http_request('GET', '/api/v1/faults')[0] == 503)
    if http_request('GET', '/api/v1/health')[1] != {'status': 'ok', 'faultManager': False}:
        fail('health does not show that the fault manager is gone')
    wait_until('adapter tells master control that fault data may be out of date',
               lambda: 'faultManagerTimeout' in mqtt.state_errors(), 15)
    with open(HISTORY) as history_file:
        stored = [json.loads(line) for line in history_file if line.strip()]
    if [event['sequence'] for event in stored] != sequences:
        fail('the history file does not hold exactly the events served')
    ok('history file holds one JSON line per event')

    start_fault_manager()
    wait_until('history is back after the restart',
               lambda: [event['sequence'] for event in api_history()] == sequences, 20)
    wait_until('adapter withdraws the warning',
               lambda: 'faultManagerTimeout' not in mqtt.state_errors(), 15)
    wait_until('monitors are connected again',
               lambda: monitors.reports.get_subscription_count() > 0, 20)
    if 'TEST-INSTANT' in api_faults():
        fail('a fault was restored as active from the history file')
    time.sleep(1.0)
    monitors.report('TEST-INSTANT', True, 'again')
    fault = wait_until('a new occurrence after the restart',
                       lambda: api_faults().get('TEST-INSTANT'))
    newest = api_history()[-1]
    if newest['sequence'] != last_sequence + 1 or fault['occurrenceCount'] != 2:
        fail(f'numbering did not continue: {newest}, {fault}')
    ok('sequence numbers and the occurrence count continue where they stopped')
    monitors.report('TEST-INSTANT', False)

    print('== the adapter dies: its watchdogs raise the bridge faults ==', flush=True)
    wait_until('no faults before the adapter is stopped', lambda: api_faults() == {}, 20)
    stop('adapter', signal.SIGKILL)
    fault = wait_until('silence from the adapter raises BRIDGE-NAV-001',
                       lambda: api_faults().get('BRIDGE-NAV-001'), 15)
    if fault['source'] != 'watchdog' or fault['severity'] != 'CRITICAL':
        fail(f'unexpected fault: {fault}')
    wait_until('and the other two bridge faults',
               lambda: {'BRIDGE-MQTT-001', 'BRIDGE-LOC-001'} <= set(api_faults()), 10)
    start_adapter()
    wait_until('adapter is back: bridge faults clear', lambda: api_faults() == {}, 30)

    monitors.running = False
    monitors.thread.join(timeout=2)
    print('== clean shutdown ==', flush=True)
    for name in ('gateway', 'fault_manager', 'adapter'):
        code = stop(name)
        if code not in (0, -signal.SIGINT):
            fail(f'{name} exited with {code} on SIGINT')
    ok('gateway, fault manager and adapter stop cleanly')
    stop_all()
    rclpy.try_shutdown()
    print('PASS', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
