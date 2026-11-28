#!/usr/bin/env python3
# Copyright 2026 vda5050_bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""
End-to-end test of the fleet manager.

Real: the fleet manager program, a Mosquitto broker, four vehicle processes
      that each run the complete VDA 5050 vehicle side of vda5050_core.
Fake: the vehicles' bodies (a point that moves in a straight line).

This script plays the warehouse system: it sends commands to the fleet
manager and reads its state. It also watches every vehicle's position and
fails if two of them ever come closer than they may.

Usage: fleet_system_test.py <directory with fleet_manager and sim_vehicle>
                            <roadmap.json>
"""

import itertools
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time

PORT = 18831
API = 'fleet/v1'
RADIUS = 0.4
# Positions arrive ten times a second from vehicles driving 1 m/s, each on
# its own clock, so two positions compared may be 0.1 s apart.
SLACK = 0.25
START = {'v1': ('P1', 0.0, -2.0), 'v2': ('P3', 4.0, -2.0), 'v3': ('P5', 8.0, -2.0),
         'v4': ('A3', 4.0, 4.0)}


class Watcher:
    """Reads everything on the broker and keeps what the test asks about."""

    def __init__(self):
        self.lock = threading.Lock()
        self.status = None
        self.responses = {}
        self.events = []
        self.connection = None
        self.positions = {}
        self.closest = (1e9, '')
        self.process = subprocess.Popen(
            ['mosquitto_sub', '-p', str(PORT), '-v', '-t', '#'],
            stdout=subprocess.PIPE, text=True)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.process.stdout:
            topic, _, payload = line.rstrip('\n').partition(' ')
            try:
                message = json.loads(payload)
            except ValueError:
                continue
            with self.lock:
                if topic == API + '/state':
                    self.status = message
                elif topic == API + '/response':
                    self.responses[message.get('requestId', '')] = message
                elif topic == API + '/events':
                    self.events.append(message)
                elif topic == API + '/connection':
                    self.connection = message.get('connectionState')
                elif topic.endswith('/visualization') and 'agvPosition' in message:
                    self._moved(topic.split('/')[3], message['agvPosition'])

    def _moved(self, serial, position):
        self.positions[serial] = (position['x'], position['y'])
        for a, b in itertools.combinations(sorted(self.positions), 2):
            distance = math.dist(self.positions[a], self.positions[b])
            if distance < self.closest[0]:
                self.closest = (distance, f'{a} and {b}')

    def get(self, reader):
        with self.lock:
            return reader(self)


class Test:
    def __init__(self, bin_dir, roadmap):
        self.bin_dir = bin_dir
        self.roadmap = roadmap
        self.processes = {}
        self.logs = tempfile.mkdtemp(prefix='fleet_system_test_')
        self.request = 0
        self.watcher = None

    # ---------------------------------------------------------- processes ----

    def start(self, name, command):
        log = open(os.path.join(self.logs, name + '.log'), 'a')
        self.processes[name] = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)

    def start_vehicle(self, serial, x, y):
        self.start(serial, [
            os.path.join(self.bin_dir, 'sim_vehicle'), '--serial', serial, '--port', str(PORT),
            '--x', str(x), '--y', str(y), '--map-id', 'warehouse', '--action-seconds', '1.0'])

    def stop_all(self):
        for process in self.processes.values():
            if process.poll() is None:
                process.terminate()
        for process in self.processes.values():
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        if self.watcher:
            self.watcher.process.kill()

    # ------------------------------------------------------------ helpers ----

    def fail(self, text):
        print('FAIL: ' + text)
        status = self.watcher.get(lambda w: w.status) if self.watcher else None
        print('---- last state of the fleet ----')
        print(json.dumps(status, indent=1)[:6000])
        print('---- last events ----')
        for event in (self.watcher.get(lambda w: w.events[-40:]) if self.watcher else []):
            print(f"  [{event['time']:8.2f}] {event['kind']} {event['vehicle']} {event['text']}")
        log = os.path.join(self.logs, 'fleet_manager.log')
        if os.path.exists(log):
            print('---- fleet manager output (end) ----')
            print(''.join(open(log).readlines()[-30:]))
        self.stop_all()
        sys.exit(1)

    def wait(self, what, condition, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                if condition():
                    print('ok: ' + what)
                    return
            except (KeyError, TypeError, IndexError):
                pass
            time.sleep(0.1)
        self.fail(what)

    def command(self, body):
        self.request += 1
        request_id = f'r{self.request}'
        body = dict(body, requestId=request_id)
        subprocess.run(
            ['mosquitto_pub', '-p', str(PORT), '-t', API + '/command', '-m', json.dumps(body)],
            check=True)
        deadline = time.time() + 10
        while time.time() < deadline:
            response = self.watcher.get(lambda w: w.responses.get(request_id))
            if response is not None:
                return response
            time.sleep(0.05)
        self.fail(f'no response to {body}')

    def submit(self, stops, **more):
        response = self.command(dict({'command': 'submit', 'stops': stops}, **more))
        if not response.get('ok'):
            self.fail(f'task refused: {response}')
        return response['taskId']

    def vehicle(self, serial):
        for item in self.watcher.get(lambda w: w.status)['vehicles']:
            if item['serialNumber'] == serial:
                return item
        return {}

    def task(self, task_id):
        for item in self.watcher.get(lambda w: w.status)['tasks']:
            if item['taskId'] == task_id:
                return item
        return {}

    def state_of(self, task_id):
        return self.task(task_id).get('state')

    def saw_event(self, kind, vehicle):
        return self.watcher.get(
            lambda w: any(e['kind'] == kind and e['vehicle'] == vehicle for e in w.events))

    # ---------------------------------------------------------------- run ----

    def run(self):
        self.start('broker', ['mosquitto', '-p', str(PORT)])
        time.sleep(1.0)
        self.watcher = Watcher()
        time.sleep(0.5)
        self.start('fleet_manager', [
            os.path.join(self.bin_dir, 'fleet_manager'), '--roadmap', self.roadmap,
            '--port', str(PORT), '--state-timeout', '20', '--status-interval', '0.2'])
        for serial, (_, x, y) in START.items():
            self.start_vehicle(serial, x, y)

        self.wait(
            'the fleet manager says it is online',
            lambda: self.watcher.get(lambda w: w.connection) == 'ONLINE', 15)
        self.wait(
            'four vehicles are found, each at its node, and usable',
            lambda: all(
                self.vehicle(s).get('available') and self.vehicle(s).get('node') == START[s][0]
                for s in START), 30)

        # 1. A command that cannot be carried out is refused, with a reason.
        refused = self.command({'command': 'submit', 'stops': [{'node': 'Nowhere'}]})
        if refused.get('ok') or 'unknown node' not in refused.get('error', ''):
            self.fail(f'a task to an unknown node was not refused properly: {refused}')
        print('ok: a task to a node that does not exist is refused')

        # 2. Eight tasks at once, criss-crossing the warehouse.
        pick = [{'actionType': 'pick'}]
        drop = [{'actionType': 'drop'}]
        first = [
            self.submit([{'node': 'A1', 'actions': pick}, {'node': 'C5', 'actions': drop}]),
            self.submit([{'node': 'A5', 'actions': pick}, {'node': 'C1', 'actions': drop}]),
            self.submit([{'node': 'C3', 'actions': pick}, {'node': 'A3', 'actions': drop}]),
            self.submit([{'node': 'A2', 'actions': pick}, {'node': 'C4', 'actions': drop}]),
            self.submit([{'node': 'C2', 'actions': pick}, {'node': 'A4', 'actions': drop}]),
            self.submit([{'node': 'B1'}, {'node': 'B5'}]),
            self.submit([{'node': 'B5'}, {'node': 'B1'}], priority=5),
            self.submit([{'node': 'B3', 'actions': pick}]),
        ]
        self.wait(
            'eight tasks through one another are all finished',
            lambda: all(self.state_of(t) == 'FINISHED' for t in first), 400)

        # 3. A task is cancelled while its vehicle is under way.
        long_task = self.submit([{'node': 'A5'}, {'node': 'P1'}], vehicle='v1')
        self.wait(
            'a vehicle is under way with a task',
            lambda: self.vehicle('v1').get('grantedMoves', 0) > 0, 30)
        time.sleep(1.5)
        if not self.command({'command': 'cancel', 'taskId': long_task}).get('ok'):
            self.fail('the task could not be cancelled')
        self.wait('the task is reported as cancelled',
                  lambda: self.state_of(long_task) == 'CANCELLED', 10)
        self.wait(
            'its vehicle is stopped, brought to a node, and usable again',
            lambda: self.vehicle('v1').get('available') and
            self.vehicle('v1').get('grantedMoves') == 0 and self.vehicle('v1').get('node'), 40)

        # 4. A vehicle dies in the middle of a task; the rest of the fleet goes on.
        doomed = self.submit([{'node': 'C5'}, {'node': 'A1'}, {'node': 'C5'}], vehicle='v2')
        self.wait(
            'another vehicle is under way',
            lambda: self.vehicle('v2').get('grantedMoves', 0) > 0, 30)
        time.sleep(2.0)
        self.processes['v2'].kill()
        self.processes['v2'].wait()
        self.wait('the dead vehicle is reported as offline',
                  lambda: self.vehicle('v2').get('online') is False, 15)
        held = self.vehicle('v2').get('claimed', [])
        if not held:
            self.fail('a vehicle that went silent gave up what it holds')
        print('ok: it keeps what it held (' + ', '.join(held) + ')')
        where = self.watcher.get(lambda w: w.positions['v2'])
        others = [
            self.submit([{'node': 'A4', 'actions': pick}], vehicle='v3'),
            self.submit([{'node': 'A2', 'actions': pick}], vehicle='v4'),
        ]
        self.wait(
            'the others finish new tasks meanwhile',
            lambda: all(self.state_of(t) == 'FINISHED' for t in others), 240)
        if self.state_of(doomed) != 'ASSIGNED':
            self.fail(f'the task of the dead vehicle is {self.state_of(doomed)}')

        # 5. It is switched on again where it stood, knowing nothing of its order.
        self.start_vehicle('v2', where[0], where[1])
        self.wait('the vehicle is back and usable',
                  lambda: self.vehicle('v2').get('available'), 40)
        self.wait('its task is finished after all',
                  lambda: self.state_of(doomed) == 'FINISHED', 300)

        # 6. The books.
        status = self.watcher.get(lambda w: w.status)
        if status['stats']['invalidPlans'] != 0:
            self.fail('the planner produced a plan that failed its own check')
        if status['stats']['tasksFinished'] != len(first) + len(others) + 1:
            self.fail(f"wrong number of finished tasks: {status['stats']}")
        closest, pair = self.watcher.get(lambda w: w.closest)
        print(f'closest approach of two vehicles: {closest:.2f} m ({pair})')
        if closest < 2 * RADIUS - SLACK:
            self.fail(f'{pair} came within {closest:.2f} m of each other')
        print('ok: no two vehicles ever touched')
        print(f"    {status['stats']}")

        # 7. A clean stop is announced.
        self.processes['fleet_manager'].send_signal(signal.SIGTERM)
        self.wait('the fleet manager says goodbye',
                  lambda: self.watcher.get(lambda w: w.connection) == 'OFFLINE', 10)
        self.stop_all()
        print('PASS')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(2)
    test = Test(sys.argv[1], sys.argv[2])
    try:
        test.run()
    except KeyboardInterrupt:
        test.stop_all()
        sys.exit(130)
