# Copyright 2026 vda5050_bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""
Tests of the HTTP layer against a stand-in backend.

A real server is started on a free port of the loopback interface and real
HTTP requests are sent to it; only the fault manager behind it is replaced.
"""

import http.client
import json
import threading
import time

from fault_gateway.http_api import Backend, BackendUnavailable, EventBroker, FaultHttpServer
import pytest

LIDAR = {'code': 'LIDAR-001', 'state': 'ACTIVE', 'severity': 'CRITICAL',
         'description': 'Lidar silent', 'message': '', 'source': 'lidar', 'known': True,
         'occurrenceCount': 1}
BUMPER = {'code': 'BUMPER 1/a', 'state': 'AWAITING_ACK', 'severity': 'WARNING',
          'description': 'Bumper', 'message': '', 'source': '', 'known': True,
          'occurrenceCount': 3}


def event(sequence, code='LIDAR-001'):
    return {'sequence': sequence, 'type': 'ACTIVATED', 'code': code, 'severity': 'ERROR',
            'time': '2026-11-26T19:50:00.000Z', 'message': '', 'source': '',
            'occurrenceCount': 1}


class FakeBackend(Backend):
    """Stands in for the fault manager; records what it was asked."""

    def __init__(self):
        self.faults = [LIDAR, BUMPER]
        self.events = [event(1), event(2), event(3)]
        self.history_calls = []
        self.acknowledged = []
        self.available = True

    def active_faults(self):
        return self.faults

    def history(self, code, min_severity, after, limit):
        if not self.available:
            raise BackendUnavailable
        self.history_calls.append((code, min_severity, after, limit))
        return [item for item in self.events if item['sequence'] > after]

    def acknowledge(self, code):
        if not self.available:
            raise BackendUnavailable
        self.acknowledged.append(code)
        if code == BUMPER['code']:
            return True, ''
        return False, 'fault is ACTIVE, not AWAITING_ACK'


class Running:
    """A server on a free port, with helpers to talk to it."""

    def __init__(self, api_token=''):
        self.backend = FakeBackend()
        self.broker = EventBroker()
        self.server = FaultHttpServer(
            ('127.0.0.1', 0), self.backend, self.broker, api_token=api_token,
            keep_alive_seconds=0.2)
        self.port = self.server.server_address[1]
        self.thread = threading.Thread(
            target=self.server.serve_forever, kwargs={'poll_interval': 0.02}, daemon=True)
        self.thread.start()

    def request(self, method, path, headers=None, body=None):
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=5)
        try:
            connection.request(method, path, body=body, headers=headers or {})
            response = connection.getresponse()
            return response.status, dict(response.getheaders()), response.read()
        finally:
            connection.close()

    def get_json(self, path):
        status, _, body = self.request('GET', path)
        return status, json.loads(body)

    def stop(self):
        self.server.stop()
        self.thread.join(timeout=5)


@pytest.fixture
def running():
    instance = Running()
    yield instance
    instance.stop()


def read_events(response, count, timeout=5.0):
    """Read `count` server-sent events as (name, id, data) tuples. Comments are skipped."""
    events = []
    name, event_id, data = None, None, None
    deadline = time.monotonic() + timeout
    while len(events) < count and time.monotonic() < deadline:
        line = response.fp.readline().decode('utf-8').rstrip('\n')
        if line.startswith('event: '):
            name = line[len('event: '):]
        elif line.startswith('id: '):
            event_id = int(line[len('id: '):])
        elif line.startswith('data: '):
            data = json.loads(line[len('data: '):])
        elif line == '' and name is not None:
            events.append((name, event_id, data))
            name, event_id, data = None, None, None
    return events


def open_stream(running, path='/api/v1/events', headers=None):
    connection = http.client.HTTPConnection('127.0.0.1', running.port, timeout=5)
    connection.request('GET', path, headers=headers or {})
    return connection, connection.getresponse()


def wait_for_subscribers(broker, count):
    deadline = time.monotonic() + 5
    while broker.subscriber_count() != count and time.monotonic() < deadline:
        time.sleep(0.01)
    assert broker.subscriber_count() == count


# ---- plain requests --------------------------------------------------------------


def test_health(running):
    assert running.get_json('/api/v1/health') == (200, {'status': 'ok', 'faultManager': True})
    running.backend.faults = None
    assert running.get_json('/api/v1/health')[1]['faultManager'] is False


def test_faults_with_summary(running):
    status, body = running.get_json('/api/v1/faults')
    assert status == 200
    assert [fault['code'] for fault in body['faults']] == ['LIDAR-001', 'BUMPER 1/a']
    assert body['summary'] == {
        'raised': 2, 'highest': 'CRITICAL',
        'bySeverity': {'INFO': 0, 'WARNING': 1, 'ERROR': 0, 'CRITICAL': 1}}


def test_faults_unavailable_until_the_fault_manager_was_heard(running):
    running.backend.faults = None
    status, body = running.get_json('/api/v1/faults')
    assert status == 503
    assert 'error' in body


def test_no_faults_is_an_empty_list_not_an_error(running):
    running.backend.faults = []
    status, body = running.get_json('/api/v1/faults')
    assert status == 200
    assert body['faults'] == []
    assert body['summary']['highest'] is None


def test_single_fault(running):
    assert running.get_json('/api/v1/faults/LIDAR-001') == (200, LIDAR)
    assert running.get_json('/api/v1/faults/NOPE')[0] == 404
    # Codes are percent-decoded, so unusual characters work.
    assert running.get_json('/api/v1/faults/BUMPER%201%2Fa') == (200, BUMPER)


def test_history_passes_filters_to_the_backend(running):
    status, body = running.get_json(
        '/api/v1/history?code=LIDAR-001&min_severity=error&after=1&limit=5')
    assert status == 200
    assert [item['sequence'] for item in body['events']] == [2, 3]
    assert running.backend.history_calls == [('LIDAR-001', 2, 1, 5)]


def test_history_defaults(running):
    running.get_json('/api/v1/history')
    assert running.backend.history_calls == [('', 0, 0, 100)]


@pytest.mark.parametrize('query', [
    'limit=abc', 'limit=-1', 'limit=10001', 'after=x', 'min_severity=LOUD',
    'code=' + 'x' * 200,
])
def test_history_rejects_bad_parameters(running, query):
    status, body = running.get_json('/api/v1/history?' + query)
    assert status == 400
    assert 'error' in body
    assert running.backend.history_calls == []


def test_backend_failure_is_503(running):
    running.backend.available = False
    assert running.get_json('/api/v1/history')[0] == 503
    assert running.request('POST', '/api/v1/faults/X/acknowledge')[0] == 503


def test_acknowledge(running):
    status, _, body = running.request('POST', '/api/v1/faults/BUMPER%201%2Fa/acknowledge')
    assert status == 200
    assert json.loads(body) == {'acknowledged': True, 'code': 'BUMPER 1/a'}
    assert running.backend.acknowledged == ['BUMPER 1/a']


def test_acknowledge_refused_is_409_with_the_reason(running):
    status, _, body = running.request('POST', '/api/v1/faults/LIDAR-001/acknowledge')
    assert status == 409
    assert json.loads(body) == {
        'acknowledged': False, 'code': 'LIDAR-001',
        'reason': 'fault is ACTIVE, not AWAITING_ACK'}


def test_unknown_paths_are_404(running):
    for path in ['/nope', '/api/v2/faults', '/api/v1', '/api/v1/faults/a/b/c',
                 '/api/v1/faults/a/delete']:
        assert running.request('GET', path)[0] == 404, path


def test_wrong_method_is_405(running):
    status, headers, _ = running.request('POST', '/api/v1/faults')
    assert (status, headers['Allow']) == (405, 'GET')
    status, headers, _ = running.request('GET', '/api/v1/faults/BUMPER/acknowledge')
    assert (status, headers['Allow']) == (405, 'POST')
    assert running.backend.acknowledged == []


def test_oversized_post_body_is_refused(running):
    status, _, _ = running.request(
        'POST', '/api/v1/faults/X/acknowledge', body=b'x' * 5000)
    assert status == 413
    assert running.backend.acknowledged == []


def test_status_page(running):
    status, headers, body = running.request('GET', '/')
    assert status == 200
    assert headers['Content-Type'].startswith('text/html')
    assert b'<title>Robot faults</title>' in body
    assert b'innerHTML' not in body  # fault texts must never be inserted as HTML


def test_responses_are_not_cacheable(running):
    _, headers, _ = running.request('GET', '/api/v1/faults')
    assert headers['Cache-Control'] == 'no-store'
    assert headers['X-Content-Type-Options'] == 'nosniff'


# ---- access token ------------------------------------------------------------------


def test_token_protects_acknowledge_but_not_reading():
    running = Running(api_token='s3cret')
    try:
        path = '/api/v1/faults/BUMPER%201%2Fa/acknowledge'
        status, headers, _ = running.request('POST', path)
        assert status == 401
        assert headers['WWW-Authenticate'] == 'Bearer'
        assert running.request('POST', path, {'Authorization': 'Bearer wrong'})[0] == 401
        assert running.request('POST', path, {'Authorization': 's3cret'})[0] == 401
        assert running.backend.acknowledged == []

        assert running.request('POST', path, {'Authorization': 'Bearer s3cret'})[0] == 200
        assert running.backend.acknowledged == ['BUMPER 1/a']
        assert running.get_json('/api/v1/faults')[0] == 200
    finally:
        running.stop()


def test_acknowledge_from_another_web_site_is_refused(running):
    path = '/api/v1/faults/BUMPER%201%2Fa/acknowledge'
    status, _, body = running.request('POST', path, {'Origin': 'http://evil.example'})
    assert status == 403
    assert 'error' in json.loads(body)
    assert running.backend.acknowledged == []

    # The gateway's own page names this server as its origin and is accepted.
    own = f'http://127.0.0.1:{running.port}'
    assert running.request('POST', path, {'Origin': own})[0] == 200
    assert running.backend.acknowledged == ['BUMPER 1/a']


# ---- server-sent events -----------------------------------------------------------


def test_stream_starts_with_the_current_faults_then_delivers_events(running):
    connection, response = open_stream(running)
    try:
        assert response.status == 200
        assert response.getheader('Content-Type') == 'text/event-stream'
        first = read_events(response, 1)
        assert first[0][0] == 'faults'
        assert len(first[0][2]['faults']) == 2

        wait_for_subscribers(running.broker, 1)
        running.broker.publish('fault_event', event(4), 4)
        running.broker.publish('faults', {'faults': [], 'summary': {}})
        received = read_events(response, 2)
        assert received[0] == ('fault_event', 4, event(4))
        assert received[1][0] == 'faults'
    finally:
        connection.close()


def test_stream_replays_missed_events(running):
    connection, response = open_stream(running, '/api/v1/events?after=1')
    try:
        received = read_events(response, 3)
        assert [(name, event_id) for name, event_id, _ in received] == [
            ('faults', None), ('fault_event', 2), ('fault_event', 3)]

        # An event already sent in the replay is not sent twice; a new one is.
        wait_for_subscribers(running.broker, 1)
        running.broker.publish('fault_event', event(3), 3)
        running.broker.publish('fault_event', event(4), 4)
        assert read_events(response, 1) == [('fault_event', 4, event(4))]
    finally:
        connection.close()


def test_stream_replays_after_a_browser_reconnect(running):
    connection, response = open_stream(running, headers={'Last-Event-ID': '2'})
    try:
        received = read_events(response, 2)
        assert [(name, event_id) for name, event_id, _ in received] == [
            ('faults', None), ('fault_event', 3)]
    finally:
        connection.close()


def test_stream_sends_keep_alive_comments(running):
    connection, response = open_stream(running)
    try:
        read_events(response, 1)
        lines = [response.fp.readline() for _ in range(2)]
        assert b': keep-alive\n' in lines
    finally:
        connection.close()


def test_stream_ends_when_the_server_stops():
    running = Running()
    connection, response = open_stream(running)
    try:
        read_events(response, 1)
        wait_for_subscribers(running.broker, 1)
        running.stop()
        assert response.fp.read() in (b'', b': keep-alive\n\n')
    finally:
        connection.close()


def test_closed_streams_are_forgotten(running):
    connection, response = open_stream(running)
    read_events(response, 1)
    wait_for_subscribers(running.broker, 1)
    response.close()  # the response holds the socket open; close both
    connection.close()
    # The server notices on its next write (keep-alive every 0.2 s here).
    deadline = time.monotonic() + 5
    while running.broker.subscriber_count() and time.monotonic() < deadline:
        running.broker.publish('faults', {'faults': [], 'summary': {}})
        time.sleep(0.05)
    assert running.broker.subscriber_count() == 0


def test_slow_reader_is_dropped_not_buffered_forever():
    broker = EventBroker(queue_size=3)
    subscriber = broker.subscribe()
    for number in range(10):
        broker.publish('fault_event', event(number), number)
    assert broker.subscriber_count() == 0
    items = []
    while not subscriber.empty():
        items.append(subscriber.get_nowait())
    assert items[-1] is None  # told to stop
    assert len(items) <= 3
