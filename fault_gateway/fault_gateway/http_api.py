# Copyright 2026 vda5050_bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""
The HTTP side of the fault gateway.

This module knows nothing about ROS. It serves a small REST API, a stream of
server-sent events and one status page, and gets its data from a "backend"
object with three methods (see `Backend`). The ROS node is one backend; the
unit tests use another.

Endpoints
    GET  /                                    status page
    GET  /api/v1/health
    GET  /api/v1/faults                       raised faults and a summary
    GET  /api/v1/faults/<code>                one raised fault
    POST /api/v1/faults/<code>/acknowledge    acknowledge a latched fault
    GET  /api/v1/history                      recorded events; query parameters
                                              code, min_severity, after, limit
    GET  /api/v1/events                       server-sent events; ?after=<seq>
                                              or Last-Event-ID replays what was
                                              missed
"""

import hmac
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import queue
import threading
from urllib.parse import parse_qs, unquote, urlsplit

from fault_gateway.convert import SEVERITIES, summarize

MAX_CODE_LENGTH = 128
MAX_LIMIT = 10000
KEEP_ALIVE_SECONDS = 15.0


class BackendUnavailable(Exception):
    """The fault manager did not answer."""


class Backend:
    """What the HTTP layer needs from the rest of the system."""

    def active_faults(self):
        """Return the raised faults as a list of dictionaries, or None if not yet known."""
        raise NotImplementedError

    def history(self, code, min_severity, after, limit):
        """Return recorded events as a list of dictionaries. May raise BackendUnavailable."""
        raise NotImplementedError

    def acknowledge(self, code):
        """Return (success, reason). May raise BackendUnavailable."""
        raise NotImplementedError


class EventBroker:
    """Hands every published item to every open event stream. Thread-safe."""

    def __init__(self, queue_size=256):
        self._lock = threading.Lock()
        self._subscribers = set()
        self._queue_size = queue_size
        self._closed = False

    def subscribe(self):
        """Return a queue that receives (kind, data, event_id) tuples."""
        subscriber = queue.Queue(maxsize=self._queue_size)
        with self._lock:
            self._subscribers.add(subscriber)
        return subscriber

    def unsubscribe(self, subscriber):
        with self._lock:
            self._subscribers.discard(subscriber)

    def publish(self, kind, data, event_id=None):
        with self._lock:
            subscribers = list(self._subscribers)
        for subscriber in subscribers:
            try:
                subscriber.put_nowait((kind, data, event_id))
            except queue.Full:
                # A client that does not read is dropped; it cannot be allowed
                # to hold memory or slow the others down.
                self.unsubscribe(subscriber)
                _force_put(subscriber, None)

    def close(self):
        """End all streams (used at shutdown)."""
        with self._lock:
            self._closed = True
            subscribers = list(self._subscribers)
            self._subscribers.clear()
        for subscriber in subscribers:
            _force_put(subscriber, None)

    @property
    def closed(self):
        return self._closed

    def subscriber_count(self):
        with self._lock:
            return len(self._subscribers)


def _force_put(target, item):
    """Put an item on a queue, making room if it is full."""
    while True:
        try:
            target.put_nowait(item)
            return
        except queue.Full:
            try:
                target.get_nowait()
            except queue.Empty:
                pass


class _BadRequest(Exception):
    """The request is malformed; answered with 400 and the message."""


class _Handled(Exception):
    """A response has already been sent; routing must stop."""


def _integer(query, name, default, maximum):
    values = query.get(name)
    if not values:
        return default
    try:
        value = int(values[-1])
    except ValueError:
        raise _BadRequest(f"'{name}' must be a whole number") from None
    if value < 0 or value > maximum:
        raise _BadRequest(f"'{name}' must be between 0 and {maximum}")
    return value


def _severity(query):
    values = query.get('min_severity')
    if not values:
        return 0
    name = values[-1].upper()
    if name not in SEVERITIES:
        raise _BadRequest("'min_severity' must be one of " + ', '.join(SEVERITIES))
    return SEVERITIES.index(name)


def _load_page():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static', 'index.html')
    with open(path, 'rb') as page:
        return page.read()


class _Handler(BaseHTTPRequestHandler):
    """One instance per request; `server` carries the shared objects."""

    server_version = 'fault_gateway/0.1'

    # ---- plumbing -----------------------------------------------------------

    def log_message(self, format, *args):  # noqa: A002  (name fixed by the base class)
        self.server.log(format % args)

    def _send(self, status, body, content_type='application/json'):
        if not isinstance(body, bytes):
            body = (json.dumps(body, separators=(',', ':')) + '\n').encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(body)

    def _error(self, status, message):
        self._send(status, {'error': message})

    def _authorized(self):
        token = self.server.api_token
        if not token:
            return True
        header = self.headers.get('Authorization', '')
        return hmac.compare_digest(header.encode('utf-8'), f'Bearer {token}'.encode('utf-8'))

    def _same_origin(self):
        """
        Refuse a state-changing request that another web site made the browser send.

        A page on any site can make a visitor's browser POST to this server.
        Browsers name the calling site in the Origin header; a request from
        this server's own page names this server. Programs such as curl send
        no Origin and are not affected.
        """
        origin = self.headers.get('Origin')
        if origin is None:
            return True
        return urlsplit(origin).netloc == self.headers.get('Host', '')

    def _route(self, method):
        try:
            parts = urlsplit(self.path)
            segments = [unquote(segment) for segment in parts.path.split('/') if segment]
            query = parse_qs(parts.query)
            if not self._dispatch(method, segments, query):
                self._error(HTTPStatus.NOT_FOUND, 'no such resource')
        except _Handled:
            pass
        except _BadRequest as error:
            self._error(HTTPStatus.BAD_REQUEST, str(error))
        except BackendUnavailable:
            self._error(HTTPStatus.SERVICE_UNAVAILABLE, 'the fault manager did not answer')
        except (BrokenPipeError, ConnectionResetError):
            pass  # the client went away; nothing to tell it

    def do_GET(self):  # noqa: N802  (name fixed by the base class)
        self._route('GET')

    def do_POST(self):  # noqa: N802
        # The body is not used, but must be consumed. Refuse large ones.
        try:
            length = int(self.headers.get('Content-Length', '0'))
        except ValueError:
            length = -1
        if length < 0 or length > 4096:
            self._error(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, 'request body too large')
            return
        if length:
            self.rfile.read(length)
        self._route('POST')

    # ---- routes -------------------------------------------------------------

    def _dispatch(self, method, segments, query):
        """Handle the request and return True, or return False for 'not found'."""
        if not segments:
            return self._only('GET', method) and self._page()
        if segments[:2] != ['api', 'v1']:
            return False
        rest = segments[2:]
        if rest == ['health']:
            return self._only('GET', method) and self._health()
        if rest == ['faults']:
            return self._only('GET', method) and self._faults()
        if rest == ['history']:
            return self._only('GET', method) and self._history(query)
        if rest == ['events']:
            return self._only('GET', method) and self._events(query)
        if len(rest) == 2 and rest[0] == 'faults':
            return self._only('GET', method) and self._fault(self._code(rest[1]))
        if len(rest) == 3 and rest[0] == 'faults' and rest[2] == 'acknowledge':
            return self._only('POST', method) and self._acknowledge(self._code(rest[1]))
        return False

    def _only(self, allowed, method):
        if method == allowed:
            return True
        self.send_response(HTTPStatus.METHOD_NOT_ALLOWED)
        self.send_header('Allow', allowed)
        self.send_header('Content-Length', '0')
        self.end_headers()
        raise _Handled  # answered here; the caller must not answer again

    def _code(self, code):
        if not code or len(code) > MAX_CODE_LENGTH:
            raise _BadRequest('invalid fault code')
        return code

    def _page(self):
        self._send(HTTPStatus.OK, self.server.page, 'text/html; charset=utf-8')
        return True

    def _health(self):
        self._send(HTTPStatus.OK, {
            'status': 'ok',
            'faultManager': self.server.backend.active_faults() is not None,
        })
        return True

    def _active(self):
        faults = self.server.backend.active_faults()
        if faults is None:
            raise BackendUnavailable
        return faults

    def _faults(self):
        faults = self._active()
        self._send(HTTPStatus.OK, {'faults': faults, 'summary': summarize(faults)})
        return True

    def _fault(self, code):
        for fault in self._active():
            if fault['code'] == code:
                self._send(HTTPStatus.OK, fault)
                return True
        self._error(HTTPStatus.NOT_FOUND, 'this fault is not raised')
        return True

    def _history(self, query):
        code = query.get('code', [''])[-1]
        if len(code) > MAX_CODE_LENGTH:
            raise _BadRequest('invalid fault code')
        events = self.server.backend.history(
            code, _severity(query), _integer(query, 'after', 0, 2 ** 63 - 1),
            _integer(query, 'limit', 100, MAX_LIMIT))
        self._send(HTTPStatus.OK, {'events': events})
        return True

    def _acknowledge(self, code):
        if not self._same_origin():
            self._error(HTTPStatus.FORBIDDEN, 'cross-site request refused')
            return True
        if not self._authorized():
            self.send_response(HTTPStatus.UNAUTHORIZED)
            self.send_header('WWW-Authenticate', 'Bearer')
            self.send_header('Content-Length', '0')
            self.end_headers()
            return True
        success, reason = self.server.backend.acknowledge(code)
        if success:
            self._send(HTTPStatus.OK, {'acknowledged': True, 'code': code})
        else:
            self._send(
                HTTPStatus.CONFLICT, {'acknowledged': False, 'code': code, 'reason': reason})
        return True

    def _events(self, query):
        after = _integer(query, 'after', 0, 2 ** 63 - 1)
        last_id = self.headers.get('Last-Event-ID', '')
        if last_id.isdigit():
            after = max(after, int(last_id))  # a browser reconnecting by itself
        replay = 'after' in query or last_id.isdigit()

        # Subscribe first, then replay: an event that happens in between is
        # delivered twice at worst (and filtered below), never lost.
        broker = self.server.broker
        subscriber = broker.subscribe()
        try:
            missed = []
            if replay:
                try:
                    missed = self.server.backend.history('', 0, after, MAX_LIMIT)
                except BackendUnavailable:
                    missed = []
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Accel-Buffering', 'no')
            self.end_headers()

            faults = self.server.backend.active_faults()
            if faults is not None:
                self._write_event('faults', {'faults': faults, 'summary': summarize(faults)})
            last_sent = after
            for event in missed:
                self._write_event('fault_event', event, event['sequence'])
                last_sent = max(last_sent, event['sequence'])

            while not broker.closed:
                try:
                    item = subscriber.get(timeout=self.server.keep_alive_seconds)
                except queue.Empty:
                    self.wfile.write(b': keep-alive\n\n')
                    self.wfile.flush()
                    continue
                if item is None:
                    break
                kind, data, event_id = item
                if kind == 'fault_event' and event_id is not None:
                    if replay and event_id <= last_sent:
                        continue  # already sent during the replay
                    last_sent = max(last_sent, event_id)
                self._write_event(kind, data, event_id)
        finally:
            broker.unsubscribe(subscriber)
        return True

    def _write_event(self, kind, data, event_id=None):
        text = f'event: {kind}\n'
        if event_id is not None:
            text += f'id: {event_id}\n'
        text += 'data: ' + json.dumps(data, separators=(',', ':')) + '\n\n'
        self.wfile.write(text.encode('utf-8'))
        self.wfile.flush()


class FaultHttpServer(ThreadingHTTPServer):
    """The HTTP server with the objects its request handlers share."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, backend, broker, api_token='', log=None,
                 keep_alive_seconds=KEEP_ALIVE_SECONDS):
        super().__init__(address, _Handler)
        self.backend = backend
        self.broker = broker
        self.api_token = api_token
        self.keep_alive_seconds = keep_alive_seconds
        self.page = _load_page()
        self.log = log if log is not None else (lambda message: None)

    def stop(self):
        """Stop serving and end all event streams."""
        self.broker.close()
        self.shutdown()
        self.server_close()
