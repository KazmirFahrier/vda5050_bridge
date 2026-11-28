# Copyright 2026 vda5050_bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""
ROS 2 node that serves the fault manager's data over HTTP.

The node subscribes to faults/active and faults/events and calls the fault
manager's services. The HTTP server runs in its own threads; rclpy spins in
the main thread. The two meet in three places, each guarded:

- the latest list of active faults (a lock),
- the event broker (thread-safe by itself),
- service calls: an HTTP thread sends the request and waits, with a timeout,
  for the rclpy thread to deliver the answer.
"""

import threading

from fault_gateway.convert import event_to_dict, fault_to_dict, summarize
from fault_gateway.http_api import Backend, BackendUnavailable, EventBroker, FaultHttpServer
from fault_msgs.msg import FaultArray, FaultEvent
from fault_msgs.srv import AcknowledgeFault, GetFaultHistory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class GatewayNode(Node, Backend):
    """The backend of the HTTP server, implemented with ROS topics and services."""

    def __init__(self):
        super().__init__('fault_gateway')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8080)
        self.declare_parameter('api_token', '')
        self.declare_parameter('service_timeout_s', 2.0)

        self._timeout = self.get_parameter('service_timeout_s').value
        self._lock = threading.Lock()
        self._faults = None  # None until the fault manager has been heard
        self.broker = EventBroker()

        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._active_subscription = self.create_subscription(
            FaultArray, 'faults/active', self._on_active, latched)
        self._event_subscription = self.create_subscription(
            FaultEvent, 'faults/events', self._on_event, 100)
        self._acknowledge_client = self.create_client(AcknowledgeFault, 'faults/acknowledge')
        self._history_client = self.create_client(GetFaultHistory, 'faults/get_history')

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        token = self.get_parameter('api_token').value
        self.server = FaultHttpServer(
            (host, port), self, self.broker, api_token=token,
            log=lambda message: self.get_logger().debug(message))
        self._server_thread = threading.Thread(
            target=self.server.serve_forever, name='http', daemon=True)
        self._server_thread.start()

        self.get_logger().info(f'serving on http://{host}:{self.server.server_address[1]}/')
        if host not in ('127.0.0.1', 'localhost', '::1') and not token:
            self.get_logger().warning(
                'reachable from other machines without an access token: '
                'anyone on the network can acknowledge faults. Set api_token.')

    # ---- ROS callbacks (rclpy thread) ----------------------------------------

    def _on_active(self, message):
        faults = [fault_to_dict(fault) for fault in message.faults]
        with self._lock:
            changed = faults != self._faults
            self._faults = faults
        if changed:  # the fault manager repeats the list periodically
            self.broker.publish('faults', {'faults': faults, 'summary': summarize(faults)})

    def _on_event(self, message):
        event = event_to_dict(message)
        self.broker.publish('fault_event', event, event['sequence'])

    # ---- Backend interface (HTTP threads) --------------------------------------

    def active_faults(self):
        # The list is latched, so the last one received would be served for
        # ever. If nobody publishes it any more, the fault manager is gone and
        # the honest answer is "unknown", not the old list.
        if self.count_publishers(self._active_subscription.topic_name) == 0:
            return None
        with self._lock:
            return None if self._faults is None else list(self._faults)

    def history(self, code, min_severity, after, limit):
        request = GetFaultHistory.Request()
        request.code = code
        request.min_severity = min_severity
        request.after_sequence = after
        request.limit = limit
        response = self._call(self._history_client, request)
        return [event_to_dict(event) for event in response.events]

    def acknowledge(self, code):
        request = AcknowledgeFault.Request()
        request.code = code
        response = self._call(self._acknowledge_client, request)
        return response.success, response.message

    def _call(self, client, request):
        if not client.service_is_ready():
            raise BackendUnavailable
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self._timeout):
            client.remove_pending_request(future)
            raise BackendUnavailable
        response = future.result()
        if response is None:
            raise BackendUnavailable
        return response

    def stop(self):
        self.server.stop()


def main(args=None):
    rclpy.init(args=args)
    node = GatewayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
