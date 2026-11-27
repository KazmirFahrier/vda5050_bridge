#!/usr/bin/env python3
# Copyright 2026 vda5050_bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""A stand-in for Nav2: accepts every NavigateToPose goal and succeeds shortly after."""

import time

from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node


class FakeNav2(Node):

    def __init__(self):
        super().__init__('fake_nav2')
        self._server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose', self.execute,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT)

    def execute(self, goal_handle):
        pose = goal_handle.request.pose.pose.position
        self.get_logger().info(f'driving to ({pose.x:.1f}, {pose.y:.1f})')
        time.sleep(0.5)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return NavigateToPose.Result()


def main():
    rclpy.init()
    try:
        rclpy.spin(FakeNav2())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
