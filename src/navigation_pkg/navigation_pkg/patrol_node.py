#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from math import sin, cos
import time


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.waypoints = [
            [-5.0,  8.0,  0.00],
            [ 5.0,  2.0, -1.57],
            [ 2.0, -8.0,  3.14],
            [-5.0, -8.0,  1.57],
        ]

        self._first_run     = True
        self._dock_state    = 'IDLE'     
        self._patrol_active = False
        self._goal_handle   = None  # NEW: Keep track of the active goal    

        self._wp_publisher = self.create_publisher(PoseArray, '/patrol_waypoints', 10)

        self.create_subscription(
            String, '/dock_manager_state', self._dock_state_cb, 10
        )

        self.create_timer(2.0, self._publish_waypoints)

    def _dock_state_cb(self, msg: String):
        prev = self._dock_state
        self._dock_state = msg.data

        # 1. Battery recharged — resuming patrol
        if prev != 'IDLE' and self._dock_state == 'IDLE' and not self._patrol_active:
            self.get_logger().info('Battery recharged — resuming patrol')
            self.send_points()

        # 2. Battery low — cancelling current patrol to go dock
        elif prev == 'IDLE' and self._dock_state != 'IDLE' and self._patrol_active:
            if self._goal_handle is not None:
                self.get_logger().warn('Low battery detected! Cancelling patrol to allow docking.')
                self._goal_handle.cancel_goal_async()
                self._goal_handle = None

    def _publish_waypoints(self):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = [self.create_pose(wp[0], wp[1], wp[2]).pose for wp in self.waypoints]
        self._wp_publisher.publish(msg)

    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = sin(yaw / 2)
        pose.pose.orientation.w = cos(yaw / 2)
        return pose

    def send_points(self):
        if self._dock_state != 'IDLE':
            self.get_logger().warn(
                f'Patrol skipped — dock state is [{self._dock_state}]. '
                'Will auto-resume when IDLE.'
            )
            return

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [
            self.create_pose(wp[0], wp[1], wp[2]) for wp in self.waypoints
        ]

        self._action_client.wait_for_server(timeout_sec=5.0)

        if self._first_run:
            self.get_logger().info('First run — waiting 10 s for Nav2 to activate...')
            time.sleep(10.0)
            self._first_run = False

        self.get_logger().info(f'Sending patrol: {len(self.waypoints)} waypoints')
        self._patrol_active = True
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self._goal_handle = future.result()  # NEW: Save the handle so we can cancel it later
        
        if not self._goal_handle.accepted:
            self.get_logger().error('Patrol goal rejected by Nav2')
            self._patrol_active = False
            self._goal_handle = None
            return
            
        self.get_logger().info('Patrol goal accepted')
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self._patrol_active = False
        self._goal_handle = None  # Clear the handle
        
        status = future.result().status
        result = future.result().result

        # Handle explicit cancellation gracefully
        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Patrol safely cancelled. Handing control to docking server.')
            return

        missed = getattr(result, 'missed_waypoints', [])

        for wp in missed:
            i = wp.index
            x = self.waypoints[i][0]
            y = self.waypoints[i][1]
            if x >= 4 and x <= 8 and y >= -3 and y <= 3:
                self.get_logger().warn(
                    f'Waypoint {i} ({x:.2f}, {y:.2f}) skipped — inside DANGER ZONE'
                )
            else:
                self.get_logger().warn(
                    f'Waypoint {i} ({x:.2f}, {y:.2f}) skipped — blocked path'
                )

        if self._dock_state != 'IDLE':
            self.get_logger().info(
                f'Patrol ended. Dock state = [{self._dock_state}]. '
                'Will restart after charging completes.'
            )
            return

        self.get_logger().info('Patrol complete — restarting loop')
        self.send_points()

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        node.send_points()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()