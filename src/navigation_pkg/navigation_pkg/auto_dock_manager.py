#!/usr/bin/env python3
"""
auto_dock_manager.py 

State machine:
  IDLE -> LOW_BATTERY -> DOCKING -> CHARGING -> UNDOCKING -> IDLE

Topics published
  /is_charging         std_msgs/Bool
  /dock_manager_state  std_msgs/String

Topics subscribed
  /battery_state       sensor_msgs/BatteryState

Action clients
  /dock_robot          nav2_msgs/action/DockRobot
  /undock_robot        nav2_msgs/action/UndockRobot
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, Point, Quaternion

try:
    from nav2_msgs.action import DockRobot, UndockRobot
    _HAVE_DOCK_ACTIONS = True
except ImportError:
    _HAVE_DOCK_ACTIONS = False

from enum import Enum, auto


class State(Enum):
    IDLE        = auto()
    LOW_BATTERY = auto()
    DOCKING     = auto()
    CHARGING    = auto()
    UNDOCKING   = auto()


class AutoDockManager(Node):
    def __init__(self):
        super().__init__('auto_dock_manager')

        self.declare_parameter('low_battery_threshold',  0.20)
        self.declare_parameter('full_battery_threshold', 0.95)
        self.declare_parameter('check_frequency',         1.0)
        self.declare_parameter('dock_id',               'home_dock')
        self.declare_parameter('dock_type',             'simple_charging_dock')
        self.declare_parameter('dock_pose', [0.0, 0.0, 0.0])   # [x, y, yaw_deg]

        self._low_thr   = self.get_parameter('low_battery_threshold').value
        self._full_thr  = self.get_parameter('full_battery_threshold').value
        self._dock_id   = self.get_parameter('dock_id').value
        self._dock_type = self.get_parameter('dock_type').value
        dp = self.get_parameter('dock_pose').value
        self._dock_pose = {'x': float(dp[0]), 'y': float(dp[1]), 'yaw_deg': float(dp[2])}

        self._state   = State.IDLE
        self._battery = 1.0
        self._lock    = threading.Lock()

        if not _HAVE_DOCK_ACTIONS:
            self.get_logger().error(
                'nav2_msgs.action.DockRobot not found. '
                'Run:  sudo apt install ros-jazzy-nav2-msgs  then rebuild.'
            )

        cb = ReentrantCallbackGroup()

        self._charging_pub = self.create_publisher(Bool,   '/is_charging',        10)
        self._state_pub    = self.create_publisher(String, '/dock_manager_state',  10)

    
        self.create_subscription(
            BatteryState, '/battery_state',
            self._battery_cb, 10, callback_group=cb
        )

  
        if _HAVE_DOCK_ACTIONS:
            self._dock_client = ActionClient(
                self, DockRobot,   '/dock_robot',   callback_group=cb
            )
            self._undock_client = ActionClient(
                self, UndockRobot, '/undock_robot', callback_group=cb
            )

        self.create_timer(
            1.0 / self.get_parameter('check_frequency').value,
            self._check_battery, callback_group=cb
        )

        self.get_logger().info(
            f'AutoDockManager ready  '
            f'low={self._low_thr*100:.0f}%  full={self._full_thr*100:.0f}%  '
            f'dock_pose={self._dock_pose}'
        )

    def _battery_cb(self, msg: BatteryState):
        with self._lock:
            self._battery = float(msg.percentage)

    def _check_battery(self):
        with self._lock:
            pct   = self._battery
            state = self._state

        self._publish_state()

        if state == State.IDLE and pct <= self._low_thr:
            self.get_logger().warn(
                f'Battery {pct*100:.1f}% <= {self._low_thr*100:.0f}% -- auto-docking'
            )
            self._transition(State.LOW_BATTERY)
            threading.Thread(target=self._do_dock, daemon=True).start()

        elif state == State.CHARGING and pct >= self._full_thr:
            self.get_logger().info(
                f'Battery {pct*100:.1f}% -- fully charged, undocking'
            )
            self._set_charging(False)
            self._transition(State.UNDOCKING)
            threading.Thread(target=self._do_undock, daemon=True).start()

    def _transition(self, new_state: State):
        self.get_logger().info(f'State {self._state.name} -> {new_state.name}')
        with self._lock:
            self._state = new_state
        self._publish_state()

    def _publish_state(self):
        with self._lock:
            s = self._state.name
        msg = String()
        msg.data = s
        self._state_pub.publish(msg)

    def _set_charging(self, charging: bool):
        msg = Bool()
        msg.data = charging
        self._charging_pub.publish(msg)

    def _make_dock_pose(self) -> PoseStamped:
        ps = PoseStamped()
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position   = Point(
            x=self._dock_pose['x'],
            y=self._dock_pose['y'],
            z=0.0
        )
        yaw = math.radians(self._dock_pose['yaw_deg'])
        ps.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0)
        )
        return ps

    def _do_dock(self):
        if not _HAVE_DOCK_ACTIONS:
            self.get_logger().error('DockRobot action unavailable -- IDLE')
            self._transition(State.IDLE)
            return

        if not self._dock_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('DockRobot server not available -- IDLE')
            self._transition(State.IDLE)
            return

        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id     = self._dock_id

        self.get_logger().info(f'DockRobot: dock_id={self._dock_id}')
        self._transition(State.DOCKING)

        send_future = self._dock_client.send_goal_async(
            goal, feedback_callback=self._dock_feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('dock_id rejected, retrying with explicit pose')
            goal.use_dock_id  = False
            goal.dock_type    = self._dock_type
            goal.dock_pose    = self._make_dock_pose()
            send_future = self._dock_client.send_goal_async(
                goal, feedback_callback=self._dock_feedback_cb
            )
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
            goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('DockRobot rejected -- IDLE')
            self._transition(State.IDLE)
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        result = result_future.result()

        if result and result.result.success:
            self.get_logger().info('Docked successfully -- CHARGING')
            self._set_charging(True)
            self._transition(State.CHARGING)
        else:
            self.get_logger().error('Docking failed -- IDLE')
            self._transition(State.IDLE)

    def _dock_feedback_cb(self, feedback_msg):
        self.get_logger().debug(f'Dock feedback: {feedback_msg.feedback}')


    def _do_undock(self):
        if not _HAVE_DOCK_ACTIONS:
            self._transition(State.IDLE)
            return

        if not self._undock_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('UndockRobot server not available')
            self._transition(State.IDLE)
            return

        goal = UndockRobot.Goal()
        goal.dock_type = self._dock_type

        self.get_logger().info('UndockRobot goal sending')
        send_future = self._undock_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('UndockRobot rejected')
            self._transition(State.IDLE)
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        self.get_logger().info('Undocking complete -- IDLE')
        self._transition(State.IDLE)


def main(args=None):
    rclpy.init(args=args)
    node = AutoDockManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()