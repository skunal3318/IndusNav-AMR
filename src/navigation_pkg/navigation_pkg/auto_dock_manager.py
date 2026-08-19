#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, Bool
from nav2_msgs.action import DockRobot, UndockRobot
from nav2_simple_commander.robot_navigator import BasicNavigator
import threading

class AutoDockManager(Node):
    def __init__(self):
        super().__init__('auto_dock_manager')
        
        self.state = 'IDLE'
        self.low_battery_threshold = 0.20
        self.full_battery_threshold = 0.95
        
        self.state_pub = self.create_publisher(String, '/dock_manager_state', 10)
        self.charging_pub = self.create_publisher(Bool, '/is_charging', 10)
        
        self.create_subscription(BatteryState, '/battery_state', self.battery_cb, 10)
        
        self._dock_client = ActionClient(self, DockRobot, 'dock_robot')
        self._undock_client = ActionClient(self, UndockRobot, 'undock_robot')
        
        self.navigator = BasicNavigator()

        self.create_timer(1.0, self.timer_cb)
        self.get_logger().info("Auto Dock Manager started and waiting...")

    def battery_cb(self, msg):
        if self.state == 'IDLE' and msg.percentage <= self.low_battery_threshold:
            self.get_logger().warn(f"Battery Low ({msg.percentage*100}%). Starting Docking sequence.")
            self.state = 'LOW_BATTERY'
            threading.Thread(target=self.execute_docking).start()
            
        elif self.state == 'CHARGING' and msg.percentage >= self.full_battery_threshold:
            self.get_logger().info("Battery Full. Starting Undocking.")
            self.state = 'UNDOCKING'
            threading.Thread(target=self.execute_undocking).start()

    def timer_cb(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)
        
        charge_msg = Bool()
        charge_msg.data = (self.state == 'CHARGING')
        self.charging_pub.publish(charge_msg)

    def execute_docking(self):
        self.get_logger().info("Cancelling active navigation goals...")
        self.navigator.cancelTask()
        
        self.state = 'DOCKING'
        if not self._dock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Docking server not available!")
            self.state = 'IDLE'
            return

        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = True
        goal_msg.dock_id = 'home_dock' 

        self.get_logger().info("Sending Docking Goal...")
        send_goal_future = self._dock_client.send_goal_async(goal_msg)
        
        while not send_goal_future.done():
            pass
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Docking goal rejected!")
            self.state = 'IDLE'
            return

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            pass
        
        self.get_logger().info("Docking successful! Transitioning to CHARGING.")
        self.state = 'CHARGING'

    def execute_undocking(self):
        if not self._undock_client.wait_for_server(timeout_sec=5.0):
            self.state = 'CHARGING'
            return

        self.get_logger().info("Sending Undocking Goal...")
        goal_msg = UndockRobot.Goal()
        undock_future = self._undock_client.send_goal_async(goal_msg)
        
        while not undock_future.done():
            pass
            
        self.get_logger().info("Undocking complete. Returning to IDLE.")
        self.state = 'IDLE'

def main(args=None):
    rclpy.init(args=args)
    node = AutoDockManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
