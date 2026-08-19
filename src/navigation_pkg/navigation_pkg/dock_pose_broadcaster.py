#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster


class DockPoseBroadcaster(Node):
    def __init__(self):
        super().__init__('dock_pose_broadcaster')

        self.declare_parameter('dock_x',       0.0)
        self.declare_parameter('dock_y',       0.0)
        self.declare_parameter('dock_yaw_deg', 0.0)   
        self.declare_parameter('publish_frequency', 1.0)

        x       = self.get_parameter('dock_x').value
        y       = self.get_parameter('dock_y').value
        yaw_deg = self.get_parameter('dock_yaw_deg').value
        freq    = self.get_parameter('publish_frequency').value

        yaw = math.radians(yaw_deg)
        self._qz = math.sin(yaw / 2.0)
        self._qw = math.cos(yaw / 2.0)
        self._x  = x
        self._y  = y

        self._tf_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp            = self.get_clock().now().to_msg()
        t.header.frame_id         = 'map'
        t.child_frame_id          = 'charging_dock'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x    = 0.0
        t.transform.rotation.y    = 0.0
        t.transform.rotation.z    = self._qz
        t.transform.rotation.w    = self._qw
        self._tf_broadcaster.sendTransform(t)

        self._pose_pub = self.create_publisher(PoseStamped, '/charging_dock_pose', 10)
        self.create_timer(1.0 / freq, self._publish_pose)

        self.get_logger().info(
            f'DockPoseBroadcaster: dock at map[{x:.2f}, {y:.2f}, {yaw_deg:.1f}°]'
        )

    def _publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x    = self._x
        msg.pose.position.y    = self._y
        msg.pose.position.z    = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = self._qz
        msg.pose.orientation.w = self._qw
        self._pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DockPoseBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
