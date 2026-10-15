#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class BatterySimulator(Node):
    def __init__(self):
        super().__init__('battery_simulator')

        self.declare_parameter('initial_percentage',  1.0)
        self.declare_parameter('idle_drain_rate',     0.0005)
        self.declare_parameter('motion_drain_rate',   0.004)
        self.declare_parameter('cmd_vel_timeout',     0.5)
        self.declare_parameter('charge_rate',         0.003)
        self.declare_parameter('publish_frequency',  10.0)
        self.declare_parameter('low_battery_threshold', 0.20)
        self.declare_parameter('full_battery_threshold', 0.95)
        self.declare_parameter('voltage_nominal',    24.0)
        self.declare_parameter('capacity_ah',         5.0)

        self._pct              = self.get_parameter('initial_percentage').value
        self._idle_drain_rate   = self.get_parameter('idle_drain_rate').value
        self._motion_drain_rate = self.get_parameter('motion_drain_rate').value
        self._cmd_vel_timeout   = self.get_parameter('cmd_vel_timeout').value
        self._charge_rate      = self.get_parameter('charge_rate').value
        self._freq             = self.get_parameter('publish_frequency').value
        self._low_thr          = self.get_parameter('low_battery_threshold').value
        self._full_thr         = self.get_parameter('full_battery_threshold').value
        self._voltage          = self.get_parameter('voltage_nominal').value
        self._capacity         = self.get_parameter('capacity_ah').value

        self._is_charging  = False
        self._dt           = 1.0 / self._freq
        self._speed        = 0.0
        self._last_cmd_vel_time = None

        # --- pub / sub ---
        self._pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self._sub = self.create_subscription(Bool, '/is_charging', self._charging_cb, 10)
        self._cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'BatterySimulator started  initial={self._pct*100:.0f}%  '
            f'idle_drain={self._idle_drain_rate}/s  '
            f'motion_drain={self._motion_drain_rate}/s per (m/s)  '
            f'charge={self._charge_rate}/s'
        )

    def _charging_cb(self, msg: Bool):
        if msg.data != self._is_charging:
            self._is_charging = msg.data
            self.get_logger().info(
                'Charging ' + ('STARTED' if self._is_charging else 'STOPPED')
            )

    def _cmd_vel_cb(self, msg: Twist):
        # Commanded speed, treating rotation-in-place as partial draw too.
        self._speed = abs(msg.linear.x) + 0.5 * abs(msg.angular.z)
        self._last_cmd_vel_time = self.get_clock().now()

    def _current_speed(self) -> float:
        if self._last_cmd_vel_time is None:
            return 0.0
        age_sec = (self.get_clock().now() - self._last_cmd_vel_time).nanoseconds * 1e-9
        if age_sec > self._cmd_vel_timeout:
            return 0.0
        return self._speed

    def _tick(self):
        if self._is_charging:
            self._pct = min(1.0, self._pct + self._charge_rate * self._dt)
            drain_rate = 0.0
        else:
            speed = self._current_speed()
            drain_rate = self._idle_drain_rate + self._motion_drain_rate * speed
            self._pct = max(0.0, self._pct - drain_rate * self._dt)

        msg = BatteryState()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = 'base_link'
        msg.percentage       = float(self._pct)
        msg.voltage          = self._voltage * (0.85 + 0.15 * self._pct)
        # Discharge current scales with drain rate (idle electronics + motion draw);
        # charge current is a fixed nominal charger output.
        msg.current          = 3.5 if self._is_charging else -(2.0 + 40.0 * drain_rate)
        msg.charge           = self._pct * self._capacity
        msg.capacity         = float(self._capacity)
        msg.design_capacity  = float(self._capacity)
        msg.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING
            if self._is_charging
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        msg.power_supply_health   = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True

        self._pub.publish(msg)

    @property
    def percentage(self) -> float:
        return self._pct


def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
