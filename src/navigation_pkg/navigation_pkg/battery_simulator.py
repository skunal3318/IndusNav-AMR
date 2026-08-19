#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool


class BatterySimulator(Node):
    def __init__(self):
        super().__init__('battery_simulator')

        self.declare_parameter('initial_percentage',  1.0)   
        self.declare_parameter('drain_rate',          0.001) 
        self.declare_parameter('charge_rate',         0.003) 
        self.declare_parameter('publish_frequency',  10.0)   
        self.declare_parameter('low_battery_threshold', 0.20)
        self.declare_parameter('full_battery_threshold', 0.95)
        self.declare_parameter('voltage_nominal',    24.0)  
        self.declare_parameter('capacity_ah',         5.0)   

        self._pct         = self.get_parameter('initial_percentage').value
        self._drain_rate  = self.get_parameter('drain_rate').value
        self._charge_rate = self.get_parameter('charge_rate').value
        self._freq        = self.get_parameter('publish_frequency').value
        self._low_thr     = self.get_parameter('low_battery_threshold').value
        self._full_thr    = self.get_parameter('full_battery_threshold').value
        self._voltage     = self.get_parameter('voltage_nominal').value
        self._capacity    = self.get_parameter('capacity_ah').value

        self._is_charging = False
        self._dt          = 1.0 / self._freq

        # --- pub / sub ---
        self._pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self._sub = self.create_subscription(Bool, '/is_charging', self._charging_cb, 10)
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'BatterySimulator started  initial={self._pct*100:.0f}%  '
            f'drain={self._drain_rate}/s  charge={self._charge_rate}/s'
        )

    def _charging_cb(self, msg: Bool):
        if msg.data != self._is_charging:
            self._is_charging = msg.data
            self.get_logger().info(
                'Charging ' + ('STARTED' if self._is_charging else 'STOPPED')
            )

    def _tick(self):
        if self._is_charging:
            self._pct = min(1.0, self._pct + self._charge_rate * self._dt)
        else:
            self._pct = max(0.0, self._pct - self._drain_rate * self._dt)

        msg = BatteryState()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = 'base_link'
        msg.percentage       = float(self._pct)
        msg.voltage          = self._voltage * (0.85 + 0.15 * self._pct)  
        msg.current          = -2.0 if not self._is_charging else 3.5      
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
