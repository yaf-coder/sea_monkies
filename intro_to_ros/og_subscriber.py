#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu

from mavros_msgs.msg import RCIn

import numpy as np

class Bluerov2_Sensors(Node):
    def __init__(self):
        super().__init__("battery_subscriber")
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.subscriber = self.create_subscription(
            BatteryState,
            "/mavros/battery",
            self.callback,
            qos_profile
        )
        self.subscriber2 = self.create_subscription(
            RCIn,
            "/mavros/rc/in",
            self.callback2,
            qos_profile
        )
        self.subscriber3 = self.create_subscription(
            mavros_msgs/MAVLINK,
            "/mavros/imu/data",
            self.callback2,
            qos_profile
        )
        self.get_logger().info("starting subscriber nodes")
        time.sleep(1)

    def callback(self, msg):
        self.get_logger().info(f"BatteryState: {msg.voltage}")
        self.battery = msg.voltage
        self.check_battery()
 
    def callback2(self, msg):
        self.get_logger().info(f"Imu: {msg}")
        self.IMU = msg

    def check_battery(self):
        if self.battery > 12:
            return
        self.get_logger().info("voltage below safe!!!")
        return
            


def main(args=None):
    rclpy.init(args=args)
    node = Bluerov2_Sensors()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()
