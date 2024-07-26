#!/usr/bin/env python

import rclpy
from rclpy.node import Node
# from mavros_msgs.msg import Imu
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16

class HeadingPIDNode(Node):

    def __init__(self):
        super().__init__("heading_pid_node")

        self.previous_error = 0.0

        self.heading_sub = self.create_subscription(
            Int16, "bluerov2/bluerov2/heading", self.heading_callback, 10
        )

        self.angularv_sub = self.create_subscription(
            Imu, "bluerov2/bluerov2/imu", self.angularv_callback, 10
        )

        self.desired_heading_sub = self.create_subscription(
            Int16, "bluerov2/desired_heading", self.desired_heading_callback, 10
        )

        self.desired_heading_pub = self.create_publisher(
            Int16,"bluerov2/desired_heading", 10
        )
        
        self.get_logger().info("starting nodes")

    def angularv_callback(self, msg):
        self.angularv = msg.angular_velocity
        self.get_logger().info(f"angular velocity: {self.angularv}")


def main(args=None):
    rclpy.init(args=args)
    headingPIDNode = HeadingPIDNode()

    try:
        rclpy.spin(headingPIDNode)
    except KeyboardInterrupt:
        pass
    finally:
        headingPIDNode.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()