#!/usr/bin/env python

import rclpy
from rclpy.node import Node
# from mavros_msgs.msg import Imu
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16
import math
from mavros_msgs.msg import ManualControl

class HeadingPIDNode(Node):

    def __init__(self):
        super().__init__("heading_pid_node")

        # self.previous_error = 0.0
        self.kp = 1
        self.kd = 0.1
        self.angle = 0

        self.heading_sub = self.create_subscription(
            Int16, "bluerov2/heading", self.heading_callback, 10
        )

        self.angularv_sub = self.create_subscription(
            Imu, "bluerov2/imu", self.angularv_callback, 10
        )

        self.desired_heading_sub = self.create_subscription(
            Int16, "bluerov2/desired_heading", self.desired_heading_callback, 10
        )

        self.move_publisher = self.create_publisher(
            ManualControl, 'bluerov2/manual_control', 10
        )

        # self.desired_heading_pub = self.create_publisher(
        #     Int16,"bluerov2/desired_heading", 10
        # )
        
        self.get_logger().info("starting nodes")

    def angularv_callback(self, msg):
        self.angularv = msg.angular_velocity
        self.get_logger().info(f"angular velocity: {self.angularv}")

    def heading_callback(self, msg):
        self.angle = msg.data

    def desired_heading_callback(self, msg):
        self.angle_thrust(msg.data, self.angle)

    def angle_thrust(desired, initial):
        # this section calculates the porportional thrust
        theta = min(abs(desired + 360 - initial), abs(initial + 360 - desired))
        theta2 = min(abs(desired - initial), abs(initial - desired))
        angle = min(theta, theta2)
        
        # the first number (0-100) is the amount of speed 180 degrees is set to
        thrust = 40 * math.sin(math.pi/360*angle)
        # clockwise = True
        if (initial - desired) <= 0:
            # clockwise = False
            thrust *= (-1)
        # porportional ends here

        pid = self.kp * thrust + self.kd * self.angularv
        
        movement = ManualControl()
        movement.r = pid
        self.move_publisher.publish(pid)


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