#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl, Altitude

class DepthPIDNode(Node):
    previous_error = 0.0
    integral = 0.0
    desired_depth: Altitude = None
    previous_depth: Altitude = None

    def __init__(self):
        super().__init__("depth_pid_node")

        self.Kp = -50.0
        self.Ki = 0.0
        self.Kd = 30.0
        self.max_integral = 1.0
        self.max_throttle = 100.0
        self.desired_depth_value = 1

        self.depth_sub = self.create_subscription(
            Altitude, "bluerov2/bluerov2/depth", self.depth_callback, 10
        )
        self.desired_depth_sub = self.create_subscription(
            Altitude, "bluerov2/bluerov2/desired_depth", self.desired_depth_callback, 10
        )

        self.manual_control_pub = self.create_publisher(
            ManualControl, "bluerov2/bluerov2/manual_control", 10
        )
        
        self.get_logger().info("starting nodes")
        
        self.desired_depth = Altitude()
        self.desired_depth.local = self.desired_depth_value

    def depth_callback(self, msg):
        depth: Altitude = msg
        self.get_logger().info(f"Depth: {depth}")

        if self.desired_depth is None:
            return

        error = self.desired_depth.local - depth.local

        if self.previous_depth is None:
            self.previous_depth = depth
            return

        dt = (
            depth.header.stamp.sec
            + depth.header.stamp.nanosec * 1e-9
            - self.previous_depth.header.stamp.sec
            - self.previous_depth.header.stamp.nanosec * 1e-9
        )
        
        if dt == 0:
            self.get_logger().warn("dt is zero, skipping this update")
            return

        # Propotional term
        propotional = self.Kp * error

        # Integral term
        self.integral += self.Ki * error * dt
        self.integral = min(max(self.integral, -self.max_integral), self.max_integral)

        # Derivative term
        derivative = self.Kd * (error - self.previous_error) / dt

        # Update previous values
        self.previous_error = error
        self.previous_depth = depth

        throttle = propotional + self.integral + derivative
        throttle = min(max(throttle, -self.max_throttle), self.max_throttle)

        manual_control_msg = ManualControl()
        manual_control_msg.z = throttle
        self.manual_control_pub.publish(manual_control_msg)

    def desired_depth_callback(self, msg):
        self.desired_depth = msg
        self.get_logger().info(f"Desired depth: {self.desired_depth.local}")

def main(args=None):
    rclpy.init(args=args)
    depthPIDNode = DepthPIDNode()

    try:
        rclpy.spin(depthPIDNode)
    except KeyboardInterrupt:
        pass
    finally:
        depthPIDNode.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()