# import rclpy
# from rclpy.node import Node
# from mavros_msgs.msg import ManualControl, Altitude


# class DepthPIDNode(Node):
#     previous_error = 0.0
#     integral = 0.0
#     desired_depth: Altitude = None
#     previous_depth: Altitude = None

#     def __init__(self):
#         super().__init__("depth_pid_node")

#         self.declare_parameter("Kp", -50.0)
#         self.Kp = self.get_parameter("Kp").value
#         self.declare_parameter("Ki", 0.0)
#         self.Ki = self.get_parameter("Ki").value
#         self.declare_parameter("Kd", 30.0)
#         self.Kd = self.get_parameter("Kd").value
#         self.declare_parameter("max_integral", 1.0)
#         self.max_integral = self.get_parameter("max_integral").value
#         self.declare_parameter("max_throttle", 100.0)
#         self.max_throttle = self.get_parameter("max_throttle").value

#         self.depth_sub = self.create_subscription(
#             Altitude, "bluerov2/depth", self.depth_callback, 10
#         )
#         self.desired_depth_sub = self.create_subscription(
#             Altitude, "bluerov2/desired_depth", self.desired_depth_callback, 10
#         )

#         self.manual_control_pub = self.create_publisher(
#             ManualControl, "bluerov2/manual_control", 10
#         )
#         self.get_logger().info("starting nodes")

#     def depth_callback(self, msg):
#         self.get_logger().info("depth callback was run")
#         depth: Altitude = msg
#         self.get_logger().debug(f"Depth: {depth}")

#         if self.desired_depth is None:
#             self.get_logger().info("no desired depth")
#             return

#         error = self.desired_depth.local - depth.local

#         if self.previous_depth is None:
#             self.previous_depth = depth
#             return

#         dt = (
#             depth.header.stamp.sec
#             + depth.header.stamp.nanosec * 1e-9
#             - self.previous_depth.header.stamp.sec
#             - self.previous_depth.header.stamp.nanosec * 1e-9
#         )

#         # Propotional term
#         propotional = self.Kp * error

#         # Integral term
#         self.integral += self.Ki * error * dt
#         self.integral = min(max(self.integral, -self.max_integral), self.max_integral)

#         # Derivative term
#         derivative = self.Kd * (error - self.previous_error) / dt

#         # Update previous values
#         self.previous_error = error
#         self.previous_depth = depth

#         throttle = propotional + self.integral + derivative
#         throttle = min(max(throttle, -self.max_throttle), self.max_throttle)

#         manual_control_msg = ManualControl()
#         manual_control_msg.z = throttle
#         self.manual_control_pub.publish(manual_control_msg)
#         self.get_logger().info("control was published")

#     def desired_depth_callback(self, msg):
#         self.desired_depth = msg
#         self.get_logger().debug(f"Desired depth: {self.desired_depth}")


# def main(args=None):
#     rclpy.init(args=args)
#     depthPIDNode = DepthPIDNode()

#     try:
#         rclpy.spin(depthPIDNode)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         depthPIDNode.destroy_node()
#         rclpy.try_shutdown()


# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3

#~/ardupilot/Tools/autotest/sim_vehicle.py --vehicle=ArduSub --aircraft="bwsibot" -L RATBeach --out=udp:YOUR_COMPUTER_IP:14550
#ros2 launch /home/kenayosh/auvc_ws/src/AUV-Group-Github/launch/example.yaml

#cd ~/auvc_ws
#colcon build --symlink-install
#source ~/auvc_ws/install/setup.zsh

#ros2 topic list
#ros2 topic type /your/topic
#ro2 topic echo /your/topic :)))))
#ros2  interface show your_msg_library/msg/YourMessageType
#ros2 topic pub bluerov2/desired_depth mavros_msgs/msg/Altitude "{relative: 0.8}" 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mavros_msgs.msg import ManualControl, Altitude
import numpy as np

import matplotlib.pyplot as plt


class PIDdepthNode(Node):
    def __init__(self):
        super().__init__('depth_node')
        #subscribers/publishers for necessary topics
        self.move_publisher = self.create_publisher(
            ManualControl,
            'bluerov2/manual_control',
            10
        )
        self.depth_subscriber = self.create_subscription(
            Altitude,
            'bluerov2/depth',
            self.depth_callback,
            10
        )

        self.desired_depth_subscriber = self.create_subscription(
            Altitude,
            'bluerov2/desired_depth',
            self.desired_depth_callback,
            10
        )
        """
        PID CONSTANTS
        """

        # self.kp = 60
        # self.ki = 7
        # self.kd = 27
        
        # Masking tape robot/double O/4lights
        # self.kp = 45
        # self.ki = 7
        # self.kd = 18
        
        # AUV with the sticker
        self.kp = 55
        self.ki = 7
        self.kd = 15
        
        self.max_integral = 4.0
        self.min_output = -100.0
        self.max_output = 100.0
        self.integral = 0.0
        self.previous_error = 0.0
        """"""
        self.get_logger().info('starting publisher node')
        #self.pid_yaw = PIDController(0.5, 0.1, 0.05, 1.0, -50, 50)
        """
        tracking constants
        """
        self.depth = float()
        self.desired_depth = None
        self.prev_time = None
        
        self.array = np.array([])

    def compute(self, error, dt):        
        """
        computes and logs the correction power based on the angle error and angular velocity in rad/s as derivative
        """
        #tracking
        self.array = np.append(self.array, [error])
        #integral calc
        self.integral += error*dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        #derivative calc
        derivative = (error - self.previous_error) / dt
        #p and summing errors
        proportional = self.kp * error
        output = proportional + (self.ki * self.integral) + (self.kd * derivative)
        self.get_logger().info(f'\n Kp: {proportional} Ki: {self.ki * self.integral} Kd: {self.kd *derivative}')
        #updating error and clamping outputs
        output = max(min(output, self.max_output), self.min_output)
        self.previous_error = error
        return output

    def depth_callback(self, msg):
        """gets timestamp in secs and nanosecs from subscriber as well as depth in meters"""
        self.depth = msg.relative
        self.timestamp = msg.header.stamp.sec + 1e-09*msg.header.stamp.nanosec
        if self.prev_time != None and self.desired_depth != None:
            self.calc_publish_vertical()
        self.prev_time = self.timestamp
        #self.get_logger().info(f'Depth: {self.depth}, Timestamp: {self.timestamp}')


    def desired_depth_callback(self, msg):
        """gets desired depth in m from subscriber"""
        self.desired_depth = msg.relative
        self.get_logger().info(self.desired_depth)
        
    def calc_publish_vertical(self):
        """publishes vertical movement based on desired depth and ouput from pid"""
        #checking if depth has been recieved and a dt has been produced
        if self.depth is not None and self.timestamp - self.prev_time > 0:
            depth_correction = self.compute(self.depth - self.desired_depth, self.timestamp - self.prev_time)
            movement = ManualControl()
            movement.z = depth_correction
            self.get_logger().info(f'\nCurrent Power: {depth_correction}/100\nDepth: {self.depth}')
            self.move_publisher.publish(movement)


def main(args=None):
    rclpy.init(args=args)
    move_node = PIDdepthNode()
    try:
        rclpy.spin(move_node)
    except KeyboardInterrupt:
        print('\nKeyboardInterrupt received, shutting down...')
    finally:
        x = np.arange(0,len(move_node.array))

        plt.plot(x,move_node.array)
        plt.savefig("/home/kenayosh/auvc_ws/src/AUV-Group-Github/intro_to_ros/depth_err.png")
        
        move_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()


"""- node:
    pkg: "intro_to_ros"
    exec: "pid_depth"
    name: "pid_depth"
    namespace: """""