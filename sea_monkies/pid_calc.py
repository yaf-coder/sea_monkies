import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mavros_msgs.msg import ManualControl, Altitude
from mavros_msgs.msg import OverrideRCIn

class PIDController:
    def __init__(self, kp, ki, kd, max_integral, min_output, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.min_output = min_output
        self.max_output = max_output
        self.integral = 0.0
        self.previous_error = 0.0
        self.error_accumulator = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)

        derivative = (error - self.previous_error) / dt

        proportional = self.kp * error

        output = proportional + (self.ki * self.integral) + (self.kd * derivative)
        output = max(min(output, self.max_output), self.min_output)

        self.previous_error = error
        return output