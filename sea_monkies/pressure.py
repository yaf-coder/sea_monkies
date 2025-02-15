# #!/usr/bin/env python

# import rclpy
# from rclpy.node import Node
# from mavros_msgs.msg import Altitude
# from sensor_msgs.msg import FluidPressure as Pressure

# class pressure_node(Node):

#     def __init__(self):
#         super().__init__("pressure_node")

#         self.pressure = 0

#         self.sub = self.create_subscription(
#             Pressure,
#             "bluerov2/pressure",
#             self.pressure_callback,
#             10
#         )

#         self.depth_pub = self.create_publisher(
#             Altitude,
#             "bluerov2/depth",
#             10
#         )

#         self.desired_depth_pub = self.create_publisher(
#             Altitude,
#             "bluerov2/desired_depth",
#             10
#         )

#         self.timer_period = 0.1  # seconds
#         self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
#         self.get_logger().info("starting nodes")

#     def pressure_callback(self, msg):
#         self.get_logger().info(f"Pressure: {msg.fluid_pressure}")
#         self.pressure = msg.fluid_pressure
#         self.depth_calculation()

#     def depth_calculation(self):
#         # p = p*g*h
#         gravity = 9.81 # m/s2
#         density = 1000 # kg/m3
#         msg = Altitude()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.local = self.pressure / (gravity * density)
#         self.depth_pub.publish(msg)
#         dd = Altitude()
#         dd.local = -15.0
#         self.desired_depth_pub.publish(dd)

#     def timer_callback(self):
#         if self.pressure > 0:
#             self.depth_calculation()

#     def destroy_node(self):
#         return super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = pressure_node()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print("\nKeyboardInterrupt received, shutting down...")
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()

# if __name__=="__main__":
#     main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import Altitude
from sensor_msgs.msg import (
    FluidPressure as Pressure,
    Temperature,
)

FLUID_DENSITY = 1000
GRAVITATION_CONST = 9.81
ONEATM_TO_PASCAL = 101325

class PressureConverter(Node):
    def __init__(self):
        super().__init__("pressure_subscriber")
        self.pressure_subscriber = self.create_subscription(
            Pressure,
            "bluerov2/pressure",
            self.pressure_callback,
            10
        )       
        self.depth_publisher = self.create_publisher(
            Altitude,
            "bluerov2/depth",
            10
        )
        
        self.get_logger().info("starting subscriber node")
        
        
        
    def pressure_callback(self, msg):
        '''subscribes to pressure, converts it to depth, and publishes depth in meters'''
        self.depth_real = Altitude()
        self.depth_real.relative = self.depth(msg.fluid_pressure)
        self.depth_real.header.stamp = msg.header.stamp
        self.depth_publisher.publish(self.depth_real)

       # self.get_logger().info(f"Pressure: {msg.fluid_pressure}, Depth: {self.depth_real.relative}")
        
    def depth(self, pressure):
        """Converts from pressure in Pa to Depth in meters"""
        return (pressure-ONEATM_TO_PASCAL)/GRAVITATION_CONST/FLUID_DENSITY
        
def main(args=None):
    rclpy.init(args=args)
    node = PressureConverter()
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