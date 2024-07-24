#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
import time
import random

class TutorialPublisher(Node):
    def __init__(self):
        super().__init__("tutorial_publisher")
        self.publisher = self.create_publisher(
            OverrideRCIn,
            "bluerov2/override_rc",
            10
        )
        self.publisher_timer = self.create_timer(40.0, self.run_node)
        self.get_logger().info("starting publisher node")

    def run_node(self):
        self.go_up()
        time.sleep(4)
        self.go_right()
        time.sleep(2)
        self.go_down()
        time.sleep(2)
        self.go_left()
        time.sleep(2)
        self.go_right()
        time.sleep(2)
        self.go_down()
        time.sleep(2)
        self.go_left()
        time.sleep(2)
        # B

        self.go_down()
        time.sleep(4)
        self.go_right()
        time.sleep(4)
        self.go_up()
        time.sleep(4)
        # Move to the right

        self.go_up()
        time.sleep(4)
        self.go_down()
        time.sleep(4)
        self.go_right()
        time.sleep(2)
        self.go_up()
        time.sleep(4)
        self.go_down
        time.sleep(4)
        self.go_right()
        time.sleep(2)
        self.go_up()
        time.sleep(4)
        # W

    def go_left(self):
        msg = OverrideRCIn()
        msg.channels[0] = 1500
        msg.channels[1] = 1500
        msg.channels[2] = 1500 # roll
        msg.channels[3] = 1500 # up/down
        msg.channels[4] = 2000 # rotate
        msg.channels[5] = 1500 # forward
        msg.channels[6] = 1500 # backwards
        msg.channels[7] = 1500 # strafe
        self.publisher.publish(msg)

    def go_right(self):
        msg = OverrideRCIn()
        msg.channels[0] = 1500
        msg.channels[1] = 1500
        msg.channels[2] = 1500 # roll
        msg.channels[3] = 1500 # up/down
        msg.channels[4] = 1000 # rotate
        msg.channels[5] = 1500 # forward
        msg.channels[6] = 1500 # backwards
        msg.channels[7] = 1500 # strafe
        self.publisher.publish(msg)

    def go_up(self):
        msg = OverrideRCIn()
        msg.channels[0] = 1500
        msg.channels[1] = 1500
        msg.channels[2] = 1500
        msg.channels[3] = 1500
        msg.channels[4] = 1500
        msg.channels[5] = 2000
        msg.channels[6] = 1500
        msg.channels[7] = 1500
        self.publisher.publish(msg)

    def go_down(self):
        msg = OverrideRCIn()
        msg.channels[0] = 1500
        msg.channels[1] = 1500
        msg.channels[2] = 1500
        msg.channels[3] = 1500
        msg.channels[4] = 1500
        msg.channels[5] = 1000
        msg.channels[6] = 1500
        msg.channels[7] = 1500
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TutorialPublisher()

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