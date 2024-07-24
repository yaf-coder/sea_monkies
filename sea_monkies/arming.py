#!/usr/bin/env python

from time import sleep

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class ArmingDisarmingNode(Node):
    done = False

    def __init__(self):
        super().__init__("arming_disarming_node")

        self.arming_client = self.create_client(SetBool, "bluerov2/arming")

        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("BlueROV2 arming service not available, waiting...")

    def dance_for_me(self):
        # Arm
        arm_future = self._arm()
        rclpy.spin_until_future_complete(self, arm_future)
        self.get_logger().info("Armed!")

        # Dance for me for 10 seconds
        sleep(60)

        # Disarm
        disarm_future = self._disarm()
        rclpy.spin_until_future_complete(self, disarm_future)
        self.get_logger().info("Disarmed!")

        self.done = True

    def _arm(self):
        self.get_logger().info(f"Arming...")
        future = self.arming_client.call_async(SetBool.Request(data=True))
        return future

    def _disarm(self):
        self.get_logger().info(f"Disarming...")
        future = self.arming_client.call_async(SetBool.Request(data=False))
        return future

    def destroy_node(self):
        disarm_future = self._disarm()
        rclpy.spin_until_future_complete(self, disarm_future)
        self.get_logger().info("Disarmed before shutting down the node")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    armingNode = ArmingDisarmingNode()
    # danceNode = DanceNode()

    try:
        armingNode.dance_for_me()

        while rclpy.ok():
            # rclpy.spin_once(danceNode)
            rclpy.spin_once(armingNode)
            if armingNode.done:
                break
    except KeyboardInterrupt:
        pass
    finally:
        armingNode.destroy_node()
        # danceNode.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()