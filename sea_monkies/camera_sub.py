#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")

        self.cvb = CvBridge()

        self.subscription = self.create_subscription(
            Image, "bluerov2/camera", self.image_callback, 10
        )

    def image_callback(self, msg: Image):
        """
        Callback function for the image subscriber.
        It receives an image message and saves it.

        Args:
            msg (Image): The image message
        """
        # Convert Image message to OpenCV image
        image = self.cvb.imgmsg_to_cv2(msg)

        # Save the image
        cv2.imwrite("image.png", image)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()