#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import matplotlib.pyplot as plt
from dt_apriltags import Detector

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")

        self.cvb = CvBridge()

        self.subscription = self.create_subscription(
            Image, "bluerov2/camera", self.image_callback, 10
        )
        self.get_logger().info('starting camera subscriber')

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
        
        # Apriltag stuff below
        img = cv2.imread("image.png", cv2.IMREAD_GRAYSCALE)

        at_detector = Detector(families='tag36h11',
                            nthreads=1,
                            quad_decimate=1.0,
                            quad_sigma=0.0,
                            refine_edges=1,
                            decode_sharpening=0.25,
                            debug=0)

        tags = at_detector.detect(img, estimate_tag_pose=False, camera_params=None, tag_size=None)
        color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

        for tag in tags:
            print
            for idx in range(len(tag.corners)):
                cv2.line(color_img, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

            cv2.putText(color_img, str(tag.tag_id),
                        org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.8,
                        color=(0, 0, 255))

        cv2.imwrite("detected.png", color_img)


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