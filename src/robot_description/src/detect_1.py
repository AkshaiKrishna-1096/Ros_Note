#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node

import rclpy.qos
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.get_logger().info('object Detection Node has started!!')
        
        # Initialise CV Bridge
        self.bridge = CvBridge()

        # Create a Qos Profile
        qos_profile = rclpy.qos.qos_profile_sensor_data

        # Subscribe from /image_raw
        self.camera_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.detector_callback,
            qos_profile
        )
        self.camera_sub
        
        self.get_logger().info("Object Detection ready!!!")

    def detector_callback(self, msg):
        try:
            # Convert Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Convert to HSV
            hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
            cv.imshow("HSV", hsv)

            # Define Orange Color range
            lower_orange = np.array([10, 100, 100])
            upper_orange = np.array([25, 255, 255])

            # Thershold the HSV image
            mask = cv.inRange(hsv, lower_orange, upper_orange)
            cv.imshow("Mask", mask)

            # Find Contours
            contours, heirachies = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv.contourArea(contour)
                if area > 300:
                    x, y, w, h = cv.boundingRect(contour)
                    aspect_ratio = float(w)/h
                    if 0.5 < aspect_ratio < 2.0:
                        cv.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

                        # Add text above the rectangle
                        text = "Cone"
                        font = cv.FONT_HERSHEY_SIMPLEX
                        font_scale = 0.6
                        thickness = 2
                        text_size, _ = cv.getTextSize(text, font, font_scale, thickness)

                        # Position text slightly above the rectangle
                        text_x = x
                        text_y = y - 10 if y - 10 > 10 else y + text_size[1] + 10  # avoid going above frame

                        cv.putText(cv_image, text, (text_x, text_y), font, font_scale, (0, 255, 0), thickness)

            # Display the result
            cv.imshow("Cone Detect", cv_image)
            cv.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Could not convert image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Fatal error: {e}", file=sys.stderr)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv.destroyAllWindows()


if __name__ == '__main__':
    main()