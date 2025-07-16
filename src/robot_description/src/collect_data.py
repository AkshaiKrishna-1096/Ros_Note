#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class TrainingImageSaver(Node):
    def __init__(self):
        super().__init__('training_image_saver')
        self.bridge = CvBridge()
        
        # Create dataset directory with train/val subdirectories
        self.base_dir = os.path.expanduser('~/ros_ws/src/robot_description/dataset')
        self.train_dir = os.path.join(self.base_dir, 'train')
        self.val_dir = os.path.join(self.base_dir, 'val')
        os.makedirs(self.train_dir, exist_ok=True)
        os.makedirs(self.val_dir, exist_ok=True)
        
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.counter = 0
        self.get_logger().info(f"Training image saver started. Saving to: {self.base_dir}")

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format (BGR by default)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Split into train/val (80/20 split)
            save_dir = self.train_dir if self.counter % 5 != 0 else self.val_dir
            self.counter += 1
            
            # Generate filename with timestamp and sequential number
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(save_dir, f"img_{timestamp}_{self.counter:06d}.png")
            
            # Save as lossless PNG (better for training than JPEG)
            cv2.imwrite(filename, rgb_image, [cv2.IMWRITE_PNG_COMPRESSION, 3])
            
            if self.counter % 10 == 0:  # Log every 10 images
                self.get_logger().info(f"Saved {filename} (RGB, PNG)")
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = TrainingImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Print summary when stopped
        node.get_logger().info(f"Total images saved: {node.counter}")
        node.get_logger().info(f"Train: {len(os.listdir(node.train_dir))}, Val: {len(os.listdir(node.val_dir))}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()