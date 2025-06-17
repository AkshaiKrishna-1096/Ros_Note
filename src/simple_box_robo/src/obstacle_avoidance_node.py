#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance_node")
        self.get_logger().info("Obstacle avoidance node activated")

        # Qos Profile 
        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )

        # safety paramteres
        self.safe_distance = 0.5
        self.slow_distance = 0.8

        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)


    def scan_callback(self, msg: LaserScan):
        front_ranges = []

        angle_range_deg = 60
        angle_increment_deg = msg.angle_increment*180/math.pi
        center_index = len(msg.ranges) // 2
        index_range = int(angle_range_deg / angle_increment_deg)

        for i in range(center_index - index_range, center_index + index_range + 1):
            if 0 <= i < len(msg.ranges):
                if msg.range_min < msg.ranges[i] < msg.range_max:
                    front_ranges.append(msg.ranges[i])

        if not front_ranges:
            self.get_logger().warn("NO valid Lidar data in front")
            return

        min_distance = min(front_ranges)

        twist = Twist()

        if min_distance < self.safe_distance :
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self.get_logger().warn("Obstacle too close warning")
        elif min_distance < self.slow_distance:
            twist.linear.x = 0.1
            self.get_logger().info("Slowing down, Obstacle nearby")
        else:
            twist.linear.x = 0.5

        self.velocity_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
