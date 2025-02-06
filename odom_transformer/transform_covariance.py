#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class TransformCovariance(Node):
    def __init__(self):
        super().__init__("transform_covariance_node")

        self.get_logger().info("Starting transform_covariance node...") 

        # Initialize parameters
        self.pose_cov_x = self.declare_parameter('pose_cov_x', 0.01).value
        self.pose_cov_y = self.declare_parameter('pose_cov_y', 0.01).value
        self.pose_cov_yaw = self.declare_parameter('pose_cov_yaw', 0.02).value

        # Subscriber to the original topic
        self.subscription = self.create_subscription(
            Odometry, "/laser_scan_matcher/odom", self.odom_callback, 10)

        # Publisher for the corrected topic
        self.publisher = self.create_publisher(Odometry, "/laser_scan_matcher/odom_covariance", 10)

        # Declare parameter callback to update parameters dynamically
        self.add_on_set_parameters_callback(self.parameter_callback)

    def odom_callback(self, msg):
        # Pose covariance matrix
        msg.pose.covariance = [
            self.pose_cov_x, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.pose_cov_y, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.pose_cov_yaw
        ]

        # Twist covariance matrix (fixed)
        msg.twist.covariance = [
            99999.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 99999.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 99999.0
        ]

        # Publish corrected message
        self.publisher.publish(msg)

    def parameter_callback(self, params):
        """Callback function to update parameters dynamically."""
        for param in params:
            if param.name == 'pose_cov_x':
                self.pose_cov_x = param.value
            elif param.name == 'pose_cov_y':
                self.pose_cov_y = param.value
            elif param.name == 'pose_cov_yaw':
                self.pose_cov_yaw = param.value
        return rclpy.parameter.ParameterEventResponse()

def main(args=None):
    rclpy.init(args=args)
    
    transform_covariance_node = TransformCovariance()

    try:
        rclpy.spin(transform_covariance_node)
    except KeyboardInterrupt:
        transform_covariance_node.get_logger().info('Keyboard Interrupt (Ctrl+C) detected. Exiting...')
    except rclpy.exceptions.ROSInterruptException:
        transform_covariance_node.get_logger().info('ROSInterruptException detected. Exiting...')
    finally:
        transform_covariance_node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
