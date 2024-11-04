#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry

# TODO CHECK: include needed ROS msg type headers and libraries

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Declare parameters
        self.declare_parameter('speed', float(0))

        # TODO: create ROS subscribers and publishers

        self.acker_publisher = self.create_publisher(
            AckermannDriveStamped,
            'drive',
            10
        )

        self.acker_subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.acker_callback,
            10
        )

        self.pose_subscription = self.create_subscription(
            Odometry,
            '/pf/pose/odom',
            self.pose_callback,
            10
        )

        self.get_logger().info('Python pursuer saids hello :)')

    def acker_callback(self, acker_msg): 

        pass

    def pose_callback(self, pose_msg):
        pass
        # TODO: find the current waypoint to track using methods mentioned in lecture

        # TODO: transform goal point to vehicle frame of reference

        # TODO: calculate curvature/steering angle

        # TODO: publish drive message, don't forget to limit the steering angle.

    def publish_ackerman(self, car_speed = 0.0, steering = 0.0, debug = False):
        if (debug):
            self.get_logger().info(
                "\n********** Try Publish **********\n" +
                "Speed: %f\tSteering_Angle: %f\n",
                car_speed,
                steering
            )

        ack_msg = AckermannDriveStamped()

        ack_msg.drive.speed = float(car_speed)
        ack_msg.drive.steering_angle = float(steering)
        self.acker_publisher.publish(ack_msg)

        if (debug):
            self.get_logger().info(
                "\n********** Published **********\n" +
                "Speed: %f\tSteering_Angle: %f\n",
                ack_msg.drive.speed,
                ack_msg.drive.steering_angle
            )

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
