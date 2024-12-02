#!/usr/bin/env python3
import os
import atexit
import rclpy
from rclpy.node import Node
import numpy as np
from time import gmtime, strftime
from numpy import linalg as LA
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import time

# Create the log file in the current working directory
log_dir = os.getcwd()  # Get the current working directory
file = open(strftime('waypoints') + '.csv', 'w')

class WaypointsLogger(Node):
    def __init__(self):
        super().__init__('waypoints_logger')

        # Declare Parameters
        self.declare_parameter('mode', 'sim')
        self.declare_parameter('timer', float(0.5))

        # Set Parameters
        mode = self.get_parameter('mode').get_parameter_value().string_value
        self.timer = self.get_parameter('timer').get_parameter_value().double_value

        # Where to listen to?
        if mode == 'sim':
            mode = '/ego_racecar/odom'
        else:
            mode = 'pf/pose/odom'

        self.get_logger().info('Marker listening to: %s\n' % mode)

        self.subscription = self.create_subscription(Odometry, mode, self.save_waypoint, 10)

    def save_waypoint(self, data):
        quaternion = np.array([data.pose.pose.orientation.x, 
                               data.pose.pose.orientation.y, 
                               data.pose.pose.orientation.z, 
                               data.pose.pose.orientation.w])

        euler = euler_from_quaternion(quaternion)
        speed = LA.norm(np.array([data.twist.twist.linear.x, 
                                  data.twist.twist.linear.y, 
                                  data.twist.twist.linear.z]), 2)

        if data.twist.twist.linear.x > 0.0:
            self.get_logger().info(f'Speed: {data.twist.twist.linear.x}')

        # Log message for the saved waypoint
        self.get_logger().info(f'Waypoint saved: x={data.pose.pose.position.x}, y={data.pose.pose.position.y}, heading={euler[2]}, speed={speed}')

        file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                         data.pose.pose.position.y,
                                         euler[2],
                                         speed))

        time.sleep(self.timer)

    def shutdown(self):
        file.close()
        self.get_logger().info('Goodbye')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointsLogger()
    atexit.register(node.shutdown)  # Register the shutdown function
    node.get_logger().info('Saving waypoints...')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
