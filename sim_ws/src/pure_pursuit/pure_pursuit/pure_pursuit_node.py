#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


# TODO CHECK: include needed ROS msg type headers and libraries

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    #region INIT
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Declare parameters
        self.declare_parameter('mode', 'sim')
        self.declare_parameter('speed', float(1.0))
        self.declare_parameter('lookahead', float(1.0))
        self.declare_parameter('turbo', float(1.0))
        
        mode = self.get_parameter('mode').get_parameter_value().as_string()

        if mode == 'sim':
            mode = '/ego_racecar/odom'
        else:
            mode = '/pf/pose/odom'

        # TODO: create ROS subscribers and publishers

        self.acker_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.acker_subscription = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.acker_callback,
            10
        )

        self.pose_subscription = self.create_subscription(
            Odometry,
            mode,
            self.pose_callback,
            10
        )

        self.get_logger().info('Python pursuer saids hello :)')

    def acker_callback(self, acker_msg): 

        pass

    #region POSE
    def pose_callback(self, pose_msg):
        self.get_logger().info(
            'Odom is being ran.'
        )

        self.get_logger().info(
            'Value: %f' \
            % pose_msg
        )

        # Declare command-line parameters
        lookahead = self.get_parameter('lookahead').get_parameter_value().as_double()

        # Position
        position_x = pose_msg.pose.position.x
        position_y = pose_msg.pose.position.y
        position_z = pose_msg.pose.position.z

        # Orientation
        orientation_x = pose_msg.pose.orientation.x
        orientation_y = pose_msg.pose.orientation.y
        orientation_z = pose_msg.pose.orientation.z
        orientation_w = pose_msg.pose.orientation.w

        # temporary
        lateral_offset = 0.0
        #region WAYPOINT

        # TODO: translate vehicle frame to world frame
        world_pose_msg = 10

        # TODO: find the current closest waypoint to track using methods mentioned in lecture
        waypoint_array = self.readCSV("waypoints.csv")
        closest_waypoint = self.find_nearest_waypoint(waypoint_array, world_pose_msg)
        #set following variables equal to closest_waypoint[] from index 0 to 3
        x_coordinate_of_waypoint, y_coordinate_of_waypoint, yaw, speed = closest_waypoint


        # TODO: transform goal point to vehicle frame of reference
        #transform closest_waypoint to the vehicle frame of reference to calculate steering angle


        # TODO: calculate curvature/steering angle

        #region STEER ANGLE
        #steer_angle = (2 * |y|) / lookeahead^2
        steering_angle = (2 * abs(lateral_offset)) / pow(lookahead, 2)

        # TODO: publish drive message, don't forget to limit the steering angle.
        if (steering_angle > 24.0):
            steering_angle = 24.0
        elif (steering_angle < -24.0):
            steering_angle = -24.0

        
        turbo = self.get_parameter('turbo').get_parameter_value().as_double()
        #region SPEED
        if steering_angle >= -4.0000 & steering_angle <= 4.0000:
            # Speed will be specified by the launch parameters
            # If less than 20 and greater than 10
            speed *= turbo
        elif ((steering_angle > 4.0000 & steering_angle <= 8.0000) |
                 (steering_angle < -4.0000 & steering_angle >= -8.0000)):
            speed = speed
        elif ((steering_angle > 8.0000 & steering_angle <= 12.0000) |
                 (steering_angle < -8.0000 & steering_angle >= -12.0000)):
            speed *= .85
        elif ((steering_angle > 12.0000 & steering_angle <= 14.0000) |
                 (steering_angle < -12.0000 & steering_angle >= -14.0000)):
            speed *= .75
        elif ((steering_angle > 14.0000 & steering_angle < 24.0001) |
                 (steering_angle < -14.0000 & steering_angle > -24.0001)):
            if (speed * .65 > .75):
                speed = .75
            else: 
                speed *= .65
        else:
            speed = 0.5


        self.publish_ackerman(speed, steering_angle)
        pass

    #region PUBLISH
    def publish_ackerman(self, car_speed = 0.0, steering = 0.0, debug = False):
        if (debug):
            self.get_logger().info(
                "\n********** Try Publish **********\n" +
                "Speed: %f\tSteering_Angle: %f\n" \
                % (car_speed, steering)
            )

        ack_msg = AckermannDriveStamped()

        ack_msg.drive.speed = float(car_speed)
        ack_msg.drive.steering_angle = float(steering)
        self.acker_publisher.publish(ack_msg)

        if (debug):
            self.get_logger().info(
                "\n********** Published **********\n" +
                "Speed: %f\tSteering_Angle: %f\n" \
                % (ack_msg.drive.speed, ack_msg.drive.steering_angle)
            )
    # Publish_ackerman, END

    def readCSV(self, directory):
        #./ for current directory. 
        #../ to go back a directory
        array_of_waypoints = list(csv.reader(open(directory)))
        return array_of_waypoints

    def create_pose_stamped(self):
        pose_stamped = PoseStamped()

        # Set the header
        # pose_stamped.header.stamp = rospy.Time.now()
        # pose_stamped.header.frame_id = "base_link"  # Replace with the appropriate frame ID

        # Set the pose
        pose_stamped.pose.position.x = 1.0
        pose_stamped.pose.position.y = 2.0
        pose_stamped.pose.position.z = 3.0

        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0

        return pose_stamped

    #region minDist
    def find_nearest_waypoint(self, waypoint_array, world_pose_msg):
        # selected_waypoint.pose.position.x = waypoint.position.x
        # selected_waypoint.pose.position.y = waypoint.position.y
        # selected_waypoint.pose.position.z = waypoint.position.z
        # selected_waypoint.pose.orientation.x = waypoint.orientation.x
        # selected_waypoint.pose.orientation.y = waypoint.orientation.y
        # selected_waypoint.pose.orientation.z = waypoint.orientation.z
        # selected_waypoint.pose.orientation.w = waypoint.orientation.w
        minimum_dist = 1000.0
        for waypoint in waypoint_array:
            # if least distance
            waypoint_x = waypoint[0]
            # some sort of check to see if its ahead of us

            waypoint_y = waypoint[1]
            x2_x1_square = pow(waypoint_x - world_pose_msg.pose.position.x, 2)
            y2_y1_square = pow(waypoint_y - world_pose_msg.pose.position.y, 2)
            temp_dist = pow(x2_x1_square + y2_y1_square, 0.5)
            if (temp_dist < minimum_dist):
                closest_waypoint = waypoint

        return closest_waypoint

    def shutdown(self):
        self.get_logger().info('Pursuit node ended itself.')

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
