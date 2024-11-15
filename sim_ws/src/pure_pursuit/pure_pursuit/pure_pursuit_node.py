#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv

import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import math
from visualization_msgs.msg import Marker, MarkerArray
import os

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
        self.declare_parameter('path', "pure_pursuit/pure_pursuit/")
        self.declare_parameter('filename', "waypoints.csv")


        mode = self.get_parameter('mode').get_parameter_value().string_value
        self.turbo = self.get_parameter('turbo').get_parameter_value().double_value
        self.lookahead = self.get_parameter('lookahead').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.default_speed = self.speed
        self.path = self.get_parameter('path').get_parameter_value().string_value
        self.filename = self.get_parameter('filename').get_parameter_value().string_value

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

        self.pose_subscription = self.create_subscription(
            Odometry,
            mode,
            self.pose_callback,
            10
        )
        
        self.visualize_marker_array_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.visualize_marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # Build a relative path.
        # First, get the path of where the command was ran
        # Second, get the path from 1 to the csv file (User Input)
        # Third, get the file name (User Input)
        # Last, append everything for the file path
        self.path = f"{os.path.abspath(os.getcwd())}/{self.path}{self.filename}"

        self.get_logger().info(f'Reading file from: %s' %
                               self.path)

        wp = np.loadtxt(self.path, delimiter=',', dtype=float)  # x, y
        wp = np.concatenate((wp, np.zeros(shape=(len(wp), 1), dtype=float)), axis=1)    # x, y, z

        self.visualize_waypoints(wp)

        self.get_logger().info(f'Python listening to: {mode}')

    #region Visualize Waypoint
    def visualize_target_waypoint(self, waypoint):
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.ns = 'waypoint'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = waypoint[0]
        marker.pose.position.y = waypoint[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0

        # self.get_logger().info(
        #     "Marker X: %f\tMarker Y: %f" %
        #     (waypoint[0], waypoint[1])
        # )

        self.visualize_marker_pub.publish(marker)

    def visualize_waypoints(self, waypoints):
        marker_array = MarkerArray()

        for index, waypoint in enumerate(waypoints):
            marker = Marker()

            marker.header.frame_id = 'map'
            marker.ns = 'waypoint'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)
        
        self.visualize_marker_array_pub.publish(marker_array)

    #region POSE
    def pose_callback(self, pose_msg):
        waypoint_array = self.readCSV(self.path)

        vehicle_pos = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, 0])
        vehicle_orientation = np.array([pose_msg.pose.pose.orientation.w,
                                        pose_msg.pose.pose.orientation.x,
                                        pose_msg.pose.pose.orientation.y,
                                        pose_msg.pose.pose.orientation.z])

        # Calculate distances to waypoints and filter by LOOKAHEAD_DISTANCE
        # Selecting all waypoints within lookahead for transformation to car frame
        within_lookahead_points = self.find_nearest_waypoint(waypoint_array, vehicle_pos)

        # w is the real scalar value
        # Calculate inverse quaternion by
        # normalizing it, find it's conjunction, then divide by the sum^2
        # (w, x, y, z) --> (w, -x, -y, -z)      Inv_Q
        #   w^2 + x^2 + y^2 + z^2       Denominator
        # Inverse Q = Inv_Q/Denominator
        rotation_inv = vehicle_orientation * [1, -1, -1, -1] / np.sum(vehicle_orientation ** 2)

        waypoints_vehicle_frame = self.transform_waypoints(vehicle_pos, within_lookahead_points, rotation_inv)

        if (len(waypoints_vehicle_frame) == 0):
            self.get_logger().info(
                "No waypoint found. Drive straight..."
            )

            # Nothing found, just drive straight
            self.publish_ackerman(self.speed, 0.0)
            return

        # Select target waypoint (furthest within lookahead in front of vehicle)
        # Get all waypoints with x values greater than 0
        front_waypoints = waypoints_vehicle_frame[waypoints_vehicle_frame[:, 0] > 0]
        # error checking waypoints greater than 0
        if not len(front_waypoints):
            return

        target_idx = np.argmax(front_waypoints[:, 0])

        # picks biggest x value (Pick the furthest waypoint) 
        # arg max returns index of waypoint with greatest x value 
        target_waypoint = front_waypoints[target_idx]
        
        # Compute steering angle and velocity
        # uses distance to waypoint instead of lookahead
        dist_to_target = np.linalg.norm(target_waypoint)
        steering_angle = 2 * abs(target_waypoint[1]) / dist_to_target ** 2

        #arc
        steering_angle = steering_angle * 0.5 * np.sign(target_waypoint[1])
        #MARKER

        #angle cut off
        if (steering_angle > 24.0):
            steering_angle = 24.0
        if (steering_angle < -24.0):
            steering_angle = -24.0

        #region SPEED
        if ((steering_angle >= -4.0000) and (steering_angle <= 4.0000)):
            # Speed will be specified by the launch parameters
            # If less than 20 and greater than 10
            self.speed = self.default_speed
            self.speed *= self.turbo
        elif ((steering_angle > 4.0000 and steering_angle <= 8.0000) or
                 (steering_angle < -4.0000 and steering_angle >= -8.0000)):
            self.speed = self.default_speed
            self.speed = self.speed
        elif ((steering_angle > 8.0000 and steering_angle <= 12.0000) or
                 (steering_angle < -8.0000 and steering_angle >= -12.0000)):
            self.speed = self.default_speed
            self.speed *= .85
        elif ((steering_angle > 12.0000 and steering_angle <= 14.0000) or
                 (steering_angle < -12.0000 and steering_angle >= -14.0000)):
            self.speed = self.default_speed
            self.speed *= .75
        elif ((steering_angle > 14.0000 and steering_angle < 24.0001) or
                 (steering_angle < -14.0000 and steering_angle > -24.0001)):
            if (self.speed * .65 > .75):
                self.speed = self.default_speed
                self.speed = .75
            else: 
                self.speed = self.default_speed
                self.speed *= .65
        else:
            self.speed = 0.5

        self.publish_ackerman(self.speed, steering_angle)
        pass


    def transform_waypoints(self, vehicle_pos, within_lookahead_points, rotation_inv):
        if len(within_lookahead_points) == 0:
            return []       # Return nothing
            return [[1,0]]

        # translation, place origin onto vehicle
        # if the vehicle is at (3, 2) and wp is (5, 6)
        # local frame is now (2, 4)
        waypoints_translated = within_lookahead_points - vehicle_pos
        way_points_x_y = waypoints_translated[:, :-1] 

        # rotation, rotate to where the car is looking
        orientation_w = rotation_inv[0]
        orientation_z = rotation_inv[-1]

        theta = 2 * np.arctan2(orientation_z, orientation_w)
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        rotation_matrix = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])

        waypoints_rotated = np.dot(way_points_x_y, rotation_matrix.T)

        return waypoints_rotated


    #region PUBLISH
    def publish_ackerman(self, car_speed: float = 0.0, steering: float = 0.0, debug: bool = False):
        if (debug):
            self.get_logger().info(
                "\n********** Try Publish **********\n" +
                "Speed: %f\tSteering_Angle: %f\n" \
                % (car_speed, steering)
            )

        ack_msg = AckermannDriveStamped()

        ack_msg.drive.speed = car_speed
        ack_msg.drive.steering_angle = steering
        self.acker_publisher.publish(ack_msg)

        if (debug):
            self.get_logger().info(
                "\n********** Published **********\n" +
                "Speed: %f\tSteering_Angle: %f\n" \
                % (ack_msg.drive.speed, ack_msg.drive.steering_angle)
            )
    # Publish_ackerman, END

    def readCSV(self, directory):
        array_of_waypoints = []
        with open(directory, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                # Convert each item in the row to a float
                waypoint = [float(value) for value in row]
                array_of_waypoints.append(waypoint)
        return array_of_waypoints

    #region minDist
    def find_nearest_waypoint(self, waypoint_array, vehicle_pos):
        vehicle_pos_x = vehicle_pos[0]
        vehicle_pos_y = vehicle_pos[1]
        within_lookahead = []
        for waypoint in waypoint_array:
            waypoint_x = float(waypoint[0])
            waypoint_y = float(waypoint[1])

            x2_x1_square = pow(waypoint_x - vehicle_pos_x, 2)
            y2_y1_square = pow(waypoint_y - vehicle_pos_y, 2)
            temp_dist = pow(x2_x1_square + y2_y1_square, 0.5)

            # self.get_logger().info("temp_dist: %f" % temp_dist)
            if (temp_dist < self.lookahead):
               within_lookahead.append([waypoint_x, waypoint_y, 0])

        return within_lookahead

    def shutdown(self):
        self.get_logger().info('Pursuit node ended itself.')

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()

    try:
        rclpy.spin(pure_pursuit_node)
    except KeyboardInterrupt:
        pass
    finally:
        pure_pursuit_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#WINNER WINNER CHICKEN DINNER