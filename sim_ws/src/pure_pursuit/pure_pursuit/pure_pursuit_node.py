#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math
import os
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray


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
        self.declare_parameter('x_neg', float(-1.0))
        self.declare_parameter('y_neg', float(-1.0))


        mode = self.get_parameter('mode').get_parameter_value().string_value
        self.turbo = self.get_parameter('turbo').get_parameter_value().double_value
        self.lookahead = self.get_parameter('lookahead').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.default_speed = self.speed
        self.x_neg = self.get_parameter('x_neg').get_parameter_value().double_value
        self.y_neg = self.get_parameter('y_neg').get_parameter_value().double_value


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
        
        self.visualize_marker_array_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.visualize_marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        wp = np.loadtxt('/sim_ws/src/pure_pursuit/pure_pursuit/waypoints.csv', delimiter=',', dtype=float)  # x, y
        wp = np.concatenate((wp, np.zeros(shape=(len(wp), 1), dtype=float)), axis=1)    # x, y, z

        self.visualize_waypoints(wp)

        self.get_logger().info(f'Python listening to: {mode}')

    def acker_callback(self, acker_msg): 

        pass

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
        marker.scale.x = 0.11
        marker.scale.y = 0.11
        marker.scale.z = 0.11
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0

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
        relative_directory = "/sim_ws/src/pure_pursuit/pure_pursuit/"

        waypoint_array = self.readCSV(relative_directory + "waypoints.csv")

        
        vehicle_pos = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, 0])
        vehicle_orientation = np.array([pose_msg.pose.pose.orientation.w,
                                        pose_msg.pose.pose.orientation.x,
                                        pose_msg.pose.pose.orientation.y,
                                        pose_msg.pose.pose.orientation.z])

        # Calculate distances to waypoints and filter by LOOKAHEAD_DISTANCE
        within_lookahead_points = self.find_nearest_waypoint(waypoint_array, vehicle_pos)

        # Transform waypoints to vehicle frame
        rotation_inv = vehicle_orientation * [1, -1, -1, -1] / np.sum(vehicle_orientation ** 2)
        waypoints_vehicle_frame = self.transform_waypoints(vehicle_pos, within_lookahead_points, rotation_inv)

        # Select target waypoint (furthest within lookahead in front of vehicle)
        front_waypoints = waypoints_vehicle_frame[waypoints_vehicle_frame[:, 0] > 0]
        if not len(front_waypoints):
            return
        target_waypoint = front_waypoints[np.argmax(front_waypoints[:, 0])]

        # Compute steering angle and velocity
        dist_to_target = np.linalg.norm(target_waypoint)
        curvature = 2 * abs(target_waypoint[1]) / dist_to_target ** 2
        steering_angle = np.clip(curvature * 0.5 * np.sign(target_waypoint[1]), -24.0, 24.0)

        self.visualize_target_waypoint([target_waypoint[0], target_waypoint[1]])

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

        self.publish_ackerman(self.speed, steering_angle, False)
        pass

    def transform_waypoints(self, vehicle_pos, within_lookahead_points, rotation_inv):
        waypoints_translated = within_lookahead_points - vehicle_pos
        rotation_inv_w, rotation_inv_vec = rotation_inv[0], rotation_inv[1:]
        t = np.cross(2 * rotation_inv_vec, waypoints_translated)
        return waypoints_translated + rotation_inv_w * t + np.cross(rotation_inv_vec, t)


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

    #region Convert
    def convert_waypoints_to_car_frame(self, waypoint_array, world_pose_msg):

        orientation_z = world_pose_msg.pose.pose.orientation.z #ros2 odometry message of car

        position_x = world_pose_msg.pose.pose.position.x #ros2 odometry message of car
        position_y = world_pose_msg.pose.pose.position.y #ros2 odometry message of car

        points_to_point = []
        for waypoint in waypoint_array:
            x_coordinate_of_waypoint = float(waypoint[0]) #x value of waypoint
            y_coordinate_of_waypoint = float(waypoint[1]) #y value of waypoint
          
            # test_yaw = yaw
            test_yaw = (math.asin(orientation_z)) * 2
            test_yaw = orientation_z
            
            new_x = (x_coordinate_of_waypoint * math.cos(test_yaw) - y_coordinate_of_waypoint * math.sin(test_yaw)) - position_x
            new_y = (x_coordinate_of_waypoint * math.sin(test_yaw) + y_coordinate_of_waypoint * math.cos(test_yaw)) - position_y
            new_x *= self.x_neg
            new_y *= self.y_neg

            

            if (new_x > 0.0):
                points_to_point.append([new_x, new_y])
        return points_to_point

    #region Nearest
    def find_nearest_waypoint_in_car_frame(self, list_of_waypoints_in_car_frame, world_pose_msg):
        closest_waypoint = [0,0]
        
        minimum_dist = 0
        for waypoint in list_of_waypoints_in_car_frame:
            waypoint_x = float(waypoint[0])
            waypoint_y = float(waypoint[1])

            self.get_logger().info("looking X: %f\looking Y: %f" %
                                   (waypoint_x, waypoint_y))
            
            if (waypoint_x == 0 and waypoint_y == 0):
                continue


            x2_x1_square = pow(waypoint_x - 0, 2)
            y2_y1_square = pow(waypoint_y - 0, 2)
            temp_dist = pow(x2_x1_square + y2_y1_square, 0.5)

            # self.get_logger().info("temp_dist: %f" % temp_dist)
            if (temp_dist < self.lookahead):
                if (temp_dist > minimum_dist):
                    closest_waypoint = waypoint
                    minimum_dist = temp_dist

        # Recreate the list as a list of float values instead of str
        self.get_logger().info("looking X: %f\looking Y: %f" %
                                   (closest_waypoint[0], closest_waypoint[1]))
        return [float(i) for i in closest_waypoint]

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
