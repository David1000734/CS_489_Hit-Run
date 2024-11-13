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
        # self.get_logger().info(
        #     f"Value: {pose_msg}\n"
        # )

        # Position
        position_x = pose_msg.pose.pose.position.x
        position_y = pose_msg.pose.pose.position.y
        position_z = pose_msg.pose.pose.position.z

        # Orientation
        orientation_x = pose_msg.pose.pose.orientation.x
        orientation_y = pose_msg.pose.pose.orientation.y
        orientation_z = pose_msg.pose.pose.orientation.z
        orientation_w = pose_msg.pose.pose.orientation.w

        # temporary
        lateral_offset = 0.0
        #region WAYPOINT

        # TODO: translate vehicle frame to world frame
        world_pose_msg = pose_msg

        relative_directory = "/sim_ws/src/pure_pursuit/pure_pursuit/"

        # TODO: find the current closest waypoint to track using methods mentioned in lecture
        waypoint_array = self.readCSV(relative_directory + "waypoints.csv")

        # self.get_logger().info(
        #     'Test array here\n'
        # )
        # for waypoint in waypoint_array:
        #     self.get_logger().info(
        #         f"{waypoint}"
        #     )

        list_of_waypoints_in_car_frame = self.convert_waypoints_to_car_frame(waypoint_array, world_pose_msg)
        result_matrix = self.find_nearest_waypoint_in_car_frame(list_of_waypoints_in_car_frame, world_pose_msg)

        # closest_waypoint = self.find_nearest_waypoint(waypoint_array, world_pose_msg)
        #set following variables equal to closest_waypoint[] from index 0 to 3
        # x_coordinate_of_waypoint, y_coordinate_of_waypoint, yaw, speed = closest_waypoint

        self.get_logger().info(
            'Passed nearest waypoint: %s\n'% 
            str(result_matrix)
        )


        # # TODO: transform goal point to vehicle frame of reference
        # #transform closest_waypoint to the vehicle frame of reference to calculate steering angle
        # quaternion = (orientation_x, orientation_y, orientation_z, orientation_w)

        # euler = euler_from_quaternion(quaternion)

        # #grab the rotation
        # test_yaw = euler[2]

        # # create a temp matrix for multiplication
        # temp_matrix = np.array([[x_coordinate_of_waypoint], [y_coordinate_of_waypoint]])

        # # create the transform matrix
        # rotate_matrix = np.array([[math.cos(test_yaw) , -math.sin(test_yaw)],
        #                              [math.sin(test_yaw) , math.cos(test_yaw)]])

        # # multiply
        # result_matrix = np.dot(rotate_matrix, temp_matrix)

        # self.get_logger().info(
        #     str(result_matrix.shape)
        # )

        new_x_in_car_frame = result_matrix[0]
        new_y_in_car_frame = result_matrix[1]

        self.get_logger().info(
            'Pointing towards: %f, %f\n'% 
            (new_x_in_car_frame, new_y_in_car_frame)
        )

        # TODO: calculate curvature/steering angle
        lateral_offset = new_y_in_car_frame
        # lateral_offset *= -1

        #region STEER ANGLE
        #steer_angle = (2 * |y|) / lookeahead^2
        steering_angle = (2 * (lateral_offset)) / pow(self.lookahead, 2)

        # TODO: publish drive message, don't forget to limit the steering angle.
        if (steering_angle > 24.0):
            steering_angle = 24.0
        elif (steering_angle < -24.0):
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

        self.publish_ackerman(self.speed, steering_angle, False)
        pass

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
        #./ for current directory. 
        #../ to go back a directory
        array_of_waypoints = list(csv.reader(open(directory)))
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

        # Recreate the list as a list of float values instead of str
        self.get_logger().info("looking X: %f\looking Y: %f" %
                                   (closest_waypoint[0], closest_waypoint[1]))
        return [float(i) for i in closest_waypoint]

    #region minDist
    def find_nearest_waypoint(self, waypoint_array, world_pose_msg):
        # self.get_logger().info("World Pose: %s" % str(world_pose_msg))
        
        closest_waypoint = waypoint_array[0]

        # Car's orientation (assuming quaternion is converted to yaw)
        
        minimum_dist = 10.0
        for idx, waypoint in enumerate(waypoint_array):
            #self.get_logger().info("Waypoint: %s" % waypoint)

            # if least distance
            waypoint_x = float(waypoint[0])
            waypoint_y = float(waypoint[1])

            # self.get_logger().info("x_waypoint: %f\ty_waypoint: %f" % (waypoint_x, waypoint_y))

            x2_x1_square = pow(waypoint_x - world_pose_msg.pose.pose.position.x, 2)
            y2_y1_square = pow(waypoint_y - world_pose_msg.pose.pose.position.y, 2)
            temp_dist = pow(x2_x1_square + y2_y1_square, 0.5)

            # self.get_logger().info("temp_dist: %f" % temp_dist)

            if (temp_dist < minimum_dist or
                (idx == len(waypoint_array) - 1) and (closest_waypoint == waypoint_array[0])):
                closest_waypoint = waypoint

        # Recreate the list as a list of float values instead of str
        return [float(i) for i in closest_waypoint]

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
