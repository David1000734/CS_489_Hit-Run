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

        # How many iterative waypoints to return. (lookahead)
        self.declare_parameter('wpnum', 5)

        mode = self.get_parameter('mode').get_parameter_value().string_value
        self.turbo = self.get_parameter('turbo').get_parameter_value().double_value
        self.lookahead = self.get_parameter('lookahead').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.default_speed = self.speed
        self.path = self.get_parameter('path').get_parameter_value().string_value
        self.filename = self.get_parameter('filename').get_parameter_value().string_value
        self.wp_num = self.get_parameter('wpnum').get_parameter_value().integer_value

        # Const values that should not change
        self.STATIC_WP_NUM = self.wp_num
        self.STATIC_LOOKAHEAD = self.lookahead

        # Keep track of the previous waypoint
        self.prev_idx = None

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
        # within_lookahead_points = self.find_nearest_waypoint(waypoint_array, vehicle_pos)

        within_lookahead_points = self.find_next_waypoint(waypoint_array, vehicle_pos)

        # w is the real scalar value
        # Calculate inverse quaternion by
        # normalizing it, find it's conjunction, then divide by the sum^2
        # (w, x, y, z) --> (w, -x, -y, -z)      Inv_Q
        #   w^2 + x^2 + y^2 + z^2       Denominator
        # Inverse Q = Inv_Q/Denominator
        rotation_inv = vehicle_orientation * [1, -1, -1, -1] / np.sum(vehicle_orientation ** 2)

        waypoints_vehicle_frame = self.transform_waypoints(vehicle_pos, within_lookahead_points, rotation_inv)


        # self.get_logger().info(
        #     "Within Lookahead: %s\nVehicle Frame: %s\n" %
        #     (str(within_lookahead_points), str(waypoints_vehicle_frame))
        # )
        # rclpy.shutdown()

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

        # Steering angle is currently in radians. The conditions below are in degrees
        # Convert from radians to degrees
        converted_steering_angle = self.radian_to_degree(steering_angle)

        #angle cut off
        if (converted_steering_angle > 24.0):
            converted_steering_angle = 24.0
        if (converted_steering_angle < -24.0):
            converted_steering_angle = -24.0

        #region SPEED
        if ((converted_steering_angle >= -4.0000) and (converted_steering_angle <= 4.0000)):
            # Speed will be specified by the launch parameters
            # If less than 20 and greater than 10
            self.speed = self.default_speed
            self.speed *= self.turbo

            # # Just going straight, we should increase lookahead.
            # # Use the static one in case the regular one has changed
            # self.lookahead = self.dynamic_increment(self.STATIC_LOOKAHEAD, True)
            # self.wp_num = self.dynamic_increment(self.STATIC_WP_NUM, True)

        elif ((converted_steering_angle > 4.0000 and converted_steering_angle <= 8.0000) or
                 (converted_steering_angle < -4.0000 and converted_steering_angle >= -8.0000)):
            self.speed = self.default_speed
            self.speed = self.speed

            # # Still straight enough, we should increase lookahead.
            # # Use the static one in case the regular one has changed
            # self.lookahead = self.dynamic_increment(self.STATIC_LOOKAHEAD, True)
            # self.wp_num = self.dynamic_increment(self.STATIC_WP_NUM, True)

        elif ((converted_steering_angle > 8.0000 and converted_steering_angle <= 12.0000) or
                 (converted_steering_angle < -8.0000 and converted_steering_angle >= -12.0000)):
            self.speed = self.default_speed
            self.speed *= .85

            # # Turn is tighter, use normal lookahead and decreased speed
            # self.lookahead = self.STATIC_LOOKAHEAD
            # self.wp_num = self.STATIC_WP_NUM

        elif ((converted_steering_angle > 12.0000 and converted_steering_angle <= 14.0000) or
                 (converted_steering_angle < -12.0000 and converted_steering_angle >= -14.0000)):
            self.speed = self.default_speed
            self.speed *= .75

            # # Tight turn, use smaller lookahead and decrease speed
            # self.lookahead = self.dynamic_increment(self.STATIC_LOOKAHEAD, False)
            # self.wp_num = self.dynamic_increment(self.STATIC_WP_NUM, False)

        elif ((converted_steering_angle > 14.0000 and converted_steering_angle < 24.0001) or
                 (converted_steering_angle < -14.0000 and converted_steering_angle > -24.0001)):
            self.speed = self.default_speed
            self.speed *= .65

            # # Tighter turn, use smaller lookahead and decrease speed
            # self.lookahead = self.dynamic_increment(self.STATIC_LOOKAHEAD, False)
            # self.wp_num = self.dynamic_increment(self.STATIC_WP_NUM, False)

        else:
            self.speed = self.default_speed
            self.speed *= 0.5

            # # Tightest turn, use smaller lookahead and decrease speed
            # self.lookahead = self.dynamic_increment(self.STATIC_LOOKAHEAD, False)
            # self.wp_num = self.dynamic_increment(self.STATIC_WP_NUM, False)
        # After setting the correct speed, we can use the original
        # steering angle in radians to specify where to turn

        self.publish_ackerman(self.speed, steering_angle)
        pass


    def transform_dict_waypoints(self, vehicle_pos, within_lookahead_points, rotation_inv):
        if (len(within_lookahead_points) ==0):
            return {}

        waypoints_translated = {}

        for key, value in within_lookahead_points.items():
            # Subtract corresponding elements
            translated = [v - vp for v, vp in zip(value, vehicle_pos)]
            waypoints_translated[key] = translated

        # Extract only the x, y components (remove the z component)
        way_points_x_y = {key: value[:-1] for key, value in waypoints_translated.items()}

        orientation_w = rotation_inv[0]
        orientation_z = rotation_inv[-1]

        theta = 2 * np.arctan2(orientation_z, orientation_w)
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        rotation_matrix = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])

        waypoints_rotated = {}

        for key, value in way_points_x_y.items():
            point = np.array(value)
            # Apply the rotation matrix
            rotated_point = np.dot(rotation_matrix, point)
            # Store the rotated point in the dictionary
            waypoints_rotated[key] = rotated_point.tolist()

        return waypoints_rotated

    # Region Transform
    def transform_waypoints(self, vehicle_pos, within_lookahead_points, rotation_inv):
        if len(within_lookahead_points) == 0:
            return []       # Return nothing

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
        for (idx, waypoint) in enumerate(waypoint_array):
            waypoint_x = float(waypoint[0])
            waypoint_y = float(waypoint[1])

            x2_x1_square = pow(waypoint_x - vehicle_pos_x, 2)
            y2_y1_square = pow(waypoint_y - vehicle_pos_y, 2)
            temp_dist = pow(x2_x1_square + y2_y1_square, 0.5)

            # self.get_logger().info("temp_dist: %f" % temp_dist)
            # self.get_logger().info("Lookahead: %f" % self.lookahead)
            if (temp_dist < self.lookahead):
                # NOTE: Last value is not used, we will use it to remember the index
               within_lookahead.append([waypoint_x, waypoint_y, idx])

        return within_lookahead

    def find_next_waypoint(self, waypoint_array, vehicle_pos):
        # Vehicle X: vehicle_pos[0]
        # Vehicle Y: vehicle_pos[1]
        within_iteration = []

        csvsize = len(waypoint_array)

        # Index of the waypoint the vehicle is closest to
        center_idx = 0

        # Find the waypoint closest to the vehicle right now
        within_iteration = self.find_nearest_waypoint(waypoint_array, vehicle_pos)

        try:
            # Determine if we have a previous waypoint already
            if (self.prev_idx is None):
                # Find the smallest waypoint and get only the idx out of it
                center_idx = min(within_iteration)[-1]      # ValueError

                # Update previous index
                self.prev_idx = center_idx

                # self.get_logger().info(
                #     "Localizer ran this time: %i\n" %
                #     self.prev_idx
                # )
            else:
                wp_in_range = []
                # If we do have a previous waypoint,
                # the next waypoint must be near this one
                wp_in_range = [wp for wp in within_iteration if self.within_range(wp[-1], self.prev_idx, 20, csvsize)]

                # Grab center most index from this list
                center_idx = wp_in_range[math.floor(len(wp_in_range) / 2)][-1]      # IndexError

                # Update previous index
                self.prev_idx = center_idx
            # if else, END

        except (IndexError, ValueError) as error:
            self.get_logger().info(
                "Index Selection Error: \n%s" %
                (error)
            )

            # self.get_logger().info(
            #         "\nRange: %s\nwithin: %s\n" %
            #         (str(wp_in_range), str(within_iteration))
            #     )

            # Array was empty, no middle found
            self.prev_idx = None
            return []

        # Grab the next wp_num waypoints from the center_idx
        within_iteration = self.array_splicing(waypoint_array, center_idx, self.wp_num)

        ### Drive Counter Clock-Wise, UNTESTED

        # # From that waypoint, grab the next N waypoints
        # # First figure out if idx + N is out of bounds
        # if (center_idx + self.wp_num > len(waypoint_array)):
        #     # It is too big, split it up
        #     difference = len(waypoint_array) - center_idx
        #     second_split = self.wp_num - difference

        #     # First split will be from center_idx up to the max
        #     temp_array = waypoint_array[ center_idx: ][ :-1 ]

        #     # Second split will be from beginning up to the rest
        #     temp_array += waypoint_array[ 0:second_split ][ :-1 ]
        # else:
        #     # It is not too big, just grab the values
        #     temp_array = waypoint_array[center_idx : center_idx + self.wp_num][ :-1 ]
        # Note, if the current waypoint + N waypoints goes out of bounds,
        # Grab the difference from the beginning of the list

        ### Drive Counter Clock-Wise, UNTESTED

        # Remove the very last element off of each waypoint
        within_iteration = [row[:-1] for row in within_iteration]

        # Return those waypoints
        return within_iteration


    def within_range(self, X, target, within_range, csvsize) -> bool:
        """ Function will simply answer the question, Is X within target with
            a range of within_range. 
            
            Ex.
            Is 5 within 6 with a range of 2.

            Thus is 5 greater than 6 - 2 and less than than 6 + 2

            4 < 5 < 8 will return True

            NOTE: x + within_range < target < x - within_range
        """
        is_within_range = True

        # If X is too small or too large, return false
        # if (X < target - within_range or
        #     X > target + within_range):
        #     is_within_range = False

        lower_bound = (target - within_range) % csvsize
        upper_bound = (target + within_range) % csvsize

        # Check if X is within the looped range
        if lower_bound <= upper_bound:
            is_within_range = lower_bound <= X <= upper_bound
        else:
            # Handles the case where the range wraps around
            is_within_range = X >= lower_bound or X <= upper_bound

        return is_within_range

    def array_splicing(self, array: list, point: int, N: int) -> list:
        ### Drive Clock-Wise
        # From that waypoint, grab the next N waypoints
        # First figure out if idx - N is out of bounds
        if (point - N < 0):
            # It is, split it up
            difference = abs(point - N)
            first_split = len(array) - (N - difference)

            # self.get_logger().info(
            #     "\nMath: %d\nDifference: %d\nFirst Split: %d" %
            #     (point - N, difference, first_split)
            # )

            # First split will be from max - difference up to max
            temp_array = array[ first_split: ][ : ]

            # Second split will be from 0 up to the difference
            temp_array += array[ 0:difference ][ : ]

            # self.get_logger().info(
            #     "Second array: %s" %
            #     (str(temp_array))
            # )
        else:
            # It is not, just grab the values
            temp_array = array[point - N : point][ : ]

            # self.get_logger().info(
            #     "No split: %s\nPoint - N: %i\npoint: %iarray: %s" %
            #     (str(temp_array), point - N, point, str(array))
            # )        # If else, END

        return temp_array
    
    def dynamic_increment(self, baseValue: float, isInc: bool) -> float:
        """
        Function returns the operation
        ( X +- (X / 2))

        Parameters:
            baseValue (float): Initial value to increment
            isInc (bool): Should this function add or subtract

        Returns:
            Returns X plus or minus half of X
            
        Notes:
            This function should not return anything less than X
        """
        operation = baseValue / 2

        if (isInc):
            # When adding, take the lowest
            operation = math.floor(baseValue + operation)
        else:
            # When subtracting, take the highest
            operation = math.ceil(baseValue - operation)

        return operation

    def radian_to_degree(self, radian: float) -> float:
        """
        Helper function to convert from radians to degrees
        """
        return (radian * (180 / math.pi))
    
    def degree_to_radian(self, degree: float) -> float:
        """
        Helper function to convert from degrees to radians
        """
        return (degree * (math.pi / 180))

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