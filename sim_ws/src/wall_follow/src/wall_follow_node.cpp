#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers

        // Initilize variables
        this -> declare_parameter("speed", 0.0);
        this -> declare_parameter("P", 0.0);
        this -> declare_parameter("I", 0.0);
        this -> declare_parameter("D", 0.0);
        this -> declare_parameter("mode", "sim");

        // Set the topic for sim or physical car
        if (this -> get_parameter("mode").as_string() == "sim") {
            sim_car = "/ego_racecar/odom";      // Sim car
        }

        // Create publisher
        acker_Publisher_ = this -> create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive",
            10
        );

        // Odometry subscriber
        odom_Subscription_ = this -> create_subscription<nav_msgs::msg::Odometry>(
            sim_car,
            10,
            std::bind(&WallFollow::drive_callback, this, _1)
        );

        // Laser scan subscriber
        scan_Subscription_ = this -> create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&WallFollow::scan_callback, this, _1)
        );

        RCLCPP_INFO(this -> get_logger(),
            "Currently listening to %s", this -> sim_car.c_str());

    }

private:
    // PID CONTROL PARAMS
    // TODO: double kp =
    // TODO: double kd =
    // TODO: double ki =
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    std::string sim_car = "/odom";          // Physical car

    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr acker_Publisher_;    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_Subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_Subscription_;

    double get_range(float* range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // TODO: implement
        return 0.0;
    }

    double get_error(float* range_data, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        // TODO:implement
        return 0.0;
    }

    void pid_control(double error, double velocity)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        double angle = 0.0;
        // TODO: Use kp, ki & kd to implement a PID controller
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {

        // First we need to get our 'a' and 'b'
        // 'a' is the distance (meters) at 90% from the car
        // 'b' is the distance at 0 to 70 degrees from our car
        // We will use 45 degrees for now.

        /// To find what value in our range that corresponds to each degree
        /// we will do the following:
        /// Length of the array: 1080
        /// Angle min = -270, Angle max = 270
        /// Target angle for 90% from car = 270 - 90 = 180
        /// Thus I need the 180'th degree scan that corresponds to the ranges.
        /// To find out how much degrees each index in our array represents, we can do this
        /// 270 + 270 = 540     1080 / 540 = 2
        /// Each index corresponds to 2 degrees so, 
        /// 180 * 2 = 360       the 360's scan from the max is our target
        /// 1080 - 360 = 720
        ///
        /// by the same calculation, we can find b
        /// 90 + 45 = 135       target 135%
        /// We can follow the same logic as we did for a or
        /// 45 * 2 = 90         720 - 90 = 630

        // 90% from the car
        double a = scan_msg -> ranges[720];

        // 135% from the car 
        double b = scan_msg -> ranges[630];

        double lookahead = 2;

        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */

        // We set our degrees for 'a' and 'b' to be 45%
        double alpha = (a * cos(45) - b) / (a * sin(45));
        alpha = atan(alpha);
        ///               (a * cos(45) - b)
        /// alpha = atan   ____________
        ///                 (a * sin(45)

        double dt = b * cos(alpha);

        // Project the car accoding to L lookahead
        // dt_1 = dt + lookahead * sin(alpha)
        double dt_1 = dt + lookahead * sin(alpha);

        /// DEBUG
        /*
        RCLCPP_INFO(this -> get_logger(),
            "a: %f\tb: %f\nLookahead: %f\talpha: %f",
            a,
            b,
            dt,
            alpha
        );
        */

        // Error is simply desired distance - actual
        double error = 1 - dt; // TODO: replace with error calculated by get_error()

        double velocity = 0.0; // TODO: calculate desired car velocity based on error
        // TODO: actuate the car with PID

    }

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        // RCLCPP_INFO(this -> get_logger(), "Odom Reading X: %f\tY: %f\tZ: %f.",
        // msg -> twist.twist.linear.x,
        // msg -> twist.twist.linear.y,
        // msg -> twist.twist.linear.z);
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}