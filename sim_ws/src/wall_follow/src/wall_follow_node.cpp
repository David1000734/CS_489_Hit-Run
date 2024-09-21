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

        RCLCPP_INFO(this -> get_logger(),
            "Currently listening to %s", this -> sim_car.c_str());

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
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        double error = 0.0; // TODO: replace with error calculated by get_error()
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