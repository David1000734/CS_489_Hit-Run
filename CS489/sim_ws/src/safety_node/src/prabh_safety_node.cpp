#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
//#include <math.h>

class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("prabh_safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */
        this->declare_parameter("ttc",0.0);
        this->declare_parameter("mode","sim");
        std::string sim_car = "/odom";

        //set topic for car
        if(this->get_parameter("mode").as_string()=="sim")
        {
            sim_car = "/ego_racecar/odom";
        }
        // Subscriber to LaserScan messages
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1));
        
        // Subscriber to Odometry messages
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            sim_car, 10, std::bind(&Safety::drive_callback, this, std::placeholders::_1));

        // Publisher to AckermannDriveStamped messages
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        RCLCPP_INFO(this->get_logger(), "SafetyNode initialized.");
        
    }

private:
    double speed = 0.0;
    /// TODO: create ROS subscribers and publishers

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        //get the velocity from odom
        this -> speed = msg->twist.twist.linear.x;
        
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC
        size_t num_ranges = scan_msg->ranges.size();
        // double threshold_ttc = 1.0; // Threshold for iTTC in seconds

        for (size_t i = 0; i < num_ranges; ++i) {
            double range = scan_msg->ranges[i];

            // Ignore invalid ranges
            if (std::isinf(range) || range < scan_msg->range_min || range > scan_msg->range_max) {
                continue;
            }

            // get angle to this range measurement
            double angle = scan_msg->angle_min + i * scan_msg->angle_increment;//get angle_min and angle_inc from the laser scan measurements

            // Calculate range rate: v * cos(angle)
            double range_rate = this -> speed * std::cos(angle);

            // divide the range by current range rate
            double iTTC = range / std::max(range_rate, 0.0); // max to avoid division by zero

            // check if iTTc <threshold
            if (iTTC < this -> get_parameter("ttc").as_double() &&
                iTTC > 0) {
                stop_car(); //stop the car if iTTc < threshold
                break;
            }
        }

    }
    // Function to stop the car by publishing a brake command
    void stop_car()
    {
        // ackermann_msgs::msg::AckermannDriveStamped drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.drive.speed = 0.0; // Stop the car
        drive_publisher_->publish(drive_msg);

        RCLCPP_INFO(this->get_logger(), "Car stopped (speed: %f)", drive_msg.drive.speed);
    }

    // Subscribers and publisher
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}