// Get the pi constant
#define _USE_MATH_DEFINES
#include <math.h>

#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("David_Cpp_Safety_Node"), count_(0)
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: create ROS subscribers and publishers
        // Create publisher
        publisher_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive",
            10
        );

        timer_ = this -> create_wall_timer(
            500ms,
            std::bind(&Safety::timer_callback, this)
        );

        // Create subscriber
        subscription_ = this -> create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive",
            10,
            std::bind(&Safety::topic_callback, this, _1)
        );
        
        // Odometry subscriber
        odom_Subscription_ = this -> create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom",
            10,
            std::bind(&Safety::drive_callback, this, _1)
        );
        /// It don't work :(
        // scan_Publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
        //     "/scan",
        //     1
        // );

        // sensor_msgs::msg::LaserScan min_max_scan = sensor_msgs::msg::LaserScan();
        // min_max_scan.angle_min = -1.8;
        // min_max_scan.angle_max = 1.8;
        // scan_Publisher_ -> publish(min_max_scan);


        // Laser scan subscriber
        scan_Subscription_ = this -> create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&Safety::scan_callback, this, _1)
        );
        
        // Print a message
        RCLCPP_INFO(this -> get_logger(), "Hello from C++");
    }

private:
    double speed = 0.0;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_Subscription_;
    // rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_Publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_Subscription_;

    size_t count_;
    /// TODO: create ROS subscribers and publishers

    void timer_callback() {
        ackermann_msgs::msg::AckermannDriveStamped message = ackermann_msgs::msg::AckermannDriveStamped();
        // RCLCPP_INFO(this -> get_logger(), "I published: %f", message.drive.speed);
        publisher_ -> publish(message);

    }

    void topic_callback(ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg) const {
        std::cout << "subscribed value: " << msg -> drive.speed << std::endl;
        // RCLCPP_INFO(this -> get_logger(), "I heard: '%f'.", msg -> drive.speed);

    }

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        speed = msg -> twist.twist.linear.x;

        // RCLCPP_INFO(this -> get_logger(), "Odom Reading X: %f\tY: %f\tZ: %f.",
        // msg -> twist.twist.linear.x,
        // msg -> twist.twist.linear.y,
        // msg -> twist.twist.linear.z);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC
        // RCLCPP_INFO(this -> get_logger(), "Scan Min: %f\tScan Max: %f\tScan Increment: %f.",
        // scan_msg -> angle_min,
        // scan_msg -> angle_max,
        // scan_msg -> angle_increment);

        // RCLCPP_INFO(this -> get_logger(), "Min: %f%\tMax: %f%\tIncrement: %f%",
        // radian_to_degree(scan_msg -> angle_min),
        // radian_to_degree(scan_msg -> angle_max),
        // radian_to_degree(scan_msg -> angle_increment));

        try {

            calculate_TTC(scan_msg -> ranges,
                        scan_msg -> angle_increment,
                        scan_msg -> angle_min,
                        scan_msg -> angle_max);
        } catch (int x) {
            // Post to ackermann where speed = 0
            RCLCPP_INFO(this -> get_logger(), "Speed before: %f", this -> speed);

            ackermann_msgs::msg::AckermannDriveStamped message = ackermann_msgs::msg::AckermannDriveStamped();
            message.drive.speed = 0;
            RCLCPP_INFO(this -> get_logger(), "post: %f", message.drive.speed);
            publisher_ -> publish(message);
        }

        /// TODO: publish drive/brake message
    }

    // Convert radian to degrees
    double radian_to_degree(double radian) {
        return (radian * (180 / M_PI));
    }

    // Calculate the angle given the index of the array
    // and the angle increment
    double calculate_angle(int index, double increment) {
        return (radian_to_degree(index * increment));
    }

    void calculate_TTC(const std::vector<float> arr, double increment, double min, double max) {
        int length = arr.size();

        // Iterate through our array and calculate the TTC for each
        for (int i = 0; i < length; i++) {
            // TTC = (range[idx]) / (-1 * (speed * cos(angle[idx])))
            // TLDR: Top part will be the range at each index
            // The range is in meters
            if (arr[i] > max || arr[i] < min) {
                // Values outside of the min and max are ignored
                continue;
            }

            // Bottom part: current speed * cos of the current angle <negated>
            double bottom_part = this -> speed * cos(calculate_angle(i, increment));
            bottom_part *= -1;

            double calc = arr[i] / bottom_part;

            // If our value is less than the threshhold, then stop the car
            if (calc < 2 && calc > 0) {
                // Values that is -inf or higher than our threshold is ignored
                throw -1;
            }
        }
        
    }


};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
