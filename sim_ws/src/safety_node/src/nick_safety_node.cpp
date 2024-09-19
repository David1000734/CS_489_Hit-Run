#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <math.h>

class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("nick_safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        this -> declare_parameter("ttc", 0.0);
        this -> declare_parameter("mode", "sim");
        std::string sim_car = "/odom";

        if (this -> get_parameter("mode").as_string() == "sim"){
            sim_car = "/ego_racecar/odom";
        }

        /// TODO: create ROS subscribers and publishers
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            sim_car,
            10,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                this->odom_callback(std::move(msg));
            });
        
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
                this->scan_callback(msg);
            });
        
        acker_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive",
            10);

        RCLCPP_INFO(this->get_logger(), "SafetyNode initialized.");
    }

private:
    double speed = 0.0;
    /// TODO: create ROS subscribers and publishers

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        speed = msg->twist.twist.linear.x;
        // RCLCPP_INFO(this->get_logger(), "Current speed: '%f'", speed);

    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        // RCLCPP_INFO(this->get_logger(), "LaserScan received. %s");
        
        /// TODO: calculate TTC
        // scan_msg->angle_min = - 108 * scan_msg->angle_increment;
        // scan_msg->angle_max = 180 * scan_msg->angle_increment;

        // double angle_min = - 108 * scan_msg->angle_increment;
        // double angle_max = 180 * scan_msg->angle_increment;
        int scan_size = scan_msg->ranges.size();


        for (int i = 0; i < scan_size; i++){
            if (speed == 0.0){
                continue;
            }
            if (std::isinf(scan_msg->ranges[i]) || std::isnan(scan_msg->ranges[i]))
                continue;
            double range = scan_msg->ranges[i];
            double range_rate = speed * std::cos(i * scan_msg->angle_increment);

            double ttc = range / std::max( - range_rate, 0.0);
            if (ttc < 0 || ttc > 4)
                continue;
            if (ttc > 0)
                RCLCPP_INFO(this->get_logger(), "TTC. %f", ttc);
            if (ttc < this -> get_parameter("ttc").as_double()){
                acker_callback();
                break;
            }
            }    
        /// TODO: publish drive/brake message
    }

    void acker_callback(){
            auto acker_msg = ackermann_msgs::msg::AckermannDriveStamped();
            acker_msg.drive.speed = 0.0;
            acker_publisher_->publish(acker_msg);
            RCLCPP_INFO(this->get_logger(), "EMERGENCY BRAKE: TTC under threshold");
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::ConstSharedPtr scan_subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr acker_publisher_;
};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
