#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <string>
#include <cmath>
using namespace std::chrono_literals;
using std::placeholders::_1;

class Safety : public rclcpp::Node {
public:
    Safety() : Node("nick_safety_node")
    {
        // get parm to determine if in sim or on car
        this->declare_parameter("mode");
        this->declare_parameter("ttc");

        std::string mode = this->get_parameter("mode").as_string();

        // Subscribe to the /ego_racecar/odom topic to get speed data from Odometry
        if(mode == "sim")
        {
            odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, std::bind(&Safety::drive_callback, this, _1));
        }
        else
        {
            odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&Safety::drive_callback, this, _1));            
        }

        // Subscribe to the /scan topic to get LaserScan data
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Safety::scan_callback,this, _1));

        // Create a publisher for AckermannDriveStamped messages
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    }

private:
    // Speed variable to store the current speed from odometry
    double speed = 0.0;

    // ROS 2 subscribers and publisher
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;

    // Callback for /ego_racecar/odom topic
    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        // get speed
        speed = msg->twist.twist.linear.x;
    }

    // Callback for /scan topic (LaserScan)
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC
        double angle = 0.0;
        double range_rate = 0.0;
        double col_time = 0.0;
        double distance = 0.0;
        double ttc = this->get_parameter("ttc").as_double();

        //lets put it in a loop that gives us the values in a 20% cone in front of us, ignores nan and inf
        for(int i = 432; i < 648; i++)
        {
            if(std::isinf(scan_msg->ranges[i]) || std::isnan(scan_msg->ranges[i]))
            {
                continue;
            }
            
            if(speed == 0.0){continue;}

            distance = scan_msg->ranges[i];
            angle = scan_msg -> angle_min + (i * scan_msg->angle_increment);
            range_rate = speed * std::cos(angle);
            // Skip cases where range_rate is negative or zero
            if (range_rate <= 0.0)
            {
                continue;
            }
            // Skip calculation if range_rate is zero (when the vehicle is not moving) or infinity or nan lol
            col_time = distance / range_rate;
            //RCLCPP_INFO(this->get_logger(), "col time %f", col_time);
            if(col_time < 0 || std::isinf(col_time) || std::isnan(col_time))
            {
                continue;
            }
            if(col_time < ttc)
            {
                // publish drive break message here
                RCLCPP_INFO(this->get_logger(), "Crash Imminent! Brake! %f", col_time);
                brake_message();
                break;
            }
        }
    }

    // publish the ackermann message to get the car to stop
    void brake_message()
    {
        // create ackermann msg
        auto brake_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // set speed to 0
        brake_msg.drive.speed = 0.0;
        // Publish the Ackermann message
        ackermann_publisher_->publish(brake_msg);
        RCLCPP_INFO(this->get_logger(), "Brake Message Published");
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
