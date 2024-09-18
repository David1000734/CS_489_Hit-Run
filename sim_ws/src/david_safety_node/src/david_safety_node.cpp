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

        this -> declare_parameter("ttc", 0.0);
        this -> declare_parameter("mode", "sim");
        std::string sim_car = "/odom";

        // Set the topic for sim or physical car
        if (this -> get_parameter("mode").as_string() == "sim") {
            sim_car = "/ego_racecar/odom";
        }

        /// TODO: create ROS subscribers and publishers
        // Create publisher
        publisher_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive",
            10
        );

        // Odometry subscriber
        odom_Subscription_ = this -> create_subscription<nav_msgs::msg::Odometry>(
            sim_car,
            10,
            std::bind(&Safety::drive_callback, this, _1)
        );

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
    const double PI = 3.1415926535;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_Subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_Subscription_;

    size_t count_;
    /// TODO: create ROS subscribers and publishers

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

        /// TODO: publish drive/brake message
        try {
            calculate_TTC(scan_msg -> ranges,
                        scan_msg -> angle_increment);
        } catch (double x) {
            // Post to ackermann where speed = 0
            ackermann_msgs::msg::AckermannDriveStamped message = ackermann_msgs::msg::AckermannDriveStamped();
            message.drive.speed = 0;
            RCLCPP_INFO(this -> get_logger(),
                        "Emergency Break with TTC at %f", x);
            publisher_ -> publish(message);
        }

    }

    // Convert radian to degrees
    double radian_to_degree(double radian) {
        return (radian * (180 / PI));
    }

    // Calculate the angle given the index of the array
    // and the angle increment
    double calculate_angle(int index, double increment) {
        return (index * increment);
    }

    /// @brief Function will calculate the time to colission. This function will
    /// also ignore a lot of the values that exceed or is smaller than a particular range.
    ///
    /// @param arr Laser scan array. This array is in terms of meters.
    ///
    /// @param increment The increment of each angle from 0 to the max in radians
    ///
    /// @param min The max meters to consider for the the TTC calculation
    ///
    /// @param max The minimum meaters to consider. Set to 0
    void calculate_TTC(const std::vector<float> arr, double increment, double min = 0.0) {
        // We will only consider 60% from the center of the scan
        // this mean arr.size() * (300 / 360) = 900
        // So, we will start from 450 and go up to 1080 - 450
        double ttc = this -> get_parameter("ttc").as_double();
        double max = log2(ttc) + 1;

        // Iterate through our array and calculate the TTC for each
        for (int i = 450; i < 630; i++) {
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

            double calc = arr[i] / std::max(bottom_part, 0.0);

            // If our value is less than the threshhold, then stop the car
            if (calc < ttc && calc > 0) {
                // Values that is -inf or higher than our threshold is ignored
                throw calc;
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
