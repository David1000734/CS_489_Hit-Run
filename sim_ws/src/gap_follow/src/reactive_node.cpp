#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        // Initilize variables
        this -> declare_parameter("mode", "sim");
        this -> declare_parameter("bubble", 0.0);
        this -> declare_parameter("speed", 0.0);
        std::string sim_car = "/odom";      // Physical Car

        if (this -> get_parameter("mode").as_string() == "sim") {
            sim_car = "/ego_racecar/odom";      // Sim Car
        }

        /// TODO: create ROS subscribers and publishers
        // Ackerman Publisher
        acker_Publisher_ =
        this -> create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive",
            10);

        // Laser scan Subscriber
        scan_Subscription_ =
        this -> create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&ReactiveFollowGap::lidar_callback,
                      this,
                      std::placeholders::_1)
        );

        RCLCPP_INFO(this -> get_logger(), 
                    "Currently listening to %s.", sim_car.c_str());
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr acker_Publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_Subscription_;

    void preprocess_lidar(float* ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        return;
    }

    void find_max_gap(float* ranges, int* indice)
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        return;
    }

    void find_best_point(float* ranges, int* indice)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        return;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        RCLCPP_INFO(
            this -> get_logger(), 
            "\nmode: %s\tBubble: %f\tSpeed: %f.\n",
            this -> get_parameter("mode").as_string().c_str(),
            this -> get_parameter("bubble").as_double(),
            this -> get_parameter("speed").as_double()
        );
        */

        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        /// TODO:
        // Find closest point to LiDAR

        // Eliminate all points inside 'bubble' (set them to zero) 

        // Find max length gap 

        // Find the best point in the gap 

        // Publish Drive message
        // publish_ackerman(2.5, 1.9);
    }

    void publish_ackerman(const double car_speed = 0.0, const double steer_angle = 0.0, const bool debug = false) {
        ackermann_msgs::msg::AckermannDriveStamped acker_message = ackermann_msgs::msg::AckermannDriveStamped();

        if (debug) {
            RCLCPP_INFO(
                this -> get_logger(), 
                "\n***** Try Publish *****\nSpeed: %f\tSteering_Angle: %f\n",
                car_speed, steer_angle
            );
        }

        // Set the speed and angle
        acker_message.drive.speed = car_speed;
        acker_message.drive.steering_angle = steer_angle;

        // Publish
        acker_Publisher_ -> publish(acker_message);

        if (debug) {
            RCLCPP_INFO(
                this -> get_logger(), 
                "\n***** Published *****\nSpeed: %f\tSteering_Angle: %f\n",
                acker_message.drive.speed, acker_message.drive.steering_angle
            );
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}