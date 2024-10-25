#include <sstream>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

class PurePursuit : public rclcpp::Node
{
    // Implement PurePursuit
    // This is just a template, you are free to implement your own node!
private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr acker_Publisher_;

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        // Initilize Variables
        this->declare_parameter("speed", 1.0);

        // TODO: create ROS subscribers and publishers
        // Ackerman Publisher
        acker_Publisher_ =
            this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
                "/drive",
                10
        );

        // Ready message
        RCLCPP_INFO(
            this -> get_logger(),
            "\nNode ready to go.\n"
        );
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstPtr &pose_msg)
    {
        // TODO: find the current waypoint to track using methods mentioned in lecture

        // TODO: transform goal point to vehicle frame of reference

        // TODO: calculate curvature/steering angle

        // TODO: publish drive message, don't forget to limit the steering angle.
    }

    ~PurePursuit() {}

    /// @brief Helper function to post the speed and angle for the car
    ///
    /// @param car_speed Specified speed for the car. Nullable
    ///
    /// @param steer_angle Specified angle. Nullable
    ///
    /// @param debug True if you want to print the
    /// speed and angle being published. Default is false.
    void publish_ackerman(const double car_speed = 0.0, const double steer_angle = 0.0, const bool debug = false)
    {
        ackermann_msgs::msg::AckermannDriveStamped acker_message = ackermann_msgs::msg::AckermannDriveStamped();

        if (debug)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "\n***** Try Publish *****\nSpeed: %f\tSteering_Angle: %f\n",
                car_speed, steer_angle);
        }

        // Set the speed and angle
        acker_message.drive.speed = car_speed;
        acker_message.drive.steering_angle = steer_angle;

        // Publish
        acker_Publisher_->publish(acker_message);

        if (debug)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "\n***** Published *****\nSpeed: %f\tSteering_Angle: %f\n",
                acker_message.drive.speed, acker_message.drive.steering_angle);
        }
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}