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
        this->declare_parameter("speedmult", 1.0);
        this->declare_parameter("lookahead", 2.0);
        this->declare_parameter("angleclamp", 24.0);

        // TODO: create ROS subscribers and publishers
        // Ackerman Publisher
        acker_Publisher_ =
            this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
                "/drive",
                10
        );

        pose_Subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped::ConstPtr>(
            "/pose",
            10,
            std::bind(&PurePursuit::pose_callback, this, _1));

        // Ready message
        RCLCPP_INFO(
            this -> get_logger(),
            "\nNode ready to go.\n"
        );
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstPtr &pose_msg)
    {
        //Position 
        double position_x = pose_msg->pose->position->x
        double position_y = pose_msg->pose->position->y
        double position_z = pose_msg->pose->position->z

        //Orientation
        double orientation_x = pose_msg->pose->orientation->x
        double orientation_y = pose_msg->pose->orientation->y
        double orientation_z = pose_msg->pose->orientation->z
        double orientation_w = pose_msg->pose->orientation->w

        //temporary
        double steer_angle = 0.0;
        double lateral_offset = 0.0;
        double L = this->get_parameter("lookahead").as_double(); 
        

        // TODO: find the current waypoint to track using methods mentioned in lecture




        // TODO: transform goal point to vehicle frame of reference



        // TODO: calculate curvature/steering angle
        //this will change once we calculate y in local frame of reference
        lateral_offset = position_y;

        //steer_angle = (2 * |y|) / L^2
        steer_angle = (2 * abs(lateral_offset)) / pow(L, 2);

        // TODO: publish drive message, don't forget to limit the steering angle.
        double angle_clamp = this->get_parameter("angleclamp").as_double(); 


        if (steer_angle > 24.0){
            steer_angle = 24.0;
        }
        else if (steer_angle < -24.0){
            steer_angle = -24.0;
        }

        // speedmult = 1 for already determined speeds.
        // speedmult between 1.0 to 2.0 for up to 2x speed
        //if we implement our speed with our waypoints correctly we wont ever need to multiply by more than 2 EVER.

        double speedmult = this->get_parameter("speedmult").as_double(); // biggest change we are looking for when finding corners
        speed *= speedmult;

        publish_ackerman(speed, steer_angle);
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