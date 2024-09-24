#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;

class WallFollow : public rclcpp::Node
{

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers

        // Initilize variables
        this->declare_parameter("speed", 0.0);
        this->declare_parameter("P", 0.0);
        this->declare_parameter("I", 0.0);
        this->declare_parameter("D", 0.0);
        this->declare_parameter("mode", "sim");

        // Set the topic for sim or physical car
        if (this->get_parameter("mode").as_string() == "sim")
        {
            sim_car = "/ego_racecar/odom"; // Sim car
        }

        // Create publisher
        acker_Publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive",
            10);

        // Odometry subscriber
        odom_Subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            sim_car,
            10,
            std::bind(&WallFollow::odom_callback, this, _1));

        // Laser scan subscriber
        scan_Subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&WallFollow::scan_callback, this, _1));

        RCLCPP_INFO(this->get_logger(),
                    "Currently listening to %s", this->sim_car.c_str());
    }

private:
    // PID CONTROL PARAMS
    // TODO: double kp =
    // TODO: double kd =
    // TODO: double ki =
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0; // Running sum of errors
    double derivative = 0.0;
    double dt_1 = 0.0;

    std::vector<float> integral_Vector;
    std::vector<float> derivative_vector;
    int integral_counter = 0;
    int derivative_counter = 0;

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    std::string sim_car = "/odom"; // Physical car

    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr acker_Publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_Subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_Subscription_;

    double get_range(const std::vector<float> range_data, double angle)
    {
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

        // First ensure that the angle is within our min and max
        if (angle > 270 || angle < -270)
        {
            // throw std::out_of_range("Value cannot be less than the minimum
            //                         or greater than the max.")

            return -1; // Maybe throw range exception...
        }

        // Find length of the array
        int arr_length = range_data.size();

        // Recall our angle min and max is -270% and 270%
        double max_angle = 270;

        // Find how much degrees we should move
        // from max or min to get to target angle
        if (angle < 0)
        {
            // Is negative
            max_angle *= -1; // Negate the angle
            arr_length = 0;  // We will use that new angle for negative values
        }
        return range_data[angle];
    }

    void pid_control()
    {
        ackermann_msgs::msg::AckermannDriveStamped drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        double p = this->get_parameter("p").as_double();
        double i = this->get_parameter("i").as_double();
        double d = this->get_parameter("d").as_double();
        double steering_angle = 0.0;

            /*
            Based on the calculated error, publish vehicle control
            Args:
                error: calculated error
                velocity: desired velocity
            Returns:
                None
            */
            double angle = 0.0;
        // if speed is 0, don't calculate values
        if (speed == 0)
        {
            continue;
        }
        // TODO: Use kp, ki & kd to implement a PID controller
        double kp = porportional_Component();
        double ki = integral_Component();
        double kd = error / prev_error;

        // Derivitive component
        prev_error = error;

        // double kd = Derivative_Component();

        steering_angle = (p * kp) + (i * ki) + (d * kd);

        // TODO: fill in drive message and publish
        // If the steering angle is between 0 degrees and 10 degrees, the car should drive at 1.5 meters per second.
        // If the steering angle is between 10 degrees and 20 degrees, the speed should be 1.0 meters per second.
        // Otherwise, the speed should be 0.5 meters per second.
        if (steering_angle > -0.0001 && steering_angle < 10.0001){
            acker_message.drive.speed = 1.5;

        // If less than 20 and greater than 10
        } else if (steering_angle > 9.999 && steering_angle < 20.001) {
            acker_message.drive.speed = 1.0;
        
        // Any other steering angle, speed is 0.5
        } else {
            acker_message.drive.speed = 0.5;
        }

        acker_Publisher_ -> publish(acker_message);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        ackermann_msgs::msg::AckermannDriveStamped acker_message = ackermann_msgs::msg::AckermannDriveStamped();

        // Error is simply desired distance - actual
        //double derivative = (dt + dt_1) / 1

        //double velocity = 1.5; // TODO: calculate desired car velocity based on error
        // TODO: actuate the car with PID

        pid_control();
    }

    double integral_Component()
    {
        int max_range = 100; // Max # of values for integral is 100

        if (integral_counter < max_range)
        {
            // Add to array
            integral_Vector.push_back(error);

            // Counter is less than max, just add to it
            integral += error;
            integral_counter++;
        }
        else
        {
            // Counter is more than max, minus counter and a add new error
            integral -= integral_Vector[0];

            // Pop the first element from the vector
            integral_Vector.erase(integral_Vector.begin());

            // Push new value onto the array
            integral_Vector.push_back(error);

            // Add new error onto vector
            integral += error;

            // No need to increment counter anymore
        }

        return integral;
    }

    double porportional_Component()
    {
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

        double a = range_data[630];

        // 135% from the car
        double b = range_data[710];

        // DEBUG
        // RCLCPP_INFO(this -> get_logger(),
        //     "a: %f\tb: %f\n",
        //     get_range(range_data, 90),
        //     get_range(range_data, 45)
        // );

        double lookahead = 2;

        // We set our degrees for 'a' and 'b' to be 45%
        double alpha = (a * cos(45) - b) / (a * sin(45));
        alpha = atan(alpha);
        ///               (a * cos(45) - b)
        /// alpha = atan   ____________
        ///                 (a * sin(45)

        double dt = b * cos(alpha);

        // Future distance
        this->dt_1 = distance + lookahead * sin(alpha);
        this->error = 1 - dt;

        return this->error;
    }

    double derivativeFunc(double error)
    {

        int max_range = 100; // Max # of values for integral is 100

        if (derivative_counter < max_range)
        {

            derivative_vector.push_back(error);
            // derivative of one point
            if (derivative_vector.size() == 1)
                return 0.0;

            // slope of most current two points ONLY
            current_derivative = (derivative_vector.back() - derivative_vector[derivative.size() - 2])

                // if there is only two points, return the current derivative
            if (derivative_vector.size() == 2){
                derivaitve = current_derivative;
                return derivative;
            }

            // if there is more than two points, average derivatives.
            derivative = (derivative + current_derivative) / 2

            derivative_counter++;
            return derivative;
        }
        else
        {
            // Counter is more than max, minus counter and a add new error
            derivative -= (derivative_vector[1] - derivative_vector[0]);

            // Pop the first element from the vector
            derivative_vector.erase(derivative_vector.begin());

            // Push new value onto the array
            derivative_vector.push_back(error);

            // Add new error onto vector
            integral += error;

            // No need to increment counter anymore
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        // RCLCPP_INFO(this -> get_logger(), "Odom Reading X: %f\tY: %f\tZ: %f.",
        // msg -> twist.twist.linear.x,
        // msg -> twist.twist.linear.y,
        // msg -> twist.twist.linear.z);
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}