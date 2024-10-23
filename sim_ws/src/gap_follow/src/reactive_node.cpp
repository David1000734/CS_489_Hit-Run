#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <map>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node
{
    // Implement Reactive Follow Gap on the car
    // This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        // Initilize variables
        this->declare_parameter("mode", "sim");
        this->declare_parameter("bubble", 0.1);
        this->declare_parameter("disp" , 5);
        this->declare_parameter("speed", 0.0);
        this->declare_parameter("gap", 4);
        this->declare_parameter("dist", 5.0);
        this->declare_parameter("change", 0.5);
        std::string sim_car = "/odom"; // Physical Car

        if (this->get_parameter("mode").as_string() == "sim")
        {
            sim_car = "/ego_racecar/odom"; // Sim Car
        }
        // ros2 launch src/gap_follow/gap_follow/gap_follow_launch.py speed:=1.0 disp:=10 dist:=1.5 gap:=20 change:=2.5 bubble:=0.24 mode:=sim

        /// TODO: create ROS subscribers and publishers
        // Ackerman Publisher
        acker_Publisher_ =
            this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
                "/drive",
                10);

        // Laser scan Subscriber
        scan_Subscription_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(&ReactiveFollowGap::lidar_callback,
                          this,
                          std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "Currently listening to %s.", sim_car.c_str());
    }

private:
    const double PI = 3.1415926535;
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr acker_Publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_Subscription_;

    std::vector<float> preprocess_lidar(std::vector<float> ranges, float increment)
    {
        const double max_change = this->get_parameter("change").as_double(); // biggest change we are looking for when finding corners
        const double bubble = this->get_parameter("bubble").as_double();           // number of values we want to make 0 when we find a corner
        const double disp = this->get_parameter("disp").as_int();           // number of values we want to make 0 when we find a corner

        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)

        for (int i = 0; i < ranges.size() - 1; i++)
        {
            if (i < 180 || i > 900) {
                ranges[i] = 0.0;
            }

            // All infinity values are changed to 30.0
            if (std::isinf(ranges[i]))
            {
                ranges[i] = 30.0;
            }

            // if ranges are below a threshold, make them 0 as well
            // Ignore angles outside of less than or greater than 90%
            // to the car
            if (fabs(ranges[i] - ranges[i + 1]) > max_change)
            {
                // in this case, the corner is on the 'right', and the change is on the left.
                //  So we extend the values of i, towards the left (positive index on the scan)
                //  and skip past it
                if (ranges[i] < ranges[i + 1])
                {
                    // Start from i and move up to i + bubble
                    // Sign extend from whatever value i is
                    // Notice we have to ensure we do not move out side the array
                    for (int j = i; (j < i + disp && j < ranges.size()); j++)
                    {
                        // RCLCPP_INFO (
                        //     this -> get_logger(),
                        //     "Left Extended, Before: %f\tAfter: %f\n",
                        //     ranges[j], ranges[i]
                        // );

                        ranges[j] = ranges[i]; // Extend the corner
                    }
                    i += disp;
                }
                // in this case, the corner is on the 'left' and the change is on the right
                // so we extend the values of i, back down the angle scan. (negative index)
                // and overwrite those values
                else if (ranges[i] > ranges[i + 1])
                {
                    // Start from i - bubble and go up to j
                    // We will sign extend from whatever i's value is
                    // Notice we only have to ensure that i - bubble does not get a negative value
                    for (int j = i - disp; (j < i && j > -1); j++)
                    {
                        // RCLCPP_INFO (
                        //     this -> get_logger(),
                        //     "Right Extended, Before: %f\tAfter: %f\n",
                        //     ranges[j], ranges[i + 1]
                        // );

                        ranges[j] = ranges[i + 1]; // Extend the corner
                    }
                }
                else
                {
                }
            }
            else
            {
            }
        }

        double smallest_value = 1000;
            int smallest_index = (ranges.size() / 2);
            for (long unsigned int i = 180; i < ranges.size()-180; i++) {
                if (ranges[i] < smallest_value) {
                    smallest_value = ranges[i];
                    smallest_index = i;
                }
            }

            double half_rad_angle_to_bubble_radius = atan(bubble / ranges[smallest_index]);
            int half_num_of_indices = round(half_rad_angle_to_bubble_radius / increment);

            for (int i = smallest_index - half_num_of_indices; i < smallest_index + half_num_of_indices; i++){
                if (i >= 0 && i < ranges.size()){
                    ranges[i] = 0;
                }
                else continue;
            }
        //             RCLCPP_INFO(this -> get_logger(),
        //                 "\nPre-processed Array: "
        //             );
        // for (int i = 0; i < size; i++) {
        //     RCLCPP_INFO(this -> get_logger(),
        //         "%f",
        //         ranges[i]
        //     );
        // }
        //             RCLCPP_INFO(this -> get_logger(),
        //                 "Pre-processed End Array.\n"
        //             );

        return ranges;
    }

    /// @brief Funciton will calculate the target degrees to reach each degree
    /// Presumable able to handle different size arrays but untested.
    ///
    /// @param arr_size Size of the array it's searching for
    ///
    /// @param target_index The index we want to steer towards.
    ///
    /// @return The angle to steer to reach the target. In DEGREES.
    ///
    /// @note This function assumes that the angle from one index
    /// to the next is 2 degrees.
    double get_steering_angle(int arr_size, int target_index, float increment)
    {
        // Since we are working with a size that may change
        // We will need to find out what our min and max angles are.
        //
        // To do this, recall that we know every index represents 2%
        // Thus the size / 2 would get us the full angle pos and neg
        //
        // If we take that (size / 2) / 2, this would get us the
        // positive range of our angles. Thus our angle is +- that value
        double center = arr_size / 2; // Center of the angles

        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "target i: %i\n",
        //     target_index);

        float increment_angle = increment * 180 / PI;
        // To get to our target index, all we do is target - center
        // Assume we have a range of 100, target is 80
        // Our center is 50, to get from 50 to 80, we need to
        //     move 30 index to the right. This translates to 60 degrees
        // Assume center is 50 and target is 40
        //     We need to move 20 idx to the left. Which is -20 degrees

        return (center - target_index) * increment_angle;
    }

    int find_best_point(std::map<int, double> gap)
    {
        // Between the end and begining / 2
        int const center =
            (gap.rbegin()->first + gap.begin()->first) / 2;
        int index = center;

        // Start_i & end_i are start and end indicies of max-gap range, respectively

        // Pick the best point by finding the furthest one
        std::map<int, double>::iterator itr;
        for (itr = gap.begin(); itr != gap.end(); itr++)
        {
            // Find the largest value AND the closest
            // one to the center
            // TODO: Depending on distance, we may want to target slighly off center
            // Ex. If we are close, and aiming for a left turn,
            // pick a point that is a little more left of the center
            //
            // If we are far, and aiming for a right turn,
            // pick a point that is closer to the center.
            // And vise versa
            if ((itr->second >= gap[index]) &&
                abs(center - itr->first) <
                    abs(center - index))
            {
                // Take the index of the largest value
                index = itr->first;
            }
        }

        RCLCPP_INFO(
            this->get_logger(),
            "center: %i\tindex: %i\nbegin: %i\tend: %i",
            center, index,
            abs(gap.begin()->first), abs(gap.rbegin()->first));

        return index;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        double distance_threshold = this->get_parameter("dist").as_double();
        int gap_size = this->get_parameter("gap").as_int();
        double speed = this->get_parameter("speed").as_double();

        std::vector<float> range_data = this->preprocess_lidar(scan_msg->ranges, scan_msg -> angle_increment);
        std::map<int, double> largest_gap; // Global Scope
        std::map<int, double> temp_gap;    // Local Scope
        double steering_angle = 0.0;
        double largest_average = 0.0; // Average depth of largest gap.

        // Just to save a variable slot, will be using
        // steering_angle as running sum for now
        for (long unsigned int i = 0; i < range_data.size(); i++)
        {
            // Check the curent point, if it meets our requirements, do stuff
            if (range_data[i] > distance_threshold)
            {
                // Store current gap into temp
                temp_gap[i] = range_data[i];

                // Increment running sum
                // steering_angle += range_data[i];

                // Check if gap is larger than minimum requirements
            }
            else if (((int)temp_gap.size()) > gap_size - 1)
            {
                // Get the average
                // steering_angle /= temp_gap.size();

                // Is the current depth deeper than largest
                if ((int)temp_gap.size() > (int)largest_gap.size())
                {
                    // Clear before adding our new gap
                    largest_gap = temp_gap;

                // RCLCPP_INFO(
                //     this->get_logger(),
                //     "Gap %i\n",
                //     i
                // );

                // for (auto it = temp_gap.begin(); it != temp_gap.end(); ++it)
                //     {
                //         RCLCPP_INFO(
                //             this -> get_logger(),
                //             "Found a gap: %i",
                //             it -> first
                //         );
                //     }
                }

                // Reset average and clear running sum
                largest_average = steering_angle;
                steering_angle = 0;
                temp_gap.clear();
            }
            else
            {
                // Does not meet distance and size requirements
                temp_gap.clear();
            }
        }
        // Steering_angle goes back to it's original purpose

        // If the whole scan is a gap, take that gap.
        if (temp_gap.size() > gap_size - 1 && temp_gap.size() > largest_gap.size())
        {
            largest_gap = temp_gap;
        }

        if (largest_gap.size() == 0)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "ERROR NO GAP\n\n\n\n\n\nERROR NO GAP"
            );

            // steering_angle = get_steering_angle(scan_msg->ranges.size(), 540, scan_msg->angle_increment);
            // double steering_rad = degree_to_radian(steering_angle);
            // publish_ackerman(1.0, steering_rad);
            return;
        }

        int target_idx = this->find_best_point(largest_gap);

        steering_angle = get_steering_angle(
            scan_msg->ranges.size(),
            target_idx,
            scan_msg->angle_increment);

        steering_angle *= -1; // Turn away from walls.

        // Angle clamping
        if (steering_angle > 20.0)
        {
            steering_angle = 20.0;
        }
        if (steering_angle < -20.0)
        {
            steering_angle = -20.0;
        }

        // If the steering angle is between 0 degrees and 10 degrees, the car should drive at 1.5 meters per second.
        // If the steering angle is between 10 degrees and 20 degrees, the speed should be 1.0 meters per second.
        // Otherwise, the speed should be 0.5 meters per second.
        if (steering_angle >= -4.0000 && steering_angle <= 4.0000)
        {
            // Speed will be specified by the launch parameters

            // If less than 20 and greater than 10
        }
        else if ((steering_angle > 4.0000 && steering_angle <= 10.0000) ||
                 (steering_angle < -4.0000 && steering_angle >= -10.0000))
        {
            // Speed is 75% of user specified speed.
            speed += 2;
            speed /= 2;

            // If less than 20 and greater than 10
        }
        else if ((steering_angle > 10.0000 && steering_angle < 20.0001) ||
                 (steering_angle < -10.0000 && steering_angle > -20.0001))
        {
            speed += 1;
            speed /= 2;
            // Any other steering angle, speed is 0.5
        }
        else
        {
            speed += 1;
            speed /= 2;
            // speed = (speed+1)/2
        }

        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        /// TODO:
        // Find closest point to LiDAR

        // Eliminate all points inside 'bubble' (set them to zero)

        // Find max length gap

        // Find the best point in the gap
        // rclcpp::shutdown();

        // Publish Drive message
        publish_ackerman(speed, degree_to_radian(steering_angle));
    }

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

    double degree_to_radian(double degrees)
    {
        // Helper function, change degrees to radian.
        return (degrees * (PI / 180));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}