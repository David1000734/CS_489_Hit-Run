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
        this->declare_parameter("bubble", 0);
        this->declare_parameter("speed", 0.0);
        this->declare_parameter("gap", 4);
        this->declare_parameter("dist", 5.0);
        std::string sim_car = "/odom"; // Physical Car

        if (this->get_parameter("mode").as_string() == "sim")
        {
            sim_car = "/ego_racecar/odom"; // Sim Car
        }

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

    std::vector<float> preprocess_lidar(std::vector<float> ranges)
    {
        // can take out comments later, just wanted y'all to understand my logic!
        // only question I have is if we want to set each value to the mean over some window?
        // and why does it want us rejecting high values in the comment below that was given to us? doesn't make sense since we want high values for gap follow

        double max_change = 0.5;                                    // biggest change we are looking for when finding corners
        const int skipVal = this->get_parameter("bubble").as_int(); // number of values we want to make 0 when we find a corner
        const double low_threshold = 1.0;                           // we don't care if values are lower than a certain threshold, get rid of them

        // Find what angle ranges is our array in. Ex. -270% to 270%
        double range_angle = ranges.size() / 4;
        int target_angle = 90;    // Min AND Max for the new array
        int difference_angle = 0; // Difference from our target

        if (range_angle > target_angle)
        {
            // Find the difference to ignore
            difference_angle = target_angle - target_angle;
        }

        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)

        // look for difference above our max_change threshold
        for (int i = 0; i < ranges.size() - 1; i++)
        {
            if (std::isinf(ranges[i]))
            {
                ranges[i] = 30.0;
            }

            // if ranges are below a threshold, make them 0 as well
            // Ignore angles outside of less than or greater than 90%
            // to the car
            if (ranges[i] <= low_threshold ||
                i < difference_angle ||
                i > ranges.size() - difference_angle)
            {
                ranges[i] = 0.0;
            }
            else if (abs(ranges[i] - ranges[i + 1]) > max_change)
            {
                int j = 1;
                // make next 5 values 0

                // in this case, the corner is on the 'right', and the change is on the left.
                //  So we extend the values of i, towards the left (positive index on the scan)
                //  and skip past it
                if (ranges[i] < ranges[i + 1])
                {
                    while (j <= skipVal && (i + j) < ranges.size())
                    {
                        ranges[i + j] = ranges[i]; // changed from 0.0
                        j++;
                    }
                    i += skipVal - 1;
                }
                // in this case, the corner is on the 'left' and the change is on the right
                // so we extend the values of i, back down the angle scan. (negative index)
                // and overwrite those values
                else if (ranges[i] > ranges[i + 1])
                {
                    while (j <= skipVal && (i + j) < ranges.size())
                    {
                        ranges[i - j] = ranges[i]; // changed from 0.0
                        j++;
                    }
                }
            }
            else
            {
            }
        }

        return ranges;
    }

    void find_max_gap(float *ranges, int *indice)
    {
        // Return the start index & end index of the max gap in free_space_ranges
        return;
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
        double center = arr_size / 2;      // Center of the angles
        double range_angle = arr_size / 4; // Dividing by 2 twice. Just divide by 4

        RCLCPP_INFO(
            this->get_logger(),
            "target i: %i\n",
            target_index);
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
            if ((itr->second > gap[index]) &&
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
        double distance_threshold = this -> get_parameter("dist").as_double();
        int gap_size = this -> get_parameter("gap").as_int();
        double speed = this -> get_parameter("speed").as_double();

        std::vector<float> range_data = this -> preprocess_lidar(scan_msg -> ranges);
        // std::vector<float> range_data = scan_msg->ranges;
        std::map<int, double> largest_gap; // Global Scope
        std::map<int, double> temp_gap;    // Local Scope
        double steering_angle = 0.0;

        for (long unsigned int i = 0; i < range_data.size(); i++)
        {
            // Check the curent point, if it meets our requirements, do stuff
            if (range_data[i] > distance_threshold)
            {
                // Store current gap into temp
                temp_gap[i] = range_data[i];

                // Check if gap is larger than current largest_gap
            }
            else if (((int)temp_gap.size()) > gap_size - 1)
            {
                // Temp gap meets the size and distance requirements

                // Is it larger than the current largest?
                if (temp_gap.size() > largest_gap.size())
                {
                    // Clear before adding our new gap
                    largest_gap.clear();
                    largest_gap = temp_gap;
                }
                // Gap is smaller or new largest determined, clear temp

                temp_gap.clear();
            }
            else
            {
                // Does not meet distance and size requirements
                temp_gap.clear();
            }
        }
            // largest gap
            /*
            // DEBUG
            for (auto index : largest_gap){
                RCLCPP_INFO(this -> get_logger(),
                "\nTemp-Idx: %i\tTemp-Value: %f.\n",
                index.first,
                index.second);
            }

            // DEBUG
            int temp = this -> find_best_point(largest_gap);
            RCLCPP_INFO(this -> get_logger(),
                "\nTarget: %i\tAngle: %f\n",
                temp,
                get_steering_angle(scan_msg -> ranges.size(), temp)
            );

            // DEBUG
            for (int i = -20; i < 40; i += 10) {
                RCLCPP_INFO(this -> get_logger(),
                    "\nTarget: %f\tAngle: %f\n",
                    i,
                    get_steering_angle(scan_msg -> ranges, i)
                );
            }
            */
            if (temp_gap.size() > gap_size - 1 && temp_gap.size() > largest_gap.size())
            {
                largest_gap = temp_gap;
            }
            if (largest_gap.size() == 0)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "ERROR NO GAP\n\n\n\n\n\nERROR NO GAP");
                steering_angle = get_steering_angle(scan_msg->ranges.size(), 540, scan_msg->angle_increment);
                double steering_rad = degree_to_radian(steering_angle);
                publish_ackerman(1.0, steering_rad);
                return;
            }

            int target_idx = this->find_best_point(largest_gap);

            int random = rand() % 100;

            

            steering_angle = get_steering_angle(
                scan_msg->ranges.size(),
                target_idx,
                scan_msg->angle_increment);

            steering_angle *= -1; // Turn away from walls.

            if (steering_angle > 20.0)
            {
                steering_angle = 20.0;
            }
            if (steering_angle < -20.0)
            {
                steering_angle = -20.0;
            }

            // steering_angle = get_steering_angle(
            //                     scan_msg -> ranges.size(),
            //                     find_best_point(largest_gap)
            // );

            // If the steering angle is between 0 degrees and 10 degrees, the car should drive at 1.5 meters per second.
            // If the steering angle is between 10 degrees and 20 degrees, the speed should be 1.0 meters per second.
            // Otherwise, the speed should be 0.5 meters per second.
            if (steering_angle > -10.0001 && steering_angle < 10.0001)
            {
                // speed = speed

                // If less than 20 and greater than 10
            }
            else if ((steering_angle > 9.999 && steering_angle < 20.0001) ||
                     (steering_angle > -10.0001 && steering_angle < -20.0001))
            {
                speed = 1.0;
                // Any other steering angle, speed is 0.5
            }
            else
            {
                speed = 0.5;
            }

            // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

            /// TODO:
            // Find closest point to LiDAR

            // Eliminate all points inside 'bubble' (set them to zero)

            // Find max length gap

            // Find the best point in the gap

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