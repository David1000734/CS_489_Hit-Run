#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <map>
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
        this -> declare_parameter("gap", 4);
        this -> declare_parameter("dist", 5.0);
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
    const double PI = 3.1415926535;
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr acker_Publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_Subscription_;

    std::vector<float> preprocess_lidar(std::vector<float> ranges)
    {
        /*
        vector<double> temp = null;
        double chg = 0.0; // grab the first range
        double temp_chg = 0.0;
        int r_indx = 0;
        double max_change = 2.0

        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window

        for(int i = 0; i < ranges.size()-1; i++){
            temp_chg = abs(ranges[i] - ranges[i+1]);
            if(abs(ranges[i] - ranges[i + 1]) > max_change){
                // Difference between two values is greater
                // than specified difference.
                // Corrner was found.
                // change the next 5 values
                chg = temp_chg;
                r_indx = i;
            }
        }
        
        // keep this maybe for later
        for(int i = 0; i < 5 ; i++){

            results[r_indx] = 0;
            r_indx++;
        }
        
        // 2.Rejecting high values (eg. > 3m)
        */

        return ranges;
    }

    void find_max_gap(float* ranges, int* indice)
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
    double get_steering_angle(int arr_size, int target_index) {
        // Since we are working with a size that may change
        // We will need to find out what our min and max angles are.
        //
        // To do this, recall that we know every index represents 2%
        // Thus the size / 2 would get us the full angle pos and neg
        //
        // If we take that (size / 2) / 2, this would get us the
        // positive range of our angles. Thus our angle is +- that value
        double center = arr_size / 2;         // Center of the angles
        double range_angle = arr_size / 4;      // Dividing by 2 twice. Just divide by 4

        // To get to our target index, all we do is target - center
        // Assume we have a range of 100, target is 80
        // Our center is 50, to get from 50 to 80, we need to
        //     move 30 index to the right. This translates to 60 degrees
        // Assume center is 50 and target is 40
        //     We need to move 20 idx to the left. Which is -20 degrees

        return (center - target_index);
    }

    int find_best_point(std::map<int, double> gap)
    {
        int index = 0;          // Target index

        // Start_i & end_i are start and end indicies of max-gap range, respectively

        // Pick the best point by finding the furthest one
        std::map<int, double>::iterator itr;
        for (itr = gap.begin(); itr != gap.end(); itr++) {
            if (itr -> second > gap[index]) {
                // Take the index of the largest value
                index = itr -> first;
            }
        }

        // RCLCPP_INFO(
        //     this -> get_logger(),
        //     "\nBest point IDX: %f\t Value: %f.\n",
        //     largest,
        //     gap[largest])
        // ;

        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        return index;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        double distance_threshold = this -> get_parameter("dist").as_double();
        int gap_size = this -> get_parameter("gap").as_int();
        double speed = this -> get_parameter("speed").as_double();

        /*
        RCLCPP_INFO(
            this -> get_logger(), 
            "\nmode: %s\tBubble: %f\tSpeed: %f.\n",
            this -> get_parameter("mode").as_string().c_str(),
            this -> get_parameter("bubble").as_double(),
            this -> get_parameter("speed").as_double()
        );
        */

        std::vector<float> range_data = this -> preprocess_lidar(scan_msg -> ranges);
        std::map<int, double> largest_gap;      // Global Scope
        std::map<int, double> temp_gap;         // Local Scope
        double steering_angle = 0.0;

        for (int i = 0; i < range_data.size(); i++) {
            // Check the curent point, if it meets our requirements, do stuff
            if (range_data[i] > distance_threshold) {
                // Store current gap into temp
                temp_gap[i] = range_data[i];

            // Check if gap is larger than current largest_gap
            } else if (temp_gap.size() > gap_size - 1) {
                // Temp gap meets the size and distance requirements

                // Is it larger than the current largest?
                if (temp_gap.size() > largest_gap.size()){
                    largest_gap = temp_gap;
                }
                // Gap is smaller or new largest determined, clear temp

                temp_gap.clear();
            } else {
                // Does not meet distance and size requirements
                temp_gap.clear();
            }
        }

        //largest gap
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
        int target_idx = this -> find_best_point(largest_gap);
        steering_angle = get_steering_angle(
                        scan_msg -> ranges.size(),
                        target_idx
        );

        // steering_angle = get_steering_angle(
        //         scan_msg -> ranges.size(),
        //         find_best_point(largest_gap)
        // );
        steering_angle *= -1;       // Turn away from walls.
        
        // steering_angle = get_steering_angle(
        //                     scan_msg -> ranges.size(),
        //                     find_best_point(largest_gap)
        // );

        // If the steering angle is between 0 degrees and 10 degrees, the car should drive at 1.5 meters per second.
        // If the steering angle is between 10 degrees and 20 degrees, the speed should be 1.0 meters per second.
        // Otherwise, the speed should be 0.5 meters per second.
        if (steering_angle > -10.0001 && steering_angle < 10.0001){
            // speed = speed

        // If less than 20 and greater than 10
        } else if ((steering_angle > 9.999 && steering_angle < 20.0001) ||
                   (steering_angle > -10.0001 && steering_angle < -20.0001)) {
            speed = 1.0;
        
        // Any other steering angle, speed is 0.5
        } else {
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

    double degree_to_radian(double degrees) {
        // Helper function, change degrees to radian.
        return (degrees * (PI / 180));
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}