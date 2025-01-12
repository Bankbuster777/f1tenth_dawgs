#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
/// CHECK: include needed ROS msg type headers and libraries

#define _USR_MATH_DEFINES
#include <cmath>

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        lidarscan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));

    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidarscan_subscriber_;

    const int scan_indices = 1080;

    const float far_threshold = 8.0f;
    const float near_threshold = 0.1f;

    const float continuity_gap = 0.05f;

    const float vehicle_width = 0.3f;

    void preprocess_lidar(float* ranges)
    {
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        for(int i =0; i < scan_indices; i++)
        {
            if(!std::isfinite(ranges[i]))
            {
                ranges[i] = far_threshold;
                // preprocess for nan or inf
                // nan: ?? how is this happened?
                // inf: it's too far
            }
            else if(ranges[i] > far_threshold)
            {
                // Far enough to consider as gap.
                ranges[i] = far_threshold;
            }
            else if(ranges[i] < near_threshold)
            {
                // Too close to run into
                ranges[i] = 0.01f;
            }
        }

        return;
    }
    void find_discontinuity_chunks(float* ranges, int* indice)
    {
        int j = 1;
        for(int i = j; i < scan_indices; i++)
        {
            if(std::abs(ranges[i] - ranges[i-1]) > continuity_gap)
            {
                indice[j] = i;
                j++;
            }
        }
        indice[j] = scan_indices - 1;

        return;
    }

    void find_max_gap(float* ranges, int* indice)
    {
        // Return the start index & end index of the max gap in free_space_ranges
        float gap_arr[1080] = {0};
        float * gap_width = gap_arr; // or = new float[] ?
        int idx_arr[1080] = {0};
        int * chunk_idx = idx_arr;
        int j = 1;

        for(int i =1; i < scan_indices; i++)
        {
            if(std::abs(ranges[i] - ranges[i-1]) > continuity_gap)
            {
                chunk_idx[j] = i;
                j++;
            }
        }
        chunk_idx[j] = scan_indices - 1; // end of scan

        // wider gap version
        int wide_idx =0;
        for (int i = 0; i < j; i++)
        {
            gap_width[i] = M_PI/180 * 0.25 * (chunk_idx[i+1] - chunk_idx[i]) * ranges[chunk_idx[i]]; // average distance or shortest distance for gap
            if(gap_width[i]> gap_width[wide_idx])
            {
                wide_idx = i;
            }
        }

        indice[0] = chunk_idx[wide_idx];
        indice[1] = chunk_idx[wide_idx + 1];
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1300, "Wider gap: %.2lf between %d and %d", gap_width[wide_idx], indice[0], indice[1]);

        // TODO: deeper gap version

        // TODO: spacier gap version

        return;
    }

    void find_best_point(float* ranges, int* indice)
    {
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there

        // apply safety bubble (in incremental way)
        int safety_bubble_left, safety_bubble_right;
        safety_bubble_left = vehicle_width/2 * 4 * 180/M_PI/ranges[indice[0]-1];
        safety_bubble_right = vehicle_width/2 * 4 * 180/M_PI/ranges[indice[1]];

        int start_i, end_i, best_i;
        start_i = indice[0] + safety_bubble_left;
        end_i = indice[1] - safety_bubble_right;
        if(start_i > end_i)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 700, "Invalid gap: start_index = %d, end_index = %d ", start_i, end_i);
            // if multiple happens, then change gap version
        }
        else
        {
            best_i = (end_i + start_i)/2;
            indice[2] = best_i;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1700, "Best point: %d ", indice[2]);
        }

        return;
    }

    float speed_profile(float gap_width, float gap_depth)
    {
        // more wider, much faster

        // more deeper, much faster

        // or just lookup table
        return 3;
    }

    float steering_profile(int index)
    {
        float steering_angle = (index - scan_indices/2) * 270/scan_indices *  M_PI/180;
        return steering_angle;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

        /// TODO:
        // Find closest point to LiDAR
        // Eliminate all points inside 'bubble' (set them to zero)

        // Find max length gap
        float* ranges = const_cast<float*>(scan_msg->ranges.data());
        int ind_arr[1080] = {0};
        int * indice = ind_arr;
        // new int[2];
        preprocess_lidar(ranges);
        // Find the best point in the gap
        find_max_gap(ranges, indice);
        find_best_point(ranges, indice);

        float gap_width = ranges[indice[0]] * (indice[1]-indice[0]) * M_PI/180*270/1080;
        float gap_depth = ranges[indice[2]];

        // steering to indice[3]
        drive_msg.drive.steering_angle = steering_profile(indice[2]);
        drive_msg.drive.speed = speed_profile(gap_width, gap_depth);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 700, "Steer to: %.2lf ", drive_msg.drive.steering_angle);
        // Publish Drive message
        ackermann_publisher_->publish(drive_msg);
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}