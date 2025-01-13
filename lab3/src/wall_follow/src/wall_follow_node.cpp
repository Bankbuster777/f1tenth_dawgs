#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

#define _USR_MATH_DEFINES
#include <cmath>

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers
        ackermann_topic = declare_parameter<std::string>("ackermann_topic", "/drive");
        // sim topic: /drive
        // dawgs topic: same
        laser_scan_topic = declare_parameter<std::string>("laser_scan_topic", "/scan");

        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ackermann_topic, 10);
        laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_scan_topic, 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));

        error_topic = declare_parameter<std::string>("error_topic", "/error");
        error_publisher_ = this->create_publisher<std_msgs::msg::Float64>(error_topic, 10);

        // PID Control parameters
        this->declare_parameter<double>("kp", kp_init_);
        this->declare_parameter<double>("kd", kd_init_);
        this->declare_parameter<double>("ki", ki_init_);

        this->declare_parameter<double>("dist_lookahead", 0.3);
        this->declare_parameter<double>("angle_a", angle_a_init_);
        this->declare_parameter<double>("angle_b", angle_b_init_);

        this->declare_parameter<double>("target_dist", target_dist_init_);
    }

private:
    // PID CONTROL PARAMS
    double kp, kd, ki;
    double kp_init_ = 3.0;
    double kd_init_ = 0.3;
    double ki_init_ = 0.0;

    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;
    double error_angle = 0.0;
    double target_angle = 0.0;

    double scan_length = 1080;
    double scan_angle = 270;

    const double R2D = 180/M_PI;
    const double D2R = M_PI/180;
    double angle_min = -2.3562;
    double angle_max = 2.3562;

    double dist_lookahead;
    double angle_a, angle_b;
    double angle_a_init_ = 40 * D2R;
    double angle_b_init_ = 90 * D2R;

    double target_dist;
    double target_dist_init_ = 1;

    // velocity profile
    double vel_high = 1.5;
    double vel_moderate = 1.2;
    double vel_low = 0.9;
    double vel_xlow = 0.3;

    // Topics
    std::string ackermann_topic, laser_scan_topic;

    std::string error_topic;

    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;

    // Error publisher
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_publisher_;

    double get_range(float* range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // TODO: implement
        // 1. modify angle in range of 0 ~ 270
        // no need ?

        // 2. change angle to index
        int index = (angle - angle_min) * 4 * R2D;
        if (index < 0 || index > scan_length)
        {
            // invalid index
            RCLCPP_ERROR(this->get_logger(), "invalid index : %d, angle: %.2lf", index, angle);
            return 0.0;
        }

        // 3. check NaNs and infs
        if(std::isfinite(range_data[index]))
        {
            return range_data[index];

        }

        return 0.0;
    }

    double get_error(float* range_data, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        // TODO:implement
        // Follow left wall
        double dist_a, dist_b, dist_D, error;

        angle_a = this->get_parameter("angle_a").get_value<double>();
        angle_b = this->get_parameter("angle_b").get_value<double>();

        dist_a = get_range(range_data, angle_a);
        dist_b = get_range(range_data, angle_b);

        double angle_alpha = std::atan2(dist_b - dist_a*std::cos(angle_b-angle_a), dist_a*std::sin(angle_b-angle_a));
        dist_D = dist_b * std::cos(angle_alpha) - dist_lookahead * std::sin(angle_alpha);

        error = dist_D - dist;
        auto error_msg = std_msgs::msg::Float64();
        error_msg.data = error;
        error_publisher_->publish(error_msg);
        error_angle = std::cos(angle_alpha);

        return error;
    }

    void pid_control(double error, double velocity)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        double angle = 0.0;
        // TODO: Use kp, ki & kd to implement a PID controller
        auto kp = this->get_parameter("kp").get_value<double>();
        auto ki = this->get_parameter("ki").get_value<double>();
        auto kd = this->get_parameter("kd").get_value<double>();

        // kd term is too big
        angle = kp * error + ki * integral + kd * (error - prev_error);

        prev_error = error;
        integral += error;

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish

        drive_msg.drive.speed = velocity;
        drive_msg.drive.steering_angle = angle;
        ackermann_publisher_->publish(drive_msg);
    }

    double speed_profiler(const double error, const double velocity_setpoint, const double error_angle)
    {
        return 0.0;
    }


    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        target_dist = this->get_parameter("target_dist").as_double();
        float* ranger = const_cast<float*>(scan_msg->ranges.data());
        error = get_error(ranger, target_dist);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),310, "Error: %.2lf", error);

        double velocity = vel_xlow;
        // TODO: calculate desired car velocity based on error
        // Make it as function. and more sophisticated
        if(error/target_dist < 0.25 && error/target_dist > - 0.25)
        {
            velocity = vel_low;
        }
        if(error/target_dist < 0.15 && error/target_dist > - 0.15)
        {
            velocity = vel_moderate;
        }
        if(error/target_dist < 0.05 && error/target_dist > - 0.05)
        {
            velocity = vel_high;
        }

        // TODO: actuate the car with PID
        pid_control(error, velocity);

    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}