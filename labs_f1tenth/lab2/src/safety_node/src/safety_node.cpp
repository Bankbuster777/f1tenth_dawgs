#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
#include <cmath>

class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        // Parameters
        ackermann_topic = declare_parameter<std::string>("ackermann_topic", "/drive");
        laser_scan_topic = declare_parameter<std::string>("laser_scan_topic", "/scan");
        odometry_topic = declare_parameter<std::string>("odometry_topic", "/odom"); // dawgs: /odom & sim: /ego_racecar/odom


        this->declare_parameter<double>("time_threshold", time_threshold_init);
        this->declare_parameter<double>("target_speed", target_speed_init);

        // TODO: create ROS subscribers and publishers
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ackermann_topic, 10);
        laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_scan_topic, 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1));
        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic, 10, std::bind(&Safety::odom_callback, this, std::placeholders::_1));

        ackermann_msg_.header.frame_id = "odom";    // this's for driving
        // ackermann_msg_.header.frame_id = "ego_racecar/base_link";    // this's for simulation

        // For debugging purpose,
        timer_ = this->create_wall_timer(std::chrono::milliseconds(5000), std::bind(&Safety::timer_callback, this));
    }

private:
    // Parameters
    std::string ackermann_topic, laser_scan_topic, odometry_topic;

    double time_threshold_init = 2.5;
    double target_speed_init = 0.6;

    double target_speed, current_speed;
    double speed_lower_limit = 0.001;
    double scan_range = 30;

    // Constant Parameters
    const int scan_size = 1080;
    const int scan_angle = 270;
    const double D2R = M_PI/180;
    const double R2D = 180/M_PI;

    ackermann_msgs::msg::AckermannDriveStamped ackermann_msg_;
    // ackermann_msg_.header.frame_id = "ego_racecar/base_link";
    // error: ‘ackermann_msg’ does not name a type; did you mean ‘ackermann_msgs’?

    double time_threshold_;   // receiving time + processing time + propagation time

    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
    {
        /// TODO: update current speed
        current_speed = odom_msg->twist.twist.linear.x;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "In odom, C.Speed: %.2lf", current_speed);

        if(std::abs(current_speed) < speed_lower_limit)
        {
            current_speed = 0;
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        // Initialize parameters
        time_threshold_ = this->get_parameter("time_threshold").as_double();
        target_speed = this->get_parameter("target_speed").as_double();

        /// TODO: calculate TTC
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "In scan, T.Speed: %.2lf", target_speed);
        double TTC[scan_size], dist[scan_size], theta[scan_size];
        double iTTC = scan_range;

        for(int i=0; i< scan_size; i++){
            dist[i] = scan_msg->ranges[i];
            if(!std::isfinite(dist[i]))
            {
                dist[i] = scan_range;
            }


            theta[i] = D2R*(i*scan_angle/scan_size-135);
            TTC[i] = dist[i]/(current_speed*std::cos(theta[i]));

            if(i == 180){
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 270, "i: %d\ttheta: %.2lf\tcos: %.2lf", i, theta[i], std::cos(theta[i]));
            }else if(i == 540){
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 270, "i: %d\ttheta: %.2lf\tcos: %.2lf", i, theta[i], std::cos(theta[i]));
            }else if(i == 900){
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 270, "i: %d\ttheta: %.2lf\tcos: %.2lf", i, theta[i], std::cos(theta[i]));
            }


            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 310, "TTC: %.2lf,\tdist: %.2lf,\tspeed: %.2lf,\ttheta: %.2lf",
                                                                                TTC[i], dist[i], target_speed, theta[i]);

            if (TTC[i] > 0)
            {
                if(TTC[i] < iTTC)
                {
                    iTTC = TTC[i];
                    if(iTTC < time_threshold_)
                    {
                        RCLCPP_INFO(this->get_logger(), "i: %d\ttheta: %.1lf\tdist: %.1lf\tspeed: %.1lf\tiTTC:%.1lf", i, theta[i]*R2D, dist[i], current_speed, iTTC);
                    }
                }
            }
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "iTTC: %.2lf,\tTime_threshold: %.2lf", iTTC, time_threshold_);

        /// TODO: publish drive/brake message
        if(iTTC < time_threshold_){
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "iTTC :  %lf \n", iTTC);
            RCLCPP_INFO_STREAM(this->get_logger(), "Iminent to collide " << iTTC);
            this->set_parameter(rclcpp::Parameter("target_speed", 0.0));
            target_speed = 0.0;
        }

        ackermann_msg_.drive.speed = target_speed;
        ackermann_publisher_->publish(ackermann_msg_);
    }

    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "target speed changed to: %.1lf,\twas: %.1lf", target_speed_init, target_speed);
        this->set_parameter(rclcpp::Parameter("target_speed", target_speed_init));
    }

};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}