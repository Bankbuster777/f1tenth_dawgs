#include "rclcpp/rclcpp.hpp"
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class Relay: public rclcpp::Node
{
public:
    Relay(): Node("relay")
    {
        // ROS2 parameter

        std::string drive_topic = "/drive";
        std::string drive_relay_topic = "/drive_relay";

        // ROS2 publisher, subscriber
        relay_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        ackermann_subscriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(drive_relay_topic, 10, std::bind(&Relay::ackermann_callback, this, std::placeholders::_1));

    }

private:

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr relay_publisher_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_subscriber_;

    void ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr ackermann_msg)
    {
        ackermann_msgs::msg::AckermannDriveStamped drive_relay_msg;

        double speed_new = ackermann_msg->drive.speed * 3;
        double steering_angle_new = ackermann_msg->drive.steering_angle * 3;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1300, "Relay msg v: %.2lf, d: %.2lf", speed_new, steering_angle_new);

        drive_relay_msg.drive.speed = speed_new;
        drive_relay_msg.drive.steering_angle = steering_angle_new;

        relay_publisher_->publish(drive_relay_msg);

    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Relay>());
    rclcpp::shutdown();
    return 0;
}