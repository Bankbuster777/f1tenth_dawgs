#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class Talker: public rclcpp::Node
{
public:
    Talker(): Node("talker")
    {
        // ROS2 topic
        std::string ackermann_topic = "/drive";
        std::string ackermann_relay_topic = "/drive_relay";

        // ROS2 parameter
        this->declare_parameter<double>("v", 0.0);
        this->declare_parameter<double>("d", 0.0);

        // ROS2 publisher, subscriber
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ackermann_topic, 10);

        publish_ackermann_topic();

    }

private:

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;


    void publish_ackermann_topic()
    {
        ackermann_msgs::msg::AckermannDriveStamped ackermann_msgs;

        double speed = this->get_parameter("v").get_value<double>();
        double steering_angle = this->get_parameter("d").get_value<double>();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1300, "Param get v: %.2lf, d: %.2lf", speed, steering_angle);

        ackermann_msgs.drive.speed = speed;
        ackermann_msgs.drive.steering_angle = steering_angle;

        ackermann_publisher_->publish(ackermann_msgs);
    }

};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();
    return 0;
}