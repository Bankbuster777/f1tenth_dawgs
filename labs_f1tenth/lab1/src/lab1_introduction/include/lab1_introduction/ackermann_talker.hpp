#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

class AckermannTalker : public rclcpp::Node
{
public:
  explicit AckermannTalker();

private:
  // ROS parameter
  double left_steering_end_point_, right_steering_end_point_;
  double steering_gain_, steering_offset_;

  // ROS publisher, subscriber
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

  // ROS callback
  void joy_callback_(const sensor_msgs::msg::Joy::SharedPtr joy_msg);

  // ROS peripheral
  void calibration_report_(double steering_gain_old, double steering_gain_new);
};
