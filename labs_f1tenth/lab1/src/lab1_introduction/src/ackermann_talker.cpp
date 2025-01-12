// 1. publishes an AckermannDriveStamped
// 2. topic name driver(publish as fast as possible)
// 3.
#include "lab1_introduction/ackermann_talker.hpp"
#include <cmath>

AckermannTalker::AckermannTalker(): Node("ackermann_talker")
{
  // ROS parameter
  this->declare_parameter<double>("left_steering_end_point_", -1.0);
  this->declare_parameter<double>("right_steering_end_point_", 1.0);
  this->declare_parameter<double>("steering_gain_", 1.0);
  this->declare_parameter<double>("steering_offset_", 0.0);

  // ROS publisher, subscriber
  ackermann_publisher_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
  joy_subscriber_ = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&AckermannTalker::joy_callback_, this, std::placeholders::_1));
}

void AckermannTalker::joy_callback_(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;
  left_steering_end_point_ = this->get_parameter("left_steering_end_point_").as_double();
  right_steering_end_point_ = this->get_parameter("right_steering_end_point_").as_double();
  steering_gain_ = this->get_parameter("steering_gain_").as_double();
  steering_offset_ = this->get_parameter("steering_offset_").as_double();

  if(joy_msg->buttons[0] == 1)
  {
    double steering_gain_new = this->steering_gain_ * abs(joy_msg->axes[0]);

    calibration_report_(this->steering_gain_, steering_gain_new);
    this->set_parameter(rclcpp::Parameter("steering_gain_", steering_gain_new));
    ackermann_msg.drive.steering_angle = steering_gain_new;
  }
  else if(joy_msg->buttons[1] == 1)
  {
    double steering_gain_new = this->steering_gain_ * abs(joy_msg->axes[0]);

    calibration_report_(this->steering_gain_, steering_gain_new);
    this->set_parameter(rclcpp::Parameter("steering_gain_", steering_gain_new));
    ackermann_msg.drive.steering_angle = steering_gain_new;
  }
  else
  {
    ackermann_msg.drive.steering_angle = joy_msg->axes[0];
  }


  ackermann_publisher_->publish(ackermann_msg);

}

void AckermannTalker::calibration_report_(double steering_gain_old, double steering_gain_new)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "This is short report for EPA" << std::endl <<
                    "steering gain was: " << steering_gain_old << std::endl <<
                    "steering gain will be: " << steering_gain_new);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannTalker>());
  rclcpp::shutdown();

  return 0;
}



