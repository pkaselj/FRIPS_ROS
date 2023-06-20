#include <cstdio>
#include <functional>
#include <cmath>
#include "rclcpp\rclcpp.hpp"
#include "rover_messages\msg\rover_heading.hpp"
#include "rover_messages\msg\command.hpp"

#ifndef M_PI
  #define M_PI 3.141f
#endif

using std::placeholders::_1;

using HeadingMsg = rover_messages::msg::RoverHeading;
using CommandMsg = rover_messages::msg::Command;

class RoverInverseKinematicsNode : public rclcpp::Node
{
public:
  RoverInverseKinematicsNode();

private:
  rclcpp::Subscription<HeadingMsg>::SharedPtr p_heading_subscriber_;
  rclcpp::Publisher<CommandMsg>::SharedPtr p_command_publisher_;

  void on_received_heading_(HeadingMsg::ConstSharedPtr p_msg);
  void do_send_command_(const CommandMsg& msg);

  // Distance from center of rover to center of wheel in meters
  float Lw_m_;
  // Wheel radius in meters
  float Rw_m_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin( std::make_shared<RoverInverseKinematicsNode>() );
  rclcpp::shutdown();
  
  return 0;
}

RoverInverseKinematicsNode::RoverInverseKinematicsNode()
: rclcpp::Node("rover_inv_kinm_node")
{

  this->declare_parameter("wheel_to_body_center_distance_m", 0.0f);
  this->declare_parameter("wheel_radius_m", 0.0f);
  this->declare_parameter("heading_input_topic", "/heading_input_topic");
  this->declare_parameter("heading_input_qos", 10);
  this->declare_parameter("command_output_topic", "/command_output_topic");
  this->declare_parameter("command_output_qos", 10);


  RCLCPP_DEBUG(this->get_logger(), "TEST");

  Lw_m_ = this->get_parameter("wheel_to_body_center_distance_m").as_double();
  Rw_m_ = this->get_parameter("wheel_radius_m").as_double(); 

  RCLCPP_INFO(
    this->get_logger(),
    "Rover inverse kinematics parameters: Lw = %03.3f m, Rw = %03.3f m.",
    Lw_m_, Rw_m_
  );

  const std::string heading_input_topic = this->get_parameter("heading_input_topic").as_string();
  const int heading_input_qos = this->get_parameter("heading_input_qos").as_int();
  const std::string command_output_topic = this->get_parameter("command_output_topic").as_string();
  const int command_output_qos = this->get_parameter("command_output_qos").as_int();

  p_command_publisher_ = this->create_publisher<CommandMsg>(command_output_topic, command_output_qos);

  auto heading_input_callback = std::bind(
    &RoverInverseKinematicsNode::on_received_heading_,
    this,
    _1
  );

  p_heading_subscriber_ = this->create_subscription<HeadingMsg>(
    heading_input_topic, heading_input_qos, heading_input_callback
  );
}

void RoverInverseKinematicsNode::on_received_heading_(HeadingMsg::ConstSharedPtr p_msg)
{
  // Rover's X axis points to motor 1, Y axis is 90 deg CCW
  // Ignore rotation for inverse kinematics (i.e. rover will never rotate)

  float vx = p_msg->speed_m_s * cosf(p_msg->relative_direction_rad);
  float vy = p_msg->speed_m_s * sinf(p_msg->relative_direction_rad);

  const float sqrt_3_over_2 = sqrtf(3.0f)/2.0f;

  CommandMsg command_msg;

  command_msg.motor_1_rps = 1.0f/Rw_m_ * (                    0 * vx + 1.0f * vy);
  command_msg.motor_2_rps = 1.0f/Rw_m_ * (-1.0f * sqrt_3_over_2 * vx - 0.5f * vy);
  command_msg.motor_3_rps = 1.0f/Rw_m_ * (        sqrt_3_over_2 * vx - 0.5f * vy);

  command_msg.duration_ms = p_msg->durations_ms;

  RCLCPP_DEBUG(
    this->get_logger(),
    "Inverse kinematics gave speeds <φ1, φ2, φ3> = <%03.3f, %03.3f, %03.3f> RPS for <v, θ> = <%03.3f m/s, %03.3f deg> for %d ms",
    command_msg.motor_1_rps, command_msg.motor_2_rps, command_msg.motor_3_rps,
    p_msg->speed_m_s, (p_msg->relative_direction_rad / 2 * M_PI) * 360,
    command_msg.duration_ms
  );

  do_send_command_(command_msg);
}

void RoverInverseKinematicsNode::do_send_command_(const CommandMsg &msg)
{
  p_command_publisher_->publish(msg);
}
