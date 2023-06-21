#include <cstdio>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rover_messages/msg/odometry.hpp"
#include "rover_messages/msg/rover_heading.hpp"

using std::placeholders::_1;

using OdometryMsg = rover_messages::msg::Odometry;
using RelativeHeadingMsg = rover_messages::msg::RoverHeading;

class RoverForwardKinematics : public rclcpp::Node
{
public:
  RoverForwardKinematics();

private:
  rclcpp::Subscription<OdometryMsg>::SharedPtr p_odometry_subscriber_;
  rclcpp::Publisher<RelativeHeadingMsg>::SharedPtr p_relative_heading_publisher_;

  void on_received_odometry_msg_(OdometryMsg::ConstSharedPtr p_msg);

  // Distance from wheel center to body center in meters 
  // float Lw_m_;
  // Wheel radius in meters
  float Rw_m_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin( std::make_shared<RoverForwardKinematics>() );
  rclcpp::shutdown();

  return 0;
}

RoverForwardKinematics::RoverForwardKinematics()
: rclcpp::Node("rover_fwd_kinm_node")
{
  this->declare_parameter("odometry_input_topic", "/odometry_input_topic");
  this->declare_parameter("odometry_input_qos", 10);
  this->declare_parameter("relative_heading_output_topic", "/relative_heading_output_topic");
  this->declare_parameter("relative_heading_output_qos", 10);
  // this->declare_parameter("wheel_to_body_center_distance_m", 0);
  this->declare_parameter("wheel_radius_m", 0.0f /* ! IMPORTANT HAS TO BE 0.0f because 0 crashes the node */ );

  auto odometry_input_topic = this->get_parameter("odometry_input_topic").as_string();
  auto odometry_input_qos = this->get_parameter("odometry_input_qos").as_int();
  auto relative_heading_output_topic = this->get_parameter("relative_heading_output_topic").as_string();
  auto relative_heading_output_qos = this->get_parameter("relative_heading_output_qos").as_int();

  // Lw_m_ = this->get_parameter("wheel_to_body_center_distance_m").as_double();
  Rw_m_ = this->get_parameter("wheel_radius_m").as_double();

  auto odometry_subscriber_callback = std::bind(
    &RoverForwardKinematics::on_received_odometry_msg_,
    this,
    _1
  );

  p_odometry_subscriber_ = this->create_subscription<OdometryMsg>(
    odometry_input_topic,
    odometry_input_qos,
    odometry_subscriber_callback
  );

  p_relative_heading_publisher_ = this->create_publisher<RelativeHeadingMsg>(
    relative_heading_output_topic,
    relative_heading_output_qos
  );

}

void RoverForwardKinematics::on_received_odometry_msg_(OdometryMsg::ConstSharedPtr p_msg)
{
  RCLCPP_DEBUG(
    this->get_logger(),
    "Received odometry message <Φ1, Φ2, Φ3> = <%03.3f, %03.3f, %03.3f> RPS",
    p_msg->motor_1_rps, 
    p_msg->motor_2_rps, 
    p_msg->motor_3_rps
  );

  auto heading_msg = RelativeHeadingMsg();

  const auto sqrt_3_over_3 = sqrtf(3)/3.0f;

  float v_x__m_s = sqrt_3_over_3 * Rw_m_ * (p_msg->motor_3_rps - p_msg->motor_2_rps);
  float v_y__m_s =  (Rw_m_ / 3.0f) * (2 * p_msg->motor_1_rps - p_msg->motor_2_rps - p_msg->motor_3_rps);

  heading_msg.speed_m_s = sqrt(v_x__m_s * v_x__m_s + v_y__m_s * v_y__m_s);
  heading_msg.relative_direction_rad = atan2f(v_y__m_s, v_x__m_s);

  RCLCPP_DEBUG(
    this->get_logger(),
    "Inferred heading: <v, Φ> = <%03.3f m/s, %03.3f DEG>",
    heading_msg.speed_m_s,
    (heading_msg.relative_direction_rad / 6.282) * 360
  );

  p_relative_heading_publisher_->publish(heading_msg);
}
