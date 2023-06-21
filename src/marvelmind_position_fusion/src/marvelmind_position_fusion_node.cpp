#include <cstdio>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "marvelmind_ros2_msgs/msg/hedge_position_angle.hpp"

using std::placeholders::_1;

using PositionMsg = marvelmind_ros2_msgs::msg::HedgePositionAngle;

class MarvelmindPositionFusionNode : public rclcpp::Node
{
public:
  MarvelmindPositionFusionNode();

private:
  rclcpp::Subscription<PositionMsg>::SharedPtr p_left_beacon_position_subscription_;
  rclcpp::Subscription<PositionMsg>::SharedPtr p_right_beacon_position_subscription_;
  rclcpp::Publisher<PositionMsg>::SharedPtr p_fused_position_publisher_;
  rclcpp::TimerBase::SharedPtr p_fused_position_broadcast_timer_;

  PositionMsg fused_position_;

  void on_left_position_receive_(PositionMsg::ConstSharedPtr p_msg);
  void on_right_position_receive_(PositionMsg::ConstSharedPtr p_msg);

  void on_position_broadcast_timer_trigger_();
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin( std::make_shared<MarvelmindPositionFusionNode>() );
  rclcpp::shutdown();
  return 0;
}

MarvelmindPositionFusionNode::MarvelmindPositionFusionNode()
: rclcpp::Node("marvelmind_position_fusion_node"),
  fused_position_()
{
  this->declare_parameter("position_measurement_left_topic", "/position_measurement_left");
  this->declare_parameter("position_measurement_left_qos", 10);
  this->declare_parameter("position_measurement_right_topic", "/position_measurement_right");
  this->declare_parameter("position_measurement_right_qos", 10);
  this->declare_parameter("fused_position_topic", "/fused_position");
  this->declare_parameter("fused_position_qos", 10);
  this->declare_parameter("fused_position_broadcast_rate_hz", 1.0f);

  auto position_measurement_left_topic = this->get_parameter("position_measurement_left_topic").as_string();
  auto position_measurement_left_qos = this->get_parameter("position_measurement_left_qos").as_int();
  auto position_measurement_left_callback = std::bind(
    &MarvelmindPositionFusionNode::on_left_position_receive_,
    this,
    _1
  );

  p_left_beacon_position_subscription_ = this->create_subscription<PositionMsg>(
    position_measurement_left_topic,
    position_measurement_left_qos,
    position_measurement_left_callback
  );



  auto position_measurement_right_topic = this->get_parameter("position_measurement_right_topic").as_string();
  auto position_measurement_right_qos = this->get_parameter("position_measurement_right_qos").as_int();
  auto position_measurement_right_callback = std::bind(
    &MarvelmindPositionFusionNode::on_right_position_receive_,
    this,
    _1
  );

  p_right_beacon_position_subscription_ = this->create_subscription<PositionMsg>(
    position_measurement_right_topic,
    position_measurement_right_qos,
    position_measurement_right_callback
  );



  // TODO: Maybe make it double
  auto fused_position_broadcast_rate_hz = this->get_parameter("fused_position_broadcast_rate_hz").as_double();
  auto fused_position_broadcast_timer_period = rclcpp::Rate(fused_position_broadcast_rate_hz).period();
  auto fused_position_timer_callback = std::bind(
    &MarvelmindPositionFusionNode::on_position_broadcast_timer_trigger_,
    this
  );

  p_fused_position_broadcast_timer_ = this->create_wall_timer(
    fused_position_broadcast_timer_period,
    fused_position_timer_callback
  );


  auto fused_position_topic = this->get_parameter("fused_position_topic").as_string();
  auto fused_position_qos = this->get_parameter("fused_position_qos").as_int();

  p_fused_position_publisher_ = this->create_publisher<PositionMsg>(
    fused_position_topic,
    fused_position_qos
  );

}

void MarvelmindPositionFusionNode::on_left_position_receive_(PositionMsg::ConstSharedPtr p_msg)
{
  RCLCPP_DEBUG(
    this->get_logger(),
    "Received position from left beacon: <x, y, heading> = <%03.3f m, %03.3f m, %03.3f DEG???>",
    p_msg->x_m, p_msg->y_m, p_msg->angle
  );
  // TODO: What to do with position
  // For now just save
  fused_position_ = *p_msg;
}

void MarvelmindPositionFusionNode::on_right_position_receive_(PositionMsg::ConstSharedPtr p_msg)
{
  RCLCPP_DEBUG(
    this->get_logger(),
    "Received position from right beacon: <x, y, heading> = <%03.3f m, %03.3f m, %03.3f DEG???>",
    p_msg->x_m, p_msg->y_m, p_msg->angle
  );
  // TODO: What to do with position
  // For now just save
  fused_position_ = *p_msg;
}

void MarvelmindPositionFusionNode::on_position_broadcast_timer_trigger_()
{
  RCLCPP_DEBUG(
    this->get_logger(),
    "Fused position timer triggered. Broadcasting fused position: "
    "<x, y, heading> = <%03.3f m, %03.3f m, %03.3f DEG???>",
    fused_position_.x_m, fused_position_.y_m, fused_position_.angle
  );

  p_fused_position_publisher_->publish(fused_position_);
}
