#include <cstdio>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "rover_interface_definition/stxetx_protocol.h"
#include "rover_messages/msg/odometry.hpp"
#include "rover_messages/msg/command.hpp"
#include "rover_messages/msg/stxetx_frame.hpp"
#include "rover_protocol_hub/little_endian_decoding.h"

using CommandMsg = rover_messages::msg::Command;
using FrameMsg = rover_messages::msg::StxetxFrame;
using OdometryMsg = rover_messages::msg::Odometry;

using std::placeholders::_1;

class RoverProtocolHub : public rclcpp::Node
{
public:
  RoverProtocolHub();
private:
  rclcpp::Publisher<FrameMsg>::SharedPtr p_outgoing_message_publisher_;
  rclcpp::Subscription<FrameMsg>::SharedPtr p_incoming_message_subscriber_;

  rclcpp::Publisher<OdometryMsg>::SharedPtr p_odometry_from_rover_publisher_;
  rclcpp::Subscription<CommandMsg>::SharedPtr p_command_to_rover_listener_;

  void on_command_msg_to_rover_(CommandMsg::ConstSharedPtr p_msg);
  void on_odometry_msg_from_rover_(FrameMsg::ConstSharedPtr p_msg);
  
  void transmit_msg_to_rover_(FrameMsg msg);
  void on_message_received_from_rover_(FrameMsg::ConstSharedPtr p_msg);
  void on_unknown_message_received_from_rover_(FrameMsg::ConstSharedPtr p_msg);

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin( std::make_shared<RoverProtocolHub>() );
  rclcpp::shutdown();
  return 0;
}

RoverProtocolHub::RoverProtocolHub()
: rclcpp::Node("rover_protocol_hub_node")
{
  /* Subscriber for messages incoming from rover */
  this->declare_parameter("rover_msg_subscriber_topic", "/rover_msg_subscriber_topic");
  this->declare_parameter("rover_msg_subscriber_qos", 10);

  auto rover_msg_subscriber_topic = this->get_parameter("rover_msg_subscriber_topic").as_string();
  auto rover_msg_subscriber_qos = this->get_parameter("rover_msg_subscriber_qos").as_int();
  auto rover_msg_subscriber_callback = std::bind(
    &RoverProtocolHub::on_message_received_from_rover_,
    this,
    _1
  );

  p_incoming_message_subscriber_ = this->create_subscription<FrameMsg>(
    rover_msg_subscriber_topic,
    rover_msg_subscriber_qos,
    rover_msg_subscriber_callback
  );
  /* ------------------------------------ - ----------------------------------- */
  /* Publisher for messages outgoing for rover */
  this->declare_parameter("rover_msg_publisher_topic", "/rover_msg_publisher_topic");
  this->declare_parameter("rover_msg_publisher_qos", 10);

  auto rover_msg_publisher_topic = this->get_parameter("rover_msg_publisher_topic").as_string();
  auto rover_msg_publisher_qos = this->get_parameter("rover_msg_publisher_qos").as_int();

  p_outgoing_message_publisher_ = this->create_publisher<FrameMsg>(
    rover_msg_publisher_topic,
    rover_msg_publisher_qos
  );
  /* ------------------------------------ - ----------------------------------- */

  /* Publisher for ODOMETRY messages coming from rover */
  this->declare_parameter("odometry_publisher_topic", "/odometry_publisher_topic");
  this->declare_parameter("odometry_publisher_qos", 10);

  auto odometry_publisher_topic = this->get_parameter("odometry_publisher_topic").as_string();
  auto odometry_publisher_qos = this->get_parameter("odometry_publisher_qos").as_int();

  p_odometry_from_rover_publisher_ = this->create_publisher<OdometryMsg>(
    odometry_publisher_topic,
    odometry_publisher_qos
  );
  /* ------------------------------------ - ----------------------------------- */

  /* Listener for COMMAND messages outgoing to rover */
  this->declare_parameter("command_listener_topic", "/command_listener_topic");
  this->declare_parameter("command_listener_qos", 10);

  auto command_listener_topic = this->get_parameter("command_listener_topic").as_string();
  auto command_listener_qos = this->get_parameter("command_listener_qos").as_int();

  auto command_listener_callback = std::bind(
    &RoverProtocolHub::on_command_msg_to_rover_,
    this,
    _1
  );

  p_command_to_rover_listener_ = this->create_subscription<CommandMsg>(
    command_listener_topic,
    command_listener_qos,
    command_listener_callback
  );
  /* ------------------------------------ - ----------------------------------- */
}

void RoverProtocolHub::on_command_msg_to_rover_(CommandMsg::ConstSharedPtr p_msg)
{
  RCLCPP_DEBUG(
    this->get_logger(),
    "Sending command message to rover."
  );

  auto frame_msg = FrameMsg();
  frame_msg.msg_type = MSG_TYPE_GO_FORWARD; // TODO
  transmit_msg_to_rover_(frame_msg);
}

void RoverProtocolHub::on_odometry_msg_from_rover_(FrameMsg::ConstSharedPtr p_msg)
{
  auto odometry_msg = OdometryMsg();

  const uint8_t* it_payload = (uint8_t*)(void*)(p_msg->payload.data());

  odometry_msg.motor_1_rps = decode_little_endian_float32((void*)(it_payload + 0));
  odometry_msg.motor_2_rps = decode_little_endian_float32((void*)(it_payload + 4));
  odometry_msg.motor_3_rps = decode_little_endian_float32((void*)(it_payload + 8));

  p_odometry_from_rover_publisher_->publish(odometry_msg);
}

void RoverProtocolHub::transmit_msg_to_rover_(FrameMsg msg)
{
  RCLCPP_DEBUG(
    this->get_logger(),
    "Sending message to rover with message type [%d].",
    msg.msg_type
  );

  p_outgoing_message_publisher_->publish(msg);
}

void RoverProtocolHub::on_message_received_from_rover_(FrameMsg::ConstSharedPtr p_msg)
{
  RCLCPP_DEBUG(
    this->get_logger(),
    "Received message from rover with message type [%d].",
    p_msg->msg_type
  );

  switch (p_msg->msg_type)
  {
  case 255: /* TESTING */
  case MSG_TYPE_ODOMETRY:
    on_odometry_msg_from_rover_(p_msg);
    break;
  
  default:
    on_unknown_message_received_from_rover_(p_msg);
    break;
  }
}

void RoverProtocolHub::on_unknown_message_received_from_rover_(FrameMsg::ConstSharedPtr p_msg)
{
  RCLCPP_WARN(
    this->get_logger(),
    "Received unknown message from rover with message type [%d].",
    p_msg->msg_type
  );
}
