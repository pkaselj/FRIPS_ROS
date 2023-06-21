#include <cstdio>
#include <functional>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "rover_interface_definition/stxetx_protocol.h"
#include "rover_messages/msg/odometry.hpp"
#include "rover_messages/msg/command.hpp"
#include "rover_messages/msg/stxetx_frame.hpp"
#include "rover_protocol_hub/little_endian_encoding.hpp"

using CommandMsg = rover_messages::msg::Command;
using FrameMsg = rover_messages::msg::StxetxFrame;
using OdometryMsg = rover_messages::msg::Odometry;

using std::placeholders::_1;


std::string byte_buffer_to_hex_string_(const uint8_t *p_buffer, size_t length)
{
  const size_t bytes_in_row = 8;

  std::stringstream ss;

  ss << std::hex << std::setfill('0');

  for (size_t i = 0; i < length; i++)
  {
    if (i % bytes_in_row == 0)
    {
      ss << std::endl;
    }
    ss << "0x" << std::setw(2) << (unsigned)p_buffer[i] << " ";
  }
  return ss.str();
}

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

  if (
    /* IF ALL MOTORS ARE COMMANDED TO 0 */
    (p_msg->motor_1_rps == 0
    && p_msg->motor_2_rps == 0
    && p_msg->motor_3_rps == 0)
    ||
    /* OR COMMAND DURATION IS SET TO 0 */
    (p_msg->duration_ms == 0)
  )
  {
    // Then, send STOP instead of COMMAND
    frame_msg.msg_type = MSG_TYPE_STOP;
  }
  else
  {
    // It is a normal command
    frame_msg.msg_type = MSG_TYPE_COMMAND;
    
    size_t payload_size = 3 * sizeof(float) + sizeof(uint32_t);

    // Fill command payload with zeros (so we can write using iterators)
    frame_msg.payload = std::vector<uint8_t>(payload_size, 0);
    frame_msg.length = payload_size;
    
    auto it_payload = frame_msg.payload.data();

    // Pack
    encode_little_endian((void*)(it_payload +  0), (const void*)&(p_msg->motor_1_rps), sizeof(float));
    encode_little_endian((void*)(it_payload +  4), (const void*)&(p_msg->motor_2_rps), sizeof(float));
    encode_little_endian((void*)(it_payload +  8), (const void*)&(p_msg->motor_3_rps), sizeof(float));
    encode_little_endian((void*)(it_payload + 12), (const void*)&(p_msg->duration_ms), sizeof(uint32_t));
  }
  
  transmit_msg_to_rover_(frame_msg);
}

void RoverProtocolHub::on_odometry_msg_from_rover_(FrameMsg::ConstSharedPtr p_msg)
{
  auto odometry_msg = OdometryMsg();

  const uint8_t* it_payload = (uint8_t*)(void*)(p_msg->payload.data());

  odometry_msg.motor_1_rps = decode_little_endian_float32((void*)(it_payload + 0));
  odometry_msg.motor_2_rps = decode_little_endian_float32((void*)(it_payload + 4));
  odometry_msg.motor_3_rps = decode_little_endian_float32((void*)(it_payload + 8));
  odometry_msg.time_delta_ms = decode_little_endian_uint32((void*)(it_payload + 12));

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
  case MSG_TYPE_ODOMETRY:
    on_odometry_msg_from_rover_(p_msg);
    break;

  case MSG_TYPE_FINISHED:
  default:
    on_unknown_message_received_from_rover_(p_msg);
    break;
  }
}

void RoverProtocolHub::on_unknown_message_received_from_rover_(FrameMsg::ConstSharedPtr p_msg)
{
  RCLCPP_WARN(
    this->get_logger(),
    "Received unknown message from rover with message type [%d]. Payload:\n%s",
    p_msg->msg_type,
    byte_buffer_to_hex_string_(p_msg->payload.data(), p_msg->length).c_str()
  );
}
