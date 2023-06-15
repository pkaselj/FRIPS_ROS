#if defined(WIN32)
  #include <SDKDDKVer.h>
#endif

#include <cstdio>
#include <memory>
#include <sstream>
#include <functional>
#include "asio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rover_interface_messages/msg/byte_buffer.hpp"
#include "rover_interface_messages/msg/stxetx_frame.hpp"
#include "rover_interface/fsm.hpp"
#include "rover_interface/stxetx_protocol.h"

using std::placeholders::_1;
using std::placeholders::_2;

#define RECEIVE_BUFFER_SIZE 1024
#define FRAME_BUFFER_SIZE 1024
#define TRANSMIT_BUFFER_SIZE 1024
#define PAYLOAD_BUFFER_SIZE 1024

static volatile bool g_io_context_ready = false;

using FrameMsg = rover_interface_messages::msg::StxetxFrame;
using ByteArrayMsg = rover_interface_messages::msg::ByteBuffer;

static std::string byte_buffer_to_hex_string_(const uint8_t* p_buffer, size_t length);

class RoverInterfaceNode : public rclcpp::Node
{
public:
  RoverInterfaceNode(std::shared_ptr<asio::io_context> p_io);
  virtual ~RoverInterfaceNode();

private:
  std::shared_ptr<asio::io_context> p_io_;
  asio::serial_port serial_port_;
  rclcpp::Publisher<FrameMsg>::SharedPtr p_publisher_;
  rclcpp::Subscription<FrameMsg>::SharedPtr p_subscriber_;

  FiniteStateMachine fsm_;

  void on_timer_();
  void on_data_received_(const asio::error_code& ec, size_t bytes_transferred);
  void on_data_transmit_completed_(const asio::error_code& ec, size_t bytes_transferred);
  void process_received_bytes_(size_t bytes_to_process);
  void rearm_serial_port_();
  void initialize_fsm_();
  void reset_frame_buffer_();
  void reset_transmit_buffer_();
  void write_byte_to_processed_buffer_(uint8_t x);
  void on_full_frame_received_();
  void on_transmit_request_received_(FrameMsg::ConstSharedPtr p_msg);

  uint8_t p_receive_buffer_[RECEIVE_BUFFER_SIZE];
  uint8_t p_frame_buffer_[FRAME_BUFFER_SIZE];
  uint8_t p_transmit_buffer_[TRANSMIT_BUFFER_SIZE];
  uint8_t p_payload_buffer_[PAYLOAD_BUFFER_SIZE];

  size_t frames_byte_count_;
  size_t frames_payload_byte_count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto p_io_ctx = std::make_shared<asio::io_context>();
  
  // auto io_ctx_handle = p_io_ctx.get();
  
  auto asio_event_thread = std::thread([p_io_ctx](){
    std::cout << "ASIO Event Thread Starting..." << std::endl;
    auto io_work = asio::make_work_guard(p_io_ctx->get_executor());
    std::cout << "ASIO Event Thread Started." << std::endl;
    g_io_context_ready = true;
    p_io_ctx->run();
    std::cout << "ASIO Event Thread Finished." << std::endl;
  });

  auto node = std::make_shared<RoverInterfaceNode>(p_io_ctx);

  rclcpp::spin(node);
  rclcpp::shutdown();

  p_io_ctx->stop();
  asio_event_thread.join();

  return 0;
}

RoverInterfaceNode::RoverInterfaceNode(std::shared_ptr<asio::io_context> p_io)
: rclcpp::Node("rover_interface_node"),
  p_io_(p_io),
  fsm_(FiniteStateMachine::STATE_IDLE, nullptr, *this),
  frames_byte_count_(0),
  frames_payload_byte_count_(0),
  serial_port_(*p_io)
{
  const std::string port_name = "COM3";
  const int baud_rate = 9600;
  const std::string publisher_name = "/rover_receiver";
  const std::string subscriber_name = "/rover_transmitter";
  const int publisher_qos = 10;
  const int subscriber_qos = 10;

  RCLCPP_DEBUG( this->get_logger(), "Creating publisher at '%s'...", publisher_name.c_str());
  p_publisher_ = this->create_publisher<FrameMsg>(publisher_name, publisher_qos);
  RCLCPP_DEBUG( this->get_logger(), "Publisher created...");

  RCLCPP_DEBUG( this->get_logger(), "Creating subscriber at '%s'...", subscriber_name.c_str());
  auto callback_publisher = std::bind(&RoverInterfaceNode::on_transmit_request_received_, this, _1);
  p_subscriber_ = this->create_subscription<FrameMsg>(subscriber_name, subscriber_qos, callback_publisher);
  RCLCPP_DEBUG( this->get_logger(), "Subscriber created...");

  RCLCPP_DEBUG( this->get_logger(), "Initializing FSM...");
  initialize_fsm_();
  RCLCPP_DEBUG( this->get_logger(), "FSM initialized.");

  RCLCPP_DEBUG( this->get_logger(), "Initializing serial port...");
  RCLCPP_DEBUG( this->get_logger(), "Trying to open serial port...");
  asio::error_code ec;
  serial_port_.open(port_name, ec);

  if (ec)
  {
    RCLCPP_FATAL(
      this->get_logger(),
      "Could not open '%s' with Baud Rate '%d'",
      port_name,
      baud_rate
    );
    exit(-1);
  }

  serial_port_.cancel();

  RCLCPP_DEBUG( this->get_logger(), "Setting serial port settings...");
  serial_port_.set_option(asio::serial_port_base::baud_rate(baud_rate));
  serial_port_.set_option(asio::serial_port_base::character_size(8));
  serial_port_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
  serial_port_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
  RCLCPP_DEBUG( this->get_logger(), "Serial port settings set.");


  RCLCPP_INFO(
    this->get_logger(),
    "Port '%s' opened with Baud Rate '%d'",
    port_name,
    baud_rate
  );

  RCLCPP_DEBUG( this->get_logger(), "Waiting for ASIO io_context to be ready...");
  while (!g_io_context_ready)
  {
    RCLCPP_DEBUG( this->get_logger(), "ASIO io_context not running, waiting...");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  // Give some time to io_context.run()
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  RCLCPP_DEBUG(this->get_logger(), "ASIO io_context running.");
  rearm_serial_port_();
  RCLCPP_DEBUG( this->get_logger(), "Callbacks armed.");
}

RoverInterfaceNode::~RoverInterfaceNode()
{
  RCLCPP_INFO(this->get_logger(), "Closing serial port...");
  serial_port_.cancel();
  serial_port_.close();
  RCLCPP_INFO(this->get_logger(), "Serial port closed.");
}

void RoverInterfaceNode::on_data_received_(const asio::error_code &ec, size_t bytes_transferred)
{
  if (!ec)
  {
    RCLCPP_DEBUG(
      this->get_logger(),
      "Received %d bytes.\n%s",
      bytes_transferred,
      byte_buffer_to_hex_string_(p_receive_buffer_, bytes_transferred).c_str()
    );

    const int received_data_upper_limit = RECEIVE_BUFFER_SIZE; 
    int bytes_to_save = (bytes_transferred > received_data_upper_limit)
        ? received_data_upper_limit
        : bytes_transferred;

    process_received_bytes_(bytes_to_save);

    std::fill(p_receive_buffer_, p_receive_buffer_ + bytes_to_save, 0);

    rearm_serial_port_();
  }
  else
  {
    RCLCPP_FATAL(this->get_logger(), "ASIO data reception error '%s'", ec.message().c_str());
  }
}

void RoverInterfaceNode::reset_transmit_buffer_()
{
  std::fill(p_transmit_buffer_, p_transmit_buffer_ + TRANSMIT_BUFFER_SIZE, 0);
  std::fill(p_payload_buffer_, p_payload_buffer_ + PAYLOAD_BUFFER_SIZE, 0);
}

void RoverInterfaceNode::on_data_transmit_completed_(const asio::error_code &ec, size_t bytes_transferred)
{
  reset_transmit_buffer_();

  if(ec)
  {
    RCLCPP_FATAL(
      this->get_logger(),
      "ASIO data write error '%s'",
      ec.message().c_str()
    );
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Successfully written %d bytes.", bytes_transferred);
}

void RoverInterfaceNode::process_received_bytes_(size_t bytes_to_process)
{
  const size_t space_left_in_buffer = (FRAME_BUFFER_SIZE - frames_byte_count_);
  const size_t n = (bytes_to_process >= space_left_in_buffer) ? space_left_in_buffer : bytes_to_process;
  
  // const size_t offset = frames_byte_count_;
  uint8_t* p_buffer_ = p_receive_buffer_;

  for (size_t i = 0; i < n; i++)
  {
      uint8_t byte_ = p_buffer_[i];
      FiniteStateMachine::FSM_Event event = FiniteStateMachine::EVT_NONE;

      if (stxetx_is_character_frame_start_delimiter(byte_))
      {
        event = FiniteStateMachine::EVT_RECEIVED_STX;
      }
      else if(stxetx_is_character_frame_end_delimiter(byte_))
      {
        event = FiniteStateMachine::EVT_RECEIVED_ETX;
      }
      else if(stxetx_is_character_escape(byte_))
      {
        event = FiniteStateMachine::EVT_RECEIVED_ESC;
      }
      else
      {
        event = FiniteStateMachine::EVT_RECEIVED_DATA;
      }

      fsm_.ProcessEvent(event, byte_);
  }
  
}


void RoverInterfaceNode::rearm_serial_port_()
{
  RCLCPP_DEBUG(this->get_logger(), "Rearming serial port");
  uint8_t* p_buffer_ = p_receive_buffer_;
  auto receive_buffer = asio::buffer(p_buffer_, RECEIVE_BUFFER_SIZE);
  auto receive_callback = std::bind(&RoverInterfaceNode::on_data_received_, this, _1, _2);
  serial_port_.async_read_some(receive_buffer, receive_callback);
}

void RoverInterfaceNode::initialize_fsm_()
{
  // Finite State Machine for reading a full frame
  
  auto callback_error = [this](uint8_t x){
    RCLCPP_DEBUG(this->get_logger(), "callback_error '0x%02X'", x);
    RCLCPP_WARN(this->get_logger(), "Invalid character 0x'%02X' received", x);
    this->reset_frame_buffer_();
  };

  auto callback_frame_finished = [this](uint8_t x /* ETX */){
    RCLCPP_DEBUG(this->get_logger(), "callback_frame_finished '0x%02X'", x);
    this->write_byte_to_processed_buffer_(x);
    RCLCPP_DEBUG(this->get_logger(), "Full frame received.");
    this->on_full_frame_received_();
  };

  auto callback_frame_started = [this](uint8_t x /* STX */){
    RCLCPP_DEBUG(this->get_logger(), "callback_frame_started '0x%02X'", x);
    RCLCPP_DEBUG(this->get_logger(), "Started receiving frame.");
    this->write_byte_to_processed_buffer_(x);
  };

  auto callback_byte_escaped = [this](uint8_t x /* ASCII_ESCAPE */){
    RCLCPP_DEBUG(this->get_logger(), "callback_byte_escaped '0x%02X'", x);
    // This is not STXETX decoding, write all escape charactets,
    // STXETX decoding library will take care of other
    this->write_byte_to_processed_buffer_(x);
  };

  auto callback_write_data_byte = [this](uint8_t x){
      RCLCPP_DEBUG(this->get_logger(), "callback_write_data_byte '0x%02X'", x);
      this->write_byte_to_processed_buffer_(x);
  };

  auto callback_do_void = [this](uint8_t){
    RCLCPP_DEBUG(this->get_logger(), "callback_do_void");
    ; // DO NOTHING
  };

  using State = FiniteStateMachine::FSM_State;
  using Event = FiniteStateMachine::FSM_Event;

  // STATE_IDLE
  fsm_.AddTransition(State::STATE_IDLE, Event::EVT_RECEIVED_STX,  State::STATE_FRAME_STARTED, callback_frame_started);
  fsm_.AddTransition(State::STATE_IDLE, Event::EVT_RECEIVED_ETX,  State::STATE_IDLE,          callback_error);
  fsm_.AddTransition(State::STATE_IDLE, Event::EVT_RECEIVED_DATA, State::STATE_IDLE,          callback_error);
  fsm_.AddTransition(State::STATE_IDLE, Event::EVT_RECEIVED_ESC,  State::STATE_IDLE,          callback_error);

  // STATE_FRAME_STARTED
  fsm_.AddTransition(State::STATE_FRAME_STARTED, Event::EVT_RECEIVED_STX,  State::STATE_IDLE,           callback_error);
  fsm_.AddTransition(State::STATE_FRAME_STARTED, Event::EVT_RECEIVED_ETX,  State::STATE_IDLE,           callback_frame_finished);
  fsm_.AddTransition(State::STATE_FRAME_STARTED, Event::EVT_RECEIVED_DATA, State::STATE_FRAME_STARTED,  callback_write_data_byte);
  fsm_.AddTransition(State::STATE_FRAME_STARTED, Event::EVT_RECEIVED_ESC,  State::STATE_BYTE_ESCAPED,   callback_write_data_byte);

  // STATE_BYTE_ESCAPED
  fsm_.AddTransition(State::STATE_BYTE_ESCAPED, Event::EVT_RECEIVED_STX,  State::STATE_FRAME_STARTED, callback_write_data_byte);
  fsm_.AddTransition(State::STATE_BYTE_ESCAPED, Event::EVT_RECEIVED_ETX,  State::STATE_FRAME_STARTED, callback_write_data_byte);
  fsm_.AddTransition(State::STATE_BYTE_ESCAPED, Event::EVT_RECEIVED_DATA, State::STATE_FRAME_STARTED, callback_write_data_byte);
  fsm_.AddTransition(State::STATE_BYTE_ESCAPED, Event::EVT_RECEIVED_ESC,  State::STATE_FRAME_STARTED, callback_write_data_byte);
}

void RoverInterfaceNode::reset_frame_buffer_()
{
  std::fill(p_frame_buffer_, p_frame_buffer_ + frames_byte_count_, 0);
  frames_byte_count_ = 0;
  std::fill(p_payload_buffer_, p_payload_buffer_ + frames_payload_byte_count_, 0);
  frames_payload_byte_count_ = 0;
}

void RoverInterfaceNode::write_byte_to_processed_buffer_(uint8_t x)
{
  if (frames_byte_count_ >= FRAME_BUFFER_SIZE)
  {
    RCLCPP_FATAL(
      this->get_logger(),
      "Cannot process anymore bytes. Processed %ld out of max %ld",
      frames_byte_count_,
      FRAME_BUFFER_SIZE
    );
    return;
  }

  uint8_t* p_buffer_ = p_frame_buffer_;

  p_buffer_[frames_byte_count_++] = x; 
}

void RoverInterfaceNode::on_full_frame_received_()
{
  
  stxetx_frame_t frame;
  (void)stxetx_init_empty_frame(&frame);

  uint8_t ec = stxetx_decode_n(
    p_frame_buffer_,
    &frame,
    FRAME_BUFFER_SIZE,
    p_payload_buffer_,
    PAYLOAD_BUFFER_SIZE
  );

  if (ec != STXETX_ERROR_NO_ERROR)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not decode received frame. Error code: %d",
      ec
    );
  }
  else
  {
    auto msg = FrameMsg();

    msg.msg_type = frame.msg_type;
    msg.flags = frame.flags;
    msg.checksum = frame.checksum;
    msg.length = frame.len_bytes;

    msg.payload = std::vector<uint8_t>(p_payload_buffer_, p_payload_buffer_ + frame.len_bytes);

    // TODO: Verify checksum
    p_publisher_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "Frame published!");
  }

  this->reset_frame_buffer_();
}

void RoverInterfaceNode::on_transmit_request_received_(FrameMsg::ConstSharedPtr p_msg)
{
  std::fill(p_transmit_buffer_, p_transmit_buffer_ + TRANSMIT_BUFFER_SIZE, 0);

  stxetx_frame_t frame;
  (void)stxetx_init_empty_frame(&frame);

  frame.msg_type = p_msg->msg_type;
  frame.flags = p_msg->flags;
  frame.checksum = p_msg->checksum;
  frame.len_bytes = p_msg->length;

  // ! WARNING: Discarding `const` qualifier
  frame.p_payload = (uint8_t*)p_msg->payload.data();

  uint8_t actual_msg_size = 0;

  uint8_t ec = stxetx_encode_n(
    p_transmit_buffer_,
    frame,
    TRANSMIT_BUFFER_SIZE,
    &actual_msg_size
  );

  if (ec != STXETX_ERROR_NO_ERROR)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to encode stxetx frame. Error code %d.",
      ec
    );

    return;
  }

  RCLCPP_DEBUG(
    this->get_logger(),
    "Transmitting message\n%s",
    byte_buffer_to_hex_string_(p_transmit_buffer_, actual_msg_size).c_str()
  );

  auto callback_write_complete = std::bind(&RoverInterfaceNode::on_data_transmit_completed_, this, _1, _2);
  serial_port_.async_write_some(
    asio::buffer(p_transmit_buffer_, actual_msg_size),
    callback_write_complete
  );

}

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
