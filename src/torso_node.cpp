#include <algorithm>
#include <memory>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "torso_interfaces/srv/move_torso.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

using namespace std::chrono_literals;

// ======================================================
// JRK INTERFACE
// ======================================================
class JrkInterface
{
public:
  bool open_port(const std::string &device)
  {
    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0)
      return false;

    struct termios options{};
    if (tcgetattr(fd_, &options) != 0)
      return false;

    options.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    options.c_iflag |= IGNPAR;
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_oflag &= ~(ONLCR | OCRNL);
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 10;

    if (tcsetattr(fd_, TCSANOW, &options) != 0)
      return false;

    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void close_port()
  {
    if (fd_ >= 0)
    {
      close(fd_);
      fd_ = -1;
    }
  }

  int read_feedback()
  {
    return read_variable_2b(0xA5);
  }

  int read_and_clear_errors()
  {
    return read_variable_2b(0xB3);
  }

  bool set_target(int target)
  {
    const uint8_t bytes[2] = {
        static_cast<uint8_t>(0xC0 + (target & 0x1F)),
        static_cast<uint8_t>((target >> 5) & 0x7F)};

    std::lock_guard<std::mutex> lock(mutex_);
    return write_bytes(bytes, 2);
  }

private:
  int fd_ = -1;
  std::mutex mutex_;

  bool write_bytes(const uint8_t *data, size_t size)
  {
    ssize_t written = write(fd_, data, size);
    return (written == static_cast<ssize_t>(size));
  }

  int read_variable_2b(uint8_t cmd)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!write_bytes(&cmd, 1))
      return -1;

    uint8_t resp[2] = {0, 0};
    ssize_t n = read(fd_, resp, 2);

    if (n != 2)
      return -1;

    return resp[0] + 256 * resp[1];
  }
};

// ======================================================
// TORSO NODE
// ======================================================
class TorsoNode : public rclcpp::Node
{
public:
  TorsoNode() : Node("torso_node")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing torso_node...");

    if (!declare_and_get_parameters() || !initialize_hardware())
    {
      RCLCPP_ERROR(this->get_logger(), "Initialization failed");
      rclcpp::shutdown();
      return;
    }

    setup_interfaces();
    initialize_messages();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz_)),
        std::bind(&TorsoNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Torso node ready");
  }

  ~TorsoNode()
  {
    jrk_.close_port();
  }

private:
  // =========================
  // ROS
  // =========================
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_current_pose_;
  rclcpp::Service<torso_interfaces::srv::MoveTorso>::SharedPtr srv_move_;
  rclcpp::TimerBase::SharedPtr timer_;

  // =========================
  // PARAMETERS
  // =========================
  std::string port_;
  std::string joint_states_topic_;
  std::string current_pose_topic_;
  std::string torso_controller_service_;
  std::string joint_name_;

  double min_m_;
  double max_m_;
  int fb_min_;
  int fb_max_;
  int target_min_;
  int target_max_;
  double publish_rate_hz_;
  bool simulation_;

  // =========================
  // STATE
  // =========================
  double current_position_m_ = 0.0;
  double goal_position_m_    = 0.0;
  int    current_feedback_   = 0;

  // =========================
  // HARDWARE
  // =========================
  JrkInterface jrk_;

  // =========================
  // MESSAGES
  // =========================
  sensor_msgs::msg::JointState      joint_msg_;
  std_msgs::msg::Float64MultiArray  current_pose_msg_;

  // ======================================================
  // PARAMETERS
  // ======================================================
  bool declare_and_get_parameters()
  {
    this->declare_parameter("simulation",               false);
    this->declare_parameter("port",                     "/dev/torso");
    this->declare_parameter("joint_states_topic",       "joint_states");
    this->declare_parameter("current_pose_topic",       "current_pose");
    this->declare_parameter("torso_controller_service", "move_torso");
    this->declare_parameter("joint_name",               "torso_joint");
    this->declare_parameter("min_position_m",           0.0);
    this->declare_parameter("max_position_m",           0.30);
    this->declare_parameter("feedback_min",             0);
    this->declare_parameter("feedback_max",             4095);
    this->declare_parameter("target_min",               0);
    this->declare_parameter("target_max",               4095);
    this->declare_parameter("publish_rate",             30.0);

    simulation_               = this->get_parameter("simulation").as_bool();
    port_                     = this->get_parameter("port").as_string();
    joint_states_topic_       = this->get_parameter("joint_states_topic").as_string();
    current_pose_topic_       = this->get_parameter("current_pose_topic").as_string();
    torso_controller_service_ = this->get_parameter("torso_controller_service").as_string();
    joint_name_               = this->get_parameter("joint_name").as_string();
    min_m_                    = this->get_parameter("min_position_m").as_double();
    max_m_                    = this->get_parameter("max_position_m").as_double();
    fb_min_                   = this->get_parameter("feedback_min").as_int();
    fb_max_                   = this->get_parameter("feedback_max").as_int();
    target_min_               = this->get_parameter("target_min").as_int();
    target_max_               = this->get_parameter("target_max").as_int();
    publish_rate_hz_          = this->get_parameter("publish_rate").as_double();

    return true;
  }

  // ======================================================
  // INIT HARDWARE
  // ======================================================
  bool initialize_hardware()
  {
    if (simulation_)
    {
      RCLCPP_WARN(this->get_logger(), "Running in SIMULATION mode");
      current_position_m_ = min_m_;
      goal_position_m_    = min_m_;
      return true;
    }

    if (!jrk_.open_port(port_))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", port_.c_str());
      return false;
    }

    int fb = jrk_.read_feedback();
    if (fb >= 0)
    {
      current_feedback_   = fb;
      current_position_m_ = feedback_to_meters(fb);
      goal_position_m_    = current_position_m_;

      int initial_target = meters_to_target(current_position_m_);
      jrk_.set_target(initial_target);
      RCLCPP_INFO(this->get_logger(),
                  "Initial target set to %d (%.3f m)", initial_target, current_position_m_);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to read initial feedback from Jrk");
      return false;
    }

    return true;
  }

  // ======================================================
  // ROS INTERFACES
  // ======================================================
  void setup_interfaces()
  {
    pub_joint_ = this->create_publisher<sensor_msgs::msg::JointState>(
        joint_states_topic_, 10);
    pub_current_pose_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        current_pose_topic_, 10);

    srv_move_ = this->create_service<torso_interfaces::srv::MoveTorso>(
        torso_controller_service_,
        std::bind(&TorsoNode::move_service, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

  void initialize_messages()
  {
    joint_msg_.name = {joint_name_};
    joint_msg_.position.resize(1);
    joint_msg_.velocity.resize(1);
    joint_msg_.effort.resize(1);

    current_pose_msg_.data.resize(1);
  }

  // ======================================================
  // SERVICE
  // ======================================================
  void move_service(
      const std::shared_ptr<torso_interfaces::srv::MoveTorso::Request> req,
      std::shared_ptr<torso_interfaces::srv::MoveTorso::Response> res)
  {
    double desired = std::clamp(req->position, min_m_, max_m_);
    goal_position_m_ = desired;

    int target = meters_to_target(desired);

    RCLCPP_INFO(this->get_logger(),
                "Sending target: %d (%.3f m)", target, desired);

    if (!simulation_)
    {
      if (!jrk_.set_target(target))
      {
        res->success = false;
        res->message = "Failed to send target";
        return;
      }
    }

    res->success = true;
    res->message = "Command sent";
  }

  // ======================================================
  // TIMER
  // ======================================================
  void timer_callback()
  {
    if (simulation_)
    {
      current_position_m_ = goal_position_m_;
    }
    else
    {
      int fb = jrk_.read_feedback();
      if (fb >= 0)
      {
        current_feedback_   = fb;
        current_position_m_ = feedback_to_meters(fb);
      }
    }

    publish();
  }

  // ======================================================
  // HELPERS
  // ======================================================
  void publish()
  {
    auto now = this->now();

    joint_msg_.header.stamp  = now;
    joint_msg_.position[0]   = current_position_m_;
    joint_msg_.velocity[0]   = 0.0;
    joint_msg_.effort[0]     = 0.0;
    pub_joint_->publish(joint_msg_);

    current_pose_msg_.data[0] = current_position_m_;
    pub_current_pose_->publish(current_pose_msg_);
  }

  double feedback_to_meters(int fb)
  {
    fb = std::clamp(fb, fb_min_, fb_max_);
    double alpha = static_cast<double>(fb - fb_min_) / (fb_max_ - fb_min_);
    return min_m_ + alpha * (max_m_ - min_m_);
  }

  int meters_to_target(double m)
  {
    m = std::clamp(m, min_m_, max_m_);
    double alpha = (m - min_m_) / (max_m_ - min_m_);
    return std::clamp(
        static_cast<int>(std::lround(target_min_ + alpha * (target_max_ - target_min_))),
        target_min_, target_max_);
  }
};

// ======================================================
// MAIN
// ======================================================
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TorsoNode>());
  rclcpp::shutdown();
  return 0;
}
