#include <algorithm>
#include <atomic>
#include <future>
#include <memory>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
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
    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0)
      return false;

    struct termios options{};
    if (tcgetattr(fd_, &options) != 0)
    {
      close_port();
      return false;
    }

    cfmakeraw(&options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 10;  // 1.0 s

    if (tcsetattr(fd_, TCSANOW, &options) != 0)
    {
      close_port();
      return false;
    }

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

  int read_errors_active()
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
    if (fd_ < 0)
      return false;

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

    // Move to startup position (top of range)
    RCLCPP_INFO(this->get_logger(), "Moving to starting position...");
    std::string msg;
    if (move_to_position(max_m_, msg))
      RCLCPP_INFO(this->get_logger(), "Starting position reached");
    else
      RCLCPP_WARN(this->get_logger(), "Failed to reach starting position: %s", msg.c_str());

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz_)),
        std::bind(&TorsoNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Torso node ready");
  }

  ~TorsoNode()
  {
    motion_cancelled_.store(true);
    if (motion_future_.valid())
      motion_future_.wait();

    jrk_.close_port();
  }

private:
  // =========================
  // ROS
  // =========================
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_current_pose_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_goal_reached_;
  rclcpp::Service<torso_interfaces::srv::MoveTorso>::SharedPtr srv_move_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_goal_pose_;
  rclcpp::TimerBase::SharedPtr timer_;

  // =========================
  // PARAMETERS
  // =========================
  std::string port_;
  std::string joint_states_topic_;
  std::string current_pose_topic_;
  std::string torso_controller_service_;
  std::string goal_pose_topic_;
  std::string goal_reached_topic_;
  std::string joint_name_;

  double move_timeout_s_        = 20.0;
  int    feedback_tolerance_    = 500;
  int    stable_reads_required_ = 3;

  double min_m_;
  double max_m_;
  int    fb_min_;
  int    fb_max_;
  int    target_min_;
  int    target_max_;
  double publish_rate_hz_;
  bool   simulation_;

  // =========================
  // STATE
  // =========================
  double current_position_m_ = 0.0;
  double goal_position_m_    = 0.0;
  int    current_feedback_   = 0;
  std::atomic<bool> motion_in_progress_{false};

  // Motion concurrency control
  std::atomic<bool> motion_active_{false};
  std::atomic<bool> motion_cancelled_{false};
  std::future<void> motion_future_;

  // =========================
  // HARDWARE
  // =========================
  JrkInterface jrk_;

  // =========================
  // MESSAGES
  // =========================
  sensor_msgs::msg::JointState     joint_msg_;
  std_msgs::msg::Float64MultiArray current_pose_msg_;

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
    this->declare_parameter("goal_pose_topic",          "goal_pose");
    this->declare_parameter("goal_reached_topic",       "goal_reached");
    this->declare_parameter("joint_name",               "torso_joint");
    this->declare_parameter("min_position_m",           0.0);
    this->declare_parameter("max_position_m",           0.3014);
    this->declare_parameter("feedback_min",             0);
    this->declare_parameter("feedback_max",             4095);
    this->declare_parameter("target_min",               0);
    this->declare_parameter("target_max",               4095);
    this->declare_parameter("publish_rate",             30.0);
    this->declare_parameter("move_timeout_s",           8.0);
    this->declare_parameter("feedback_tolerance",       20);
    this->declare_parameter("stable_reads_required",    3);

    simulation_               = this->get_parameter("simulation").as_bool();
    port_                     = this->get_parameter("port").as_string();
    joint_states_topic_       = this->get_parameter("joint_states_topic").as_string();
    current_pose_topic_       = this->get_parameter("current_pose_topic").as_string();
    torso_controller_service_ = this->get_parameter("torso_controller_service").as_string();
    goal_pose_topic_          = this->get_parameter("goal_pose_topic").as_string();
    goal_reached_topic_       = this->get_parameter("goal_reached_topic").as_string();
    joint_name_               = this->get_parameter("joint_name").as_string();
    min_m_                    = this->get_parameter("min_position_m").as_double();
    max_m_                    = this->get_parameter("max_position_m").as_double();
    fb_min_                   = this->get_parameter("feedback_min").as_int();
    fb_max_                   = this->get_parameter("feedback_max").as_int();
    target_min_               = this->get_parameter("target_min").as_int();
    target_max_               = this->get_parameter("target_max").as_int();
    publish_rate_hz_          = this->get_parameter("publish_rate").as_double();
    move_timeout_s_           = this->get_parameter("move_timeout_s").as_double();
    feedback_tolerance_       = this->get_parameter("feedback_tolerance").as_int();
    stable_reads_required_    = this->get_parameter("stable_reads_required").as_int();

    if (max_m_ <= min_m_)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid position range: min_position_m=%.6f max_position_m=%.6f",
                   min_m_, max_m_);
      return false;
    }

    if (fb_max_ <= fb_min_)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid feedback range: feedback_min=%d feedback_max=%d",
                   fb_min_, fb_max_);
      return false;
    }

    if (target_max_ <= target_min_)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid target range: target_min=%d target_max=%d",
                   target_min_, target_max_);
      return false;
    }

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
      current_feedback_   = fb_min_;
      current_position_m_ = min_m_;
      goal_position_m_    = min_m_;
      return true;
    }

    if (!jrk_.open_port(port_))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", port_.c_str());
      return false;
    }

    int fb_before = jrk_.read_feedback();
    if (fb_before < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to read initial feedback from Jrk");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Feedback inicial: %d", fb_before);

    int err = jrk_.read_errors_active();
    if (err >= 0)
      RCLCPP_INFO(this->get_logger(), "Errores activos: 0x%04X (%d)", err, err);
    else
      RCLCPP_WARN(this->get_logger(), "No se pudo leer errores activos");

    current_feedback_   = fb_before;
    current_position_m_ = feedback_to_meters(fb_before);
    goal_position_m_    = current_position_m_;

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

    pub_goal_reached_ = this->create_publisher<std_msgs::msg::Bool>(
        goal_reached_topic_, 10);

    srv_move_ = this->create_service<torso_interfaces::srv::MoveTorso>(
        torso_controller_service_,
        std::bind(&TorsoNode::move_service, this,
                  std::placeholders::_1, std::placeholders::_2));

    sub_goal_pose_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        goal_pose_topic_, 10,
        std::bind(&TorsoNode::goal_pose_callback, this, std::placeholders::_1));
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
  // MOVEMENT CONTROL (shared by service, topic, and startup)
  // ======================================================
  bool move_to_position(double desired, std::string &message)
  {
    desired          = std::clamp(desired, min_m_, max_m_);
    goal_position_m_ = desired;

    int target      = meters_to_target(desired);
    int expected_fb = meters_to_feedback(desired);

    RCLCPP_INFO(this->get_logger(),
                "Requested=%.6f m | Clamped=%.6f m | Target=%d | ExpectedFb=%d",
                desired, desired, target, expected_fb);

    if (simulation_)
    {
      current_feedback_   = expected_fb;
      current_position_m_ = desired;
      message = "Movement completed (simulation)";
      return true;
    }

    if (!jrk_.set_target(target))
    {
      message = "Failed to send target";
      return false;
    }

    bool reached = wait_until_reached_feedback(expected_fb);
    message = reached ? "Movement completed" : "Target sent, but feedback did not reach expected position";
    return reached;
  }

  // ======================================================
  // TOPIC CALLBACK (goal_pose)
  // ======================================================
  void goal_pose_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.empty())
    {
      RCLCPP_WARN(this->get_logger(), "goal_pose received empty data, ignoring");
      return;
    }

    if (motion_active_.load())
    {
      RCLCPP_WARN(this->get_logger(), "New goal_pose received while moving. Cancelling current motion.");
      motion_cancelled_.store(true);
      if (motion_future_.valid())
        motion_future_.wait();
    }

    double desired = msg->data[0];

    motion_cancelled_.store(false);
    motion_active_.store(true);
    motion_in_progress_.store(true);

    motion_future_ = std::async(std::launch::async, [this, desired]()
    {
      std::string result_msg;
      bool reached = move_to_position(desired, result_msg);

      RCLCPP_INFO(this->get_logger(), "goal_pose result: %s", result_msg.c_str());

      std_msgs::msg::Bool out;
      out.data = reached;
      pub_goal_reached_->publish(out);

      motion_in_progress_.store(false);
      motion_active_.store(false);
    });
  }

  // ======================================================
  // SERVICE
  // ======================================================
  void move_service(
      const std::shared_ptr<torso_interfaces::srv::MoveTorso::Request> req,
      std::shared_ptr<torso_interfaces::srv::MoveTorso::Response> res)
  {
    motion_in_progress_.store(true);
    res->success = move_to_position(req->position, res->message);
    motion_in_progress_.store(false);
  }

  // ======================================================
  // TIMER
  // ======================================================
  void timer_callback()
  {
    if (motion_in_progress_.load())
      return;

    if (simulation_)
    {
      current_position_m_ = goal_position_m_;
      current_feedback_   = meters_to_feedback(current_position_m_);
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

    joint_msg_.header.stamp = now;
    joint_msg_.position[0]  = current_position_m_;
    joint_msg_.velocity[0]  = 0.0;
    joint_msg_.effort[0]    = 0.0;
    pub_joint_->publish(joint_msg_);

    current_pose_msg_.data[0] = current_position_m_;
    pub_current_pose_->publish(current_pose_msg_);
  }

  double feedback_to_meters(int fb)
  {
    fb = std::clamp(fb, fb_min_, fb_max_);
    double alpha = static_cast<double>(fb - fb_min_) /
                   static_cast<double>(fb_max_ - fb_min_);
    return min_m_ + alpha * (max_m_ - min_m_);
  }

  int meters_to_target(double m)
  {
    constexpr double eps = 1e-6;

    m = std::clamp(m, min_m_, max_m_);

    if (m <= min_m_ + eps) return target_min_;
    if (m >= max_m_ - eps) return target_max_;

    double alpha = (m - min_m_) / (max_m_ - min_m_);

    return std::clamp(
        static_cast<int>(std::lround(target_min_ + alpha * (target_max_ - target_min_))),
        target_min_, target_max_);
  }

  int meters_to_feedback(double m)
  {
    constexpr double eps = 1e-6;

    m = std::clamp(m, min_m_, max_m_);

    if (m <= min_m_ + eps) return fb_min_;
    if (m >= max_m_ - eps) return fb_max_;

    double alpha = (m - min_m_) / (max_m_ - min_m_);

    return std::clamp(
        static_cast<int>(std::lround(fb_min_ + alpha * (fb_max_ - fb_min_))),
        fb_min_, fb_max_);
  }

  bool wait_until_reached_feedback(int expected_fb)
  {
    if (simulation_)
    {
      current_feedback_   = expected_fb;
      current_position_m_ = feedback_to_meters(expected_fb);
      return true;
    }

    auto start = std::chrono::steady_clock::now();
    int stable_reads = 0;

    while (rclcpp::ok() && !motion_cancelled_.load())
    {
      int fb = jrk_.read_feedback();
      if (fb >= 0)
      {
        current_feedback_   = fb;
        current_position_m_ = feedback_to_meters(fb);
        publish();

        int error = std::abs(fb - expected_fb);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "Esperando movimiento... expected_fb=%d fb=%d error=%d",
            expected_fb, fb, error);

        if (error <= feedback_tolerance_)
        {
          stable_reads++;
          if (stable_reads >= stable_reads_required_)
            return true;
        }
        else
        {
          stable_reads = 0;
        }
      }

      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start).count();

      if (elapsed > static_cast<long>(move_timeout_s_ * 1000.0))
        return false;

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return false;
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
