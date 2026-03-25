#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>

#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

using namespace std::chrono_literals;

class TorsoNode : public rclcpp::Node
{
public:
  TorsoNode()
  : Node("torso_node")
  {
    port_ = this->declare_parameter<std::string>("port", "/dev/torso");
    joint_name_ = this->declare_parameter<std::string>("joint_name", "torso_joint");

    // Recorrido físico en metros
    min_position_m_ = this->declare_parameter<double>("min_position_m", 0.0);
    max_position_m_ = this->declare_parameter<double>("max_position_m", 0.3014);

    // Rango de feedback/target del Jrk
    feedback_min_ = this->declare_parameter<int>("feedback_min", 0);
    feedback_max_ = this->declare_parameter<int>("feedback_max", 4095);
    target_min_   = this->declare_parameter<int>("target_min", 0);
    target_max_   = this->declare_parameter<int>("target_max", 4095);

    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 30.0);

    command_topic_ = this->declare_parameter<std::string>("command_topic", "/torso/command");
    joint_states_topic_ = this->declare_parameter<std::string>("joint_states_topic", "/joint_states");

    RCLCPP_INFO(this->get_logger(), "Abriendo Jrk en: %s", port_.c_str());

    if (!openSerial(port_)) {
      RCLCPP_FATAL(this->get_logger(), "No se pudo abrir el puerto del Jrk.");
      throw std::runtime_error("No se pudo abrir el puerto del Jrk");
    }

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, 10);

    cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      command_topic_,
      10,
      std::bind(&TorsoNode::commandCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&TorsoNode::updateAndPublish, this));

    // Leer posición inicial
    int feedback = readVariable2B(0xA5);  // Feedback
    if (feedback < 0) {
      RCLCPP_WARN(this->get_logger(), "No se pudo leer el feedback inicial.");
      current_position_m_ = min_position_m_;
    } else {
      current_feedback_ = feedback;
      current_position_m_ = feedbackToMeters(feedback);
      RCLCPP_INFO(
        this->get_logger(),
        "Feedback inicial: %d  -> posición inicial: %.5f m",
        feedback, current_position_m_);
    }

    last_publish_time_ = this->now();
  }

  ~TorsoNode() override
  {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

private:
  // ROS
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Params
  std::string port_;
  std::string joint_name_;
  std::string command_topic_;
  std::string joint_states_topic_;

  double min_position_m_;
  double max_position_m_;
  int feedback_min_;
  int feedback_max_;
  int target_min_;
  int target_max_;
  double publish_rate_hz_;

  // State
  int fd_ = -1;
  std::mutex serial_mutex_;

  int current_feedback_ = 0;
  double current_position_m_ = 0.0;
  double last_position_m_ = 0.0;
  rclcpp::Time last_publish_time_;

  static double clampDouble(double x, double lo, double hi)
  {
    return std::max(lo, std::min(x, hi));
  }

  static int clampInt(int x, int lo, int hi)
  {
    return std::max(lo, std::min(x, hi));
  }

  bool openSerial(const std::string & device)
  {
    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ == -1) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo abrir %s: %s", device.c_str(), std::strerror(errno));
      return false;
    }

    struct termios options{};
    if (tcgetattr(fd_, &options) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr falló: %s", std::strerror(errno));
      close(fd_);
      fd_ = -1;
      return false;
    }

    // Igual que el ejemplo POSIX de Pololu: no canónico, sin echo
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_oflag &= ~(ONLCR | OCRNL);

    // Timeout de lectura pequeño
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;  // 0.1 s

    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr falló: %s", std::strerror(errno));
      close(fd_);
      fd_ = -1;
      return false;
    }

    return true;
  }

  bool writeBytes(const uint8_t * data, size_t size)
  {
    ssize_t written = write(fd_, data, size);
    if (written < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error escribiendo al Jrk: %s", std::strerror(errno));
      return false;
    }
    if (static_cast<size_t>(written) != size) {
      RCLCPP_ERROR(this->get_logger(), "Escritura incompleta al Jrk.");
      return false;
    }
    return true;
  }

  int readVariable2B(uint8_t command)
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);

    tcflush(fd_, TCIFLUSH);

    if (!writeBytes(&command, 1)) {
      return -1;
    }

    uint8_t response[2] = {0, 0};
    ssize_t n = read(fd_, response, 2);

    if (n < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error leyendo respuesta del Jrk: %s", std::strerror(errno));
      return -1;
    }

    if (n != 2) {
      RCLCPP_WARN(this->get_logger(), "Respuesta incompleta del Jrk: llegaron %ld bytes", static_cast<long>(n));
      return -1;
    }

    return static_cast<int>(response[0]) + 256 * static_cast<int>(response[1]);
  }

  bool setTargetHighResolution(int target)
  {
    target = clampInt(target, target_min_, target_max_);

    const uint8_t bytes[2] = {
      static_cast<uint8_t>(0xC0 + (target & 0x1F)),
      static_cast<uint8_t>((target >> 5) & 0x7F)
    };

    std::lock_guard<std::mutex> lock(serial_mutex_);
    return writeBytes(bytes, 2);
  }

  double feedbackToMeters(int feedback) const
  {
    feedback = clampInt(feedback, feedback_min_, feedback_max_);

    const double denom = static_cast<double>(feedback_max_ - feedback_min_);
    if (denom <= 0.0) {
      return min_position_m_;
    }

    const double alpha = (static_cast<double>(feedback) - feedback_min_) / denom;
    return min_position_m_ + alpha * (max_position_m_ - min_position_m_);
  }

  int metersToTarget(double meters) const
  {
    meters = clampDouble(meters, min_position_m_, max_position_m_);

    const double denom = (max_position_m_ - min_position_m_);
    if (denom <= 0.0) {
      return target_min_;
    }

    const double alpha = (meters - min_position_m_) / denom;
    const double raw = static_cast<double>(target_min_) +
                       alpha * static_cast<double>(target_max_ - target_min_);

    return clampInt(static_cast<int>(std::lround(raw)), target_min_, target_max_);
  }

  void commandCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    const double desired_m = clampDouble(
      static_cast<double>(msg->data),
      min_position_m_,
      max_position_m_);

    const int target = metersToTarget(desired_m);

    if (!setTargetHighResolution(target)) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo mandar target al Jrk.");
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Comando torso: %.5f m -> target %d",
      desired_m, target);
      
  }

  void updateAndPublish()
  {
    const int feedback = readVariable2B(0xA5);  // Feedback
    if (feedback >= 0) {
      current_feedback_ = feedback;
      current_position_m_ = feedbackToMeters(feedback);
    }

    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name.push_back(joint_name_);
    js.position.push_back(current_position_m_);

    // velocidad estimada simple
    const auto now = this->now();
    const double dt = (now - last_publish_time_).seconds();
    if (dt > 1e-6) {
      js.velocity.push_back((current_position_m_ - last_position_m_) / dt);
    } else {
      js.velocity.push_back(0.0);
    }

    js.effort.push_back(0.0);

    joint_pub_->publish(js);

    last_position_m_ = current_position_m_;
    last_publish_time_ = now;
  }

  int readSignedVariable2B(uint8_t command)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  tcflush(fd_, TCIFLUSH);

  if (!writeBytes(&command, 1)) {
    return -1;
  }

  uint8_t response[2] = {0, 0};
  ssize_t n = read(fd_, response, 2);

  if (n != 2) {
    RCLCPP_WARN(this->get_logger(), "Respuesta incompleta para comando 0x%02X: %ld bytes",
                command, static_cast<long>(n));
    return -1;
  }

  int16_t value = static_cast<int16_t>(
    static_cast<uint16_t>(response[0]) |
    (static_cast<uint16_t>(response[1]) << 8));

  return static_cast<int>(value);
}

int getHaltingErrors()
{
  return readSignedVariable2B(0xB3);
}

void printJrkDiagnostics(const char * context)
{
  int target = readVariable2B(0xA3);          // Target
  int feedback = readVariable2B(0xA5);        // Feedback
  int duty_target = readSignedVariable2B(0xAB); // Duty cycle target
  int duty = readSignedVariable2B(0xAD);        // Duty cycle
  int halting = getHaltingErrors();             // Error flags halting

  RCLCPP_INFO(
    this->get_logger(),
    "[%s] target=%d feedback=%d duty_target=%d duty=%d halting_errors=0x%04X",
    context, target, feedback, duty_target, duty, halting);
}
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TorsoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}