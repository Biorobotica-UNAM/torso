#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <stdexcept>
#include <chrono>
#include <thread>

class TorsoNode : public rclcpp::Node
{
public:
    TorsoNode() : Node("torso_node")
    {
        declare_parameter("port", "/dev/jrk_torso");
        declare_parameter("baudrate", 9600);
        declare_parameter<std::string>("goal_topic", "/hardware/torso/goal");

        std::string port;
        int baudrate;
        std::string goal_topic;

        get_parameter("port", port);
        get_parameter("baudrate", baudrate);
        get_parameter("goal_topic", goal_topic);

        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0)
        {
            RCLCPP_FATAL(get_logger(), "No se pudo abrir el puerto %s", port.c_str());
            throw std::runtime_error("Serial open failed");
        }

        configureSerial(baudrate);

        // Pequeña espera por si el dispositivo lo necesita
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Clear errors 0xB3
        if (!sendInitByte())
        {
            throw std::runtime_error("Init byte failed");
        }

        RCLCPP_INFO(get_logger(), "Get Error Flags Halting 0xB3 enviado correctamente");

        sub_ = create_subscription<std_msgs::msg::Float32>(
            goal_topic,
            10,
            std::bind(&TorsoNode::targetCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "Torso node listo y escuchando /jrk_target");
    }

    ~TorsoNode()
    {
        if (fd_ >= 0)
            close(fd_);
    }

private:
    int fd_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;

    void configureSerial(int baudrate)
    {
        struct termios tty{};
        if (tcgetattr(fd_, &tty) != 0)
        {
            throw std::runtime_error("tcgetattr failed");
        }

        speed_t speed = B9600;
        if (baudrate == 115200) speed = B115200;

        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

        tty.c_lflag = 0;
        tty.c_iflag = 0;
        tty.c_oflag = 0;

        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 5;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0)
        {
            throw std::runtime_error("tcsetattr failed");
        }
    }

    bool sendInitByte()
    {
        unsigned char b = 0xB3;

        ssize_t n = write(fd_, &b, 1);
        if (n != 1)
        {
            RCLCPP_ERROR(get_logger(), "Error enviando byte 0xB3");
            return false;
        }

        tcdrain(fd_);  // asegurar envío
        return true;
    }

    int jrkSetTarget(float target)
    {
        int target_bits = (int)(target * 15996.09375f + 0.5f);

        if (target_bits > 4095) target_bits = 4095;
        if (target_bits < 0)    target_bits = 0;

        unsigned char command[] = {
            static_cast<unsigned char>(0xC0 + (target_bits & 0x1F)),
            static_cast<unsigned char>((target_bits >> 5) & 0x7F)
        };

        if (write(fd_, command, sizeof(command)) != (ssize_t)sizeof(command))
        {
            RCLCPP_ERROR(get_logger(), "Error enviando target");
            return -1;
        }

        return 0;
    }

    void targetCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        jrkSetTarget(msg->data);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        rclcpp::spin(std::make_shared<TorsoNode>());
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error fatal: " << e.what() << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}