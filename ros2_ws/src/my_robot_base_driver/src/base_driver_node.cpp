#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cmath>

class BaseDriverNode : public rclcpp::Node {
public:
    BaseDriverNode() : Node("base_driver_node") {
        // Parameters
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<bool>("publish_odom", true); // <--- TÙY CHỌN CHẾ ĐỘ

        std::string port = this->get_parameter("port").as_string();
        int baud = this->get_parameter("baudrate").as_int();
        publish_odom_ = this->get_parameter("publish_odom").as_bool();

        // Setup Serial
        if (!setup_serial(port, baud)) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial: %s", port.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Serial connected: %s", port.c_str());
        }

        // Sub/Pub
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&BaseDriverNode::cmd_callback, this, std::placeholders::_1));

        if (publish_odom_) {
            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            RCLCPP_INFO(this->get_logger(), "Odom/TF publishing ENABLED (Real Mode)");
        } else {
            RCLCPP_WARN(this->get_logger(), "Odom/TF publishing DISABLED (Backup Mode - Use Fake Odom)");
        }

        // Timer 20Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&BaseDriverNode::loop, this));
        
        last_time_ = this->get_clock()->now();
    }

    ~BaseDriverNode() { if(fd_ != -1) close(fd_); }

private:
    int fd_ = -1;
    bool publish_odom_;
    double x_ = 0, y_ = 0, th_ = 0;
    rclcpp::Time last_time_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool setup_serial(const std::string &port, int baud) {
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ == -1) return false;
        struct termios opt;
        tcgetattr(fd_, &opt);
        cfsetispeed(&opt, (baud==115200)?B115200:B9600);
        cfsetospeed(&opt, (baud==115200)?B115200:B9600);
        opt.c_cflag |= (CLOCAL|CREAD|CS8);
        opt.c_cflag &= ~(PARENB|CSTOPB|CSIZE);
        opt.c_lflag |= ICANON; // Read line mode
        tcsetattr(fd_, TCSANOW, &opt);
        return true;
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (fd_ == -1) return;
        char buf[50];
        int len = sprintf(buf, "%.3f,%.3f\n", msg->linear.x, msg->angular.z);
        write(fd_, buf, len);
    }

    void loop() {
        if (fd_ == -1) return;
        
        // Luôn đọc Serial để tránh đầy bộ đệm (kể cả khi không dùng odom)
        char buf[64];
        int n = read(fd_, buf, sizeof(buf)-1);
        if (n > 0) {
            buf[n] = '\0';
            // Chỉ tính toán nếu cần publish
            if (publish_odom_) {
                std::string data(buf);
                size_t sep = data.find(',');
                if (sep != std::string::npos) {
                    try {
                        double v = std::stof(data.substr(0, sep));
                        double w = std::stof(data.substr(sep+1));
                        update_odom(v, w);
                    } catch (...) {}
                }
            }
        }
    }

    void update_odom(double v, double w) {
        rclcpp::Time now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        double dx = (v * cos(th_)) * dt;
        double dy = (v * sin(th_)) * dt;
        double dth = w * dt;

        x_ += dx; y_ += dy; th_ += dth;

        tf2::Quaternion q; q.setRPY(0, 0, th_);
        
        // TF
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.rotation.x = q.x(); t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z(); t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);

        // Odom Msg
        nav_msgs::msg::Odometry o;
        o.header = t.header;
        o.child_frame_id = "base_link";
        o.pose.pose.position.x = x_;
        o.pose.pose.position.y = y_;
        o.pose.pose.orientation = t.transform.rotation;
        o.twist.twist.linear.x = v;
        o.twist.twist.angular.z = w;
        odom_pub_->publish(o);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseDriverNode>());
    rclcpp::shutdown();
    return 0;
}