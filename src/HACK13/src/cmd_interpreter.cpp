#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include "hack13/msg/angle.hpp"
#include "hack13/msg/velocity.hpp"

class CmdInterpreter : public rclcpp::Node {
public:
  CmdInterpreter() : Node("cmd_interpreter") {

    m_theta_pub = this->create_publisher<hack13::msg::Angle>("/theta_ideal", 10);
    m_vel_pub   = this->create_publisher<hack13::msg::Velocity>("/vel_ideal", 10);

    m_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist &msg) {
          auto vel     = hack13::msg::Velocity();
          vel.velocity = msg.linear.x;

          m_vel_pub->publish(vel);

          float radius = (msg.angular.z == 0) ? 0.0 : msg.linear.x / msg.angular.z;

          auto angle  = hack13::msg::Angle();
          angle.theta = (radius != 0.0) ? std::clamp(std::atan(WHEEL_BASE / radius),
                                                     float(-0.95), float(0.95))
                                        : 0.0;

          m_theta_pub->publish(angle);
        });

    RCLCPP_INFO(this->get_logger(), "cmd_interpreter init()");
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_sub;
  rclcpp::Publisher<hack13::msg::Angle>::SharedPtr m_theta_pub;
  rclcpp::Publisher<hack13::msg::Velocity>::SharedPtr m_vel_pub;

  constexpr static const float WHEEL_BASE{0.7};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CmdInterpreter>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}