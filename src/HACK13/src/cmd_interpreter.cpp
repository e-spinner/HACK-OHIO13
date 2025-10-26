#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include "hack13/msg/angle.hpp"
// #include "hack13/msg/diff_state.hpp"

class CmdInterpreter : public rclcpp::Node {
public:
  CmdInterpreter() : Node("cmd_interpreter") {

    m_theta_pub = this->create_publisher<hack13::msg::Angle>("/theta_ideal", 10);
    // m_diff_pub  = this->create_publisher<hack13::msg::DiffState>("/diff_state",
    // 10);

    m_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist &msg) {
          float radius = (msg.angular.z == 0) ? 0.0 : msg.linear.x / msg.angular.z;

          auto angle  = hack13::msg::Angle();
          angle.theta = (radius != 0.0) ? std::clamp(std::atan(WHEEL_BASE / radius),
                                                     float(-0.95), float(0.95))
                                        : 0.0;

          if (msg.linear.x > 0) { angle.theta = -angle.theta; }

          if (msg.linear.x != 0) { m_theta_pub->publish(angle); }

          // auto diff = hack13::msg::DiffState();

          // diff.v_left = (angle.theta < 0)
          //                   ? msg.angular.z * (radius - (WHEEL_BASE / 2))
          //                   : msg.angular.z * (radius + (WHEEL_BASE / 2));

          // diff.v_right = (angle.theta < 0)
          //                    ? msg.angular.z * (radius + (WHEEL_BASE / 2))
          //                    : msg.angular.z * (radius - (WHEEL_BASE / 2));

          // m_diff_pub->publish(diff);
        });

    RCLCPP_INFO(this->get_logger(), "cmd_interpreter init()");
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_sub;
  rclcpp::Publisher<hack13::msg::Angle>::SharedPtr m_theta_pub;
  // rclcpp::Publisher<hack13::msg::DiffState>::SharedPtr m_diff_pub;

  constexpr static const float WHEEL_BASE{0.7};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CmdInterpreter>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}