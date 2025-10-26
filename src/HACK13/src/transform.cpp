#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "hack13/msg/ack_state.hpp"
#include "hack13/msg/diff_state.hpp"

class AckTransform : public rclcpp::Node {
public:
  AckTransform() : Node("ack_transform") {

    m_tf_broad = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    m_ack_state_sub = this->create_subscription<hack13::msg::AckState>(
        "/ack_state", 10, [this](const hack13::msg::AckState &msg) {
          // publish each tf
          send_tf("frame", "left_steering_arm", 0.0, 0.3045, 0.0, msg.theta_2);
          send_tf("rack", "left_tie_ros", 0.0, 0.1737, 0.0, msg.theta_3);
          send_tf("base_link", "rack", -0.0315, msg.d, 0.0, 0.0);
          send_tf("rack", "right_tie_rod", 0.0, -0.01737, 0.0, msg.theta_5);
          send_tf("frame", "right_steering_rod", 0.0, -0.3045, 0.0, msg.theta_4);
          send_tf("base_link", "pinion", -0.0315, 0.0, 0.04445, msg.theta_pin, 1.57);
        });

    m_diff_state_sub = this->create_subscription<hack13::msg::AckState>(
        "/diff_state", 10, [this](const hack13::msg::AckState &msg) {
          // publish each tf
        });

    RCLCPP_INFO(this->get_logger(), "ack_transformer init()");
  }

private:
  rclcpp::Subscription<hack13::msg::AckState>::SharedPtr m_ack_state_sub;
  rclcpp::Subscription<hack13::msg::AckState>::SharedPtr m_diff_state_sub;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broad;

  void send_tf(const std::string &parent, const std::string &child, double x,
               double y, double z, double yaw, double pitch = 0.0) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp            = this->get_clock()->now();
    t.header.frame_id         = parent;
    t.child_frame_id          = child;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;

    tf2::Quaternion q;
    q.setRPY(0.0, pitch, yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    m_tf_broad->sendTransform(t);
  };
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<AckTransform>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}