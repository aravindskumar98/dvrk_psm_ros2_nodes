#include "dvrk_psm_ros2_nodes/psm_jointlevel_ros2node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace dvrk_psm_ros2_nodes
{

PSMJointLevelNode::PSMJointLevelNode(const rclcpp::NodeOptions & options)
: Node("psm_jointlevel_ros2node", options)
{
    declare_parameter("arm_namespace_prefix", "PSM1");
    auto ns = "/" + get_parameter("arm_namespace_prefix").as_string();
    arm_positions_[2] = 0.08;

    setpoint_js_pub_ = create_publisher<sensor_msgs::msg::JointState>(ns + "/setpoint_js", 10);
    jaw_setpoint_pub_ = create_publisher<sensor_msgs::msg::JointState>(ns + "/jaw/setpoint_js", 10);
    jaw_measured_pub_ = create_publisher<sensor_msgs::msg::JointState>(ns + "/jaw/measured_js", 10);

    servo_jp_sub_ = create_subscription<sensor_msgs::msg::JointState>(ns + "/servo_jp", 10,
        [this](sensor_msgs::msg::JointState::SharedPtr msg) {
            std::lock_guard lock(mutex_);
            std::copy_n(msg->position.begin(), std::min(msg->position.size(), arm_positions_.size()),
                       arm_positions_.begin());
        });
    servo_jaw_jp_sub_ = create_subscription<sensor_msgs::msg::JointState>(ns + "/jaw/servo_jp", 10,
        [this](sensor_msgs::msg::JointState::SharedPtr msg) {
            if (!msg->position.empty()) {
                std::lock_guard lock(mutex_);
                jaw_position_ = msg->position[0];
            }
        });

    timer_ = create_wall_timer(2ms, [this]() { publish_loop(); });
    RCLCPP_INFO(get_logger(), "Streaming at 500 Hz on %s", ns.c_str());
}

void PSMJointLevelNode::publish_loop()
{
    auto now = get_clock()->now();
    std::lock_guard lock(mutex_);

    sensor_msgs::msg::JointState arm_msg;
    arm_msg.header.stamp = now;
    arm_msg.position = arm_positions_;
    setpoint_js_pub_->publish(arm_msg);

    sensor_msgs::msg::JointState jaw_msg;
    jaw_msg.header.stamp = now;
    jaw_msg.position = {jaw_position_};
    jaw_setpoint_pub_->publish(jaw_msg);
    jaw_measured_pub_->publish(jaw_msg);
}

}  // namespace dvrk_psm_ros2_nodes

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<dvrk_psm_ros2_nodes::PSMJointLevelNode>());
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("psm_jointlevel"), "Failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
