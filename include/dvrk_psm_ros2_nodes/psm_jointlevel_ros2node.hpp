#ifndef DVRK_PSM_ROS2_NODES__PSM_JOINTLEVEL_ROS2NODE_HPP_
#define DVRK_PSM_ROS2_NODES__PSM_JOINTLEVEL_ROS2NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>
#include <vector>

namespace dvrk_psm_ros2_nodes
{

class PSMJointLevelNode : public rclcpp::Node
{
public:
    explicit PSMJointLevelNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void publish_loop();

    std::mutex mutex_;
    std::vector<double> arm_positions_{6, 0.0};
    double jaw_position_{0.0};

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr setpoint_js_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jaw_setpoint_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jaw_measured_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr servo_jp_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr servo_jaw_jp_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dvrk_psm_ros2_nodes

#endif  // DVRK_PSM_ROS2_NODES__PSM_JOINTLEVEL_ROS2NODE_HPP_
