#ifndef DVRK_PSM_ROS2_NODES__PSM_KINEMATIC_ROS2NODE_HPP_
#define DVRK_PSM_ROS2_NODES__PSM_KINEMATIC_ROS2NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <crtk_msgs/msg/operating_state.hpp>
#include <crtk_msgs/msg/string_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cisstRobot/robManipulator.h>
#include <cisstVector/vctDynamicVector.h>
#include <cisstVector/vctFrame4x4.h>

namespace dvrk_psm_ros2_nodes
{

class PSMKinematicNode : public rclcpp::Node
{
public:
    explicit PSMKinematicNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void on_state_command(const crtk_msgs::msg::StringStamped::SharedPtr);
    void on_servo_cp(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void on_setpoint_js(const sensor_msgs::msg::JointState::SharedPtr msg);

    std::unique_ptr<robManipulator> robot_;
    size_t num_joints_;
    vctDynamicVector<double> last_joint_position_;
    std::vector<std::string> joint_names_;

    rclcpp::Publisher<crtk_msgs::msg::OperatingState>::SharedPtr state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr servo_jp_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_cp_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr measured_cp_pub_;
    rclcpp::Subscription<crtk_msgs::msg::StringStamped>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr servo_cp_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr setpoint_js_sub_;
};

}  // namespace dvrk_psm_ros2_nodes

#endif  // DVRK_PSM_ROS2_NODES__PSM_KINEMATIC_ROS2NODE_HPP_
