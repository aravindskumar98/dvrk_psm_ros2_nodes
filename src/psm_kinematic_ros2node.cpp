#include "dvrk_psm_ros2_nodes/psm_kinematic_ros2node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace dvrk_psm_ros2_nodes
{

PSMKinematicNode::PSMKinematicNode(const rclcpp::NodeOptions & options)
: Node("psm_kinematic_ros2node", options), robot_(std::make_unique<robManipulator>())
{
    declare_parameter("arm_namespace_prefix", "PSM1");
    declare_parameter("arm_pkg", "dvrk_config");
    declare_parameter("tool_pkg", "dvrk_config");
    declare_parameter("arm_relpath", "kinematic/PSM.json");
    declare_parameter("tool_relpath", "tool/LARGE_NEEDLE_DRIVER_400006.json");

    auto ns = "/" + get_parameter("arm_namespace_prefix").as_string();
    auto arm_path = std::filesystem::path(ament_index_cpp::get_package_share_directory(
        get_parameter("arm_pkg").as_string())) / get_parameter("arm_relpath").as_string();
    auto tool_path = std::filesystem::path(ament_index_cpp::get_package_share_directory(
        get_parameter("tool_pkg").as_string())) / get_parameter("tool_relpath").as_string();

    RCLCPP_INFO(get_logger(), "Loading %s + %s", arm_path.c_str(), tool_path.c_str());

    if (robot_->LoadRobot(arm_path) != robManipulator::ESUCCESS ||
        robot_->LoadRobot(tool_path) != robManipulator::ESUCCESS) {
        throw std::runtime_error("Failed to load robot: " + robot_->LastError());
    }

    num_joints_ = robot_->links.size();
    last_joint_position_.SetSize(num_joints_);
    last_joint_position_.SetAll(0.0);
    joint_names_ = {"Base_Yaw", "Yaw_Pitch_End", "Pitch_End_Main_Insert",
                    "Main_Insert_Tool_Roll", "Tool_Roll_Tool_Pitch", "Tool_Yaw_Tool_Pitch"};

    auto latched = rclcpp::QoS(rclcpp::KeepLast(1000)).transient_local();
    state_pub_ = create_publisher<crtk_msgs::msg::OperatingState>(ns + "/operating_state", latched);
    servo_jp_pub_ = create_publisher<sensor_msgs::msg::JointState>(ns + "/servo_jp", 10);
    setpoint_cp_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(ns + "/setpoint_cp", 10);
    measured_cp_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(ns + "/measured_cp", 10);

    state_sub_ = create_subscription<crtk_msgs::msg::StringStamped>(ns + "/state_command", 10,
        [this](crtk_msgs::msg::StringStamped::SharedPtr) {
            crtk_msgs::msg::OperatingState msg;
            msg.header.stamp = get_clock()->now();
            msg.state = "ENABLED";
            msg.is_homed = true;
            msg.is_busy = false;
            state_pub_->publish(msg);
        });
    servo_cp_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(ns + "/servo_cp", 10,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { on_servo_cp(msg); });
    setpoint_js_sub_ = create_subscription<sensor_msgs::msg::JointState>(ns + "/setpoint_js", 10,
        [this](sensor_msgs::msg::JointState::SharedPtr msg) { on_setpoint_js(msg); });

    RCLCPP_INFO(get_logger(), "Initialized with %zu joints on %s", num_joints_, ns.c_str());
}

void PSMKinematicNode::on_state_command(const crtk_msgs::msg::StringStamped::SharedPtr)
{
    crtk_msgs::msg::OperatingState msg;
    msg.header.stamp = get_clock()->now();
    msg.state = "ENABLED";
    msg.is_homed = true;
    msg.is_busy = false;
    state_pub_->publish(msg);
}

void PSMKinematicNode::on_servo_cp(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    vctFrame4x4<double> target;
    target.Translation().Assign(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                      msg->pose.orientation.z, msg->pose.orientation.w);
    tf2::Matrix3x3 rot(q);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            target.Rotation().Element(i, j) = rot[i][j];
        }
    }

    vctDynamicVector<double> jp = last_joint_position_;
    if (robot_->InverseKinematics(jp, target) != robManipulator::ESUCCESS) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "IK failed");
        return;
    }

    last_joint_position_ = jp;
    sensor_msgs::msg::JointState js;
    js.header.stamp = get_clock()->now();
    js.header.frame_id = "map";
    js.name = joint_names_;
    js.position.assign(jp.Pointer(), jp.Pointer() + num_joints_);
    servo_jp_pub_->publish(js);
}

void PSMKinematicNode::on_setpoint_js(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.size() < num_joints_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Insufficient joints");
        return;
    }

    vctDynamicVector<double> q(num_joints_);
    std::copy_n(msg->position.begin(), num_joints_, q.Pointer());
    auto frame = robot_->ForwardKinematics(q);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = "map";
    pose.pose.position.x = frame.Translation().X();
    pose.pose.position.y = frame.Translation().Y();
    pose.pose.position.z = frame.Translation().Z();

    tf2::Matrix3x3 rot(frame.Rotation().Element(0,0), frame.Rotation().Element(0,1), frame.Rotation().Element(0,2),
                       frame.Rotation().Element(1,0), frame.Rotation().Element(1,1), frame.Rotation().Element(1,2),
                       frame.Rotation().Element(2,0), frame.Rotation().Element(2,1), frame.Rotation().Element(2,2));
    tf2::Quaternion quat;
    rot.getRotation(quat);
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    pose.pose.orientation.w = quat.w();

    setpoint_cp_pub_->publish(pose);
    measured_cp_pub_->publish(pose);
}

}  // namespace dvrk_psm_ros2_nodes

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<dvrk_psm_ros2_nodes::PSMKinematicNode>());
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("psm_kinematic"), "Failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
