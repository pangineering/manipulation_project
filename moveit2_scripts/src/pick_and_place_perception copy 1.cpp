#include <iostream>
#include <vector>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

class PickPlacePerception {
private:
    moveit::planning_interface::MoveGroupInterface move_group_arm;
    moveit::planning_interface::MoveGroupInterface move_group_gripper;
    const moveit::core::JointModelGroup* joint_model_group_arm;
    const moveit::core::JointModelGroup* joint_model_group_gripper;

    bool goToHomePosition() {
        // Implementation for going home
        // ...
        return true; // Replace with appropriate logic
    }

    bool openGripper() {
        // Implementation for opening gripper
        // ...
        return true; // Replace with appropriate logic
    }

    bool closeGripper() {
        // Implementation for closing gripper
        // ...
        return true; // Replace with appropriate logic
    }

    void preGrasp(const geometry_msgs::msg::Pose& target_pose) {
        RCLCPP_INFO(rclcpp::get_logger("move_group_demo"), "Pregrasp Position");

        move_group_arm.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
        bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                            moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success_arm) {
            move_group_arm.execute(my_plan_arm);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("move_group_demo"), "Failed to plan preGrasp movement");
        }
    }

public:
    PickPlacePerception(rclcpp::Node::SharedPtr& node, const std::string& planning_group_arm, const std::string& planning_group_gripper)
        : move_group_arm(node, planning_group_arm), move_group_gripper(node, planning_group_gripper) {
        joint_model_group_arm = move_group_arm.getCurrentState()->getJointModelGroup(planning_group_arm);
        joint_model_group_gripper = move_group_gripper.getCurrentState()->getJointModelGroup(planning_group_gripper);
    }

    // Public method to trigger the preGrasp operation
    void initiatePreGrasp(const geometry_msgs::msg::Pose& target_pose) {
        goToHomePosition();
        preGrasp(target_pose);
        openGripper();
        // Additional actions if needed...
    }

    // Other public methods for manipulation and perception
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    PickPlacePerception perception(move_group_node, "ur_manipulator", "gripper");

    // Assuming you have a target pose defined as target_pose1
    geometry_msgs::msg::Pose target_pose1;
    perception.initiatePreGrasp(target_pose1);

    rclcpp::shutdown();
    return 0;
}
