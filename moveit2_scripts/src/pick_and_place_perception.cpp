#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <iostream>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>

class PickPlacePerception : public rclcpp::Node {
public:
  using Find = grasping_msgs::action::FindGraspableObjects;
  using GoalHandleFind = rclcpp_action::ClientGoalHandle<Find>;

  explicit PickPlacePerception(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("pick_place_perception", node_options), goal_done_(false) {
    // Create an action client for the "find_objects" action

    client_ptr_ = rclcpp_action::create_client<Find>(
        get_node_base_interface(), get_node_graph_interface(),
        get_node_logging_interface(), get_node_waitables_interface(),
        "find_objects");

    // Create a timer to send the goal periodically
    timer_ =
        create_wall_timer(std::chrono::milliseconds(500),
                          std::bind(&PickPlacePerception::send_goal, this));
  }

  bool is_goal_done() const { return goal_done_; }
  void pickPlace() {
    preGrasp();
    approachObject();
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      move_group_arm_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      move_group_gripper_;

  rclcpp_action::Client<Find>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  void send_goal() {
    // Cancel the timer
    timer_->cancel();

    // Set the goal_done_ flag to false
    goal_done_ = false;

    // Check if the action client is initialized
    if (!client_ptr_) {
      RCLCPP_ERROR(get_logger(), "Action client not initialized");
      goal_done_ = true;
      return;
    }

    // Wait for the action server to become available
    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      goal_done_ = true;
      return;
    }

    // Create the goal message
    auto goal_msg = Find::Goal();
    goal_msg.plan_grasps = false;

    RCLCPP_INFO(get_logger(), "Sending goal");

    // Set the options for sending the goal
    auto send_goal_options = rclcpp_action::Client<Find>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&PickPlacePerception::goal_response_callback, this,
                  std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&PickPlacePerception::feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(
        &PickPlacePerception::result_callback, this, std::placeholders::_1);

    // Send the goal and store the future handle
    auto goal_handle_future =
        client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(const GoalHandleFind::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleFind::SharedPtr,
                         const std::shared_ptr<const Find::Feedback> feedback) {
    RCLCPP_INFO(get_logger(), "Ignoring feedback...");
  }

  void result_callback(const GoalHandleFind::WrappedResult &result) {
    // Set the goal_done_ flag to true
    goal_done_ = true;

    // Check the result code
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown result code");
      return;
    }

    RCLCPP_INFO(get_logger(), "Result received");
    for (auto object : result.result->objects) {
      if (object.object.primitives[0].type == 1 &&
          object.object.primitives[0].dimensions[0] < 0.05 &&
          object.object.primitives[0].dimensions[1] < 0.05 &&
          object.object.primitives[0].dimensions[2] < 0.1) {
        RCLCPP_INFO(get_logger(), "X: %f",
                    object.object.primitive_poses[0].position.x);
        RCLCPP_INFO(get_logger(), "Y: %f",
                    object.object.primitive_poses[0].position.y);
      }
      RCLCPP_INFO(get_logger(), "Next");
    }
  }
  void preGrasp() {
    RCLCPP_INFO(get_logger(), "Pregrasp Position");

    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.x = -1.0;
    target_pose1.orientation.y = 0.00;
    target_pose1.orientation.z = 0.00;
    target_pose1.orientation.w = 0.00;
    target_pose1.position.x = 0.343;
    target_pose1.position.y = 0.132;
    target_pose1.position.z = 0.264;

    move_group_arm_->setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    bool success_arm = (move_group_arm_->plan(my_plan_arm) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success_arm) {
      move_group_arm_->execute(my_plan_arm);
      RCLCPP_INFO(get_logger(), "Arm in pregrasp position");
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to plan arm movement");
    }
    // ... (the rest of the code in preGrasp function)
  }

  void approachObject() {
    RCLCPP_INFO(get_logger(), "Approach to object!");
    geometry_msgs::msg::Pose target_pose1;
    // Assuming target_pose1 is a member of the class
    target_pose1.position.z -= 0.03;
    std::vector<geometry_msgs::msg::Pose> approach_waypoints;
    approach_waypoints.push_back(target_pose1);

    target_pose1.position.z -= 0.03;
    approach_waypoints.push_back(target_pose1);

    moveit_msgs::msg::RobotTrajectory trajectory_approach;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    double fraction = move_group_arm_->computeCartesianPath(
        approach_waypoints, eef_step, jump_threshold, trajectory_approach);

    move_group_arm_->execute(trajectory_approach);
  }
};

static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<PickPlacePerception>();

  // Spin asynchronously in a separate thread
  std::thread spin_thread([action_client]() {
    rclcpp::spin(action_client);
  });

  // Perform pickPlace concurrently while spinning
  action_client->pickPlace();

  // Wait for the spinning thread to finish before shutting down
  spin_thread.join();

  rclcpp::shutdown();
  return 0;
}