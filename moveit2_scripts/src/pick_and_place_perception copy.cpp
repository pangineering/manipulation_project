#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

class PickAndPlacePerception : public rclcpp::Node {
public:
  PickAndPlacePerception() : Node("pick_and_place_perception") {
    pick_and_place_group_ =
        new moveit::planning_interface::MoveGroupInterface("ur_manipulator");
    gripper_group_ =
        new moveit::planning_interface::MoveGroupInterface("gripper");

      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("get_pose_client", node_options), goal_done_(false) {
        this->client_ptr_ = rclcpp_action::create_client<Find>(
            this->get_node_base_interface(), this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(), "find_objects");

        this->timer_ =
            this->create_wall_timer(std::chrono::milliseconds(500),
                                    std::bind(&GetPoseClient::send_goal, this));
        bool is_goal_done() const { return this->goal_done_; }

        void send_goal() {
          using namespace std::placeholders;

          this->timer_->cancel();

          this->goal_done_ = false;

          if (!this->client_ptr_) {
            RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
          }

          if (!this->client_ptr_->wait_for_action_server(
                  std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(),
                         "Action server not available after waiting");
            this->goal_done_ = true;
            return;
          }

          auto goal_msg = Find::Goal();
          goal_msg.plan_grasps = false;

          RCLCPP_INFO(this->get_logger(), "Sending goal");

          auto send_goal_options =
              rclcpp_action::Client<Find>::SendGoalOptions();
          send_goal_options.goal_response_callback =
              std::bind(&GetPoseClient::goal_response_callback, this, _1);
          send_goal_options.feedback_callback =
              std::bind(&GetPoseClient::feedback_callback, this, _1, _2);
          send_goal_options.result_callback =
              std::bind(&GetPoseClient::result_callback, this, _1);
          auto goal_handle_future =
              this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }

      private:
        void pickAndPlace(const geometry_msgs::msg::Pose &object_pose) {
          // Pregrasp
          preGrasp();

          // Approach
          approachObject();

          // Close Gripper
          closeGripper();

          // Retreat
          retreathObject();

          // Place
          placeObject();

          // Open Gripper
          releaseObject();

          RCLCPP_INFO(this->get_logger(), "Pick and place actions completed.");
        }

        void preGrasp() {
          RCLCPP_INFO(this->get_logger(), "Pregrasp Position");

          moveit::planning_interface::MoveGroupInterface move_group_arm(
              "ur_manipulator");
          move_group_arm.setPoseTarget(
              target_pose_); // Use the stored target_pose

          moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
          bool success_arm =
              (move_group_arm.plan(my_plan_arm) ==
               moveit::planning_interface::MoveItErrorCode::SUCCESS);

          if (success_arm) {
            move_group_arm.execute(my_plan_arm);
          } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to plan preGrasp movement");
          }
        }

        void approachObject() {
          // Approach
          RCLCPP_INFO(LOGGER, "Approach to object!");

          std::vector<geometry_msgs::msg::Pose> approach_waypoints;
          target_pose1.position.z -= 0.03;
          approach_waypoints.push_back(target_pose1);

          target_pose1.position.z -= 0.03;
          approach_waypoints.push_back(target_pose1);

          moveit_msgs::msg::RobotTrajectory trajectory_approach;
          const double jump_threshold = 0.0;
          const double eef_step = 0.01;

          double fraction = move_group_arm.computeCartesianPath(
              approach_waypoints, eef_step, jump_threshold,
              trajectory_approach);

          move_group_arm.execute(trajectory_approach);
        }
        void closeGripper() {
          // Close Gripper

          RCLCPP_INFO(LOGGER, "Close Gripper!");

          move_group_gripper.setNamedTarget("close");

          success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                             moveit::core::MoveItErrorCode::SUCCESS);

          move_group_gripper.execute(my_plan_gripper);
        }
        void retreathObject() {
          // Retreat

          RCLCPP_INFO(LOGGER, "Retreat from object!");

          std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
          target_pose1.position.z += 0.03;
          retreat_waypoints.push_back(target_pose1);

          target_pose1.position.z += 0.03;
          retreat_waypoints.push_back(target_pose1);

          moveit_msgs::msg::RobotTrajectory trajectory_retreat;

          fraction = move_group_arm.computeCartesianPath(
              retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

          move_group_arm.execute(trajectory_retreat);
        }
        void placeObject() {
          // Place

          RCLCPP_INFO(LOGGER, "Rotating Arm");

          current_state_arm = move_group_arm.getCurrentState(10);
          current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                                     joint_group_positions_arm);

          joint_group_positions_arm[0] = 1.57; // Shoulder Pan

          move_group_arm.setJointValueTarget(joint_group_positions_arm);

          success_arm = (move_group_arm.plan(my_plan_arm) ==
                         moveit::core::MoveItErrorCode::SUCCESS);

          move_group_arm.execute(my_plan_arm);
        }
        void releaseObject() {

          // Open Gripper

          RCLCPP_INFO(LOGGER, "Release Object!");

          move_group_gripper.setNamedTarget("open");

          success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                             moveit::core::MoveItErrorCode::SUCCESS);

          move_group_gripper.execute(my_plan_gripper);
        }
        void goal_response_callback(
            const GoalHandleFind::SharedPtr &goal_handle) {
          if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
          } else {
            RCLCPP_INFO(this->get_logger(),
                        "Goal accepted by server, waiting for result");
          }
        }

        void feedback_callback(
            GoalHandleFind::SharedPtr,
            const std::shared_ptr<const Find::Feedback> feedback) {
          RCLCPP_INFO(this->get_logger(), "Ignoring feedback...");
        }

        void result_callback(const GoalHandleFind::WrappedResult &result) {
          this->goal_done_ = true;
          switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
          }

          RCLCPP_INFO(this->get_logger(), "Result received");
          for (auto object : result.result->objects) {
            if (object.object.primitives[0].type == 1 &&
                object.object.primitives[0].dimensions[0] < 0.05 &&
                object.object.primitives[0].dimensions[1] < 0.05 &&
                object.object.primitives[0].dimensions[2] < 0.1) {
              RCLCPP_INFO(this->get_logger(), "X: %f",
                          object.object.primitive_poses[0].position.x);
              RCLCPP_INFO(this->get_logger(), "Y: %f",
                          object.object.primitive_poses[0].position.y);
            }
          }
          //}
        }
        rclcpp::Client<manipulation_msgs::action::Perception>::SharedPtr
            perception_client_;
        rclcpp::Subscription<manipulation_msgs::action::Perception_Result>::
            SharedPtr perception_subscriber_;
        moveit::planning_interface::MoveGroupInterface *pick_and_place_group_;
        moveit::planning_interface::MoveGroupInterface *gripper_group_;
        rclcpp_action::Client<Find>::SharedPtr client_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;
        bool goal_done_;
      };

      int main(int argc, char **argv) {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<PickAndPlacePerception>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
      }