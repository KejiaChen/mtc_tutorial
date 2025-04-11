#ifndef MTC_TASK_NODE_H
#define MTC_TASK_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_task_constructor_msgs/msg/sub_trajectory.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif


namespace mtc = moveit::task_constructor;

using GroupPoseDict = std::map<std::string, geometry_msgs::msg::PoseStamped>;
using GroupPoseMatrixDict = std::map<std::string, Eigen::Isometry3d>;
using GroupStringDict = std::map<std::string, std::string>;
using GroupVectorDict = std::map<std::string, std::vector<double>>;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);
  ~MTCTaskNode();

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  // public getter
  rclcpp::Node::SharedPtr getNode();
  moveit::planning_interface::MoveGroupInterface& getMoveGroup();
  moveit_visual_tools::MoveItVisualTools& getVisualTools();

  // Compose an MTC task from a series of stages.
  mtc::Task createTask(std::string& goal_frame_name, bool if_use_dual, bool if_split_plan, bool if_cartesian_connect, bool if_approach);
  mtc::Task createPostTask(std::string& goal_frame_name, bool if_use_dual, bool if_split_plan, bool if_cartesian_connect, bool if_approach);
  mtc::Task createTestWaypointTask(std::string& goal_frame_name, bool if_use_dual, bool if_split_plan, bool if_cartesian_connect, bool if_approach);

  // publish mtc sub_trajectory
  void publishSolutionSubTraj(const moveit_task_constructor_msgs::msg::Solution& msg);

  void doTask(std::string& goal_clip_id, bool execute, bool plan_for_dual, bool cartesian_connect, bool approach,
              std::function<mtc::Task(std::string&, bool, bool, bool, bool)> createTaskFn);
  void updatePlanningScene();

  void setSelectOrientation(bool select_orientation) { select_orientation_ = select_orientation; }

  // void loadCustomScene(const std::string &path);

  // Robot group names
  std::string lead_arm_group_name;
  std::string lead_hand_group_name;
  std::string lead_hand_frame;

  std::string follow_arm_group_name;
  std::string follow_hand_group_name;
  std::string follow_hand_frame;

  std::string dual_arm_group_name;

  // Planners
  std::shared_ptr<mtc::solvers::PipelinePlanner> lead_sampling_planner;
  std::shared_ptr<mtc::solvers::JointInterpolationPlanner> lead_interpolation_planner;
  std::shared_ptr<mtc::solvers::CartesianPath> lead_cartesian_planner;

  std::shared_ptr<mtc::solvers::PipelinePlanner> follow_sampling_planner;
  std::shared_ptr<mtc::solvers::JointInterpolationPlanner> follow_interpolation_planner;
  std::shared_ptr<mtc::solvers::CartesianPath> follow_cartesian_planner;

private:
  geometry_msgs::msg::PoseStamped getPoseTransform(const geometry_msgs::msg::PoseStamped& pose, const std::string& target_frame);
  moveit_msgs::msg::Constraints createBoxConstraints(const std::string& link_name, geometry_msgs::msg::PoseStamped& goal_pose);

  geometry_msgs::msg::PoseStamped createClipGoal(const std::string& goal_frame, const std::vector<double>& goal_translation_vector);
  std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped> assignClipGoal(const std::string& goal_frame, 
          const std::vector<double>& goal_vector_1, const std::vector<double>& goal_vector_2);

  // // Compose an MTC task from a series of stages.
  // mtc::Task createTask(std::string& goal_frame_name, bool if_use_dual, bool if_split_plan);
  // mtc::Task createPostTask(std::string& goal_frame_name, bool if_use_dual, bool if_split_plan);
  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;

  // interfaces
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;

  // TF2 components
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // publish mtc sub_trajectory
	rclcpp::Publisher<moveit_task_constructor_msgs::msg::SubTrajectory>::SharedPtr subtrajectory_publisher_;

  // Joint state subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState::SharedPtr current_joint_state_;
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void periodicUpdate(); // Background thread function
  
  // Threading for periodic updates
  std::thread update_thread_;
  std::atomic<bool> stop_thread_;

  // Select Goal Orientation
  bool select_orientation_ = false;

  // Transform from flange to TCP
  Eigen::Isometry3d lead_flange_to_tcp_transform_;
  Eigen::Isometry3d follow_flange_to_tcp_transform_;

  // Helper methods for internal setup
  void initializeGroups();
  void initializePlanners();
};

#endif  // MTC_TASK_NODE_H
