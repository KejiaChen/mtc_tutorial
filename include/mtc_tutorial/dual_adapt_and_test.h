#ifndef TRAJ_ADAPT_TEST_NODE_H
#define TRAJ_ADAPT_TEST_NODE_H

#include <rclcpp/rclcpp.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <mtc_tutorial/dual_mtc_routing.h>

#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <algorithm>

class TrajAdaptTestNode
{
public:
  explicit TrajAdaptTestNode(const rclcpp::NodeOptions& options);
  ~TrajAdaptTestNode() = default;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface() { return node_->get_node_base_interface(); }
  rclcpp::Node::SharedPtr getNode() { return node_; }

  moveit::planning_interface::MoveGroupInterface& getMoveGroup() { return move_group_; }
  // moveit_visual_tools::MoveItVisualTools& getVisualTools() { return visual_tools_; }

  // Main entry point for this test node: read → build dual traj → stretch → dump
  void runFromFiles(const std::string& leader_joint_txt,
                    const std::string& follower_joint_txt,
                    const std::string& leader_tcp_txt,
                    const std::string& follower_tcp_txt,
                    const std::vector<double>& stretch_distance,
                    double ik_timeout,
                    const std::filesystem::path& dump_dir);

  template <typename T>
  bool getROSParam(const std::string& name, T& value) {
    // returns false if parameter not set or wrong type
    return node_->get_parameter(name, value);
  }

  // If you want a "get or default" version:
  template <typename T>
  T getROSParamOr(const std::string& name, const T& default_value) {
    T v = default_value;
    node_->get_parameter(name, v);  // leaves v as default_value if unset
    return v;
  }
  // void loadCustomScene(const std::string &path);

  // Robot group names
  std::string lead_arm_group_name;
  std::string lead_hand_group_name;
  std::string lead_hand_frame;
  std::string lead_base_frame;

  std::string follow_arm_group_name;
  std::string follow_hand_group_name;
  std::string follow_hand_frame;
  std::string follow_base_frame;

  std::string dual_arm_group_name;

private:
  // --- ROS/MoveIt ---
  rclcpp::Node::SharedPtr node_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;

  moveit::planning_interface::MoveGroupInterface move_group_;
  // moveit_visual_tools::MoveItVisualTools visual_tools_;

  // --- file parsing ---
  struct JointTxtData {
    std::vector<double> t;                       // seconds
    std::vector<std::vector<double>> q;          // [M][7]
  };

  struct TcpTxtData {
    std::vector<double> t;                       // seconds
    std::vector<Eigen::Matrix4d> T;              // [M]
  };

  static JointTxtData readJointTxt(const std::string& path, size_t dof_expected = 7);
  static TcpTxtData   readTcpTxt(const std::string& path);

  // Build a dual-arm RobotTrajectory waypoint-by-waypoint
  robot_trajectory::RobotTrajectory buildDualRobotTrajectory(const JointTxtData& lead,
                                                            const JointTxtData& follow) const;

  // Your stretcher, rewritten as a free-standing method in this class
  void stretchRobotTrajectoryInPlace(robot_trajectory::RobotTrajectory& robot_trajectory,
                                    double extension_distance,
                                    double ik_timeout);

  // Loads parameters for group/link/frame names and transforms
  void initializeGroups();
  void initializeTransforms(double default_franka_flange_to_tcp_z,
                            double sensone_height,
                            double extend_finger_length);

  Eigen::Isometry3d lead_hand_to_tcp_transform_  = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d follow_hand_to_tcp_transform_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d lead_flange_to_tcp_transform_  = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d follow_flange_to_tcp_transform_ = Eigen::Isometry3d::Identity();

  std::vector<std::string> lead_franka_joint_names_ = {"right_panda_joint1", "right_panda_joint2", "right_panda_joint3", "right_panda_joint4", "right_panda_joint5", "right_panda_joint6", "right_panda_joint7"};
  std::vector<std::string> follow_franka_joint_names_ = {"left_panda_joint1", "left_panda_joint2", "left_panda_joint3", "left_panda_joint4", "left_panda_joint5", "left_panda_joint6", "left_panda_joint7"};
};

#endif // TRAJ_ADAPT_TEST_NODE_H
