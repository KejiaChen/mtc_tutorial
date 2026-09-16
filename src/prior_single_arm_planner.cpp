#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_task_constructor_msgs/msg/sub_trajectory.hpp>

using json = nlohmann::json;

namespace {
constexpr char kRequestTopic[] = "/prior_transition/plan_request";
constexpr char kResponseTopic[] = "/prior_transition/plan_response";
constexpr char kSyncRequestTopic[] = "/prior_transition/sync_state_request";
constexpr char kSyncResponseTopic[] = "/prior_transition/sync_state_response";
constexpr char kExecuteRequestTopic[] = "/prior_transition/execute_request";
constexpr char kExecuteResponseTopic[] = "/prior_transition/execute_response";
constexpr char kLeaderMoveRequestTopic[] = "/prior_transition/leader_move_request";
constexpr char kLeaderMoveResponseTopic[] = "/prior_transition/leader_move_response";
constexpr char kLeaderExecuteRequestTopic[] = "/prior_transition/leader_execute_request";
constexpr char kLeaderExecuteResponseTopic[] = "/prior_transition/leader_execute_response";
constexpr char kFollowerFkRequestTopic[] = "/prior_transition/follower_fk_request";
constexpr char kFollowerFkResponseTopic[] = "/prior_transition/follower_fk_response";
constexpr char kTrajectoryTopic[] = "/prior_transition/follower_subtrajectory";
constexpr char kLeaderTrajectoryTopic[] = "/prior_transition/leader_subtrajectory";
constexpr char kGoalMarkerTopic[] = "/prior_transition/follower_goal_marker";
}

class PriorSingleArmPlanner final : public rclcpp::Node {
public:
  explicit PriorSingleArmPlanner(const rclcpp::NodeOptions& options)
  : Node("prior_single_arm_planner", options),
    follower_group_(declare_parameter<std::string>("follower_group", "left_panda_arm")),
    follower_link_(declare_parameter<std::string>("follower_link", "left_panda_hand")),
    leader_group_name_(declare_parameter<std::string>("leader_group", "right_panda_arm")),
    leader_link_(declare_parameter<std::string>("leader_link", "right_panda_hand")),
    default_frame_(declare_parameter<std::string>("default_frame", "world")),
    planning_time_(declare_parameter<double>("planning_time", 10.0)),
    planning_attempts_(declare_parameter<int>("planning_attempts", 5)),
    publish_legacy_topic_(declare_parameter<bool>("publish_legacy_topic", false)),
    alter_finger_left_(declare_parameter<bool>("alter_finger_left", false)),
    extended_finger_length_(declare_parameter<double>("extended_finger_length", 0.01)),
    tcp_offset_x_(declare_parameter<double>("tcp_offset_x", 0.0)),
    tcp_offset_y_(declare_parameter<double>("tcp_offset_y", 0.0)),
    tcp_offset_z_(declare_parameter<double>("tcp_offset_z", 0.1034)),
    // No dedicated calibration exists yet for the leader tool; default to the
    // same offset as the follower since both arms share the same gripper
    // geometry convention in the Python routing helpers. Revisit once the
    // real MIOS O_T_EE can be compared against this FK.
    leader_tcp_offset_x_(declare_parameter<double>("leader_tcp_offset_x", 0.0)),
    leader_tcp_offset_y_(declare_parameter<double>("leader_tcp_offset_y", 0.0)),
    leader_tcp_offset_z_(declare_parameter<double>("leader_tcp_offset_z", 0.1034)),
    open_follower_gripper_(declare_parameter<bool>("open_follower_gripper", true)),
    open_gripper_width_(declare_parameter<double>("open_gripper_width", 0.035)),
    sync_joint_state_(declare_parameter<bool>("sync_joint_state", true)),
    sync_via_move_group_(declare_parameter<bool>("sync_via_move_group", true)),
    // Keep MoveGroupInterface on this node so launch parameters are visible.
    move_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), follower_group_),
    gripper_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), "left_hand"),
    sync_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), "dual_arm"),
    leader_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), leader_group_name_) {
    response_publisher_ = create_publisher<std_msgs::msg::String>(kResponseTopic, 10);
    sync_response_publisher_ = create_publisher<std_msgs::msg::String>(kSyncResponseTopic, 10);
    execute_response_publisher_ = create_publisher<std_msgs::msg::String>(kExecuteResponseTopic, 10);
    leader_move_response_publisher_ = create_publisher<std_msgs::msg::String>(kLeaderMoveResponseTopic, 10);
    leader_execute_response_publisher_ = create_publisher<std_msgs::msg::String>(kLeaderExecuteResponseTopic, 10);
    follower_fk_response_publisher_ = create_publisher<std_msgs::msg::String>(kFollowerFkResponseTopic, 10);
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10).reliable());
    goal_marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      kGoalMarkerTopic, rclcpp::QoS(1).transient_local());
    trajectory_publisher_ = create_publisher<moveit_task_constructor_msgs::msg::SubTrajectory>(
      kTrajectoryTopic, rclcpp::QoS(1).transient_local());
    leader_trajectory_publisher_ = create_publisher<moveit_task_constructor_msgs::msg::SubTrajectory>(
      kLeaderTrajectoryTopic, rclcpp::QoS(1).transient_local());
    if (publish_legacy_topic_) {
      legacy_trajectory_publisher_ = create_publisher<moveit_task_constructor_msgs::msg::SubTrajectory>(
        "/mtc_sub_trajectory", rclcpp::QoS(1).transient_local());
    }

    const double velocity_scaling = declare_parameter<double>("velocity_scaling", 0.1);
    const double acceleration_scaling = declare_parameter<double>("acceleration_scaling", 0.1);
    move_group_.setPoseReferenceFrame(default_frame_);
    move_group_.setPlanningTime(planning_time_);
    move_group_.setNumPlanningAttempts(planning_attempts_);
    move_group_.setMaxVelocityScalingFactor(velocity_scaling);
    move_group_.setMaxAccelerationScalingFactor(acceleration_scaling);
    sync_group_.setPlanningTime(planning_time_);
    sync_group_.setNumPlanningAttempts(planning_attempts_);
    sync_group_.setMaxVelocityScalingFactor(0.1);
    sync_group_.setMaxAccelerationScalingFactor(0.1);
    leader_group_.setPoseReferenceFrame(default_frame_);
    leader_group_.setPlanningTime(planning_time_);
    leader_group_.setNumPlanningAttempts(planning_attempts_);
    leader_group_.setMaxVelocityScalingFactor(velocity_scaling);
    leader_group_.setMaxAccelerationScalingFactor(acceleration_scaling);
    auto node_alias = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {});
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_alias, "robot_description");
    if (planning_scene_monitor_->getPlanningScene()) {
      planning_scene_monitor_->requestPlanningSceneState();
      planning_scene_monitor_->startSceneMonitor();
    } else {
      RCLCPP_WARN(get_logger(), "PlanningSceneMonitor could not initialize; collision diagnostics disabled");
    }
    request_subscription_ = create_subscription<std_msgs::msg::String>(
      kRequestTopic, 10, std::bind(&PriorSingleArmPlanner::requestCallback, this, std::placeholders::_1));
    sync_request_subscription_ = create_subscription<std_msgs::msg::String>(
      kSyncRequestTopic, 10, std::bind(&PriorSingleArmPlanner::syncRequestCallback, this, std::placeholders::_1));
    execute_request_subscription_ = create_subscription<std_msgs::msg::String>(
      kExecuteRequestTopic, 10, std::bind(&PriorSingleArmPlanner::executeCallback, this, std::placeholders::_1));
    leader_move_request_subscription_ = create_subscription<std_msgs::msg::String>(
      kLeaderMoveRequestTopic, 10,
      std::bind(&PriorSingleArmPlanner::leaderPlanCallback, this, std::placeholders::_1));
    leader_execute_request_subscription_ = create_subscription<std_msgs::msg::String>(
      kLeaderExecuteRequestTopic, 10,
      std::bind(&PriorSingleArmPlanner::leaderExecuteCallback, this, std::placeholders::_1));
    follower_fk_request_subscription_ = create_subscription<std_msgs::msg::String>(
      kFollowerFkRequestTopic, 10,
      std::bind(&PriorSingleArmPlanner::followerFkCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Listening on %s (group=%s, link=%s)", kRequestTopic,
                follower_group_.c_str(), follower_link_.c_str());
    RCLCPP_INFO(get_logger(), "Listening on %s (group=%s, link=%s)", kLeaderMoveRequestTopic,
                leader_group_name_.c_str(), leader_link_.c_str());
  }

private:
  static std::vector<double> readVector(const json& object, const char* key, std::size_t length) {
    if (!object.contains(key) || !object.at(key).is_array()) {
      throw std::runtime_error(std::string("missing array: ") + key);
    }
    auto values = object.at(key).get<std::vector<double>>();
    if (values.size() != length) {
      throw std::runtime_error(std::string("wrong length for ") + key);
    }
    return values;
  }

  static void setJointVector(moveit::core::RobotState& state, const std::string& group,
                             const std::vector<double>& values) {
    const auto* joint_group = state.getJointModelGroup(group);
    if (!joint_group || values.size() != joint_group->getVariableCount()) {
      throw std::runtime_error("joint vector does not match group " + group);
    }
    state.setJointGroupPositions(joint_group, values);
  }

  // Reads the last waypoint of a planned trajectory, in the given group's
  // canonical joint order (looked up by name rather than assumed position
  // index). Used to report the joints a plan actually ends at without
  // depending on a live /joint_states subscription (MoveGroupInterface's
  // getCurrentState() needs one and is not reliably warm right after an
  // execute() in this node).
  static std::vector<double> finalJointPositions(
      const moveit_msgs::msg::RobotTrajectory& trajectory,
      const std::vector<std::string>& group_joint_names) {
    if (trajectory.joint_trajectory.points.empty()) {
      throw std::runtime_error("trajectory has no waypoints");
    }
    const auto& joint_names = trajectory.joint_trajectory.joint_names;
    const auto& last_point = trajectory.joint_trajectory.points.back();
    std::vector<double> values;
    values.reserve(group_joint_names.size());
    for (const auto& name : group_joint_names) {
      const auto it = std::find(joint_names.begin(), joint_names.end(), name);
      if (it == joint_names.end() ||
          static_cast<std::size_t>(std::distance(joint_names.begin(), it)) >= last_point.positions.size()) {
        throw std::runtime_error("planned trajectory is missing joint " + name);
      }
      values.push_back(last_point.positions[std::distance(joint_names.begin(), it)]);
    }
    return values;
  }

  geometry_msgs::msg::Pose parsePose(const json& request, const char* key = "follower_pose") const {
    const auto& pose = request.at(key);
    const auto position = readVector(pose, "position", 3);
    const auto orientation = readVector(pose, "orientation", 4);
    geometry_msgs::msg::Pose result;
    result.position.x = position[0];
    result.position.y = position[1];
    result.position.z = position[2];
    result.orientation.x = orientation[0];
    result.orientation.y = orientation[1];
    result.orientation.z = orientation[2];
    result.orientation.w = orientation[3];
    return result;
  }

  void publishResponse(const json& response) {
    std_msgs::msg::String message;
    message.data = response.dump();
    response_publisher_->publish(message);
  }

  void publishExecuteResponse(const json& response) {
    std_msgs::msg::String message;
    message.data = response.dump();
    execute_response_publisher_->publish(message);
  }

  void publishSyncResponse(const json& response) {
    std_msgs::msg::String message;
    message.data = response.dump();
    sync_response_publisher_->publish(message);
  }

  void publishSynchronizedJointState(const moveit::core::RobotState& state, int repeats = 1) {
    if (!sync_joint_state_ || !joint_state_publisher_) {
      return;
    }
    sensor_msgs::msg::JointState message;
    message.name = state.getVariableNames();
    const double* positions = state.getVariablePositions();
    message.position.assign(positions, positions + message.name.size());
    for (int i = 0; i < repeats; ++i) {
      message.header.stamp = now();
      joint_state_publisher_->publish(message);
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    RCLCPP_INFO(get_logger(), "Published synchronized request state to /joint_states (%zu joints, repeats=%d)",
                message.name.size(), repeats);
  }

  void logDualArmTargetDiagnostics(const moveit::core::RobotState& state,
                                   const std::vector<double>& target) const {
    const auto model = sync_group_.getRobotModel();
    const auto* group = model ? model->getJointModelGroup("dual_arm") : nullptr;
    if (!group) {
      RCLCPP_ERROR(get_logger(), "Diagnostics: dual_arm JointModelGroup is missing");
      return;
    }
    const auto& names = group->getVariableNames();
    RCLCPP_ERROR(get_logger(), "Diagnostics: dual_arm target count=%zu model count=%zu",
                 target.size(), names.size());
    bool target_ok = true;
    for (std::size_t i = 0; i < names.size() && i < target.size(); ++i) {
      const auto& bounds = model->getVariableBounds(names[i]);
      const bool below = bounds.position_bounded_ && target[i] < bounds.min_position_;
      const bool above = bounds.position_bounded_ && target[i] > bounds.max_position_;
      const bool ok = !below && !above;
      target_ok = target_ok && ok;
      RCLCPP_ERROR(get_logger(),
                   "Diagnostics joint[%zu] %s target=%.9f bounds=[%.9f, %.9f] bounded=%s status=%s",
                   i, names[i].c_str(), target[i], bounds.min_position_, bounds.max_position_,
                   bounds.position_bounded_ ? "true" : "false", ok ? "OK" : "OUT_OF_BOUNDS");
    }
    RCLCPP_ERROR(get_logger(), "Diagnostics target bounds result: %s; state dual_arm bounds: %s; full state bounds: %s",
                 target_ok ? "OK" : "OUT_OF_BOUNDS",
                 state.satisfiesBounds(group) ? "OK" : "INVALID",
                 state.satisfiesBounds() ? "OK" : "INVALID");
  }

  void logCollisionDiagnostics(moveit::core::RobotState start_state,
                               moveit::core::RobotState* goal_state) const {
    if (!planning_scene_monitor_) {
      return;
    }
    const auto scene = planning_scene_monitor_->getPlanningScene();
    if (!scene) {
      RCLCPP_WARN(get_logger(), "Collision diagnostics: planning scene unavailable");
      return;
    }
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 100;
    request.verbose = false;
    collision_detection::CollisionResult start_result;
    scene->checkCollision(request, start_result, start_state);
    RCLCPP_ERROR(get_logger(), "Collision diagnostics start state: %s (%zu contact pairs)",
                 start_result.collision ? "COLLIDING" : "free", start_result.contacts.size());
    for (const auto& contact : start_result.contacts) {
      RCLCPP_ERROR(get_logger(), "  start contact: %s <-> %s",
                   contact.first.first.c_str(), contact.first.second.c_str());
    }
    if (goal_state) {
      collision_detection::CollisionResult goal_result;
      scene->checkCollision(request, goal_result, *goal_state);
      RCLCPP_ERROR(get_logger(), "Collision diagnostics IK goal state: %s (%zu contact pairs)",
                   goal_result.collision ? "COLLIDING" : "free", goal_result.contacts.size());
      for (const auto& contact : goal_result.contacts) {
        RCLCPP_ERROR(get_logger(), "  goal contact: %s <-> %s",
                     contact.first.first.c_str(), contact.first.second.c_str());
      }
    }
  }

  moveit::core::RobotStatePtr makeRequestState(const json& request) {
    if (!request.contains("follower_joints") || !request.contains("leader_joints")) {
      throw std::runtime_error("request must include leader_joints and follower_joints");
    }
    auto state = std::make_shared<moveit::core::RobotState>(move_group_.getRobotModel());
    state->setToDefaultValues();
    setJointVector(*state, follower_group_, readVector(request, "follower_joints", 7));
    setJointVector(*state, "right_panda_arm", readVector(request, "leader_joints", 7));
    if (open_follower_gripper_) {
      state->setVariablePosition("left_panda_finger_joint1", open_gripper_width_);
      state->setVariablePosition("left_panda_finger_joint2", open_gripper_width_);
    }
    state->update();
    return state;
  }

  void syncRequestCallback(const std_msgs::msg::String::SharedPtr message) {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    json request;
    try {
      request = json::parse(message->data);
      const std::string task_id = request.value("task_id", "prior_transition");
      auto state = makeRequestState(request);
      last_start_state_ = std::make_shared<moveit::core::RobotState>(*state);
      publishSynchronizedJointState(*state, 6);
      if (sync_via_move_group_) {
        RCLCPP_INFO(get_logger(), "Synchronizing RViz through MoveIt dual_arm joint goal");
        sync_group_.clearPoseTargets();
        std::vector<double> dual_arm_positions;
        state->copyJointGroupPositions("dual_arm", dual_arm_positions);
        RCLCPP_INFO(get_logger(), "Dual-arm synchronization target has %zu joints", dual_arm_positions.size());
        if (!sync_group_.setJointValueTarget(dual_arm_positions)) {
          logDualArmTargetDiagnostics(*state, dual_arm_positions);
          throw std::runtime_error("MoveIt rejected synchronized dual_arm arm-joint goal");
        }
        const auto arm_result = sync_group_.move();
        RCLCPP_INFO(get_logger(), "MoveIt dual_arm synchronization result=%d", arm_result.val);
        if (arm_result != moveit::core::MoveItErrorCode::SUCCESS) {
          throw std::runtime_error("MoveIt failed to execute synchronized dual_arm joint goal");
        }
        if (open_follower_gripper_) {
          if (!gripper_group_.setNamedTarget("open")) {
            throw std::runtime_error("MoveIt rejected synchronized left_hand open goal");
          }
          const auto gripper_result = gripper_group_.move();
          RCLCPP_INFO(get_logger(), "MoveIt left_hand synchronization result=%d", gripper_result.val);
          if (gripper_result != moveit::core::MoveItErrorCode::SUCCESS) {
            throw std::runtime_error("MoveIt failed to execute synchronized left_hand open goal");
          }
        }
      }
      publishSyncResponse({{"task_id", task_id}, {"ok", true}, {"stage", "state_synchronized"},
                           {"joint_count", state->getVariableNames().size()},
                           {"rviz_synchronized", sync_via_move_group_}});
      RCLCPP_INFO(get_logger(), "State synchronization complete for task %s", task_id.c_str());
    } catch (const std::exception& error) {
      publishSyncResponse({{"task_id", request.value("task_id", "")}, {"ok", false},
                           {"stage", "state_synchronization"}, {"error", error.what()}});
      RCLCPP_ERROR(get_logger(), "State synchronization failed: %s", error.what());
    }
  }

  void publishGoalMarker(const geometry_msgs::msg::Pose& pose, const std::string& task_id) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = default_frame_;
    marker.header.stamp = now();
    marker.ns = "prior_follower_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = 0.025;
    marker.scale.y = 0.025;
    marker.scale.z = 0.025;
    marker.color.r = 0.1f;
    marker.color.g = 0.95f;
    marker.color.b = 0.2f;
    marker.color.a = 1.0f;
    marker.text = task_id;
    goal_marker_publisher_->publish(marker);
    RCLCPP_INFO(get_logger(), "Published follower goal marker for task %s at (%.4f, %.4f, %.4f)",
                task_id.c_str(), pose.position.x, pose.position.y, pose.position.z);
  }

  Eigen::Vector3d effectiveTcpOffset() const {
    return Eigen::Vector3d(
      tcp_offset_x_, tcp_offset_y_,
      tcp_offset_z_ + (alter_finger_left_ ? 0.5 * extended_finger_length_ : 0.0));
  }

  Eigen::Vector3d leaderEffectiveTcpOffset() const {
    return Eigen::Vector3d(leader_tcp_offset_x_, leader_tcp_offset_y_, leader_tcp_offset_z_);
  }

  // Requests contain the physical TCP pose. MoveGroupInterface expects the
  // pose of the selected link, so convert TCP -> hand using the same offset
  // convention as dual_mtc_routing.cpp.
  geometry_msgs::msg::Pose tcpPoseToHandPose(const geometry_msgs::msg::Pose& tcp_pose,
                                              const Eigen::Vector3d& tcp_offset) const {
    const double norm = std::sqrt(
      tcp_pose.orientation.x * tcp_pose.orientation.x +
      tcp_pose.orientation.y * tcp_pose.orientation.y +
      tcp_pose.orientation.z * tcp_pose.orientation.z +
      tcp_pose.orientation.w * tcp_pose.orientation.w);
    if (norm < 1e-9) {
      throw std::runtime_error("pose has a zero-length quaternion");
    }

    Eigen::Quaterniond tcp_orientation(
      tcp_pose.orientation.w / norm,
      tcp_pose.orientation.x / norm,
      tcp_pose.orientation.y / norm,
      tcp_pose.orientation.z / norm);
    const Eigen::Vector3d tcp_position(
      tcp_pose.position.x, tcp_pose.position.y, tcp_pose.position.z);
    const Eigen::Vector3d hand_position = tcp_position - tcp_orientation * tcp_offset;

    geometry_msgs::msg::Pose hand_pose = tcp_pose;
    hand_pose.position.x = hand_position.x();
    hand_pose.position.y = hand_position.y();
    hand_pose.position.z = hand_position.z();
    hand_pose.orientation.x = tcp_orientation.x();
    hand_pose.orientation.y = tcp_orientation.y();
    hand_pose.orientation.z = tcp_orientation.z();
    hand_pose.orientation.w = tcp_orientation.w();
    return hand_pose;
  }

  void logPose(const char* label, const geometry_msgs::msg::Pose& pose) const {
    RCLCPP_INFO(get_logger(),
                "%s position=(%.5f, %.5f, %.5f), orientation=(%.5f, %.5f, %.5f, %.5f)",
                label, pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  }

  void requestCallback(const std_msgs::msg::String::SharedPtr message) {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    json request;
    try {
      request = json::parse(message->data);
      const std::string task_id = request.value("task_id", "prior_transition");
      if (request.value("frame_id", default_frame_) != default_frame_) {
        throw std::runtime_error("only frame_id='" + default_frame_ + "' is supported");
      }

      // Synchronization is a separate callback/request. Planning only
      // consumes the same authoritative request state; it does not publish
      // /joint_states or race MoveIt's current-state monitor.
      auto start_state = makeRequestState(request);
      RCLCPP_INFO(get_logger(), "Using synchronized request state as planning start state");
      Eigen::Isometry3d hand_to_tcp = Eigen::Isometry3d::Identity();
      hand_to_tcp.translation() = effectiveTcpOffset();
      const auto start_hand_transform = start_state->getGlobalLinkTransform(follower_link_);
      const auto start_tcp_transform = start_hand_transform * hand_to_tcp;
      RCLCPP_INFO(get_logger(), "Start state bounds valid: %s", start_state->satisfiesBounds() ? "yes" : "no");
      RCLCPP_INFO(get_logger(), "Start %s FK hand=(%.5f, %.5f, %.5f), TCP=(%.5f, %.5f, %.5f)",
                  follower_link_.c_str(),
                  start_hand_transform.translation().x(), start_hand_transform.translation().y(),
                  start_hand_transform.translation().z(),
                  start_tcp_transform.translation().x(), start_tcp_transform.translation().y(),
                  start_tcp_transform.translation().z());
      move_group_.setStartState(*start_state);
      move_group_.clearPoseTargets();
      const auto follower_pose = parsePose(request);
      const auto hand_pose = tcpPoseToHandPose(follower_pose, effectiveTcpOffset());
      logPose("Requested follower TCP goal", follower_pose);
      logPose("Converted follower hand goal", hand_pose);
      const auto offset = effectiveTcpOffset();
      RCLCPP_INFO(get_logger(), "Using hand->TCP offset in hand frame=(%.5f, %.5f, %.5f)%s",
                  offset.x(), offset.y(), offset.z(), alter_finger_left_ ? " (altered finger)" : "");

      // This isolates geometric reachability from OMPL/collision failures.
      // It is only a diagnostic copy; the planning start state is unchanged.
      auto ik_state = *start_state;
      const auto* follower_jmg = ik_state.getJointModelGroup(follower_group_);
      if (!follower_jmg) {
        RCLCPP_ERROR(get_logger(), "Cannot run IK diagnostic: group '%s' is missing from the robot model",
                     follower_group_.c_str());
      } else {
        const bool ik_ok = ik_state.setFromIK(follower_jmg, hand_pose, follower_link_, 0.5);
        RCLCPP_INFO(get_logger(), "IK diagnostic for converted hand goal: %s", ik_ok ? "success" : "failure");
        if (ik_ok) {
          std::vector<double> ik_joints;
          ik_state.copyJointGroupPositions(follower_jmg, ik_joints);
          RCLCPP_INFO(get_logger(),
                      "IK solution joints: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                      ik_joints[0], ik_joints[1], ik_joints[2], ik_joints[3],
                      ik_joints[4], ik_joints[5], ik_joints[6]);
        }
        logCollisionDiagnostics(*start_state, ik_ok ? &ik_state : nullptr);
      }
      publishGoalMarker(follower_pose, task_id);
      const bool target_set = move_group_.setPoseTarget(hand_pose, follower_link_);
      RCLCPP_INFO(get_logger(), "MoveIt setPoseTarget(%s) returned %s",
                  follower_link_.c_str(), target_set ? "true" : "false");
      if (!target_set) {
        throw std::runtime_error("MoveIt rejected converted follower hand target");
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto result = move_group_.plan(plan);
      if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS ||
          plan.trajectory_.joint_trajectory.points.empty()) {
        RCLCPP_ERROR(get_logger(),
                     "MoveIt planning failed: error_code=%d, joint_names=%zu, points=%zu",
                     result.val, plan.trajectory_.joint_trajectory.joint_names.size(),
                     plan.trajectory_.joint_trajectory.points.size());
        publishResponse({{"task_id", task_id}, {"ok", false}, {"stage", "plan"},
                         {"error", "MoveIt planning failed"}});
        move_group_.clearPoseTargets();
        return;
      }

      const auto trajectory_id = next_trajectory_id_.fetch_add(1);
      moveit_task_constructor_msgs::msg::SubTrajectory subtrajectory;
      subtrajectory.info.id = trajectory_id;
      subtrajectory.info.stage_id = request.value("stage_id", 3u);
      subtrajectory.info.planner_id = task_id;
      subtrajectory.info.comment = "prior single-arm follower plan";
      subtrajectory.trajectory = plan.trajectory_;
      last_plan_ = plan;
      last_plan_task_id_ = task_id;
      last_plan_trajectory_id_ = trajectory_id;
      has_last_plan_ = true;
      trajectory_publisher_->publish(subtrajectory);
      if (legacy_trajectory_publisher_) {
        legacy_trajectory_publisher_->publish(subtrajectory);
      }
      publishResponse({{"task_id", task_id}, {"ok", true}, {"stage", "planned"},
                      {"stage_id", subtrajectory.info.stage_id}, {"trajectory_id", trajectory_id},
                      {"trajectory_topic", kTrajectoryTopic},
                      {"joint_count", subtrajectory.trajectory.joint_trajectory.joint_names.size()},
                      {"waypoint_count", subtrajectory.trajectory.joint_trajectory.points.size()}});
      move_group_.clearPoseTargets();
    } catch (const std::exception& error) {
      publishResponse({{"task_id", request.value("task_id", "")}, {"ok", false},
                       {"stage", "request"}, {"error", error.what()}});
      RCLCPP_ERROR(get_logger(), "Prior transition request failed: %s", error.what());
      move_group_.clearPoseTargets();
    }
  }

  void executeCallback(const std_msgs::msg::String::SharedPtr message) {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    json request;
    try {
      request = json::parse(message->data);
      const std::string task_id = request.value("task_id", "");
      const uint32_t trajectory_id = request.value("trajectory_id", 0u);
      if (!has_last_plan_) {
        throw std::runtime_error("no planned trajectory is available");
      }
      if (task_id != last_plan_task_id_) {
        throw std::runtime_error("execute task_id does not match the latest plan");
      }
      if (trajectory_id != 0u && trajectory_id != last_plan_trajectory_id_) {
        throw std::runtime_error("execute trajectory_id does not match the latest plan");
      }

      RCLCPP_INFO(get_logger(), "Executing accepted follower plan task=%s trajectory_id=%u",
                  task_id.c_str(), last_plan_trajectory_id_);
      if (open_follower_gripper_) {
        if (!gripper_group_.setNamedTarget("open")) {
          throw std::runtime_error("MoveIt rejected left_hand named target 'open'");
        }
        if (last_start_state_) {
          gripper_group_.setStartState(*last_start_state_);
        }
        const auto gripper_result = gripper_group_.move();
        RCLCPP_INFO(get_logger(), "MoveIt follower gripper open result=%d", gripper_result.val);
        if (gripper_result != moveit::core::MoveItErrorCode::SUCCESS) {
          publishExecuteResponse({{"task_id", task_id}, {"ok", false}, {"stage", "gripper"},
                                  {"trajectory_id", last_plan_trajectory_id_},
                                  {"error_code", gripper_result.val},
                                  {"error", "failed to open follower gripper"}});
          return;
        }
      }
      const auto result = move_group_.execute(last_plan_);
      RCLCPP_INFO(get_logger(), "MoveIt follower arm execute result=%d", result.val);
      json execute_response = {{"task_id", task_id},
                                {"ok", result == moveit::core::MoveItErrorCode::SUCCESS},
                                {"stage", "executed"},
                                {"trajectory_id", last_plan_trajectory_id_},
                                {"error_code", result.val}};
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        // So a no-hardware caller (e.g. --moveit-only) can track the
        // follower's actual post-execution joint state instead of reusing a
        // stale value across subsequent plan requests.
        execute_response["follower_joints"] = finalJointPositions(last_plan_.trajectory_, move_group_.getJointNames());
      }
      publishExecuteResponse(execute_response);
    } catch (const std::exception& error) {
      const std::string task_id = request.value("task_id", "");
      publishExecuteResponse({{"task_id", task_id}, {"ok", false}, {"stage", "execute"},
                              {"error", error.what()}});
      RCLCPP_ERROR(get_logger(), "Prior execution request failed: %s", error.what());
    }
  }

  void publishLeaderMoveResponse(const json& response) {
    std_msgs::msg::String message;
    message.data = response.dump();
    leader_move_response_publisher_->publish(message);
  }

  void publishLeaderExecuteResponse(const json& response) {
    std_msgs::msg::String message;
    message.data = response.dump();
    leader_execute_response_publisher_->publish(message);
  }

  // Plans a single Cartesian move of the leader arm alone (no execution),
  // mirroring requestCallback()'s plan-only step for the follower: the
  // caller reviews the candidate (y/r/q) before leaderExecuteCallback()
  // actually moves the arm, same accept/replan/quit loop as the follower.
  void leaderPlanCallback(const std_msgs::msg::String::SharedPtr message) {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    json request;
    try {
      request = json::parse(message->data);
      const std::string task_id = request.value("task_id", "prior_transition");
      if (request.value("frame_id", default_frame_) != default_frame_) {
        throw std::runtime_error("only frame_id='" + default_frame_ + "' is supported");
      }
      const auto leader_pose = parsePose(request, "leader_pose");
      const auto hand_pose = tcpPoseToHandPose(leader_pose, leaderEffectiveTcpOffset());
      logPose("Requested leader TCP goal", leader_pose);
      logPose("Converted leader hand goal", hand_pose);

      leader_group_.clearPoseTargets();
      const bool target_set = leader_group_.setPoseTarget(hand_pose, leader_link_);
      RCLCPP_INFO(get_logger(), "MoveIt setPoseTarget(%s) returned %s",
                  leader_link_.c_str(), target_set ? "true" : "false");
      if (!target_set) {
        throw std::runtime_error("MoveIt rejected leader hand target");
      }

      moveit::planning_interface::MoveGroupInterface::Plan leader_plan;
      const auto plan_result = leader_group_.plan(leader_plan);
      leader_group_.clearPoseTargets();
      if (plan_result != moveit::core::MoveItErrorCode::SUCCESS ||
          leader_plan.trajectory_.joint_trajectory.points.empty()) {
        RCLCPP_ERROR(get_logger(), "MoveIt leader planning failed: error_code=%d", plan_result.val);
        publishLeaderMoveResponse({{"task_id", task_id}, {"ok", false}, {"stage", "leader_plan"},
                                   {"error_code", plan_result.val},
                                   {"error", "MoveIt failed to plan the leader move"}});
        return;
      }
      // Publish the planned leader trajectory so prior_subtrajectory_subscriber
      // can forward it to the real leader MIOS server too -- shape_control_
      // clip_fixing_prior.py now always plans the leader move through MoveIt
      // (collision-checked) instead of a raw move_cart_pose(), and only
      // additionally plays this trajectory on real hardware when not running
      // --moveit-only.
      const auto leader_trajectory_id = next_trajectory_id_.fetch_add(1);
      moveit_task_constructor_msgs::msg::SubTrajectory leader_subtrajectory;
      leader_subtrajectory.info.id = leader_trajectory_id;
      leader_subtrajectory.info.stage_id = request.value("stage_id", 3u);
      leader_subtrajectory.info.planner_id = task_id;
      leader_subtrajectory.info.comment = "prior single-arm leader plan";
      leader_subtrajectory.trajectory = leader_plan.trajectory_;
      last_leader_plan_ = leader_plan;
      last_leader_plan_task_id_ = task_id;
      last_leader_plan_trajectory_id_ = leader_trajectory_id;
      has_last_leader_plan_ = true;
      leader_trajectory_publisher_->publish(leader_subtrajectory);

      publishLeaderMoveResponse({{"task_id", task_id}, {"ok", true}, {"stage", "leader_planned"},
                                 {"trajectory_id", leader_trajectory_id},
                                 {"waypoint_count", leader_plan.trajectory_.joint_trajectory.points.size()}});
      RCLCPP_INFO(get_logger(), "Leader plan complete for task %s", task_id.c_str());
    } catch (const std::exception& error) {
      publishLeaderMoveResponse({{"task_id", request.value("task_id", "")}, {"ok", false},
                                 {"stage", "leader_plan"}, {"error", error.what()}});
      RCLCPP_ERROR(get_logger(), "Leader plan request failed: %s", error.what());
    }
  }

  // Executes the last accepted leader plan, mirroring executeCallback() for
  // the follower (no gripper step, since the leader has none here).
  void leaderExecuteCallback(const std_msgs::msg::String::SharedPtr message) {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    json request;
    try {
      request = json::parse(message->data);
      const std::string task_id = request.value("task_id", "");
      const uint32_t trajectory_id = request.value("trajectory_id", 0u);
      if (!has_last_leader_plan_) {
        throw std::runtime_error("no planned leader trajectory is available");
      }
      if (task_id != last_leader_plan_task_id_) {
        throw std::runtime_error("leader execute task_id does not match the latest plan");
      }
      if (trajectory_id != 0u && trajectory_id != last_leader_plan_trajectory_id_) {
        throw std::runtime_error("leader execute trajectory_id does not match the latest plan");
      }

      RCLCPP_INFO(get_logger(), "Executing accepted leader plan task=%s trajectory_id=%u",
                  task_id.c_str(), last_leader_plan_trajectory_id_);
      const auto result = leader_group_.execute(last_leader_plan_);
      RCLCPP_INFO(get_logger(), "MoveIt leader arm execute result=%d", result.val);
      json execute_response = {{"task_id", task_id},
                                {"ok", result == moveit::core::MoveItErrorCode::SUCCESS},
                                {"stage", "leader_executed"},
                                {"trajectory_id", last_leader_plan_trajectory_id_},
                                {"error_code", result.val}};
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        execute_response["leader_joints"] =
          finalJointPositions(last_leader_plan_.trajectory_, leader_group_.getJointNames());
      }
      publishLeaderExecuteResponse(execute_response);
    } catch (const std::exception& error) {
      const std::string task_id = request.value("task_id", "");
      publishLeaderExecuteResponse({{"task_id", task_id}, {"ok", false}, {"stage", "leader_execute"},
                                    {"error", error.what()}});
      RCLCPP_ERROR(get_logger(), "Leader execute request failed: %s", error.what());
    }
  }

  void publishFollowerFkResponse(const json& response) {
    std_msgs::msg::String message;
    message.data = response.dump();
    follower_fk_response_publisher_->publish(message);
  }

  // Reports the follower TCP orientation FK-computed from a given joint
  // state, with no planning or execution. A no-hardware caller (e.g.
  // --moveit-only) needs this to know the follower's actual current
  // orientation -- e.g. lead_arm.get_current_state()'s live TCP orientation
  // is not available here -- before it can compute a grasp orientation
  // relative to it.
  void followerFkCallback(const std_msgs::msg::String::SharedPtr message) {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    json request;
    try {
      request = json::parse(message->data);
      const std::string task_id = request.value("task_id", "prior_transition");
      const auto follower_joints = readVector(request, "follower_joints", 7);

      auto state = std::make_shared<moveit::core::RobotState>(move_group_.getRobotModel());
      state->setToDefaultValues();
      setJointVector(*state, follower_group_, follower_joints);
      state->update();

      // tcpPoseToHandPose() only translates (along the orientation's local
      // Z) between the hand link and the TCP frame; the orientation itself
      // is identical, so the hand link's FK orientation IS the TCP
      // orientation -- no additional rotation to apply here.
      const auto hand_transform = state->getGlobalLinkTransform(follower_link_);
      const Eigen::Quaterniond orientation(hand_transform.rotation());
      publishFollowerFkResponse({{"task_id", task_id}, {"ok", true}, {"stage", "follower_fk"},
                                 {"orientation", {orientation.x(), orientation.y(),
                                                   orientation.z(), orientation.w()}}});
    } catch (const std::exception& error) {
      publishFollowerFkResponse({{"task_id", request.value("task_id", "")}, {"ok", false},
                                 {"stage", "follower_fk"}, {"error", error.what()}});
      RCLCPP_ERROR(get_logger(), "Follower FK request failed: %s", error.what());
    }
  }

  std::string follower_group_;
  std::string follower_link_;
  std::string leader_group_name_;
  std::string leader_link_;
  std::string default_frame_;
  double planning_time_;
  int planning_attempts_;
  bool publish_legacy_topic_;
  bool alter_finger_left_;
  double extended_finger_length_;
  double tcp_offset_x_;
  double tcp_offset_y_;
  double tcp_offset_z_;
  double leader_tcp_offset_x_;
  double leader_tcp_offset_y_;
  double leader_tcp_offset_z_;
  bool open_follower_gripper_;
  double open_gripper_width_;
  bool sync_joint_state_;
  bool sync_via_move_group_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::MoveGroupInterface gripper_group_;
  moveit::planning_interface::MoveGroupInterface sync_group_;
  moveit::planning_interface::MoveGroupInterface leader_group_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  std::mutex plan_mutex_;
  std::atomic<uint32_t> next_trajectory_id_{1};
  moveit::planning_interface::MoveGroupInterface::Plan last_plan_;
  moveit::core::RobotStatePtr last_start_state_;
  std::string last_plan_task_id_;
  uint32_t last_plan_trajectory_id_{0};
  bool has_last_plan_{false};
  moveit::planning_interface::MoveGroupInterface::Plan last_leader_plan_;
  std::string last_leader_plan_task_id_;
  uint32_t last_leader_plan_trajectory_id_{0};
  bool has_last_leader_plan_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sync_request_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr execute_request_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr leader_move_request_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr leader_execute_request_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr follower_fk_request_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sync_response_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr execute_response_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr leader_move_response_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr leader_execute_response_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr follower_fk_response_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_publisher_;
  rclcpp::Publisher<moveit_task_constructor_msgs::msg::SubTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<moveit_task_constructor_msgs::msg::SubTrajectory>::SharedPtr leader_trajectory_publisher_;
  rclcpp::Publisher<moveit_task_constructor_msgs::msg::SubTrajectory>::SharedPtr legacy_trajectory_publisher_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<PriorSingleArmPlanner>(rclcpp::NodeOptions());
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
