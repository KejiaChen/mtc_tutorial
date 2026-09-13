#include <atomic>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_task_constructor_msgs/msg/sub_trajectory.hpp>

using json = nlohmann::json;

namespace {
constexpr char kRequestTopic[] = "/prior_transition/plan_request";
constexpr char kResponseTopic[] = "/prior_transition/plan_response";
constexpr char kTrajectoryTopic[] = "/prior_transition/follower_subtrajectory";
}

class PriorSingleArmPlanner final : public rclcpp::Node {
public:
  explicit PriorSingleArmPlanner(const rclcpp::NodeOptions& options)
  : Node("prior_single_arm_planner", options),
    follower_group_(declare_parameter<std::string>("follower_group", "left_panda_arm")),
    follower_link_(declare_parameter<std::string>("follower_link", "left_panda_hand")),
    default_frame_(declare_parameter<std::string>("default_frame", "world")),
    planning_time_(declare_parameter<double>("planning_time", 10.0)),
    planning_attempts_(declare_parameter<int>("planning_attempts", 5)),
    publish_legacy_topic_(declare_parameter<bool>("publish_legacy_topic", false)),
    // Keep MoveGroupInterface on this node so launch parameters are visible.
    move_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), follower_group_) {
    response_publisher_ = create_publisher<std_msgs::msg::String>(kResponseTopic, 10);
    trajectory_publisher_ = create_publisher<moveit_task_constructor_msgs::msg::SubTrajectory>(
      kTrajectoryTopic, rclcpp::QoS(1).transient_local());
    if (publish_legacy_topic_) {
      legacy_trajectory_publisher_ = create_publisher<moveit_task_constructor_msgs::msg::SubTrajectory>(
        "/mtc_sub_trajectory", rclcpp::QoS(1).transient_local());
    }

    move_group_.setPoseReferenceFrame(default_frame_);
    move_group_.setPlanningTime(planning_time_);
    move_group_.setNumPlanningAttempts(planning_attempts_);
    move_group_.setMaxVelocityScalingFactor(declare_parameter<double>("velocity_scaling", 0.05));
    move_group_.setMaxAccelerationScalingFactor(declare_parameter<double>("acceleration_scaling", 0.05));
    request_subscription_ = create_subscription<std_msgs::msg::String>(
      kRequestTopic, 10, std::bind(&PriorSingleArmPlanner::requestCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Listening on %s (group=%s, link=%s)", kRequestTopic,
                follower_group_.c_str(), follower_link_.c_str());
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

  geometry_msgs::msg::Pose parsePose(const json& request) const {
    const auto& pose = request.at("follower_pose");
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

  void requestCallback(const std_msgs::msg::String::SharedPtr message) {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    json request;
    try {
      request = json::parse(message->data);
      const std::string task_id = request.value("task_id", "prior_transition");
      if (request.value("frame_id", default_frame_) != default_frame_) {
        throw std::runtime_error("only frame_id='" + default_frame_ + "' is supported");
      }

      auto start_state = move_group_.getCurrentState(2.0);
      if (!start_state) {
        // The real prior runner sends both arm joint vectors in the request.
        // This also supports mock controllers that publish /joint_states with
        // a zero timestamp, which MoveIt correctly rejects as stale.
        if (!request.contains("follower_joints") || !request.contains("leader_joints")) {
          throw std::runtime_error(
            "MoveIt did not provide a current robot state; include leader_joints and follower_joints in the request");
        }
        start_state = std::make_shared<moveit::core::RobotState>(move_group_.getRobotModel());
        start_state->setToDefaultValues();
        RCLCPP_WARN(get_logger(), "Using request joint state because /joint_states is stale or unavailable");
      }
      if (request.contains("follower_joints")) {
        setJointVector(*start_state, follower_group_, readVector(request, "follower_joints", 7));
      }
      if (request.contains("leader_joints")) {
        setJointVector(*start_state, "right_panda_arm", readVector(request, "leader_joints", 7));
      }
      start_state->update();
      move_group_.setStartState(*start_state);
      move_group_.clearPoseTargets();
      move_group_.setPoseTarget(parsePose(request), follower_link_);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto result = move_group_.plan(plan);
      if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS ||
          plan.trajectory_.joint_trajectory.points.empty()) {
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

  std::string follower_group_;
  std::string follower_link_;
  std::string default_frame_;
  double planning_time_;
  int planning_attempts_;
  bool publish_legacy_topic_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  std::mutex plan_mutex_;
  std::atomic<uint32_t> next_trajectory_id_{1};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_publisher_;
  rclcpp::Publisher<moveit_task_constructor_msgs::msg::SubTrajectory>::SharedPtr trajectory_publisher_;
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
