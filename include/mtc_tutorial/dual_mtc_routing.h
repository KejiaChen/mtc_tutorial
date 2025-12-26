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
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection/collision_matrix.h>


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

inline bool dumpTrajectoryTXT(const robot_trajectory::RobotTrajectory& traj,
                              const std::string& joint_filename,
                              const std::string& tcp_filename,
                              const std::string& group_name = "",
                              const Eigen::Isometry3d &offset = Eigen::Isometry3d::Identity(),
                              std::string base_link_name = "base_link",
                              char delim = ' ',          // use ' ' for space-separated
                              int precision = 6,
                              double fallback_dt = -1.0) // e.g., 0.05 if times are all zero
{
  const std::string group = group_name.empty() ? traj.getGroupName() : group_name;
  const auto* jmg = traj.getRobotModel()->getJointModelGroup(group);
  if (!jmg) return false;

  std::vector<const moveit::core::LinkModel*> tips;
  if (!jmg->getEndEffectorTips(tips) || tips.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("RobotTrajectory"), "Unable to get end effector tips from jmg");
    return false;
  }

  const auto& names = jmg->getVariableNames();
  const size_t N = names.size();
  const size_t M = traj.getWayPointCount();
  if (M == 0 || N == 0) return false;

  std::filesystem::create_directories(std::filesystem::path(joint_filename).parent_path());
  std::ofstream joint_out(joint_filename);
  if (!joint_out) return false;
  joint_out.setf(std::ios::fixed, std::ios::floatfield);
  joint_out << std::setprecision(precision);

  std::filesystem::create_directories(std::filesystem::path(tcp_filename).parent_path());
  std::ofstream tcp_out(tcp_filename);
  if (!tcp_out) return false;
  tcp_out.setf(std::ios::fixed, std::ios::floatfield);
  tcp_out << std::setprecision(precision);

  // // Header
  // out << "# time";
  // for (auto& n : names) out << delim << "q/" << n;
  // for (auto& n : names) out << delim << "dq/" << n;
  // out << "\n";

  // Gather data
  std::vector<double> T(M, 0.0);
  std::vector<std::vector<double>> Q(M, std::vector<double>(N, 0.0));
  std::vector<std::vector<double>> dQ(M, std::vector<double>(N, 0.0));
  std::vector<Eigen::Isometry3d> TCP_pose_in_world(M, Eigen::Isometry3d::Identity());
  std::vector<Eigen::Isometry3d> TCP_pose_in_base(M, Eigen::Isometry3d::Identity());

  bool any_velocity = false;
  Eigen::Isometry3d robot_base_pose = traj.getWayPoint(0).getGlobalLinkTransform(base_link_name);
  for (size_t i = 0; i < M; ++i) {
    const auto& s = traj.getWayPoint(i);
    T[i] = traj.getWayPointDurationFromStart(i);
    s.copyJointGroupPositions(jmg, Q[i]);
    s.copyJointGroupVelocities(jmg, dQ[i]); // zeros if not set

    for (const moveit::core::LinkModel* ee_parent_link : tips){
      // pose in world frame for publishing
      Eigen::Isometry3d ee_pose_in_world= s.getGlobalLinkTransform(ee_parent_link);
      // Apply the translation in the z-axis
      Eigen::Isometry3d tcp_pose_in_world = ee_pose_in_world*offset;
      TCP_pose_in_world[i] = tcp_pose_in_world;

      // ee_pose in robot base frame for storage 
      Eigen::Isometry3d ee_pose_in_base = robot_base_pose.inverse()*ee_pose_in_world;
      // Apply the translation in the z-axiss
      Eigen::Isometry3d tcp_pose_in_base = ee_pose_in_base*offset;
      TCP_pose_in_base[i] = tcp_pose_in_base;
    }

    for (double v : dQ[i]) if (std::abs(v) > 1e-12) { any_velocity = true; break; }
  }

  // Dump rows
  for (size_t i = 0; i < M; ++i) {
    joint_out << T[i];
    for (size_t j = 0; j < N; ++j) joint_out << delim << Q[i][j];
    for (size_t j = 0; j < N; ++j) joint_out << delim << dQ[i][j];
    joint_out << "\n";
  }

  // Dump TCP poses
  for (size_t i = 0; i < M; ++i) {
    tcp_out << T[i];
    for (int col = 0; col < 4; ++col) {
      for (int row = 0; row < 4; ++row) {
        tcp_out << delim << TCP_pose_in_base[i](row, col);
      }
    }
    tcp_out << "\n";  // Newline for the next matrix
  }

  return true;
}

inline std::filesystem::path nextIndexedFile(const std::filesystem::path& dir,
                                            const std::string& prefix,
                                            const std::string& ext = ".txt",
                                            int width = 3,
                                            int start_index = 1)
{
  std::filesystem::create_directories(dir);
  int max_idx = start_index - 1;

  for (const auto& entry : std::filesystem::directory_iterator(dir)) {
    if (!entry.is_regular_file()) continue;
    const auto name = entry.path().filename().string();

    // check prefix_
    if (name.size() < prefix.size() + 1 + ext.size()) continue;
    if (name.compare(0, prefix.size(), prefix) != 0) continue;
    if (name[prefix.size()] != '_') continue;

    // check suffix .ext
    if (name.compare(name.size() - ext.size(), ext.size(), ext) != 0) continue;

    // digits in the middle
    const auto digits = name.substr(prefix.size() + 1,
                                    name.size() - prefix.size() - 1 - ext.size());
    if (digits.empty() || !std::all_of(digits.begin(), digits.end(),
                                      [](unsigned char c){ return std::isdigit(c); }))
      continue;

    int idx = std::stoi(digits);
    if (idx > max_idx) max_idx = idx;
  }

  const int next = std::max(start_index, max_idx + 1);
  std::ostringstream oss;
  oss << prefix << "_" << std::setw(width) << std::setfill('0') << next << ext;
  return dir / oss.str();
}

inline bool dumpTrajectoryTXTIndexed(const robot_trajectory::RobotTrajectory& traj,
                                    const std::string& prefix,
                                    const std::string& group_name = "",
                                    const std::filesystem::path& dir = std::filesystem::current_path(),
                                    const Eigen::Isometry3d &offset = Eigen::Isometry3d::Identity(),
                                      std::string base_link_name = "base_link",
                                    char delim = ' ', int precision = 6,
                                    double fallback_dt = 0.05, // pick something reasonable
                                    int width = 3, int start_index = 1)
{
  // const auto joint_path = nextIndexedFile(dir, prefix + "_joint", ".txt", width, start_index);
  // const auto tcp_path = nextIndexedFile(dir, prefix + "_tcp", ".txt", width, start_index);

  std::ostringstream joint_oss;
  joint_oss << prefix << "_joint.txt";
  const auto joint_path = dir / joint_oss.str();
  std::ostringstream tcp_oss;
  tcp_oss << prefix << "_tcp.txt";
  const auto tcp_path = dir / tcp_oss.str();

  RCLCPP_INFO(rclcpp::get_logger("RobotTrajectory"),
            "Dumping trajectory to %s", joint_path.string().c_str());
  return dumpTrajectoryTXT(traj, joint_path.string(), tcp_path.string(), group_name, offset, base_link_name, delim, precision, fallback_dt);
}

void printACM(const collision_detection::AllowedCollisionMatrix& acm)
{
  rclcpp::Logger ACM_logger = rclcpp::get_logger("ACM");
  RCLCPP_INFO(ACM_logger, "----- AllowedCollisionMatrix dump -----");

  std::vector<std::string> entries;
  acm.getAllEntryNames(entries);

  // 1. Print entry names
  RCLCPP_INFO(ACM_logger, "ACM contains %zu entries:", entries.size());
  for (const auto& name : entries)
    RCLCPP_INFO_STREAM(ACM_logger, "  • " << name);

  // 2. Pairwise entries (explicit rules)
  RCLCPP_INFO(ACM_logger, "Pairwise collision rules (explicit entries):");
  for (size_t i = 0; i < entries.size(); ++i) {
    for (size_t j = i; j < entries.size(); ++j) {
      const auto& a = entries[i];
      const auto& b = entries[j];

      collision_detection::AllowedCollision::Type type;
      bool has_entry = acm.getEntry(a, b, type);  // <--- 3-arg version

      if (!has_entry) {
        // No explicit rule; skip or mark as unspecified
        continue;
        // or:
        // RCLCPP_INFO_STREAM(LOGGER,
        //   "  (" << a << ", " << b << ") : UNSPECIFIED");
      } else {
        std::string type_str;
        switch (type) {
          case collision_detection::AllowedCollision::ALWAYS:
            type_str = "ALWAYS (ALLOWED)";
            break;
          case collision_detection::AllowedCollision::NEVER:
            type_str = "NEVER (NOT ALLOWED)";
            break;
          case collision_detection::AllowedCollision::CONDITIONAL:
            type_str = "CONDITIONAL";
            break;
          default:
            type_str = "UNKNOWN";
            break;
        }

        RCLCPP_INFO_STREAM(ACM_logger,
          "  (" << a << ", " << b << ") : " << type_str);
      }
    }
  }

  RCLCPP_INFO(ACM_logger, "-----------------------------------------");
}

// Get JSON node at clip_clearance[ path[0] ][ path[1] ] ...
static nlohmann::json& getJsonNode(nlohmann::json& root,
                                   const std::vector<std::string>& path)
{
  nlohmann::json* node = &root;
  for (const auto& key : path)
    node = &((*node)[key]);  // creates object nodes as needed
  return *node;
}

// Get JSON node at clip_clearance[ path... ][leaf_key]
static nlohmann::json& getJsonNode(nlohmann::json& root,
                                   const std::vector<std::string>& path,
                                   const std::string& leaf_key)
{
  nlohmann::json* node = &root;
  for (const auto& key : path)
    node = &((*node)[key]);
  return (*node)[leaf_key];  // creates leaf object as needed
}

static std::unordered_map<std::string, std::set<int>> parseStageTargets(const std::vector<std::string>& targets)
{
  std::unordered_map<std::string, std::set<int>> stage_to_indices;

  for (const auto& entry : targets) {
    const std::string pat = "_subtraj_";
    auto pos = entry.rfind(pat);

    if (pos == std::string::npos) {
      // pure stage name: all subtrajectories for that stage
      stage_to_indices[entry] = {};
      continue;
    }

    std::string stage_name = entry.substr(0, pos);
    std::string idx_str    = entry.substr(pos + pat.size());

    try {
      int idx = std::stoi(idx_str);
      stage_to_indices[stage_name].insert(idx);
    } catch (const std::exception& e) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("RobotTrajectory"),
          "evaluateClearance: cannot parse target entry '"
          << entry << "': " << e.what());
    }
  }

  return stage_to_indices;
}

// returns normalized or zero
static Eigen::Vector3d safeNormalize(const Eigen::Vector3d& v, double eps = 1e-9)
{
  const double n = v.norm();
  if (n < eps) return Eigen::Vector3d::Zero();
  return v / n;
}

static inline void assignQuat(geometry_msgs::msg::Pose& pose,
                            const Eigen::Quaterniond& q)
{
  Eigen::Quaterniond nq = q.normalized();
  pose.orientation.w = nq.w();
  pose.orientation.x = nq.x();
  pose.orientation.y = nq.y();
  pose.orientation.z = nq.z();
}

double computeMinClearance(
    const planning_scene::PlanningScenePtr& scene,   // note: non-const now
    const robot_trajectory::RobotTrajectory& traj,
    const collision_detection::AllowedCollisionMatrix& acm)
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.distance = true;

  double min_d = std::numeric_limits<double>::infinity();

  // Use the scene's current state, which has the attached cable
  moveit::core::RobotState& scene_state = scene->getCurrentStateNonConst();

  for (size_t i = 0; i < traj.getWayPointCount(); ++i) {
    const moveit::core::RobotState& wp_state = traj.getWayPoint(i);

    // Copy joint values from the waypoint into the scene state
    scene_state.setVariablePositions(wp_state.getVariablePositions());
    scene_state.update();  // update transforms

    res.clear();
    scene->checkCollision(req, res, scene_state, acm);

    if (res.collision)
      min_d = 0.0;  // by convention; res.distance is usually 0 in collision
    else
      min_d = std::min(min_d, res.distance);
  }

  return min_d;
}

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
  mtc::Task createTask(std::string& start_frame_name, std::string& goal_frame_name, bool if_use_dual, bool if_split_plan, bool if_cartesian_connect, bool if_approach, bool clip_added_from_blender);
  mtc::Task createReverseTask(std::string& start_frame_name, std::string& goal_frame_name, bool if_use_dual, bool if_split_plan, bool if_cartesian_connect, bool if_approach);
  mtc::Task createPostTask(std::string& start_frame_name, std::string& goal_frame_name, bool if_use_dual, bool if_split_plan, bool if_cartesian_connect, bool if_approach, bool clip_added_from_blender);
  mtc::Task createTestWaypointTask(std::string& goal_frame_name, bool if_use_dual, bool if_split_plan, bool if_cartesian_connect, bool if_approach);
  mtc::Task createHomingTask(std::string& start_frame_name, std::string& goal_frame_name, bool if_use_dual, bool if_split_plan, bool if_cartesian_connect, bool if_approach, bool clip_added_from_blender);

  // synchronization with real-world
  void udpReceiverSync(const std::string& host, int port,
                    std::vector<double>& joint_positions,
                    std::mutex& joint_positions_mutex,
                    std::condition_variable& joint_positions_condition_variable,
                    std::vector<double>& ee_pose,
                    std::mutex& ee_pose_mutex);

  mtc::Task createGoalJointTask(std::string arm_group_name, 
                                std::string hand_group_name, 
                                std::string hand_frame,
                                std::mutex& joint_positions_mutex,
                                std::vector<double>& joint_positions,
                                std::vector<std::string>& joint_names,
                                std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner);
  
 bool doSyncTask(std::string arm_group_name, 
                std::string hand_group_name, 
                std::string hand_frame,
                std::mutex& joint_positions_mutex,
                std::vector<double>& joint_positions,
                std::vector<std::string>& joint_names,
                std::condition_variable& joint_positions_condition_variable,
                std::atomic<bool>& joint_data_received_flag,
                std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner
                );

  void syncwithRealWorld();

  // publish mtc sub_trajectory
  void publishSolutionSubTraj(std::string goal_clip_name, const moveit_task_constructor_msgs::msg::Solution& msg);

  void doTask(std::string& start_clip_id, std::string& goal_clip_id, bool execute, bool plan_for_dual, bool split, bool cartesian_connect,
            bool approach, bool clip_added_from_blender, std::function<mtc::Task(std::string&, std::string&, bool, bool, bool, bool, bool)> createTaskFn);
  void updatePlanningScene();

  void setSelectOrientation(bool select_orientation) { select_orientation_ = select_orientation; }

  void saveClearanceToJson(const std::string& file_path)
  {
    std::ofstream ofs(file_path);
    ofs << clearance_results_;
    ofs.close();
  }

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

  std::vector<double> getClipSizeFromScene(const std::string& clip_id_prefix = "clip")
  {
    moveit::planning_interface::PlanningSceneInterface psi;

    // 1) Get all object names
    std::vector<std::string> names = psi.getKnownObjectNames();

    // 2) Filter for clips (e.g., "clip_1", "clip_foo", etc.)
    std::vector<std::string> clip_ids;
    for (const auto& name : names)
    {
      if (name.rfind(clip_id_prefix, 0) == 0) // starts with "clip"
        clip_ids.push_back(name);
    }

    if (clip_ids.empty())
    {
      throw std::runtime_error("No objects with prefix '" + clip_id_prefix + "' found in planning scene.");
    }

    // 3) Retrieve their CollisionObjects
    std::map<std::string, moveit_msgs::msg::CollisionObject> objects_map = psi.getObjects(clip_ids);

    if (objects_map.empty())
    {
      throw std::runtime_error("getObjects() returned empty map for clips.");
    }

    // For simplicity: just take the first clip
    const auto& co = objects_map.begin()->second;

    if (co.primitives.empty())
    {
      throw std::runtime_error("Clip collision object has no primitives (maybe it is a mesh?).");
    }

    const auto& prim = co.primitives[0];
    if (prim.type != shape_msgs::msg::SolidPrimitive::BOX)
    {
      throw std::runtime_error("Clip is not a BOX primitive.");
    }

    if (prim.dimensions.size() < 3)
    {
      throw std::runtime_error("Clip BOX primitive has fewer than 3 dimensions.");
    }

    double dx = prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X];
    double dy = prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
    double dz = prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z];

    return {dx, dy, dz};
  }


  void updateClipOffsets(std::vector<double> start_clip_size= {0.04, 0.04, 0.06}, 
                        std::vector<double> goal_clip_size= {0.04, 0.04, 0.06},
                        bool clip_added_from_blender = false)
  {
    // U-type Clip
    // std::vector<double> insertion_vector_ = {0, 0, -(insertion_offset_magnitude_)}; // Caution: this doens't set insertion distance
    // std::vector<double> leader_pre_insert_offset_ = {0, -(clip_size[1]/2+hold_y_offset_), clip_size[2]/2-2*hold_z_offset_};
    // std::vector<double> follower_pre_insert_offset_ = {0, clip_size[1]/2+hold_y_offset_, clip_size[2]/2-2*hold_z_offset_}; 
    // std::vector<double> leader_grasp_offset_magnitude_ = {0, -(clip_size[1]/2+grasp_leader_offset_magnitude_), clip_size[2]/2-2*hold_z_offset_};
    // std::vector<double> follower_grasp_offset_magnitude_ = {0, -(clip_size[1]/2+grasp_follower_offset_magnitude_), clip_size[2]/2-2*hold_z_offset_};

    // // // C-type Clip
    if (!clip_added_from_blender){
      insertion_vector_ = {-insertion_offset_magnitude_, 0, 0};
      // insertion offset in goal clip frame
      leader_pre_insert_offset_ = {(-insertion_offset_magnitude_+goal_clip_size[0]/2), -(goal_clip_size[1]/2+hold_y_offset_), goal_clip_size[2]/2+hold_z_offset_};
      follower_pre_insert_offset_ = {(-insertion_offset_magnitude_+goal_clip_size[0]/2), goal_clip_size[1]/2+hold_y_offset_, goal_clip_size[2]/2+hold_z_offset_}; 
      // grasp offset in start clip frame
      leader_grasp_offset_magnitude_ = {0, (start_clip_size[1]/2+grasp_leader_offset_magnitude_), start_clip_size[2]/2+hold_z_offset_};
      follower_grasp_offset_magnitude_ = {0, (start_clip_size[1]/2+grasp_follower_offset_magnitude_), start_clip_size[2]/2+hold_z_offset_};
    }else{
      insertion_vector_ = {-insertion_offset_magnitude_, 0, 0};
      // insertion offset in goal clip frame
      leader_pre_insert_offset_ = {(-insertion_offset_magnitude_+goal_clip_size[0]/2), -(goal_clip_size[1]/2+hold_y_offset_), -goal_clip_size[2]/2+hold_z_offset_};
      follower_pre_insert_offset_ = {(-insertion_offset_magnitude_+goal_clip_size[0]/2), goal_clip_size[1]/2+hold_y_offset_, -goal_clip_size[2]/2+hold_z_offset_}; 
      // grasp offset in start clip frame
      leader_grasp_offset_magnitude_ = {0, (start_clip_size[1]/2+grasp_leader_offset_magnitude_), -start_clip_size[2]/2+hold_z_offset_};
      follower_grasp_offset_magnitude_ = {0, (start_clip_size[1]/2+grasp_follower_offset_magnitude_), -start_clip_size[2]/2+hold_z_offset_};
    }
  }


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

  // Planners
  std::shared_ptr<mtc::solvers::PipelinePlanner> lead_sampling_planner;
  std::shared_ptr<mtc::solvers::JointInterpolationPlanner> lead_interpolation_planner;
  std::shared_ptr<mtc::solvers::CartesianPath> lead_cartesian_planner;

  std::shared_ptr<mtc::solvers::PipelinePlanner> lead_chomp_planner;

  std::shared_ptr<mtc::solvers::PipelinePlanner> follow_sampling_planner;
  std::shared_ptr<mtc::solvers::JointInterpolationPlanner> follow_interpolation_planner;
  std::shared_ptr<mtc::solvers::CartesianPath> follow_cartesian_planner;

  std::shared_ptr<mtc::solvers::PipelinePlanner> follow_chomp_planner;

private:
  geometry_msgs::msg::PoseStamped getPoseTransform(const geometry_msgs::msg::PoseStamped& pose, const std::string& target_frame);
  Eigen::Isometry3d getTransformIsometry(const std::string& source_frame, const std::string& target_frame);

  moveit_msgs::msg::Constraints createBoxConstraints(const std::string& link_name, geometry_msgs::msg::PoseStamped& goal_pose, double x_offset, double y_offset, double z_offset);

  // Returns +1 if v aligns more with +Y_clip, -1 if more with -Y_clip.
  static int signAlongClipY(const Eigen::Isometry3d& T_w_c,
                          const Eigen::Vector3d& v_conn_w)
  {
    // Express connection vector in CLIP frame (rotation only).
    Eigen::Vector3d v_c = T_w_c.linear().transpose() * v_conn_w; // R_c^T * v_w
    const double dot_y = v_c.normalized().dot(Eigen::Vector3d::UnitY());
    return (dot_y >= 0.0) ? +1 : -1;
  }

  void attachCollisionCable(planning_scene::PlanningScenePtr scene,
                                              const std::string& id, 
                                              double length,
                                              double radius,
                                              Eigen::Vector3d vec_in_world,
                                              const std::string& attach_link, 
                                              std::vector<std::string> touch_links,
                                              bool enable_cable_collision)
  {
      moveit_msgs::msg::AttachedCollisionObject attach_msg;
      attach_msg.link_name = attach_link;
      attach_msg.object.header.frame_id = attach_link;
      attach_msg.object.id = id;

      // Add geometry of cable
      shape_msgs::msg::SolidPrimitive prim;
      prim.type = prim.CYLINDER;
      prim.dimensions = {length, radius}; // height (along local Z), radius

      // Step 1: Create pose in TCP frame (cylinder lying along +X, end at origin)
      Eigen::Isometry3d cylinder_pose_tcp = Eigen::Isometry3d::Identity();
      // Convert direction from world frame into attach_link tcp frame
      Eigen::Isometry3d world_to_hand = scene->getFrameTransform(attach_link).inverse();
      Eigen::Vector3d vec_in_hand = world_to_hand.linear() * vec_in_world.normalized();
      // Calculate the rotation of cylinder Z-axis to align with the direction
      Eigen::Quaterniond align_quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), vec_in_hand);
      cylinder_pose_tcp.linear() = align_quat.toRotationMatrix();
      // Rotate cylinder Z-axis → X-axis using +90° about Y
      // cylinder_pose_tcp.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()).toRotationMatrix();
      // Translate it so one end sits at TCP
      cylinder_pose_tcp.translation() =  vec_in_hand.normalized() * (0.5*length);
      // Step 2: Transform to `left_panda_hand` frame
      Eigen::Isometry3d cylinder_pose_in_hand = follow_hand_to_tcp_transform_ * cylinder_pose_tcp;
      // Step 3: Convert to geometry_msgs::Pose
      geometry_msgs::msg::Pose pose_msg = tf2::toMsg(cylinder_pose_in_hand);

      // RCLCPP_INFO_STREAM(LOGGER, "Attach collision object: " << id 
      //                           << " position: " << pose_msg.position.x << ", " << pose_msg.position.y << ", " << pose_msg.position.z 
      //                           << " orientation: " << pose_msg.orientation.x << ", " << pose_msg.orientation.y << ", " << pose_msg.orientation.z << ", " << pose_msg.orientation.w);

      attach_msg.object.primitives.push_back(prim);
      attach_msg.object.primitive_poses.push_back(pose_msg);
      attach_msg.object.operation = moveit_msgs::msg::CollisionObject::ADD;

      // Ignore collision with both grippers
      attach_msg.touch_links = touch_links;

      scene->processAttachedCollisionObjectMsg(attach_msg);

      // add the object but disable cable collision if enable_cable_collision is false
      if (!enable_cable_collision) {
        collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();

        bool allow = true;  // always allowed to collide

        // Let dlo_obj collide with everything by default
        acm.setDefaultEntry(id, allow);
        acm.setEntry(id, allow);
      }

      // visualization
      Eigen::Isometry3d pose_in_world = scene->getFrameTransform(attach_link) * cylinder_pose_in_hand;
      // Convert the pose to a geometry_msgs::Pose for visualization
      geometry_msgs::msg::Pose pose_msg_world = tf2::toMsg(pose_in_world);
      visual_tools_.publishCylinder(pose_msg_world, rviz_visual_tools::ORANGE, length, radius);
      visual_tools_.trigger();

  }

  void evaluateClearance(
    const std::string& task_name,                    
    const std::string& clip_id,
    const std::string& object_id,
    const std::vector<std::string>& target_stages_and_indices);

  void stretchRobotTrajectoryInPlace(
      robot_trajectory::RobotTrajectory& robot_trajectory,
      double extension_distance,
      double ik_timeout)
  {
    const moveit::core::RobotModelConstPtr& model = robot_trajectory.getRobotModel();
    if (!model)
      throw std::runtime_error("RobotTrajectory has null RobotModel");

    const auto* lead_jmg   = model->getJointModelGroup(lead_arm_group_name);
    const auto* follow_jmg = model->getJointModelGroup(follow_arm_group_name);
    if (!lead_jmg || !follow_jmg)
      throw std::runtime_error("JointModelGroup not found");

    const size_t M = robot_trajectory.getWayPointCount();
    if (M == 0)
      return;

    // Build a brand new trajectory with the same model+group
    robot_trajectory::RobotTrajectory new_traj(model, robot_trajectory.getGroupName());

    // Helper: compute dt between waypoints (works across MoveIt variants)
    auto duration_from_prev = [&](size_t i) -> double {
      // waypoint 0 is absolute start
      if (i == 0) return 0.0;

      // Prefer duration-from-previous if available in your version
      // If this doesn't compile in your branch, comment it out and use the fallback below.
      return robot_trajectory.getWayPointDurationFromPrevious(i);

      // Fallback (if needed):
      // const double t_i   = robot_trajectory.getWayPointDurationFromStart(i);
      // const double t_im1 = robot_trajectory.getWayPointDurationFromStart(i - 1);
      // return std::max(0.0, t_i - t_im1);
    };

    for (size_t i = 0; i < M; ++i)
    {
      moveit::core::RobotState state = robot_trajectory.getWayPoint(i); // copy

      // --- FK on the hand frames ---
      const Eigen::Isometry3d T_lead_ee_in_world   = state.getGlobalLinkTransform(lead_hand_frame);
      const Eigen::Isometry3d T_follow_ee_in_world = state.getGlobalLinkTransform(follow_hand_frame);

      // --- TCP frames ---
      const Eigen::Isometry3d T_lead_tcp_in_world   = T_lead_ee_in_world   * lead_hand_to_tcp_transform_;
      const Eigen::Isometry3d T_follow_tcp_in_world = T_follow_ee_in_world * follow_hand_to_tcp_transform_;

      Eigen::Vector3d dir = safeNormalize(T_lead_tcp_in_world.translation() - T_follow_tcp_in_world.translation());

      if (!dir.isZero())
      {
        // --- Stretch TCP ---
        Eigen::Isometry3d T_lead_tcp_des = T_lead_tcp_in_world;
        T_lead_tcp_des.translation() += extension_distance * dir;

        // --- Convert back to desired hand pose (IK target) ---
        const Eigen::Isometry3d T_lead_hand_des =
            T_lead_tcp_des * lead_hand_to_tcp_transform_.inverse();

        // --- Seed from current waypoint (already in 'state') ---
        bool ik_ok = state.setFromIK(
            lead_jmg,
            T_lead_hand_des,
            lead_hand_frame,
            ik_timeout);

        // If IK fails, keep original state (the copy we started with)
        (void)ik_ok;
      }

      // Preserve timing by adding the same dt as original
      const double dt = duration_from_prev(i);
      new_traj.addSuffixWayPoint(state, dt);
    }

    // Replace the original trajectory with the new one
    robot_trajectory = new_traj;
  }

  void generateAndDumpStretchedTargets(
    const std::string& task_name,
    const std::string& clip_id,
    const std::vector<std::string>& target_stages_and_indices,
    double extension_distance,
    double ik_timeout,
    const std::string& dump_prefix);

  std::tuple<int, geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped> assignClipGoalsAlongConnection(const std::string& clip_frame,
                                                                                                                        const std::string& next_clip_frame,
                                                                                                                        const std::vector<double>& leader_grasp_offset,
                                                                                                                        const std::vector<double>& follower_grasp_offset,
                                                                                                                        bool tilt_follower,
                                                                                                                        double follower_tilt_rad,
                                                                                                                        Eigen::Quaterniond& clip2ee_quat);

  geometry_msgs::msg::PoseStamped createClipGoal(const std::string& goal_frame, const std::vector<double>& goal_translation_vector);
  std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped> assignClipGoal(const std::string& goal_frame, 
                                                                                              const std::vector<double>& goal_vector_1, const std::vector<double>& goal_vector_2);

  std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped> assignClipGoalBiDirection(const std::string& goal_frame_name, 
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
  Eigen::Isometry3d lead_hand_to_tcp_transform_;
  Eigen::Isometry3d follow_hand_to_tcp_transform_;

  // Helper methods for internal setup
  void initializeGroups();
  void initializePlanners();
  void initializeTransforms(double default_franka_flange_to_tcp_z,
                          double sensone_height,
                          double extend_finger_length);

  // synchronization variables
  std::thread udp_thread_lead_sync_;
  std::thread udp_thread_follow_sync_;

  // Flags to indicate that joint data has been received
  std::atomic<bool> lead_joint_data_received_{false};
  std::atomic<bool> follow_joint_data_received_{false};
  
  std::mutex lead_joint_positions_mutex_,
              follow_joint_positions_mutex_,
              lead_ee_pose_mutex_,
              follow_ee_pose_mutex_;
  
  std::condition_variable lead_joint_positions_condition_variable_,
                          follow_joint_positions_condition_variable_;

  std::vector<double> lead_joint_positions_;
  std::vector<double> follow_joint_positions_;
  std::vector<double> lead_ee_pose_;
  std::vector<double> follow_ee_pose_;
  std::vector<std::string> lead_franka_joint_names_ = {"right_panda_joint1", "right_panda_joint2", "right_panda_joint3", "right_panda_joint4", "right_panda_joint5", "right_panda_joint6", "right_panda_joint7"};
  std::vector<std::string> follow_franka_joint_names_ = {"left_panda_joint1", "left_panda_joint2", "left_panda_joint3", "left_panda_joint4", "left_panda_joint5", "left_panda_joint6", "left_panda_joint7"};
  std::atomic<bool> sync_udp_running_{true}; // Flag to control the UDP sync thread
  nlohmann::json clearance_results_ = nlohmann::json::object();

  // scene configuration
  double insertion_offset_magnitude_; 
  double grasp_follower_offset_magnitude_;
  double grasp_leader_offset_magnitude_;
  double hold_y_offset_;
  double hold_z_offset_;
  // std::vector<double> leader_pre_insert_offset_ = {-(clip_size[0]/2+hold_x_offset), -clip_size[1]/2, clip_size[2]/2};
  // std::vector<double> follower_pre_insert_offset_ = {clip_size[0]/2+hold_x_offset, -clip_size[1]/2, clip_size[2]/2};

  // U-type Clip
  // std::vector<double> insertion_vector_ = {0, 0, -(insertion_offset_magnitude_)}; // Caution: this doens't set insertion distance
  // std::vector<double> leader_pre_insert_offset_ = {0, -(clip_size[1]/2+hold_y_offset_), clip_size[2]/2-2*hold_z_offset_};
  // std::vector<double> follower_pre_insert_offset_ = {0, clip_size[1]/2+hold_y_offset_, clip_size[2]/2-2*hold_z_offset_}; 
  // std::vector<double> leader_grasp_offset_magnitude_ = {0, -(clip_size[1]/2+grasp_leader_offset_magnitude_), clip_size[2]/2-2*hold_z_offset_};
  // std::vector<double> follower_grasp_offset_magnitude_ = {0, -(clip_size[1]/2+grasp_follower_offset_magnitude_), clip_size[2]/2-2*hold_z_offset_};

  // // C-type Clip
  std::vector<double> insertion_vector_;
  // offset in clip frame
  std::vector<double> leader_pre_insert_offset_;
  std::vector<double> follower_pre_insert_offset_;
  std::vector<double> leader_grasp_offset_magnitude_;
  std::vector<double> follower_grasp_offset_magnitude_;
};

#endif  // MTC_TASK_NODE_H
