#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
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
#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/msg/bool.hpp>
// #include <moveit_msgs>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;
using GroupPoseDict = std::map<std::string, geometry_msgs::msg::PoseStamped>;
using GroupPoseMatrixDict = std::map<std::string, Eigen::Isometry3d>;
using GroupStringDict = std::map<std::string, std::string>;
using GroupVectorDict = std::map<std::string, std::vector<double>>;

// scene configuration
std::vector<double> clip_size = {0.04, 0.04, 0.06};
double hold_x_offset = 0.03;
std::vector<double> leader_pre_clip = {-(clip_size[0]/2+hold_x_offset), -clip_size[1]/2, clip_size[2]/2};
std::vector<double> follower_pre_clip = {clip_size[0]/2+hold_x_offset, -clip_size[1]/2, clip_size[2]/2}; 

geometry_msgs::msg::PoseStamped createClipGoal(const std::string& goal_frame, const std::vector<double>& goal_translation_vector)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.pose.position.x = goal_translation_vector[0];
  goal_pose.pose.position.y = goal_translation_vector[1];
  goal_pose.pose.position.z = goal_translation_vector[2];
  // Orientation from clip frame to robot EE frame
  goal_pose.pose.orientation.x = 0.0;
  goal_pose.pose.orientation.y = 1.0;
  goal_pose.pose.orientation.z = 0.0;
  goal_pose.pose.orientation.w = 0.0;

  goal_pose.header.frame_id = goal_frame;
  return goal_pose;
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

  void doTask(std::string& goal_clip_id, bool execute=false);
  void updatePlanningScene();

  // void loadCustomScene(const std::string &path);

private:
  geometry_msgs::msg::PoseStamped getPoseTransform(const geometry_msgs::msg::PoseStamped& pose, const std::string& target_frame);
  moveit_msgs::msg::Constraints createBoxConstraints(const std::string& link_name, geometry_msgs::msg::PoseStamped& goal_pose);

  // Compose an MTC task from a series of stages.
  mtc::Task createTask(std::string& goal_frame_name);
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;

  // interfaces
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;

  // TF2 components
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Joint state subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState::SharedPtr current_joint_state_;
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void periodicUpdate(); // Background thread function
  
  // Threading for periodic updates
  std::thread update_thread_;
  std::atomic<bool> stop_thread_;

};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) },
    move_group_(node_, "right_panda_arm"),
    visual_tools_(node_, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_.getRobotModel()),
    tf_buffer_(node_->get_clock()), // Initialize TF Buffer with node clock
    tf_listener_(tf_buffer_)       // Initialize TF Listener with TF Buffer
{
  visual_tools_.loadRemoteControl();

  // Subscription to the joint states
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&MTCTaskNode::jointStateCallback, this, std::placeholders::_1));

  // Wait until joint states are received
  RCLCPP_INFO(LOGGER, "Waiting for joint states...");
  while (!current_joint_state_)
  {
    rclcpp::spin_some(node_);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(LOGGER, "Joint states received.");

  // Start the periodic update thread
  // update_thread_ = std::thread(&MTCTaskNode::periodicUpdate, this);

}

// Destructor
MTCTaskNode::~MTCTaskNode()
{
  stop_thread_ = true; // Signal the thread to stop
  if (update_thread_.joinable())
  {
    update_thread_.join(); // Wait for the thread to finish
  }
}

 void MTCTaskNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  current_joint_state_ = msg; // Cache the latest joint state
}

void MTCTaskNode::updatePlanningScene()
{
  if (!current_joint_state_)
  {
    RCLCPP_WARN(LOGGER, "Joint state not yet received, cannot update planning scene.");
    return;
  }

  // Retrieve the current robot state from MoveGroup
  moveit::core::RobotStatePtr robot_state = move_group_.getCurrentState();

  // Update robot state with the latest joint positions
  const std::vector<std::string>& joint_names = current_joint_state_->name;
  const std::vector<double>& joint_positions = current_joint_state_->position;

  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    robot_state->setJointPositions(joint_names[i], &joint_positions[i]);
  }

  // Apply the updated state to the planning scene
  move_group_.setStartState(*robot_state);
  RCLCPP_INFO(LOGGER, "Planning scene successfully updated.");
}

// Periodic Update Thread
void MTCTaskNode::periodicUpdate()
{
  rclcpp::Rate rate(1.0); // 1 Hz update rate
  while (!stop_thread_ && rclcpp::ok())
  {
    updatePlanningScene();
    rate.sleep();
  }
}

rclcpp::Node::SharedPtr MTCTaskNode::getNode()
{
  return node_;
}

moveit::planning_interface::MoveGroupInterface& MTCTaskNode::getMoveGroup()
{
  return move_group_;
}

moveit_visual_tools::MoveItVisualTools& MTCTaskNode::getVisualTools()
{
  return visual_tools_;
}



geometry_msgs::msg::PoseStamped MTCTaskNode::getPoseTransform(const geometry_msgs::msg::PoseStamped& pose, const std::string& target_frame)
{   
    std::string frame_id = pose.header.frame_id;
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform(
            target_frame,   // Target frame
            frame_id,  // Source frame
            rclcpp::Time(0),  // Get the latest transform
            rclcpp::Duration::from_seconds(1.0)// Timeout
        );
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(LOGGER, "Could not transform %s frame to %s frame: %s", frame_id.c_str(), target_frame.c_str(), ex.what());
        return pose;
    }

    geometry_msgs::msg::PoseStamped pose_world;
    tf2::doTransform(pose, pose_world, transform);
    return pose_world;
}

moveit_msgs::msg::Constraints MTCTaskNode::createBoxConstraints(const std::string& link_name, geometry_msgs::msg::PoseStamped& goal_pose)
{
  // Assume current_pose and goal_pose are of type geometry_msgs::msg::PoseStamped
  geometry_msgs::msg::PoseStamped current_pose = move_group_.getCurrentPose("right_panda_hand");
  geometry_msgs::msg::PoseStamped current_pose_transformed = getPoseTransform(current_pose, "world");
  geometry_msgs::msg::PoseStamped goal_pose_transformed = getPoseTransform(goal_pose, "world");
  RCLCPP_INFO(LOGGER, "Goal pose transformed: x: %f, y: %f, z: %f", goal_pose_transformed.pose.position.x, goal_pose_transformed.pose.position.y, goal_pose_transformed.pose.position.z);

  // Compute the box center and dimensions
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = (current_pose_transformed.pose.position.x + goal_pose_transformed.pose.position.x) / 2.0;
  box_pose.position.y = (current_pose_transformed.pose.position.y + goal_pose_transformed.pose.position.y) / 2.0;
  box_pose.position.z = (current_pose_transformed.pose.position.z + goal_pose_transformed.pose.position.z) / 2.0;
  box_pose.orientation.w = 1.0; // Identity quaternion for box orientation

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {
      fabs(goal_pose_transformed.pose.position.x - current_pose_transformed.pose.position.x)+0.1,  // Length (x)
      fabs(goal_pose_transformed.pose.position.y - current_pose_transformed.pose.position.y)+0.1,  // Width (y)
      fabs(goal_pose_transformed.pose.position.z - current_pose_transformed.pose.position.z)+0.3   // Height (z)
  };

  // Create position constraint
  moveit_msgs::msg::PositionConstraint box_constraint;
  box_constraint.header.frame_id = "world"; // Replace with the appropriate reference frame
  box_constraint.link_name = "right_panda_hand"; // Replace with the relevant link name
  box_constraint.constraint_region.primitives.emplace_back(box);
  box_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
  box_constraint.weight = 1.0;

  // Visualize the box constraint
  Eigen::Vector3d box_point_1(
      box_pose.position.x - box.dimensions[0] / 2.0,
      box_pose.position.y - box.dimensions[1] / 2.0,
      box_pose.position.z - box.dimensions[2] / 2.0
  );
  Eigen::Vector3d box_point_2(
      box_pose.position.x + box.dimensions[0] / 2.0,
      box_pose.position.y + box.dimensions[1] / 2.0,
      box_pose.position.z + box.dimensions[2] / 2.0
  );
  visual_tools_.publishCuboid(box_point_1, box_point_2, rviz_visual_tools::TRANSLUCENT_DARK);
  visual_tools_.trigger();

  // Wrap in a generic Constraints message
  moveit_msgs::msg::Constraints box_constraints;
  box_constraints.position_constraints.emplace_back(box_constraint);

  return box_constraints;
}

void MTCTaskNode::doTask(std::string& goal_clip_id, bool execute)
{
  task_ = createTask(goal_clip_id);

  // publish solution for moveit servo
  bool publish_mtc_trajectory = !execute;

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  RCLCPP_INFO_STREAM(LOGGER, "Task planned successfully");
  task_.introspection().publishSolution(*task_.solutions().front(), publish_mtc_trajectory);

//   // Access the first solution
//   const auto& solution = solutions.front();

//   // Cast SolutionBase to SubTrajectory to access the trajectory
//   auto sub_trajectory = std::dynamic_pointer_cast<const mtc::SubTrajectory>(solution);
//   if (!sub_trajectory)
//   {
//     RCLCPP_ERROR(rclcpp::get_logger("MTC"), "Failed to cast SolutionBase to SubTrajectory.");
//     return;
//   }

//   // Access the trajectory
//   const robot_trajectory::RobotTrajectoryConstPtr& trajectory = sub_trajectory->trajectory();
//   if (!trajectory)
//   {
//     RCLCPP_ERROR(rclcpp::get_logger("MTC"), "No trajectory available in the solution.");
//     return;
//   }
 
//  // Iterate through waypoints and extract joint positions
//   for (size_t i = 0; i < trajectory->getWayPointCount(); ++i)
//   {
//     const robot_state::RobotState& waypoint = trajectory->getWayPoint(i);

//     std::vector<double> joint_positions;
//     waypoint.copyJointGroupPositions(trajectory->getGroupName(), joint_positions);

//     RCLCPP_INFO(rclcpp::get_logger("MTC"), "Waypoint %zu: %s", i,
//                 std::to_string(joint_positions.size()).c_str());
//   }

  if (execute){
    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
      return;
    }
  }
  
  return;
}

mtc::Task MTCTaskNode::createTask(std::string& goal_frame_name)
{
  mtc::Task task;
  task.stages()->setName("routing task");
  task.loadRobotModel(node_);

  const auto& lead_arm_group_name = "right_panda_arm";
  const auto& lead_hand_group_name = "right_hand";
  const auto& lead_hand_frame = "right_panda_hand";

  const auto& follow_arm_group_name = "left_panda_arm";
  const auto& follow_hand_group_name = "left_hand";
  const auto& follow_hand_frame = "left_panda_hand";

  const auto& dual_arm_group_name = "dual_arm";

  // Set task properties (only valid for single arm)
  // task.setProperty("group", lead_arm_group_name);
  // task.setProperty("eef", lead_hand_group_name);
  // task.setProperty("ik_frame", lead_hand_frame);

  // delete markers
  visual_tools_.deleteAllMarkers();
  visual_tools_.trigger();

  // set target pose
  geometry_msgs::msg::PoseStamped lead_target_pose = createClipGoal(goal_frame_name, leader_pre_clip);
  geometry_msgs::msg::PoseStamped follow_target_pose = createClipGoal(goal_frame_name, follower_pre_clip);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  mtc::Stage* pre_move_stage_ptr = nullptr;

  /****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
  {
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));
  }

  // Set up planners
  auto lead_sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  lead_sampling_planner->setMaxVelocityScalingFactor(0.05);
  lead_sampling_planner->setMaxAccelerationScalingFactor(0.05);

  auto lead_interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto lead_cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  lead_cartesian_planner->setMaxVelocityScalingFactor(0.05);
  lead_cartesian_planner->setMaxAccelerationScalingFactor(0.05);
  lead_cartesian_planner->setStepSize(.001);

  auto follow_sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  follow_sampling_planner->setMaxVelocityScalingFactor(0.05);
  follow_sampling_planner->setMaxAccelerationScalingFactor(0.05);

  auto follow_interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto follow_cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  follow_cartesian_planner->setMaxVelocityScalingFactor(0.05);
  follow_cartesian_planner->setMaxAccelerationScalingFactor(0.05);
  follow_cartesian_planner->setStepSize(.001);

  /****************************************************
  ---- *               Close Hand                      *
  ***************************************************/
 {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", lead_interpolation_planner);
    stage->setGroup(lead_hand_group_name);
    stage->setGoal("close");

    pre_move_stage_ptr = stage.get();

    task.add(std::move(stage));
 }
  /****************************************************
	 *                                                  *
	 *              Connect to Pick                     *
	 *                                                  *
	 ***************************************************/
  {
    mtc::stages::Connect::GroupPlannerVector planners = {{lead_arm_group_name, lead_sampling_planner}, {follow_arm_group_name, follow_sampling_planner}};
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>("move to pick", planners);
    stage_move_to_pick->setTimeout(5.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);

    // add path constraints
    moveit_msgs::msg::Constraints path_constraints = createBoxConstraints(lead_hand_frame, lead_target_pose);
    stage_move_to_pick->setPathConstraints(path_constraints);
    RCLCPP_INFO(LOGGER, "Path constraints set");

    task.add(std::move(stage_move_to_pick));
  }
  
  /****************************************************
	 *                                                  *
	 *               Pick Container                     *
	 *                                                  *
	 ***************************************************/
  mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    // only valid for single arm
    // task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    // grasp->properties().configureInitFrom(mtc::Stage::PARENT,
    //                                       { "eef", "group", "ik_frame" });

    /****************************************************
  ---- *               Insertion in EE-z                *
    ***************************************************/
    // {
    //   auto stage =
    //       std::make_unique<mtc::stages::MoveRelative>("insertion", cartesian_planner);
    //   stage->properties().set("marker_ns", "insertion");
    //   stage->properties().set("link", lead_hand_frame);
    //   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    //   stage->setMinMaxDistance(0.1, 0.15);

    //   // Set hand forward direction
    //   geometry_msgs::msg::Vector3Stamped vec;
    //   vec.header.frame_id = lead_hand_frame;
    //   vec.vector.z = 0.5;
    //   stage->setDirection(vec);
    //   grasp->insert(std::move(stage));
    // }
    /****************************************************
  ---- *      Generate Fixed Grasp Pose for single arm *
	***************************************************/
    // {
    //   // Sample grasp pose
    // //   auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
    // //   stage->properties().configureInitFrom(mtc::Stage::PARENT);
    // //   stage->properties().set("marker_ns", "grasp_pose");
    // //   stage->setPreGraspPose("close");
    // //   stage->setObject(goal_frame_name);
    // //   stage->setAngleDelta(M_PI / 12);
    // //   stage->setMonitoredStage(current_state_ptr);  // Hook into current state
    
    //   // Fixed grasp pose
    //   auto stage = std::make_unique<mtc::stages::FixedCartesianPoses>("fixed clipping pose");
    //   stage->addPose(lead_target_pose);
    //   stage->setMonitoredStage(pre_move_stage_ptr);  // Hook into pre_move_stage_ptr

    //   // IK frame at TCP
    //   Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
    // //   Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
    // //                         Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
    // //                         Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
    // //   grasp_frame_transform.linear() = q.matrix();
    //   grasp_frame_transform.translation().z() = 0.1034;

    //   // Compute IK
    //   auto wrapper =
    //       std::make_unique<mtc::stages::ComputeIK>("clipping pose IK", std::move(stage));
    //   wrapper->setMaxIKSolutions(8);
    //   wrapper->setMinSolutionDistance(1.0);
    //   wrapper->setIKFrame(grasp_frame_transform, lead_hand_frame);
    //   wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    //   wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    //   grasp->insert(std::move(wrapper));
    // }
  //   /****************************************************
  // ---- *    Generate Fixed Grasp Pose for dual arm *
	// ***************************************************/
  //   {
  //     // Fixed grasp pose
  //     auto dual_fixed_pose = std::make_unique<mtc::stages::FixedCartesianPosesMultiple>("dual fixed clipping pose");
  //     GroupPoseDict pose_pairs = {{follow_arm_group_name, follow_target_pose}, {lead_arm_group_name, lead_target_pose}};
  //     dual_fixed_pose->addPosePair(pose_pairs);
  //     dual_fixed_pose->setMonitoredStage(pre_move_stage_ptr);

  //     // IK frame at TCP
  //     Eigen::Isometry3d lead_grasp_frame_transform = Eigen::Isometry3d::Identity();
  //     lead_grasp_frame_transform.translation().z() = 0.1034;
  //     Eigen::Isometry3d follow_grasp_frame_transform = Eigen::Isometry3d::Identity();
  //     follow_grasp_frame_transform.translation().z() = 0.1034;

  //     // IK groups
  //     std::vector<std::string> ik_groups = {follow_arm_group_name, lead_arm_group_name};
  //     GroupStringDict ik_endeffectors = {{follow_arm_group_name, follow_hand_group_name}, {lead_arm_group_name, lead_hand_group_name}};
  //     GroupStringDict ik_hand_frames = {{follow_arm_group_name, follow_hand_frame}, {lead_arm_group_name, lead_hand_frame}, };
  //     // GroupStringDict ik_links = {{lead_arm_group, "right_arm_hand"}, {follow_arm_group, "left_arm_hand"}};
  //     GroupPoseMatrixDict ik_frame_transforms = {{follow_arm_group_name, follow_grasp_frame_transform}, {lead_arm_group_name, lead_grasp_frame_transform}};

  //     // Compute IK
  //     auto ik_wrapper = std::make_unique<mtc::stages::ComputeIKMultiple>("clipping pose IK", std::move(dual_fixed_pose), ik_groups, dual_arm_group_name);
  //     ik_wrapper->setSubGroups(ik_groups);
  //     ik_wrapper->setGroup(dual_arm_group_name);
  //     ik_wrapper->setEndEffector(ik_endeffectors);
  //     // ik_wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
  //     ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_poses"});
  //     // ik_wrapper->properties().set("object", "object");
  //     ik_wrapper->setMaxIKSolutions(5);
  //     ik_wrapper->setMinSolutionDistance(1.0);
  //     ik_wrapper->setIKFrame(ik_frame_transforms, ik_hand_frames);

  //     grasp->insert(std::move(ik_wrapper));

  //   }
  /****************************************************
  ---- *    Generate Grasp Pose for dual arm *
	***************************************************/
    {
      // Target positions in clip frame
      GroupStringDict goal_frames = {{lead_arm_group_name, goal_frame_name}, {follow_arm_group_name, goal_frame_name}};

      std::vector<double> lead_goal_delta_vector = {lead_target_pose.pose.position.x, lead_target_pose.pose.position.y, lead_target_pose.pose.position.z};
      std::vector<double> follow_goal_delta_vector = {follow_target_pose.pose.position.x, follow_target_pose.pose.position.y, follow_target_pose.pose.position.z};
      std::vector<double> lead_goal_orient_vector = {lead_target_pose.pose.orientation.x, lead_target_pose.pose.orientation.y, lead_target_pose.pose.position.z, lead_target_pose.pose.orientation.w};
      std::vector<double> follow_goal_orient_vector = {follow_target_pose.pose.orientation.x, follow_target_pose.pose.orientation.y, follow_target_pose.pose.position.z, follow_target_pose.pose.orientation.w};
      GroupVectorDict delta_pairs = {{lead_arm_group_name, lead_goal_delta_vector}, {follow_arm_group_name, follow_goal_delta_vector}};
      GroupVectorDict orient_pairs = {{lead_arm_group_name, lead_goal_orient_vector}, {follow_arm_group_name, follow_goal_orient_vector}};

      // IK frame at TCP
      Eigen::Isometry3d lead_grasp_frame_transform = Eigen::Isometry3d::Identity();
      lead_grasp_frame_transform.translation().z() = 0.1034;
      Eigen::Isometry3d follow_grasp_frame_transform = Eigen::Isometry3d::Identity();
      follow_grasp_frame_transform.translation().z() = 0.1034;

      // IK groups
      std::vector<std::string> ik_groups = {follow_arm_group_name, lead_arm_group_name};
      GroupStringDict ik_endeffectors = {{follow_arm_group_name, follow_hand_group_name}, {lead_arm_group_name, lead_hand_group_name}};
      GroupStringDict ik_hand_frames = {{follow_arm_group_name, follow_hand_frame}, {lead_arm_group_name, lead_hand_frame}, };
      // GroupStringDict ik_links = {{lead_arm_group, "right_arm_hand"}, {follow_arm_group, "left_arm_hand"}};
      GroupPoseMatrixDict ik_frame_transforms = {{follow_arm_group_name, follow_grasp_frame_transform}, {lead_arm_group_name, lead_grasp_frame_transform}};
      GroupStringDict pre_grasp_pose = {{follow_arm_group_name, "close"}, {lead_arm_group_name, "close"}};

      // generate grasp pose, randomize for follower
      auto grasp_generator = std::make_unique<mtc::stages::GenerateGraspPoseDual>("generate grasp pose dual", ik_groups);
      grasp_generator->setEndEffector(ik_endeffectors);
      grasp_generator->properties().set("marker_ns", "grasp_pose");
      grasp_generator->properties().set("explr_axis", "y");
      grasp_generator->setAngleDelta(0.2); // enumerate over angles from 0 to 6.4 (less then 2 PI)
      grasp_generator->setPreGraspPose(pre_grasp_pose);
      grasp_generator->setGraspPose("close");
      grasp_generator->setObject(goal_frames); // object sets target pose frame
      grasp_generator->setTargetDelta(delta_pairs);
      grasp_generator->setTargetOrient(orient_pairs);
      grasp_generator->setMonitoredStage(pre_move_stage_ptr);
      grasp_generator->properties().set("generate_group", follow_arm_group_name);
      grasp_generator->properties().set("planning_frame", goal_frame_name);

      // Compute IK
      auto ik_wrapper = std::make_unique<mtc::stages::ComputeIKMultiple>("clipping pose IK", std::move(grasp_generator), ik_groups, dual_arm_group_name);
      ik_wrapper->setSubGroups(ik_groups);
      ik_wrapper->setGroup(dual_arm_group_name);
      ik_wrapper->setEndEffector(ik_endeffectors);
      // ik_wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
      ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_poses"});
      // ik_wrapper->properties().set("object", "object");
      ik_wrapper->setMaxIKSolutions(5);
      ik_wrapper->setMinSolutionDistance(1.0);
      ik_wrapper->setIKFrame(ik_frame_transforms, ik_hand_frames);

      grasp->insert(std::move(ik_wrapper));

    }
    /****************************************************
  ---- *               Allow Collision (hand object)   *
	***************************************************/
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                            task.getRobotModel()
                                ->getJointModelGroup(lead_hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
      grasp->insert(std::move(stage));
    }
    /****************************************************
  ---- *               Close Hand                      *
	***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", lead_interpolation_planner);
      stage->setGroup(lead_hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));
  }

  

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  
  // Create a publisher to start the follower tracking
  auto tracking_start_pub = mtc_task_node->getNode()->create_publisher<std_msgs::msg::Bool>("/start_tracking", 10);

  // moveit::planning_interface::MoveGroupInterface move_group(mtc_task_node->getNode(), "right_panda_arm");
  // moveit_visual_tools::MoveItVisualTools visual_tools(mtc_task_node->getNode(), "right_panda_link0", "dual_mtc_routing",
  //                                                     move_group.getRobotModel());
  // visual_tools.loadRemoteControl();

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  // Use visul tools to control the movement from one clip to another
  mtc_task_node->getVisualTools().prompt("After objects are loaded, press 'next' in the RvizVisualToolsGui window to start the demo");

  // initial clip
  std::string clip_id = "clip5";
  mtc_task_node->doTask(clip_id, false);

  // // Use visul tools to control the movement from one clip to another
  // mtc_task_node->getVisualTools().prompt("After moving to initial positions, press 'next' in the RvizVisualToolsGui window to start servo");
  // start follower tracking
  std_msgs::msg::Bool start_tracking_msg;
  start_tracking_msg.data = true;
  tracking_start_pub->publish(start_tracking_msg);

  // Use visul tools to control the movement from one clip to another
  mtc_task_node->getVisualTools().prompt("Press 'next' in the RvizVisualToolsGui window to contiune the next task");

  // Update planning scene after execution
  RCLCPP_INFO(LOGGER, "Updating planning scene after MTC execution.");
  mtc_task_node->updatePlanningScene();

  // the next clip
  clip_id = "clip6";
  mtc_task_node->doTask(clip_id, false);

  // Use visul tools to control the movement from one clip to another
  mtc_task_node->getVisualTools().prompt("Press 'next' in the RvizVisualToolsGui window to contiune the next task");

  // Update planning scene after execution
  RCLCPP_INFO(LOGGER, "Updating planning scene after MTC execution.");
  mtc_task_node->updatePlanningScene();

  // the next clip
  clip_id = "clip8";
  mtc_task_node->doTask(clip_id, false);

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}