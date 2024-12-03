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


static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

std::vector<double> clip_size = {0.04, 0.04, 0.06};
double hold_x_offset = 0.03;
std::vector<double> leader_pre_clip = {-(clip_size[0]/2+hold_x_offset), -clip_size[1]/2, clip_size[2]/2};

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

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  // Add this public getter
  rclcpp::Node::SharedPtr getNode();

  void doTask(std::string& goal_clip_id);

  // void loadCustomScene(const std::string &path);

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask(std::string& goal_frame_name);
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

rclcpp::Node::SharedPtr MTCTaskNode::getNode()
{
  return node_;
}

// void MTCTaskNode::loadCustomScene(const std::string &path)
// {
//     moveit::planning_interface::PlanningSceneInterface psi;
    
//     using moveit::planning_interface::MoveGroupInterface;
//     auto move_group_interface = MoveGroupInterface(node_, "right_panda_arm");

//     moveit_visual_tools::MoveItVisualTools visual_tools(node_, "right_panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
//                                                       move_group_interface.getRobotModel());

//     std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
//     std::ifstream fin(path);
//     if (!fin.is_open())
//     {
//         throw std::runtime_error("Failed to open scene file: " + path);
//     }

//     std::string line;
//     while (std::getline(fin, line))
//     {
//         // Skip invalid lines or empty lines
//         if (line.empty() || line[0] == '.' || line.find("(noname)+") != std::string::npos)
//             continue;

//         if (line[0] == '*') // Object definition starts
//         {
//             std::string object_name = line.substr(2); // Extract object name after "* "
//             RCLCPP_INFO(LOGGER, "Loading object: %s", object_name.c_str());

//             moveit_msgs::msg::CollisionObject collision_object;
//             collision_object.id = object_name;
//             collision_object.header.frame_id = "world"; // Default frame

//             // Read the next lines for the object block
//             std::string pos_line, ori_line, shape_line, dim_line, unused_line;
//             double x, y, z, qx, qy, qz, qw, dx, dy, dz;

//             // Read position
//             if (!std::getline(fin, pos_line) || !(std::istringstream(pos_line) >> x >> y >> z))
//                 throw std::runtime_error("Invalid position line for object: " + object_name);

//             // Read orientation
//             if (!std::getline(fin, ori_line) || !(std::istringstream(ori_line) >> qx >> qy >> qz >> qw))
//                 throw std::runtime_error("Invalid orientation line for object: " + object_name);

//             // Skip one line (the "1" marker)
//             if (!std::getline(fin, unused_line))
//                 throw std::runtime_error("Unexpected end of file after orientation for object: " + object_name);

//             // Read shape type (assuming it's always "box")
//             if (!std::getline(fin, shape_line) || shape_line != "box")
//                 throw std::runtime_error("Invalid or unsupported shape type for object: " + object_name);

//             // Read dimensions
//             if (!std::getline(fin, dim_line) || !(std::istringstream(dim_line) >> dx >> dy >> dz))
//                 throw std::runtime_error("Invalid dimensions line for object: " + object_name);

//             // Skip the remaining unused lines in the block
//             for (int i = 0; i < 4; ++i)
//             {
//                 if (!std::getline(fin, unused_line))
//                     throw std::runtime_error("Unexpected end of file in unused block for object: " + object_name);
//             }
            
//             shape_msgs::msg::SolidPrimitive primitive;
//             primitive.type = primitive.BOX;
//             primitive.dimensions.resize(3);
//             primitive.dimensions[primitive.BOX_X] = dx;
//             primitive.dimensions[primitive.BOX_Y] = dy;
//             primitive.dimensions[primitive.BOX_Z] = dz;

//             // Create pose
//             geometry_msgs::msg::Pose pose;
//             pose.position.x = x;
//             pose.position.y = y;
//             pose.position.z = z;
//             pose.orientation.x = qx;
//             pose.orientation.y = qy;
//             pose.orientation.z = qz;
//             pose.orientation.w = qw;

//             collision_object.primitives.push_back(primitive);
//             collision_object.primitive_poses.push_back(pose);
//             collision_object.operation = collision_object.ADD;

//             collision_objects.push_back(collision_object);
//         }
        
//     }
//     // Add object to planning scene
    
//     psi.addCollisionObjects(collision_objects);

//     // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
//     visual_tools.trigger();

//     RCLCPP_INFO(LOGGER, "Scene loaded successfully from %s", path.c_str());
// }

void MTCTaskNode::doTask(std::string& goal_clip_id)
{
  task_ = createTask(goal_clip_id);

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
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask(std::string& goal_frame_name)
{
  mtc::Task task;
  task.stages()->setName("routing task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "right_panda_arm";
  const auto& hand_group_name = "right_hand";
  const auto& hand_frame = "right_panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

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
  
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setMaxVelocityScalingFactor(0.05);
  sampling_planner->setMaxAccelerationScalingFactor(0.05);

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.05);
  cartesian_planner->setMaxAccelerationScalingFactor(0.05);
  cartesian_planner->setStepSize(.01);

  /****************************************************
  ---- *               Close Hand                      *
  ***************************************************/
 {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage->setGroup(hand_group_name);
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
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>("move to pick", 
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage_move_to_pick->setTimeout(5.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
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
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });

    /****************************************************
  ---- *               Insertion in EE-z                *
    ***************************************************/
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("insertion", cartesian_planner);
      stage->properties().set("marker_ns", "insertion");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    /****************************************************
  ---- *               Generate Grasp Pose                *
	***************************************************/
    {
      // Sample grasp pose
    //   auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
    //   stage->properties().configureInitFrom(mtc::Stage::PARENT);
    //   stage->properties().set("marker_ns", "grasp_pose");
    //   stage->setPreGraspPose("close");
    //   stage->setObject(goal_frame_name);
    //   stage->setAngleDelta(M_PI / 12);
    //   stage->setMonitoredStage(current_state_ptr);  // Hook into current state
    
      // Fixed grasp pose
      auto stage = std::make_unique<mtc::stages::FixedCartesianPoses>("fixed clipping pose");
      geometry_msgs::msg::PoseStamped target_pose;
      target_pose = createClipGoal(goal_frame_name, leader_pre_clip);
      stage->addPose(target_pose);
      stage->setMonitoredStage(pre_move_stage_ptr);  // Hook into pre_move_stage_ptr

      // IK frame at TCP
      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
    //   Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
    //                         Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
    //                         Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
    //   grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.1034;

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("clipping pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }
    /****************************************************
  ---- *               Allow Collision (hand object)   *
	***************************************************/
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
      grasp->insert(std::move(stage));
    }
    /****************************************************
  ---- *               Close Hand                      *
	***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
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

  moveit::planning_interface::MoveGroupInterface move_group(mtc_task_node->getNode(), "right_panda_arm");
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(mtc_task_node->getNode(), "right_panda_link0", "dual_mtc_routing",
                                                      move_group.getRobotModel());
  visual_tools.loadRemoteControl();

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  // initial clip
  std::string clip_id = "clip5";
  mtc_task_node->doTask(clip_id);

  // start follower tracking
  std_msgs::msg::Bool start_tracking_msg;
  start_tracking_msg.data = true;
  tracking_start_pub->publish(start_tracking_msg);

  // Use visul tools to control the movement from one clip to another
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // the next clip
  clip_id = "clip6";
  mtc_task_node->doTask(clip_id);

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}