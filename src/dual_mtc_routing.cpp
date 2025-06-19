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
#include <moveit_task_constructor_msgs/srv/get_clip_names.hpp>
#include <moveit/task_constructor/storage.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <vector>

#include <mtc_tutorial/dual_mtc_routing.h>
#include <moveit/task_constructor/stages/noop.h>
// #include <moveit/task_constructor/task_routing.h>
// #include <moveit_task_constructor_msgs/msg/clip_names.hpp>
// #include <moveit_msgs>

using boost::asio::ip::udp;
using json = nlohmann::json;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_node");

double desired_ee_distance = 0.1;

// scene configuration
std::vector<double> clip_size = {0.04, 0.04, 0.06};
double insertion_offset_magnitude = 0.02;
double grasp_follower_offset_magnitude = 0.03;
double grasp_leader_offset_magnitude = desired_ee_distance + grasp_follower_offset_magnitude;
double hold_y_offset = 0.03;
double hold_z_offset = 0.01;
// std::vector<double> leader_pre_insert_offset = {-(clip_size[0]/2+hold_x_offset), -clip_size[1]/2, clip_size[2]/2};
// std::vector<double> follower_pre_insert_offset = {clip_size[0]/2+hold_x_offset, -clip_size[1]/2, clip_size[2]/2};

// U-type Clip
// std::vector<double> insertion_vector = {0, 0, -(insertion_offset_magnitude+0.02)};
// std::vector<double> leader_pre_insert_offset = {0, -(clip_size[1]/2+hold_y_offset), clip_size[2]/2+hold_z_offset};
// std::vector<double> follower_pre_insert_offset = {0, clip_size[1]/2+hold_y_offset, clip_size[2]/2+hold_z_offset}; 

// C-type Clip
std::vector<double> insertion_vector = {-(insertion_offset_magnitude+0.02), 0, 0};
// offset in clip frame
std::vector<double> leader_pre_insert_offset = {(insertion_offset_magnitude+clip_size[0]/2), -(clip_size[1]/2+hold_y_offset), clip_size[2]/2+hold_z_offset};
std::vector<double> follower_pre_insert_offset = {(insertion_offset_magnitude+clip_size[0]/2), clip_size[1]/2+hold_y_offset, clip_size[2]/2+hold_z_offset}; 
std::vector<double> leader_grasp_offset = {0, -(clip_size[1]/2+grasp_leader_offset_magnitude), clip_size[2]/2+hold_z_offset};
std::vector<double> follower_grasp_offset = {0, -(clip_size[1]/2+grasp_follower_offset_magnitude), clip_size[2]/2+hold_z_offset};
double default_franka_flange_to_tcp_z = 0.1034;
double sensone_height = 0.036; 

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) },
    move_group_(node_, "dual_arm"),  // for dual arm or single arm
    visual_tools_(node_, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_.getRobotModel(), true),
    tf_buffer_(node_->get_clock()), // Initialize TF Buffer with node clock
    tf_listener_(tf_buffer_),       // Initialize TF Listener with TF Buffer
    lead_flange_to_tcp_transform_(Eigen::Isometry3d::Identity()),
    follow_flange_to_tcp_transform_(Eigen::Isometry3d::Identity())
{
  visual_tools_.loadRemoteControl();

  // Subtrajectory publisher
  subtrajectory_publisher_ = node_->create_publisher<moveit_task_constructor_msgs::msg::SubTrajectory>(
		    "/mtc_sub_trajectory", rclcpp::QoS(1).transient_local());

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

  // initialize udp sync with real-world
  udp_thread_lead_sync_ = std::thread(&MTCTaskNode::udpReceiverSync, // member function pointer
                                      this,                          // object pointer (this class instance)
                                      "10.157.175.222",                 // host
                                      6305,                          // port
                                      std::ref(lead_joint_positions_),              // reference to class member
                                      std::ref(lead_joint_positions_mutex_),        // mutex
                                      std::ref(lead_joint_positions_condition_variable_), // condition variable
                                      std::ref(lead_ee_pose_),                      // EE pose vector
                                      std::ref(lead_ee_pose_mutex_)                 // EE pose mutex
                                      );

  udp_thread_follow_sync_ = std::thread(&MTCTaskNode::udpReceiverSync, // member function pointer
                                        this,                          // object pointer (this class instance)
                                        "10.157.175.222",
                                        6306,                          // port
                                        std::ref(follow_joint_positions_),              // EE pose mutex
                                        std::ref(follow_joint_positions_mutex_),        // mutex
                                        std::ref(follow_joint_positions_condition_variable_), // condition variable
                                        std::ref(follow_ee_pose_),                      // EE pose vector
                                        std::ref(follow_ee_pose_mutex_)                 // EE pose mutex
                                        );

  // CAUTION: flange_to_tcp stands for the transform from panda_link_8 to TCP
  // In comparison to hand_to_tcp, there is additional rotation of 45 degree around z axis, and an optional z offset because of wrist snesor
  // adapt flange to TCP transform based on the wrist sensor's height
  if (node_->get_parameter("use_sensone_left").as_bool()){
    follow_flange_to_tcp_transform_.translation().z() = default_franka_flange_to_tcp_z + sensone_height;
  }else{
    follow_flange_to_tcp_transform_.translation().z() = default_franka_flange_to_tcp_z; 
  }
  // set rotation to 45 degree around z axis
  follow_flange_to_tcp_transform_.rotate(Eigen::AngleAxisd(-M_PI/4, Eigen::Vector3d::UnitZ())); // link8 rotates 45 degree around z axis to tcp
  RCLCPP_INFO(LOGGER, "Follower flange to TCP transform z: %f", follow_flange_to_tcp_transform_.translation().z());

  if (node_->get_parameter("use_sensone_right").as_bool()){
    lead_flange_to_tcp_transform_.translation().z() = default_franka_flange_to_tcp_z + sensone_height; 
  }else{
    lead_flange_to_tcp_transform_.translation().z() = default_franka_flange_to_tcp_z; 
  }
  lead_flange_to_tcp_transform_.rotate(Eigen::AngleAxisd(-M_PI/4, Eigen::Vector3d::UnitZ())); // link8 rotates 45 degree around z axis to tcp
  RCLCPP_INFO(LOGGER, "Leader flange to TCP transform z: %f", lead_flange_to_tcp_transform_.translation().z());
  
  // hand_to_TCP transform is different from flange_to_TCP transform, it is usually a fixed value if franka hand is not changed
  hand_to_tcp_transform_ = Eigen::Isometry3d::Identity();
  hand_to_tcp_transform_.translation().z() = 0.1034; 

  // Start the periodic update thread
  // update_thread_ = std::thread(&MTCTaskNode::periodicUpdate, this);

  // Initialize Planning Groups
  initializeGroups();
}

// Destructor
MTCTaskNode::~MTCTaskNode()
{
  // stop_thread_ = true; // Signal the thread to stop
  // if (update_thread_.joinable())
  // {
  //   update_thread_.join(); // Wait for the thread to finish
  // }

  // stop the UDP receiver thread
  sync_udp_running_ = false; // Signal the UDP threads to stop
  lead_joint_positions_condition_variable_.notify_all();
  if (udp_thread_lead_sync_.joinable()) {
    udp_thread_lead_sync_.join();
  }

  follow_joint_positions_condition_variable_.notify_all();
  if (udp_thread_follow_sync_.joinable()) {
    udp_thread_follow_sync_.join();
  }
}

void MTCTaskNode::initializeGroups()
{
    // Define robot group names
    lead_arm_group_name = "right_panda_arm";
    lead_hand_group_name = "right_hand";
    lead_hand_frame = "right_panda_hand";
    lead_base_frame = "right_panda_link0";

    follow_arm_group_name = "left_panda_arm";
    follow_hand_group_name = "left_hand";
    follow_hand_frame = "left_panda_hand";
    follow_base_frame = "left_panda_link0";

    dual_arm_group_name = "dual_arm";
}

void MTCTaskNode::initializePlanners()
{ 
    std::string chomp_pipeline_name = "chomp";
    // Lead arm planners
    lead_sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    // lead_sampling_planner->setPlannerId("RRTstarkConfigDefault");
    lead_sampling_planner->setMaxVelocityScalingFactor(0.05);
    lead_sampling_planner->setMaxAccelerationScalingFactor(0.05);

    lead_interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    lead_cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    lead_cartesian_planner->setMaxVelocityScalingFactor(1.0);
    lead_cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    lead_cartesian_planner->setStepSize(0.01);

    lead_chomp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, chomp_pipeline_name);
    lead_chomp_planner->setMaxVelocityScalingFactor(0.05);
    lead_chomp_planner->setMaxAccelerationScalingFactor(0.05);

    // Follow arm planners
    follow_sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    follow_sampling_planner->setPlannerId("BiESTkConfigDefault");
    follow_sampling_planner->setMaxVelocityScalingFactor(0.05);
    follow_sampling_planner->setMaxAccelerationScalingFactor(0.05);

    follow_interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    follow_cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    follow_cartesian_planner->setMaxVelocityScalingFactor(1.0);
    follow_cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    follow_cartesian_planner->setStepSize(0.01);

    follow_chomp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, chomp_pipeline_name);
    follow_chomp_planner->setMaxVelocityScalingFactor(0.05);
    follow_chomp_planner->setMaxAccelerationScalingFactor(0.05);
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

void MTCTaskNode::udpReceiverSync(const std::string& host, int port,
  // std::array<double, 6>& joint_positions,
  std::vector<double>& joint_positions,
  std::mutex& joint_positions_mutex,
  std::condition_variable& joint_positions_condition_variable,
  std::vector<double>& ee_pose,
  std::mutex& ee_pose_mutex
  )
{
  try {
    boost::asio::io_context io_context;

    // Bind to all interfaces to avoid issues if the IP is wrong
    udp::socket socket(io_context, udp::endpoint(boost::asio::ip::udp::v4(), port));

    std::array<char, 4096> recv_buf;

    while (sync_udp_running_) {
      udp::endpoint sender_endpoint;
      size_t len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);
      std::string data(recv_buf.data(), len);
    
      std::cout << "[UDP raw data] " << data << std::endl;
    
      // Safely parse JSON
      json received_json;
      try {
        received_json = json::parse(data);
      } catch (const json::parse_error& e) {
        std::cerr << "[UDP] JSON parse error: " << e.what() << "\n[UDP] Dropping message: " << data << std::endl;
        continue;  // Skip garbage
      }
    
      // Verify 'command' or 'magic' key to ensure it's from your system
      if (!received_json.contains("command") || received_json["command"] != "synchronize") {
        std::cerr << "[UDP] Invalid or missing 'command'. Skipping message." << std::endl;
        continue;
      }
    
      // Check and validate joint data
      if (!received_json.contains("q") || !received_json["q"].is_array()) {
        std::cerr << "[UDP] Missing or invalid 'q' array." << std::endl;
        continue;
      }
    
      std::vector<double> current_q = received_json["q"].get<std::vector<double>>();
      if (current_q.size() != 7) {
        std::cerr << "[UDP] Incorrect joint vector size: " << current_q.size() << std::endl;
        continue;
      }
    
      {
        std::lock_guard<std::mutex> lock(joint_positions_mutex);
        joint_positions = current_q;
    
        if (port == 6305) lead_joint_data_received_ = true;
        if (port == 6306) follow_joint_data_received_ = true;
      }
    
      joint_positions_condition_variable.notify_one();
    
      std::cout << "[UDP] Accepted joint data: ";
      for (double q : current_q) std::cout << q << " ";
      std::cout << std::endl;
    }    

  } catch (const std::exception& e) {
    std::cerr << "UDP Receiver Error: " << e.what() << std::endl;
  }
}

bool MTCTaskNode::doSyncTask(std::string arm_group_name, 
                              std::string hand_group_name, 
                              std::string hand_frame,
                              std::mutex& joint_positions_mutex,
                              std::vector<double>& joint_positions,
                              std::vector<std::string>& joint_names,
                              std::condition_variable& joint_positions_condition_variable,
                              std::atomic<bool>& joint_data_received_flag,
                              std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner
  )
{
  RCLCPP_INFO(LOGGER, "Waiting for joint position via UDP...");

  joint_data_received_flag = false;

  {
  std::unique_lock<std::mutex> lock(joint_positions_mutex);
  joint_positions_condition_variable.wait(lock, [&] {
  return joint_data_received_flag.load() && joint_positions.size() == 7;
  });
  }

  RCLCPP_INFO(LOGGER, "Joint position received: [%f, %f, %f, %f, %f, %f, %f]",
  joint_positions[0], joint_positions[1], joint_positions[2],
  joint_positions[3], joint_positions[4], joint_positions[5],
  joint_positions[6]);

  joint_data_received_flag = false;  // Reset

  RCLCPP_INFO(LOGGER, "Joint position received: [%f, %f, %f, %f, %f, %f, %f]", joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3], joint_positions[4], joint_positions[5], joint_positions[6]);

  mtc::Task sync_task = createGoalJointTask(arm_group_name, hand_group_name, hand_frame,
                    joint_positions_mutex, 
                    joint_positions, 
                    joint_names,
                    sampling_planner
                    );
  try
  {
  sync_task.init();
  }
  catch (mtc::InitStageException& e)
  {
  RCLCPP_ERROR_STREAM(LOGGER, "Arm sync task initialization failed: " << e.what());
  return false;
  }

  if (!sync_task.plan(5))
  {
  RCLCPP_ERROR_STREAM(LOGGER, "Arm sync task planning failed");
  return false;
  }

  // Do not publish the solution for synchronization task
  // sync_task.introspection().publishSolution(*sync_task.solutions().front());
  auto sync_result = sync_task.execute(*sync_task.solutions().front());
  if (sync_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
  RCLCPP_ERROR_STREAM(LOGGER, "Arm sync task execution failed");
  return false;
  }
  RCLCPP_INFO(LOGGER, "Arm sync task executed successfully");

  return true;
}

mtc::Task MTCTaskNode::createGoalJointTask(std::string arm_group_name, 
                                          std::string hand_group_name, 
                                          std::string hand_frame,
                                          std::mutex& joint_positions_mutex,
                                          std::vector<double>& joint_positions,
                                          std::vector<std::string>& joint_names,
                                          std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner
                                          )
{
  mtc::Task task;
  task.stages()->setName("synchronization arm task");
  task.loadRobotModel(node_);

  std::vector<double> delta = {0, 0, 0};
  std::vector<double> orients = {0, 0, 0, 1};

  // task.setProperty("left_group", arm_group_name);
  // task.setProperty("left_eef", hand_group_name);
  // task.setProperty("left_ik_frame", hand_frame);

  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop


  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  // move to synchronization joint position
  {
    // set Goal from joint positions
    std::map<std::string, double> joint_goal;
    {
    if (joint_positions.size() != joint_names.size()) {
    std::cerr << "Error: joint_positions size (" << joint_positions.size() 
    << ") does not match joint_names size (" << joint_names.size() << ")" << std::endl;
    return task;
    }

    std::lock_guard<std::mutex> joint_lock(joint_positions_mutex);

    for (size_t i = 0; i < joint_names.size(); i++)
    {
    joint_goal[joint_names[i]] = joint_positions[i];
    }

    std::cout << "printing joint goal" << std::endl;
    for (auto const& [key, val] : joint_goal)
    {
    std::cout << key << ": " << val << std::endl;
    }

    }

    // auto stage_move_to_joint = std::make_unique<mtc::stages::MoveTo>("move to synchronization position", joint_interpolation_planner);
    auto stage_move_to_joint = std::make_unique<mtc::stages::MoveTo>("move to synchronization position", sampling_planner);
    stage_move_to_joint->setGroup(arm_group_name);
    stage_move_to_joint->setGoal(joint_goal);
    task.add(std::move(stage_move_to_joint));
  }

  // // further move to synchronization cartesian ee pose
  // {
  //   // set Goal from ee pose
  //   geometry_msgs::msg::PoseStamped ee_goal;
  //   {
  //     std::lock_guard<std::mutex> cartesian_lock(ee_pose_mutex);
  //     ee_goal.header.frame_id = "left_base_link";
  //     ee_goal.pose.position.x = ee_pose[0];
  //     ee_goal.pose.position.y = ee_pose[1];
  //     ee_goal.pose.position.z = ee_pose[2];

  //     // Convert Euler angles (RX, RY, RZ) to quaternion
  //     tf2::Quaternion quaternion;
  //     quaternion.setRPY(ee_pose[3], ee_pose[4], ee_pose[5]);

  //     // Set the orientation of ee_goal
  //     ee_goal.pose.orientation = tf2::toMsg(quaternion);
  //   }

  //   // IK frame at TCP
  //   Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
  //   grasp_frame_transform.translation().z() = 0.197;

  //   auto stage_move_to_cartesian = std::make_unique<mtc::stages::MoveTo>("move to synchronization cartesian pose", cartesian_interpolation_planner);
  //   stage_move_to_cartesian->setGroup(left_arm_group_name);
  //   stage_move_to_cartesian->setGoal(ee_goal);
  //   stage_move_to_cartesian->setIKFrame(grasp_frame_transform, left_hand_frame);
  //   task.add(std::move(stage_move_to_cartesian));
  // }

  return task;
}


void MTCTaskNode::syncwithRealWorld()
{
  initializeGroups();

  initializePlanners();

  RCLCPP_INFO(LOGGER, "Synchronizing the lead arm to its real-world position...");
  doSyncTask(lead_arm_group_name, lead_hand_group_name, lead_hand_frame,
             lead_joint_positions_mutex_, 
             lead_joint_positions_, lead_franka_joint_names_,
             lead_joint_positions_condition_variable_,
             lead_joint_data_received_,
             lead_sampling_planner
             );

  RCLCPP_INFO(LOGGER, "Synchronizing the follow arm to its real-world position...");
  doSyncTask(follow_arm_group_name, follow_hand_group_name, follow_hand_frame,
             follow_joint_positions_mutex_,
             follow_joint_positions_, follow_franka_joint_names_,
             follow_joint_positions_condition_variable_,
              follow_joint_data_received_,
              follow_sampling_planner
             );

  RCLCPP_INFO(LOGGER, "Synchronization with real-world completed.");
}


void MTCTaskNode::publishSolutionSubTraj(const moveit_task_constructor_msgs::msg::Solution& msg) {
	int index = 0;
  // int task_id = std::stoi(msg.task_id);
  int task_id = 0;
  // try {
  //   if (!msg.task_id.empty()) {
  //       task_id = std::stoi(msg.task_id);
  //   } else {
  //       RCLCPP_WARN(LOGGER, "task_id is empty.");
  //   }
  // }catch (const std::invalid_argument& e) {
  //     RCLCPP_ERROR(LOGGER, "Invalid task_id: %s", msg.task_id.c_str());
  //     return;  // Handle the error appropriately
  // } catch (const std::out_of_range& e) {
  //     RCLCPP_ERROR(LOGGER, "task_id out of range: %s", msg.task_id.c_str());
  //     return;  // Handle the error appropriately
  // }

  for (const moveit_task_constructor_msgs::msg::SubTrajectory& sub_trajectory : msg.sub_trajectory) {
    if (sub_trajectory.trajectory.joint_trajectory.points.empty())
      continue;

    // visualize trajectories
    moveit_msgs::msg::RobotTrajectory robot_trajectory;
    robot_trajectory.joint_trajectory = sub_trajectory.trajectory.joint_trajectory;

    visual_tools_.publishTrajectoryLine(robot_trajectory, move_group_.getCurrentState()->getJointModelGroup(lead_arm_group_name),
                                        lead_flange_to_tcp_transform_, task_id, sub_trajectory.info.stage_id, index, "/home/tp2/ws_humble/trajectories_leader", rviz_visual_tools::ORANGE, lead_base_frame);
    visual_tools_.publishTrajectoryLine(robot_trajectory, move_group_.getCurrentState()->getJointModelGroup(follow_arm_group_name),
                                        follow_flange_to_tcp_transform_, task_id, sub_trajectory.info.stage_id, index, "/home/tp2/ws_humble/trajectories_follower", rviz_visual_tools::BLUE, follow_base_frame);
    // visual_tools_.publishTrajectoryLine(robot_trajectory, move_group_.getCurrentState()->getLinkModel(lead_hand_frame));
    visual_tools_.trigger();

    // renumber trajectory id
    auto new_sub_trajectory = sub_trajectory;
    new_sub_trajectory.info.id = index;

    // publish trajectories
    subtrajectory_publisher_->publish(new_sub_trajectory);
    RCLCPP_INFO(LOGGER, "Sub-trajectory includes the following joints:");
    for (const auto& joint_name : sub_trajectory.trajectory.joint_trajectory.joint_names) {
        RCLCPP_INFO(LOGGER, "  %s", joint_name.c_str());
    }
    RCLCPP_INFO_STREAM(LOGGER, "Published subtrajectory id " << new_sub_trajectory.info.id 
                              << " for stage " << new_sub_trajectory.info.stage_id
                              << " with " << new_sub_trajectory.trajectory.joint_trajectory.points.size()
                              << " waypoints");
    visual_tools_.prompt("[Publishing] Press 'next' to publishing the next subtrajectory");
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    index++;
  }
  return;
	
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
      fabs(goal_pose_transformed.pose.position.z - current_pose_transformed.pose.position.z)+0.1   // Height (z)
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

std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped> MTCTaskNode::assignClipGoal(const std::string& goal_frame_name, 
          const std::vector<double>& goal_vector_1, const std::vector<double>& goal_vector_2)
{
  /*For each clip, leader should always be on the right side of the clip. This gives to two possibilities of leader's sides in the clip frame.*/
  geometry_msgs::msg::PoseStamped leader_target_pose;
  geometry_msgs::msg::PoseStamped follower_target_pose;

  geometry_msgs::msg::PoseStamped target_pose_1 = createClipGoal(goal_frame_name, goal_vector_1);
  geometry_msgs::msg::PoseStamped target_pose_1_transformed = getPoseTransform(target_pose_1, "world");
  
  geometry_msgs::msg::PoseStamped target_pose_2 = createClipGoal(goal_frame_name, goal_vector_2);
  geometry_msgs::msg::PoseStamped target_pose_2_transformed = getPoseTransform(target_pose_2, "world");

  // Assign the goal pose based on y position
  if (target_pose_1_transformed.pose.position.y > target_pose_2_transformed.pose.position.y){
    leader_target_pose = target_pose_1;
    follower_target_pose = target_pose_2;

    // Rotate follow_target_pose around the x-axis by 45 degrees
    Eigen::AngleAxisd follow_rotation(-M_PI / 4, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond current_orientation(follower_target_pose.pose.orientation.w,
                                          follower_target_pose.pose.orientation.x,
                                          follower_target_pose.pose.orientation.y,
                                          follower_target_pose.pose.orientation.z);

    // Apply the rotation
    Eigen::Quaterniond updated_orientation = follow_rotation * current_orientation;
    // Update the pose's orientation
    follower_target_pose.pose.orientation.w = updated_orientation.w();
    follower_target_pose.pose.orientation.x = updated_orientation.x();
    follower_target_pose.pose.orientation.y = updated_orientation.y();
    follower_target_pose.pose.orientation.z = updated_orientation.z();
  } else {
    leader_target_pose = target_pose_2;
    follower_target_pose = target_pose_1;

    // Rotate follow_target_pose around the x-axis by 45 degrees
    Eigen::AngleAxisd follow_rotation(M_PI / 4, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond current_orientation(follower_target_pose.pose.orientation.w,
                                          follower_target_pose.pose.orientation.x,
                                          follower_target_pose.pose.orientation.y,
                                          follower_target_pose.pose.orientation.z);

    // Apply the rotation
    Eigen::Quaterniond updated_orientation = follow_rotation * current_orientation;
    // Update the pose's orientation
    follower_target_pose.pose.orientation.w = updated_orientation.w();
    follower_target_pose.pose.orientation.x = updated_orientation.x();
    follower_target_pose.pose.orientation.y = updated_orientation.y();
    follower_target_pose.pose.orientation.z = updated_orientation.z();
  }

  return std::make_pair(leader_target_pose, follower_target_pose);
}

geometry_msgs::msg::PoseStamped MTCTaskNode::createClipGoal(const std::string& goal_frame, const std::vector<double>& goal_translation_vector)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.pose.position.x = goal_translation_vector[0];
  goal_pose.pose.position.y = goal_translation_vector[1];
  goal_pose.pose.position.z = goal_translation_vector[2];

  goal_pose.header.frame_id = goal_frame;

  // Select orientaiton based on distance to current pose
  geometry_msgs::msg::PoseStamped current_pose = move_group_.getCurrentPose("right_panda_hand");
  // current pose in the goal frame
  geometry_msgs::msg::PoseStamped current_pose_transformed = getPoseTransform(current_pose, goal_frame);
  
  tf2::Quaternion current_orientation(
    current_pose_transformed.pose.orientation.x,
    current_pose_transformed.pose.orientation.y,
    current_pose_transformed.pose.orientation.z,
    current_pose_transformed.pose.orientation.w);
  
  // Get orientation in the goal frame
  tf2::Quaternion goal_orientation_1(0.7071068, -0.7071068, 0.0, 0.0);
  tf2::Quaternion goal_orientation_2(0.7071068, 0.7071068, 0.0, 0.0);

  tf2::Quaternion selected_orientation;
  if (select_orientation_){
    double angle_diff1 = current_orientation.angleShortestPath(goal_orientation_1);
    double angle_diff2 = current_orientation.angleShortestPath(goal_orientation_2);
    RCLCPP_INFO(LOGGER, "Angle diff 1: %f, Angle diff 2: %f", angle_diff1, angle_diff2);

    if (angle_diff1 < angle_diff2) {
        selected_orientation = goal_orientation_1;
    } else {
        selected_orientation = goal_orientation_2;
    }
  }else{
    selected_orientation = goal_orientation_1;
  }

  RCLCPP_INFO(LOGGER, "Selected orientation: x: %f, y: %f, z: %f, w: %f", selected_orientation.x(), selected_orientation.y(), selected_orientation.z(), selected_orientation.w());

  // Orientation from clip frame to robot EE frame
  goal_pose.pose.orientation.x = selected_orientation.x();
  goal_pose.pose.orientation.y = selected_orientation.y();
  goal_pose.pose.orientation.z = selected_orientation.z();
  goal_pose.pose.orientation.w = selected_orientation.w();

  return goal_pose;
}

void MTCTaskNode::doTask(std::string& start_clip_id, std::string& goal_clip_id, bool execute, bool plan_for_dual, bool split, bool cartesian_connect, bool approach,
                        std::function<mtc::Task(std::string&, std::string&, bool, bool, bool, bool)> createTaskFn)
{
  task_ = createTaskFn(start_clip_id, goal_clip_id, plan_for_dual, split, cartesian_connect, approach);

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

  // Publish the solution
  // task_.introspection().publishSolution(*task_.solutions().front(), publish_mtc_trajectory);
  moveit_task_constructor_msgs::msg::Solution msg;
	task_.introspection().fillSolution(msg, *task_.solutions().front());
  publishSolutionSubTraj(msg);

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

mtc::Task MTCTaskNode::createTask(std::string& start_frame_name, std::string& goal_frame_name, bool if_use_dual, bool if_split_plan, bool if_cartesian_connect, bool if_approach)
{
  mtc::Task task;
  task.stages()->setName("routing task");
  task.loadRobotModel(node_);

  // Initialize robot groups
  initializeGroups();

  // Set task properties (only valid for single arm)
  // if (!if_use_dual){
  //   task.setProperty("group", lead_arm_group_name);
  //   task.setProperty("eef", lead_hand_group_name);
  //   task.setProperty("ik_frame", lead_hand_frame);
  // }
  
  // delete markers
  visual_tools_.deleteAllMarkers();
  visual_tools_.trigger();

  // set target pose
  // geometry_msgs::msg::PoseStamped lead_target_pose = createClipGoal(goal_frame_name, leader_pre_insert_offset);
  // geometry_msgs::msg::PoseStamped follow_target_pose = createClipGoal(goal_frame_name, follower_pre_insert_offset);
  RCLCPP_INFO(LOGGER, "Grasping in clip frame : %s", start_frame_name.c_str());
  auto [lead_grasp_pose, follow_grasp_pose] = assignClipGoal(start_frame_name, leader_grasp_offset, follower_grasp_offset);

  RCLCPP_INFO(LOGGER, "Inserting in clip frame : %s", goal_frame_name.c_str());
  auto [lead_target_pose, follow_target_pose] = assignClipGoal(goal_frame_name, leader_pre_insert_offset, follower_pre_insert_offset);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  mtc::Stage* pre_move_stage_ptr = nullptr;
  mtc::Stage* pre_grasp_stage_ptr = nullptr;

  /****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
  {
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    pre_move_stage_ptr = stage_state_current.get();
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

  }
   
  // Set up planners
  initializePlanners();

  if (if_split_plan){

    /****************************************************
    ---- *               Open Hand                      *
    ***************************************************/
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", follow_interpolation_planner);
        stage->setGroup(follow_hand_group_name);
        stage->setGoal("open");

        pre_grasp_stage_ptr = stage.get();

        task.add(std::move(stage));
    }

    // check if the start_frame_name is empty
    if (start_frame_name.empty()){
      RCLCPP_WARN(LOGGER, "Start frame name is empty, skipping the grasping stage");
    }else{
      /****************************************************
       *                                                  *
       *              Connect to Grasp                     *
       *                                                  *
       ***************************************************/
      { 
        mtc::stages::Connect::GroupPlannerVector planners;
        mtc::stages::Connect::GroupPlannerVector interpolation_planners;
        mtc::stages::ConnectMF::GroupCartPlannerVector carteisan_planners;
        if (if_use_dual){
          // The order is important for collision checking!
          planners = {{lead_arm_group_name, lead_sampling_planner}, {follow_arm_group_name, follow_sampling_planner}};
          interpolation_planners = {{lead_arm_group_name, lead_interpolation_planner}, {follow_arm_group_name, follow_interpolation_planner}};
          carteisan_planners = {{lead_arm_group_name, lead_cartesian_planner}, {follow_arm_group_name, follow_cartesian_planner}};
        }else{
          planners = {{lead_arm_group_name, lead_sampling_planner}};
          interpolation_planners = {{lead_arm_group_name, lead_interpolation_planner}};
          carteisan_planners = {{lead_arm_group_name, lead_cartesian_planner}};
        }
        
        auto stage_move_to_align = std::make_unique<mtc::stages::ConnectMFSeq>("move to grasp", planners);
        stage_move_to_align->setTimeout(5.0);
        stage_move_to_align->properties().configureInitFrom(mtc::Stage::PARENT);
        stage_move_to_align->properties().set("lead_group", lead_arm_group_name);
        stage_move_to_align->properties().set("follow_group", follow_arm_group_name);  

        // add path constraints
        moveit_msgs::msg::Constraints path_constraints = createBoxConstraints(lead_hand_frame, lead_target_pose);
        stage_move_to_align->setPathConstraints(path_constraints);
        RCLCPP_INFO(LOGGER, "Path constraints set");

        // stage_move_to_align->properties().set("merge_mode", mtc::stages::ConnectMF::MergeMode::SEQUENTIAL);
      
        task.add(std::move(stage_move_to_align));
      }

    // /****************************************************
    // ---- *   Fixed Grasp Pose for dual arm *
    // ***************************************************/
      // {
      //   // Fixed align pose
      //   auto dual_grasped_pose = std::make_unique<mtc::stages::FixedCartesianPosesMultiple>("dual grasping pose");
      //   GroupPoseDict pose_pairs = {{follow_arm_group_name, follow_grasp_pose}, {lead_arm_group_name, lead_grasp_pose}};
      //   dual_grasped_pose->addPosePair(pose_pairs);
      //   dual_grasped_pose->setMonitoredStage(pre_grasp_stage_ptr);

      //   // IK frame at TCP
      //   Eigen::Isometry3d lead_grasp_frame_transform = Eigen::Isometry3d::Identity();
      //   lead_grasp_frame_transform.translation().z() = 0.1034;
      //   Eigen::Isometry3d follow_grasp_frame_transform = Eigen::Isometry3d::Identity();
      //   follow_grasp_frame_transform.translation().z() = 0.1034;

      //   // IK groups
      //   std::vector<std::string> ik_groups = {follow_arm_group_name, lead_arm_group_name};
      //   GroupStringDict ik_endeffectors = {{follow_arm_group_name, follow_hand_group_name}, {lead_arm_group_name, lead_hand_group_name}};
      //   GroupStringDict ik_hand_frames = {{follow_arm_group_name, follow_hand_frame}, {lead_arm_group_name, lead_hand_frame}, };
      //   // GroupStringDict ik_links = {{lead_arm_group, "right_arm_hand"}, {follow_arm_group, "left_arm_hand"}};
      //   GroupPoseMatrixDict ik_frame_transforms = {{follow_arm_group_name, follow_grasp_frame_transform}, {lead_arm_group_name, lead_grasp_frame_transform}};

      //   // Compute IK
      //   auto ik_wrapper = std::make_unique<mtc::stages::ComputeIKMultiple>("grasping pose IK", std::move(dual_grasped_pose), ik_groups, dual_arm_group_name);
      //   ik_wrapper->setSubGroups(ik_groups);
      //   ik_wrapper->setGroup(dual_arm_group_name);
      //   ik_wrapper->setEndEffector(ik_endeffectors);
      //   // ik_wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
      //   ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_poses"});
      //   // ik_wrapper->properties().set("object", "object");
      //   ik_wrapper->setMaxIKSolutions(20);
      //   ik_wrapper->setMinSolutionDistance(1.0);
      //   ik_wrapper->setIKFrame(ik_frame_transforms, ik_hand_frames);

      //   task.add(std::move(ik_wrapper));

      // }

        /***************************************************
      ---- * Generate Grasp Pose for dual arm              *
        ***************************************************/
      {
        // Target positions in clip frame
        GroupStringDict goal_frames = {{lead_arm_group_name, start_frame_name}, {follow_arm_group_name, start_frame_name}};

        // std::vector<double> lead_goal_delta_vector = {lead_grasp_pose.pose.position.x, lead_grasp_pose.pose.position.y, lead_grasp_pose.pose.position.z};
        // std::vector<double> follow_goal_delta_vector = {follow_grasp_pose.pose.position.x, follow_grasp_pose.pose.position.y, follow_grasp_pose.pose.position.z};
        // std::vector<double> lead_goal_orient_vector = {lead_grasp_pose.pose.orientation.x, lead_grasp_pose.pose.orientation.y, lead_grasp_pose.pose.position.z, lead_grasp_pose.pose.orientation.w};
        // std::vector<double> follow_goal_orient_vector = {follow_grasp_pose.pose.orientation.x, follow_grasp_pose.pose.orientation.y, follow_grasp_pose.pose.position.z, follow_grasp_pose.pose.orientation.w};
        // GroupVectorDict delta_pairs = {{lead_arm_group_name, lead_goal_delta_vector}, {follow_arm_group_name, follow_goal_delta_vector}};
        // GroupVectorDict orient_pairs = {{lead_arm_group_name, lead_goal_orient_vector}, {follow_arm_group_name, follow_goal_orient_vector}};
        GroupPoseDict pose_pairs = {{follow_arm_group_name, follow_grasp_pose}, {lead_arm_group_name, lead_grasp_pose}};

        // IK groups
        std::vector<std::string> ik_groups = {follow_arm_group_name, lead_arm_group_name};
        GroupStringDict ik_endeffectors = {{follow_arm_group_name, follow_hand_group_name}, {lead_arm_group_name, lead_hand_group_name}};
        GroupStringDict ik_hand_frames = {{follow_arm_group_name, follow_hand_frame}, {lead_arm_group_name, lead_hand_frame}, };
        // GroupStringDict ik_links = {{lead_arm_group, "right_arm_hand"}, {follow_arm_group, "left_arm_hand"}};
        GroupPoseMatrixDict ik_frame_transforms = {{follow_arm_group_name, hand_to_tcp_transform_}, {lead_arm_group_name, hand_to_tcp_transform_}};
        GroupStringDict pre_grasp_pose = {{follow_arm_group_name, "open"}, {lead_arm_group_name, "close"}};

        // generate grasp pose, randomize for follower
        auto grasp_generator = std::make_unique<mtc::stages::GenerateGraspPoseDual>("generate grasp pose dual", ik_groups);
        grasp_generator->setEndEffector(ik_endeffectors);
        grasp_generator->properties().set("marker_ns", "grasp_pose");
        grasp_generator->properties().set("explr_axis", "y");
        grasp_generator->setAngleDelta(0.2); // enumerate over angles from 0 to 6.4 (less then 2 PI)
        grasp_generator->setPreGraspPose(pre_grasp_pose);
        grasp_generator->setGraspPose("close");
        grasp_generator->setObject(goal_frames); // object sets target pose frame
        grasp_generator->setTargetPoseInObject(pose_pairs);
        // grasp_generator->setTargetDelta(delta_pairs);
        // grasp_generator->setTargetOrient(orient_pairs);
        grasp_generator->setMonitoredStage(pre_grasp_stage_ptr);
        grasp_generator->properties().set("generate_group", follow_arm_group_name);
        grasp_generator->properties().set("planning_frame", start_frame_name);

        // Compute IK
        auto ik_wrapper = std::make_unique<mtc::stages::ComputeIKMultiple>("grasping pose IK", std::move(grasp_generator), ik_groups, dual_arm_group_name);
        ik_wrapper->setSubGroups(ik_groups);
        ik_wrapper->setGroup(dual_arm_group_name);
        ik_wrapper->setEndEffector(ik_endeffectors);
        // ik_wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
        ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_poses"});
        // ik_wrapper->properties().set("object", "object");
        ik_wrapper->setMaxIKSolutions(10);
        ik_wrapper->setMinSolutionDistance(1.0);
        ik_wrapper->setIKFrame(ik_frame_transforms, ik_hand_frames);

        task.add(std::move(ik_wrapper));
      }
    }
  }

       /****************************************************
    ---- *               Open Hand                      *
    ***************************************************/
   {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", follow_interpolation_planner);
    stage->setGroup(follow_hand_group_name);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  /****************************************************
	 *                                                  *
	 *              Connect to Align                     *
	 *                                                  *
	 ***************************************************/
  { 
    mtc::stages::Connect::GroupPlannerVector planners;
    mtc::stages::Connect::GroupPlannerVector interpolation_planners;
    mtc::stages::ConnectMF::GroupCartPlannerVector carteisan_planners;
    mtc::stages::ConnectMFReverse::GroupPipePlannerVector chomp_planners;
    mtc::stages::Connect::GroupPlannerVector hand_planners;
    if (if_use_dual){
      // The order is important for collision checking!
      planners = {{lead_arm_group_name, lead_sampling_planner}, {follow_arm_group_name, follow_sampling_planner}};
      interpolation_planners = {{lead_arm_group_name, lead_interpolation_planner}, {follow_arm_group_name, follow_interpolation_planner}};
      carteisan_planners = {{lead_arm_group_name, lead_cartesian_planner}, {follow_arm_group_name, follow_cartesian_planner}};
      chomp_planners = {{lead_arm_group_name, lead_chomp_planner}, {follow_arm_group_name, follow_chomp_planner}};
      hand_planners = {{lead_hand_group_name, lead_interpolation_planner}, {follow_hand_group_name, follow_interpolation_planner}};
    }else{
      planners = {{lead_arm_group_name, lead_sampling_planner}};
      interpolation_planners = {{lead_arm_group_name, lead_interpolation_planner}};
      carteisan_planners = {{lead_arm_group_name, lead_cartesian_planner}};
    }
    
    if (if_cartesian_connect){
      GroupStringDict ik_endeffectors = {{follow_arm_group_name, follow_hand_group_name}, {lead_arm_group_name, lead_hand_group_name}};

      moveit::planning_interface::MoveGroupInterfacePtr lead_move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, lead_arm_group_name);
      moveit::planning_interface::MoveGroupInterfacePtr follow_move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, follow_arm_group_name);
      
      if (if_split_plan){
        auto stage_move_to_align = std::make_unique<mtc::stages::ConnectMFPrl>("move to align", planners, interpolation_planners, carteisan_planners, follow_move_group_interface, visual_tools_);
        stage_move_to_align->setTimeout(5.0);
        stage_move_to_align->properties().configureInitFrom(mtc::Stage::PARENT);
        stage_move_to_align->properties().set("lead_group", lead_arm_group_name);
        stage_move_to_align->properties().set("follow_group", follow_arm_group_name);      
        stage_move_to_align->properties().set("lead_hand_to_tcp_transform", hand_to_tcp_transform_);
        stage_move_to_align->properties().set("follow_hand_to_tcp_transform", hand_to_tcp_transform_);
        // stage_move_to_align->properties().set("merge_mode", mtc::stages::ConnectMF::MergeMode::SEQUENTIAL);
        stage_move_to_align->setEndEffector(ik_endeffectors);

        // add path constraints
        moveit_msgs::msg::Constraints path_constraints = createBoxConstraints(lead_hand_frame, lead_target_pose);
        stage_move_to_align->setPathConstraints(path_constraints);
        RCLCPP_INFO(LOGGER, "Path constraints set");

        task.add(std::move(stage_move_to_align));
      }else{
        // auto stage_move_to_align = std::make_unique<mtc::stages::ConnectMF>("move to align", planners, interpolation_planners, carteisan_planners, follow_move_group_interface, visual_tools_);
        // stage_move_to_align->setTimeout(5.0);
        // stage_move_to_align->properties().configureInitFrom(mtc::Stage::PARENT);
        // stage_move_to_align->properties().set("lead_group", lead_arm_group_name);
        // stage_move_to_align->properties().set("follow_group", follow_arm_group_name);      
        // stage_move_to_align->properties().set("lead_hand_to_tcp_transform", hand_to_tcp_transform_);
        // stage_move_to_align->properties().set("follow_hand_to_tcp_transform", hand_to_tcp_transform_);
        
        // geometry_msgs::msg::PoseStamped leader_grasp_pose_transformed = getPoseTransform(lead_grasp_pose, "world");
        // stage_move_to_align->properties().set("lead_grasp_pose", leader_grasp_pose_transformed);
        
        // stage_move_to_align->properties().set("track_offset", 0.1);
        // stage_move_to_align->properties().set("follow_grasp_offset", 0.03);
        // // stage_move_to_align->properties().set("merge_mode", mtc::stages::ConnectMF::MergeMode::SEQUENTIAL);
        // stage_move_to_align->setEndEffector(ik_endeffectors);

        // // add path constraints
        // moveit_msgs::msg::Constraints path_constraints = createBoxConstraints(lead_hand_frame, lead_target_pose);
        // stage_move_to_align->setPathConstraints(path_constraints);
        // RCLCPP_INFO(LOGGER, "Path constraints set");

        // task.add(std::move(stage_move_to_align));

        auto stage_move_to_align = std::make_unique<mtc::stages::ConnectMFReverse>("move to align", planners, interpolation_planners, carteisan_planners, chomp_planners,
                                                                                    hand_planners, lead_move_group_interface, visual_tools_);
        stage_move_to_align->setTimeout(10.0);
        stage_move_to_align->properties().configureInitFrom(mtc::Stage::PARENT);
        stage_move_to_align->properties().set("lead_group", lead_arm_group_name);
        stage_move_to_align->properties().set("lead_hand_group", lead_hand_group_name);
        stage_move_to_align->properties().set("follow_group", follow_arm_group_name);
        stage_move_to_align->properties().set("follow_hand_group", follow_hand_group_name);      
        stage_move_to_align->properties().set("hand_to_tcp_transform", hand_to_tcp_transform_);
        stage_move_to_align->properties().set("lead_flange_to_tcp_transform", lead_flange_to_tcp_transform_);
        stage_move_to_align->properties().set("follow_flange_to_tcp_transform", follow_flange_to_tcp_transform_);
        
        geometry_msgs::msg::PoseStamped follower_grasp_pose_transformed = getPoseTransform(follow_grasp_pose, "world");
        stage_move_to_align->properties().set("follow_grasp_pose", follower_grasp_pose_transformed);
        geometry_msgs::msg::PoseStamped leader_grasp_pose_transformed = getPoseTransform(lead_grasp_pose, "world");
        stage_move_to_align->properties().set("lead_grasp_pose", leader_grasp_pose_transformed);
        
        stage_move_to_align->properties().set("track_offset", 0.1);
        stage_move_to_align->properties().set("follow_grasp_offset", 0.03);
        // stage_move_to_align->properties().set("merge_mode", mtc::stages::ConnectMF::MergeMode::SEQUENTIAL);
        stage_move_to_align->setEndEffector(ik_endeffectors);

        // add path constraints
        moveit_msgs::msg::Constraints path_constraints = createBoxConstraints(lead_hand_frame, lead_target_pose);
        stage_move_to_align->setPathConstraints(path_constraints);
        RCLCPP_INFO(LOGGER, "Path constraints set");

        task.add(std::move(stage_move_to_align));
      }

    }else{
      auto stage_move_to_align = std::make_unique<mtc::stages::Connect>("move to align", planners, hand_planners);
      stage_move_to_align->setTimeout(5.0);
      stage_move_to_align->properties().configureInitFrom(mtc::Stage::PARENT);

      // add path constraints
      moveit_msgs::msg::Constraints path_constraints = createBoxConstraints(lead_hand_frame, lead_target_pose);
      stage_move_to_align->setPathConstraints(path_constraints);
      RCLCPP_INFO(LOGGER, "Path constraints set");

      // stage_move_to_align->properties().set("merge_mode", mtc::stages::ConnectMF::MergeMode::SEQUENTIAL);
    
      task.add(std::move(stage_move_to_align));
    }
  }
  
  /****************************************************
	 *                                                  *
	 *               Pick Container                     *
	 *                                                  *
	 ***************************************************/
  mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator
  {
    auto align = std::make_unique<mtc::SerialContainer>("pick object");
    // if (!if_use_dual){
    //   task.properties().exposeTo(align->properties(), { "eef", "group", "ik_frame" });
    //   align->properties().configureInitFrom(mtc::Stage::PARENT,
    //                                         { "eef", "group", "ik_frame" });
    // }

  /****************************************************
  ---- *               Follower Grasping              *
  ***************************************************/
  {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", follow_interpolation_planner);
      stage->setGroup(follow_hand_group_name);
      stage->setGoal("close");

      align->insert(std::move(stage));
  }

    /****************************************************
  ---- *               Insertion in EE-z                *
    ***************************************************/
    // {
    //   auto stage =
    //       std::make_unique<mtc::stages::MoveRelative>("insertion", lead_cartesian_planner);
    //   stage->properties().set("marker_ns", "insertion");
    //   stage->properties().set("link", lead_hand_frame);
    //   // stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    //   stage->setGroup(lead_arm_group_name);
    //   stage->setMinMaxDistance(0.1, 0.15);

    //   // Set hand forward direction
    //   geometry_msgs::msg::Vector3Stamped vec; 
    //   vec.header.frame_id = lead_hand_frame;
    //   vec.vector.z = 0.1;
    //   stage->setDirection(vec);
    //   align->insert(std::move(stage));
    // }

  //     /****************************************************
  // ---- *              Dual Insertion in EE-z            *
  //   ***************************************************/
    // { // This will add an offset to the target pose in the pose generator stage
    //   mtc::stages::MoveRelativeMultiple::GroupPlannerVector cartesian_planners;
    //   cartesian_planners = {{follow_arm_group_name, follow_cartesian_planner}, {lead_arm_group_name, lead_cartesian_planner}};

    //   auto stage =
    //       std::make_unique<mtc::stages::MoveRelativeMultiple>("insertion", cartesian_planners);
    //   stage->properties().set("marker_ns", "insertion");
    //   // stage->properties().set("link", lead_hand_frame);

    //   // IK frame at TCP
    //   Eigen::Isometry3d lead_grasp_frame_transform = Eigen::Isometry3d::Identity();
    //   lead_grasp_frame_transform.translation().z() = 0.1034;
    //   Eigen::Isometry3d follow_grasp_frame_transform = Eigen::Isometry3d::Identity();
    //   follow_grasp_frame_transform.translation().z() = 0.1034;

    //   GroupStringDict ik_hand_frames = {{follow_arm_group_name, follow_hand_frame}, {lead_arm_group_name, lead_hand_frame}};
    //   GroupPoseMatrixDict ik_frame_transforms = {{follow_arm_group_name, follow_grasp_frame_transform}, {lead_arm_group_name, lead_grasp_frame_transform}};
      
    //   stage->setIKFrame(ik_frame_transforms, ik_hand_frames);
    //   stage->setGroup({follow_arm_group_name, lead_arm_group_name});
    //   stage->setMinMaxDistance(0.05, 0.15);

    //   // Set hand forward direction
    //   geometry_msgs::msg::Vector3Stamped vec; 
    //   vec.header.frame_id = goal_frame_name;
    //   // vec.vector.z = -0.05;
    //   vec.vector.x = -0.03;
    //   stage->setDirection(vec);
    //   task.add(std::move(stage));
    // }

      /****************************************************
  ---- *              Dual Insertion in EE-z            *
    ***************************************************/
    if (if_approach){
      mtc::stages::MoveRelativeMultiple::GroupPlannerVector cartesian_planners;
      cartesian_planners = {{follow_arm_group_name, follow_cartesian_planner}, {lead_arm_group_name, lead_cartesian_planner}};

      auto stage =
          std::make_unique<mtc::stages::MoveRelativeMultiple>("insertion", cartesian_planners);
      stage->properties().set("marker_ns", "insertion");
      // stage->properties().set("link", lead_hand_frame);

      GroupStringDict ik_hand_frames = {{follow_arm_group_name, follow_hand_frame}, {lead_arm_group_name, lead_hand_frame}};
      GroupPoseMatrixDict ik_frame_transforms = {{follow_arm_group_name, follow_flange_to_tcp_transform_}, {lead_arm_group_name, lead_flange_to_tcp_transform_}};
      
      stage->setIKFrame(ik_frame_transforms, ik_hand_frames);
      stage->setGroup({follow_arm_group_name, lead_arm_group_name});
      stage->setMinMaxDistance(0.02, 0.08);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec; 
      vec.header.frame_id = goal_frame_name;
      // vec.vector.z = -0.05;
      // vec.vector.z = -(insertion_offset_magnitude+0.02);
      // vec.vector.x = -(insertion_offset_magnitude+0.02);
      vec.vector.x = insertion_vector[0];
      vec.vector.y = insertion_vector[1];
      vec.vector.z = insertion_vector[2];
      stage->setDirection(vec);
      // task.add(std::move(stage));
      align->insert(std::move(stage));
    }

    /****************************************************
  ---- *     Fixed Align Pose for single arm *
	***************************************************/
    if (!if_use_dual){    
      // Fixed align pose
      auto stage = std::make_unique<mtc::stages::FixedCartesianPoses>("fixed clipping pose");
      stage->addPose(lead_target_pose);
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
      wrapper->setIKFrame(grasp_frame_transform, lead_hand_frame);
      // wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->setGroup(lead_arm_group_name);
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose", "eef"});
      align->insert(std::move(wrapper));
    }

  /****************************************************
  ---- * Generate Align Pose for single arm (not finished)*
	***************************************************/
    // if (if_use_dual && if_split_plan){
    // // if (!if_use_dual){
    //   // Sample align pose
    //   auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
    //   // stage->properties().configureInitFrom(mtc::Stage::PARENT);
    //   stage->properties().set("eef", lead_hand_group_name);
    //   stage->properties().set("marker_ns", "align_pose");
    //   stage->setPreGraspPose("close");
    //   stage->setObject(goal_frame_name);
    //   stage->setAngleDelta(M_PI / 12);
    //   stage->setMonitoredStage(current_state_ptr);  // Hook into current state

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
    //   // wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    //   wrapper->setGroup(lead_arm_group_name);
    //   wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose", "eef"});
    //   align->insert(std::move(wrapper));
    // }

  //   /****************************************************
  // ---- *   Fixed Target Pose for dual arm *
	// ***************************************************/
    // {
    //   // Fixed align pose
    //   auto dual_fixed_pose = std::make_unique<mtc::stages::FixedCartesianPosesMultiple>("dual fixed clipping pose");
    //   GroupPoseDict pose_pairs = {{follow_arm_group_name, follow_target_pose}, {lead_arm_group_name, lead_target_pose}};
    //   dual_fixed_pose->addPosePair(pose_pairs);
    //   dual_fixed_pose->setMonitoredStage(pre_move_stage_ptr);

    //   // IK frame at TCP
    //   Eigen::Isometry3d lead_grasp_frame_transform = Eigen::Isometry3d::Identity();
    //   lead_grasp_frame_transform.translation().z() = 0.1034;
    //   Eigen::Isometry3d follow_grasp_frame_transform = Eigen::Isometry3d::Identity();
    //   follow_grasp_frame_transform.translation().z() = 0.1034;

    //   // IK groups
    //   std::vector<std::string> ik_groups = {follow_arm_group_name, lead_arm_group_name};
    //   GroupStringDict ik_endeffectors = {{follow_arm_group_name, follow_hand_group_name}, {lead_arm_group_name, lead_hand_group_name}};
    //   GroupStringDict ik_hand_frames = {{follow_arm_group_name, follow_hand_frame}, {lead_arm_group_name, lead_hand_frame}, };
    //   // GroupStringDict ik_links = {{lead_arm_group, "right_arm_hand"}, {follow_arm_group, "left_arm_hand"}};
    //   GroupPoseMatrixDict ik_frame_transforms = {{follow_arm_group_name, follow_grasp_frame_transform}, {lead_arm_group_name, lead_grasp_frame_transform}};

    //   // Compute IK
    //   auto ik_wrapper = std::make_unique<mtc::stages::ComputeIKMultiple>("clipping pose IK", std::move(dual_fixed_pose), ik_groups, dual_arm_group_name);
    //   ik_wrapper->setSubGroups(ik_groups);
    //   ik_wrapper->setGroup(dual_arm_group_name);
    //   ik_wrapper->setEndEffector(ik_endeffectors);
    //   // ik_wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
    //   ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_poses"});
    //   // ik_wrapper->properties().set("object", "object");
    //   ik_wrapper->setMaxIKSolutions(20);
    //   ik_wrapper->setMinSolutionDistance(1.0);
    //   ik_wrapper->setIKFrame(ik_frame_transforms, ik_hand_frames);

    //   align->insert(std::move(ik_wrapper));
    // }

  /****************************************************
  ---- *    Generate Target Pose for dual arm *
	***************************************************/
    if (if_use_dual){
      // Target positions in clip frame
      GroupStringDict goal_frames = {{lead_arm_group_name, goal_frame_name}, {follow_arm_group_name, goal_frame_name}};

      // std::vector<double> lead_goal_delta_vector = {lead_target_pose.pose.position.x, lead_target_pose.pose.position.y, lead_target_pose.pose.position.z};
      // std::vector<double> follow_goal_delta_vector = {follow_target_pose.pose.position.x, follow_target_pose.pose.position.y, follow_target_pose.pose.position.z};
      // std::vector<double> lead_goal_orient_vector = {lead_target_pose.pose.orientation.x, lead_target_pose.pose.orientation.y, lead_target_pose.pose.position.z, lead_target_pose.pose.orientation.w};
      // std::vector<double> follow_goal_orient_vector = {follow_target_pose.pose.orientation.x, follow_target_pose.pose.orientation.y, follow_target_pose.pose.position.z, follow_target_pose.pose.orientation.w};
      // GroupVectorDict delta_pairs = {{lead_arm_group_name, lead_goal_delta_vector}, {follow_arm_group_name, follow_goal_delta_vector}};
      // GroupVectorDict orient_pairs = {{lead_arm_group_name, lead_goal_orient_vector}, {follow_arm_group_name, follow_goal_orient_vector}};
      
      GroupPoseDict pose_pairs = {{follow_arm_group_name, follow_target_pose}, {lead_arm_group_name, lead_target_pose}};

      // IK groups
      std::vector<std::string> ik_groups = {follow_arm_group_name, lead_arm_group_name};
      GroupStringDict ik_endeffectors = {{follow_arm_group_name, follow_hand_group_name}, {lead_arm_group_name, lead_hand_group_name}};
      GroupStringDict ik_hand_frames = {{follow_arm_group_name, follow_hand_frame}, {lead_arm_group_name, lead_hand_frame}, };
      // GroupStringDict ik_links = {{lead_arm_group, "right_arm_hand"}, {follow_arm_group, "left_arm_hand"}};
      GroupPoseMatrixDict ik_frame_transforms = {{follow_arm_group_name, hand_to_tcp_transform_}, {lead_arm_group_name, hand_to_tcp_transform_}};
      GroupStringDict pre_grasp_pose = {{follow_arm_group_name, "close"}, {lead_arm_group_name, "close"}};

      // generate grasp pose, randomize for follower
      auto grasp_generator = std::make_unique<mtc::stages::GenerateGraspPoseDual>("generate clipping pose", ik_groups);
      grasp_generator->setEndEffector(ik_endeffectors);
      grasp_generator->properties().set("marker_ns", "align_pose");
      grasp_generator->properties().set("explr_axis", "x");
      grasp_generator->setAngleDelta(0.2); // enumerate over angles from 0 to 6.4 (less then 2 PI)
      grasp_generator->setPreGraspPose(pre_grasp_pose);
      grasp_generator->setGraspPose("close");
      grasp_generator->setObject(goal_frames); // object sets target pose frame
      grasp_generator->setTargetPoseInObject(pose_pairs);
      // grasp_generator->setTargetDelta(delta_pairs);
      // grasp_generator->setTargetOrient(orient_pairs);
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
      ik_wrapper->setMaxIKSolutions(10);
      ik_wrapper->setMinSolutionDistance(1.0);
      ik_wrapper->setIKFrame(ik_frame_transforms, ik_hand_frames);

      align->insert(std::move(ik_wrapper));
    }
    /****************************************************
  ---- *               Allow Collision (hand object)   *
	***************************************************/
    // {
    //   auto stage =
    //       std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
    //   stage->allowCollisions("object",
    //                         task.getRobotModel()
    //                             ->getJointModelGroup(lead_hand_group_name)
    //                             ->getLinkModelNamesWithCollisionGeometry(),
    //                         true);
    //   align->insert(std::move(stage));
    // }

    /****************************************************
  ---- *               Close Hand                      *
	***************************************************/
    // {
    //   auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", lead_interpolation_planner);
    //   stage->setGroup(lead_hand_group_name);
    //   stage->setGoal("close");
    //   align->insert(std::move(stage));
    // }

  //   /****************************************************
  // ---- *              Dual Insertion in EE-z            *
  //   ***************************************************/
  //   {
  //     mtc::stages::MoveRelativeMultiple::GroupPlannerVector cartesian_planners;
  //     cartesian_planners = {{follow_arm_group_name, follow_cartesian_planner}, {lead_arm_group_name, lead_cartesian_planner}};

  //     auto stage =
  //         std::make_unique<mtc::stages::MoveRelativeMultiple>("insertion", cartesian_planners);
  //     stage->properties().set("marker_ns", "insertion");
  //     // stage->properties().set("link", lead_hand_frame);

  //     // IK frame at TCP
  //     Eigen::Isometry3d lead_grasp_frame_transform = Eigen::Isometry3d::Identity();
  //     lead_grasp_frame_transform.translation().z() = 0.1034;
  //     Eigen::Isometry3d follow_grasp_frame_transform = Eigen::Isometry3d::Identity();
  //     follow_grasp_frame_transform.translation().z() = 0.1034;

  //     GroupStringDict ik_hand_frames = {{follow_arm_group_name, follow_hand_frame}, {lead_arm_group_name, lead_hand_frame}};
  //     GroupPoseMatrixDict ik_frame_transforms = {{follow_arm_group_name, follow_grasp_frame_transform}, {lead_arm_group_name, lead_grasp_frame_transform}};
      
  //     stage->setIKFrame(ik_frame_transforms, ik_hand_frames);
  //     stage->setGroup({follow_arm_group_name, lead_arm_group_name});
  //     // stage->setMinMaxDistance(0.1, 0.15);

  //     // Set hand forward direction
  //     geometry_msgs::msg::Vector3Stamped vec; 
  //     vec.header.frame_id = goal_frame_name;
  //     // vec.vector.z = -0.05;
  //     // vec.vector.z = -(insertion_offset_magnitude+0.02);
  //     // vec.vector.x = -(insertion_offset_magnitude+0.02);
  //     vec.vector.x = insertion_vector[0];
  //     vec.vector.y = insertion_vector[1];
  //     vec.vector.z = insertion_vector[2];
  //     stage->setDirection(vec);
  //     // task.add(std::move(stage));
  //     align->insert(std::move(stage));
  //   }

    task.add(std::move(align));
  }

  //    /****************************************************
  // ---- *               Open Hand                      *
	// ***************************************************/
  // {
  //   auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", follow_interpolation_planner);
  //   stage->setGroup(follow_hand_group_name);
  //   stage->setGoal("open");
  //   task.add(std::move(stage));
  // }

  //  /****************************************************
  // ---- *               Retrieve in EE-z                *
  //   ***************************************************/
  // {
  //   auto stage =
  //       std::make_unique<mtc::stages::MoveRelative>("insertion", follow_cartesian_planner);
  //   stage->properties().set("marker_ns", "insertion");
  //   stage->properties().set("link", follow_hand_frame);
  //   // stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //   stage->setGroup(follow_arm_group_name);
  //   stage->setMinMaxDistance(0.1, 0.15);

  //   // Set hand forward direction
  //   geometry_msgs::msg::Vector3Stamped vec;
  //   // vec.header.frame_id = follow_hand_frame;
  //   vec.header.frame_id = goal_frame_name;
  //   vec.vector.z = 0.03;
  //   stage->setDirection(vec);
  //   task.add(std::move(stage));
  // }


  return task;
}

mtc::Task MTCTaskNode::createReverseTask(std::string& start_frame_name, std::string& goal_frame_name, bool if_use_dual, bool if_split_plan, bool if_cartesian_connect, bool if_approach)
{
  /*Attempt to plan from generated IK goal back to the current scene.
    Not finished: trajectory from connect_reverse stage is promising, but task is somehow still marks as failure.*/
  mtc::Task task;
  task.stages()->setName("routing task");
  task.loadRobotModel(node_);

  // Initialize robot groups
  initializeGroups();

  // Set task properties (only valid for single arm)
  // if (!if_use_dual){
  //   task.setProperty("group", lead_arm_group_name);
  //   task.setProperty("eef", lead_hand_group_name);
  //   task.setProperty("ik_frame", lead_hand_frame);
  // }
  
  // delete markers
  visual_tools_.deleteAllMarkers();
  visual_tools_.trigger();

  // set target pose
  // geometry_msgs::msg::PoseStamped lead_target_pose = createClipGoal(goal_frame_name, leader_pre_insert_offset);
  // geometry_msgs::msg::PoseStamped follow_target_pose = createClipGoal(goal_frame_name, follower_pre_insert_offset);
  RCLCPP_INFO(LOGGER, "Grasping in clip frame : %s", start_frame_name.c_str());
  auto [lead_grasp_pose, follow_grasp_pose] = assignClipGoal(start_frame_name, leader_grasp_offset, follower_grasp_offset);

  RCLCPP_INFO(LOGGER, "Inserting in clip frame : %s", goal_frame_name.c_str());
  auto [lead_target_pose, follow_target_pose] = assignClipGoal(goal_frame_name, leader_pre_insert_offset, follower_pre_insert_offset);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  mtc::Stage* pre_move_stage_ptr = nullptr;
  mtc::Stage* pre_grasp_stage_ptr = nullptr;

 
  /****************************************************
	 *                                                  *
	 *               Start from Goal State              *
	 *                                                  *
	 ***************************************************/
  // Set up planners
  initializePlanners();

     /****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
  {
    // Add the monitored stage first, so IK sees real frame updates
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    pre_move_stage_ptr = stage_state_current.get();  // Set monitored before consuming
    current_state_ptr = stage_state_current.get();

    task.add(std::move(stage_state_current));

  }
   

  /****************************************************
	 *                                                  *
	 *              Connect to Align                     *
	 *                                                  *
	 ***************************************************/
  { 
    mtc::stages::Connect::GroupPlannerVector planners;
    mtc::stages::Connect::GroupPlannerVector interpolation_planners;
    mtc::stages::ConnectMF::GroupCartPlannerVector carteisan_planners;
    mtc::stages::ConnectMFReverse::GroupPipePlannerVector chomp_planners;
    if (if_use_dual){
      // The order is important for collision checking!
      planners = {{lead_arm_group_name, lead_sampling_planner}, {follow_arm_group_name, follow_sampling_planner}};
      interpolation_planners = {{lead_arm_group_name, lead_interpolation_planner}, {follow_arm_group_name, follow_interpolation_planner}};
      carteisan_planners = {{lead_arm_group_name, lead_cartesian_planner}, {follow_arm_group_name, follow_cartesian_planner}};
      chomp_planners = {{lead_arm_group_name, lead_chomp_planner}, {follow_arm_group_name, follow_chomp_planner}};
    }else{
      planners = {{lead_arm_group_name, lead_sampling_planner}};
      interpolation_planners = {{lead_arm_group_name, lead_interpolation_planner}};
      carteisan_planners = {{lead_arm_group_name, lead_cartesian_planner}};
    }
    
      auto stage_move_to_align = std::make_unique<mtc::stages::ConnectReverse>("move to align", planners);
      stage_move_to_align->setTimeout(5.0);
      stage_move_to_align->properties().configureInitFrom(mtc::Stage::PARENT);

      // add path constraints
      moveit_msgs::msg::Constraints path_constraints = createBoxConstraints(lead_hand_frame, lead_target_pose);
      stage_move_to_align->setPathConstraints(path_constraints);
      RCLCPP_INFO(LOGGER, "Path constraints set");

      // stage_move_to_align->properties().set("merge_mode", mtc::stages::ConnectMF::MergeMode::SEQUENTIAL);
    
      task.add(std::move(stage_move_to_align));
  
  }

  auto ik_chain = std::make_unique<mtc::SerialContainer>("IK chain");
  ik_chain->setForwardedProperties({"target_poses"});
  // ik_chain->setForwardedInterface(mtc::Stage::InterfaceFlags::ALL);

 /****************************************************
  ---- *   Fixed Clipping Pose for dual arm *
  ***************************************************/
  {
    // Fixed align pose
    auto dual_fixed_pose = std::make_unique<mtc::stages::FixedCartesianPosesMultiple>("dual fixed clipping pose");
    GroupPoseDict pose_pairs = {{follow_arm_group_name, follow_target_pose}, {lead_arm_group_name, lead_target_pose}};
    dual_fixed_pose->addPosePair(pose_pairs);
    dual_fixed_pose->setMonitoredStage(pre_move_stage_ptr);

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

    // Compute IK
    auto ik_wrapper = std::make_unique<mtc::stages::ComputeIKMultiple>("clipping pose IK", std::move(dual_fixed_pose), ik_groups, dual_arm_group_name);
    ik_wrapper->setSubGroups(ik_groups);
    ik_wrapper->setGroup(dual_arm_group_name);
    ik_wrapper->setEndEffector(ik_endeffectors);
    // ik_wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
    ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_poses"});
    // ik_wrapper->properties().set("object", "object");
    ik_wrapper->setMaxIKSolutions(20);
    ik_wrapper->setMinSolutionDistance(1.0);
    ik_wrapper->setIKFrame(ik_frame_transforms, ik_hand_frames);

    ik_chain->insert(std::move(ik_wrapper));
    // task.add(std::move(ik_wrapper));
  }


  {
    auto noop = std::make_unique<mtc::stages::NoOp>("mark end");
    noop->setForwardedProperties({"target_poses"});
    // task.add(std::move(noop));
    ik_chain->insert(std::move(noop));
  }

  task.add(std::move(ik_chain));

  return task;
}

mtc::Task MTCTaskNode::createTestWaypointTask(std::string& goal_frame_name, bool if_use_dual, bool if_split_plan, bool if_cartesian_connect, bool if_approach)
{
  mtc::Task task;
  task.stages()->setName("test waypoint task");
  task.loadRobotModel(node_);

  // Initialize robot groups
  initializeGroups();

  // delete markers
  visual_tools_.deleteAllMarkers();
  visual_tools_.trigger();

  // set target pose
  geometry_msgs::msg::PoseStamped lead_target_pose = createClipGoal(goal_frame_name, leader_pre_insert_offset);
  geometry_msgs::msg::PoseStamped follow_target_pose = createClipGoal(goal_frame_name, follower_pre_insert_offset);

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

    // pre_move_stage_ptr = stage_state_current.get();
  }

  // Set up planners
  initializePlanners();

  /****************************************************
  ---- *               Close Hand                      *
  ***************************************************/
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", follow_interpolation_planner);
    stage->setGroup(follow_hand_group_name);
    stage->setGoal("close");

    pre_move_stage_ptr = stage.get();

    task.add(std::move(stage));
  }

  /****************************************************
  ---- *  Compute Cartesian Waypoints for single arm *
	***************************************************/
  {
    moveit::planning_interface::MoveGroupInterfacePtr lead_move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, lead_arm_group_name);
    auto cartesian_stage = std::make_unique<mtc::stages::CartesianWaypointStage>("Cartesian Waypoints", lead_cartesian_planner, lead_move_group_interface);

    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    geometry_msgs::msg::PoseStamped waypoint1;
    waypoint1.header.frame_id = "world";
    waypoint1.pose.position.x = 0.3855;
    waypoint1.pose.position.y = -0.0025;
    waypoint1.pose.position.z = 1.3840;
    waypoint1.pose.orientation = lead_target_pose.pose.orientation;
    waypoints.push_back(waypoint1);

    waypoints.push_back(lead_target_pose);

    std::vector<geometry_msgs::msg::Pose> waypoints_in_world;
    for (auto& waypoint : waypoints)
    {
      // transform to world frame
      geometry_msgs::msg::Pose waypoint_transformed = getPoseTransform(waypoint, "world").pose;
      waypoints_in_world.push_back(waypoint_transformed);
    }

    // IK frame at TCP
    Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
    grasp_frame_transform.translation().z() = 0.1034;

    cartesian_stage->setWaypoints(waypoints_in_world);
    cartesian_stage->setGroup(lead_arm_group_name);
    cartesian_stage->setIKFrame(grasp_frame_transform, lead_hand_frame);
    // cartesian_stage->setPathConstraints(path_constraints);

    task.stages()->add(std::move(cartesian_stage));
  }

  return task;
}

mtc::Task MTCTaskNode::createPostTask(std::string& start_frame_name, std::string& goal_frame_name, bool if_use_dual, bool if_split_plan, bool if_cartesian_connect, bool if_approach)
{
  mtc::Task task;
  task.stages()->setName("post routing task");
  task.loadRobotModel(node_);

  // Initialize robot groups
  initializeGroups();

  // delete markers
  visual_tools_.deleteAllMarkers();
  visual_tools_.trigger();

  // set target pose
  geometry_msgs::msg::PoseStamped lead_target_pose = createClipGoal(goal_frame_name, leader_pre_insert_offset);
  geometry_msgs::msg::PoseStamped follow_target_pose = createClipGoal(goal_frame_name, follower_pre_insert_offset);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  /****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
  {
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    // pre_move_stage_ptr = stage_state_current.get();
  }

  // Set up planners
  initializePlanners();

       /****************************************************
  ---- *               Open Hand                      *
	***************************************************/
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", follow_interpolation_planner);
    stage->setGroup(follow_hand_group_name);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

   /****************************************************
  ---- *               Retrieve in EE-z                *
    ***************************************************/
  {
    auto stage =
        std::make_unique<mtc::stages::MoveRelative>("lift", follow_cartesian_planner);
    stage->properties().set("marker_ns", "lift");
    stage->properties().set("link", follow_hand_frame);
    // stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGroup(follow_arm_group_name);
    // stage->setMinMaxDistance(0.1, 0.15); //this will override the moving distance
    // Set hand forward direction
    geometry_msgs::msg::Vector3Stamped vec;
    // vec.header.frame_id = follow_hand_frame;
    vec.header.frame_id = goal_frame_name;
    vec.vector.z = 0.05;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  return task;
}

mtc::Task MTCTaskNode::createHomingTask(std::string& start_frame_name, std::string& goal_frame_name, bool if_use_dual, bool if_split_plan, bool if_cartesian_connect, bool if_approach)
{
  mtc::Task task;
  task.stages()->setName("homing task");
  task.loadRobotModel(node_);

  // Initialize robot groups
  initializeGroups();

  // delete markers
  visual_tools_.deleteAllMarkers();
  visual_tools_.trigger();

  // Current state stage
  {
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    task.add(std::move(stage_state_current));

    // pre_move_stage_ptr = stage_state_current.get();
  }

  // Set up planners
  initializePlanners();

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move follower back home", follow_sampling_planner);
    stage->setGroup(follow_arm_group_name);
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move leader back home", lead_sampling_planner);
    stage->setGroup(lead_arm_group_name);
    stage->setGoal("ready");
    task.add(std::move(stage));
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

  std::vector<std::string> clip_names;

  // Service client to get clip names for update DLO model
  auto clip_names_client = mtc_task_node->getNode()->create_client<moveit_task_constructor_msgs::srv::GetClipNames>("get_clip_names");
  auto request = std::make_shared<moveit_task_constructor_msgs::srv::GetClipNames::Request>();

  // Wait for the service to become available
  while (!clip_names_client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(LOGGER, "Waiting for the service...");
  }

  // moveit::planning_interface::MoveGroupInterface move_group(mtc_task_node->getNode(), "right_panda_arm");
  // moveit_visual_tools::MoveItVisualTools visual_tools(mtc_task_node->getNode(), "right_panda_link0", "dual_mtc_routing",
  //                                                     move_group.getRobotModel());
  // visual_tools.loadRemoteControl();

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  // Synchronize with RealWolrd
  mtc_task_node->syncwithRealWorld();
  mtc_task_node->getVisualTools().prompt("Synchronization with RealWorld is done. Press 'next' in the RvizVisualToolsGui window to continue the next task");

  // Variables for synchronization
  std::mutex mutex;
  std::condition_variable cv;
  bool response_received = false;

  // List of clip IDs to process
  std::vector<std::string> clip_ids = {"clip5", "clip6", "clip7"}; //"clip5", "clip6", "clip7 or 8"

  // initial clip
  for (auto i = 0; i < clip_ids.size(); i++)
  {
    auto clip_id = clip_ids[i];
    auto prev_clip_id = (i > 0) ? clip_ids[i - 1] : "";
    // Use visul tools to control the movement from one clip to another
    mtc_task_node->getVisualTools().prompt("[Planning] Press 'next' in the RvizVisualToolsGui window to continue the next task");

    // Update planning scene after execution
    RCLCPP_INFO(LOGGER, "Updating planning scene after MTC execution.");
    mtc_task_node->updatePlanningScene();

    if (i>0){
      // from the second clip on, set the orientation and add approach
      mtc_task_node->setSelectOrientation(true);

      mtc_task_node->doTask(prev_clip_id, clip_id, true, true, false, true, true,
                      [mtc_task_node](std::string& start, std::string& goal, bool dual, bool split, bool cartesian, bool approach) {
                      return mtc_task_node->createTask(start, goal, dual, split, cartesian, approach);
                      });
      
    }else{
      mtc_task_node->doTask(prev_clip_id, clip_id, false, true, false, false, true,
                    [mtc_task_node](std::string& start, std::string& goal, bool dual, bool split, bool cartesian, bool approach) {
                    return mtc_task_node->createTask(start, goal, dual, split, cartesian, approach);
                    });
    }

    // mtc_task_node->doTask(clip_id, false, true, false, false,
    //                     [mtc_task_node](std::string& goal, bool dual, bool split, bool cartesian, bool approach) {
    //                     return mtc_task_node->createTestWaypointTask(goal, dual, split, cartesian, approach);
    //                     });

    // mtc_task_node->doTask(clip_id, false, true, false,
    //                     [mtc_task_node](std::string& goal, bool dual, bool split, bool cartesian) {
    //                     return mtc_task_node->createTask(goal, dual, split, cartesian);
    //                     });
                        
    mtc_task_node->getVisualTools().prompt("[Planning] Press 'next' in the RvizVisualToolsGui window to continue the next task");

    mtc_task_node->doTask(prev_clip_id, clip_id, false, true, false, false, false,
                        [mtc_task_node](std::string& start, std::string& goal, bool dual, bool split, bool cartesian, bool approach) {
                        return mtc_task_node->createPostTask(start, goal, dual, split, cartesian, approach);
                        });
    
    clip_names.push_back(clip_id);

    // Send the request
    request->clip_names = clip_names;
    // Send the request asynchronously
    RCLCPP_INFO(LOGGER, "Sending service request...");
    for (const auto& name : clip_names)
    {
      RCLCPP_INFO(LOGGER, "Clip Name: %s", name.c_str());
    }
    clip_names_client->async_send_request(request, 
                            [&](rclcpp::Client<moveit_task_constructor_msgs::srv::GetClipNames>::SharedFuture future) 
    {
      auto response = future.get();
      if (response)
      {
        RCLCPP_INFO(LOGGER, "Service response: %s", response->success ? "true" : "false");
      }
      else
      {
        RCLCPP_ERROR(LOGGER, "Service call failed!");
      }

      // Signal that the response has been received
      {
        std::lock_guard<std::mutex> lock(mutex);
        response_received = true;
      }
      cv.notify_one();
    });

    // Wait for the service response before continuing
    {
      std::unique_lock<std::mutex> lock(mutex);
      cv.wait(lock, [&] { return response_received; });
    }

    // Reset the response flag for the next request
    response_received = false;

    if (clip_id == "clip5")
    {
      // // Use visul tools to control the movement from one clip to another
      mtc_task_node->getVisualTools().prompt("After moving to initial positions, press 'next' in the RvizVisualToolsGui window to start servo");
      // start follower tracking
      std_msgs::msg::Bool start_tracking_msg;
      start_tracking_msg.data = true;
      tracking_start_pub->publish(start_tracking_msg);
    }

  }

  // move back home
  std::string clip_id_place_holder = "clip0";
  mtc_task_node->doTask(clip_id_place_holder, clip_id_place_holder, false, true, false, false, false,
  [mtc_task_node](std::string& start, std::string& goal, bool dual, bool split, bool cartesian, bool approach) {
  return mtc_task_node->createHomingTask(start, goal, dual, split, cartesian, approach);
  });


  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}