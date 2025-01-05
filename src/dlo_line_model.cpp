#include <string>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/task_constructor/stage.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/task_constructor/marker_tools.h>
#include <moveit_task_constructor_msgs/srv/get_clip_names.hpp>
// #include <moveit_task_constructor_msgs/msg/clip_names.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <rclcpp/rclcpp.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dlo_tracker");
using namespace moveit::task_constructor;

std::mutex clip_names_mutex;
std::vector<std::string> clip_names = {};
bool clip_names_changed = false;  // Signal flag for clip names changes

Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose& pose_msg) {
    Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
    
    // Translation
    isometry.translation() << pose_msg.position.x, pose_msg.position.y, pose_msg.position.z;
    
    // Rotation
    Eigen::Quaterniond quat(pose_msg.orientation.w, pose_msg.orientation.x,
                            pose_msg.orientation.y, pose_msg.orientation.z);
    isometry.linear() = quat.toRotationMatrix();
    
    return isometry;
}

void handle_service(const std::shared_ptr<moveit_task_constructor_msgs::srv::GetClipNames::Request> request,
                        std::shared_ptr<moveit_task_constructor_msgs::srv::GetClipNames::Response> response) {
    // Lock the mutex to ensure thread safety
    std::lock_guard<std::mutex> lock(clip_names_mutex);

    RCLCPP_INFO(LOGGER, "Received request to update clip names.");

    // Clear and update clip_names
    clip_names = {"clip0"};
    clip_names.insert(clip_names.end(), request->clip_names.begin(), request->clip_names.end());
    // clip_names = request->clip_names;
    clip_names_changed = true;

    // Log the received clip names
    for (const auto& clip_name : clip_names) {
        RCLCPP_INFO(LOGGER, "Received clip name: %s", clip_name.c_str());
    }

    // Respond to the client
    response->success = !clip_names.empty();
    RCLCPP_INFO(LOGGER, "Service response: %s", response->success ? "true" : "false");
}

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
	rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("dlo_tracker");

    moveit::planning_interface::PlanningSceneInterface psi;
    moveit::planning_interface::MoveGroupInterface move_group_interface(node, "dual_arm");

    // TF buffer and listener
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(node);
    // std::thread executor_thread([&executor]() { executor.spin(); });

    // initialize rviz tool
    // rviz_visual_tools::RvizVisualToolsPtr visual_tools(new rviz_visual_tools::RvizVisualTools("world", "/interactive_robot_marray", node));
    auto visual_tools = moveit_visual_tools::MoveItVisualTools{node, "world", "/rviz_visual_tools"};
    // visual_tools->loadMarkerPub();
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    // Create a service server for clip_names
    auto clip_service = node->create_service<moveit_task_constructor_msgs::srv::GetClipNames>(
      "get_clip_names", handle_service);

    // Initialize marker properties
    std::string target_frame = "right_panda_hand";  // Replace with your TF frame name
    std::string reference_frame = "world";      // Replace with your reference frame name
    std::vector<Eigen::Vector3d> line_starts;
    std::vector<Eigen::Vector3d> line_ends; 
    Eigen::Vector3d gripper_tip_position;
    rviz_visual_tools::Colors line_marker_color = rviz_visual_tools::RED;
    rviz_visual_tools::Scales line_marker_scale = rviz_visual_tools::MEDIUM;
    Eigen::Isometry3d line_start_pose;
    Eigen::Isometry3d line_end_pose;

    // Line marker starts from a certain point
    // std::vector<std::string> clip_names = {"clip0"};
    
    // // TODO: why there has to be more than one segements to show the line?
    // std::string clip_name = clip_names[0];
    // std::map<std::string, geometry_msgs::msg::Pose> clip_poses = psi.getObjectPoses(clip_names);
    // RCLCPP_INFO(LOGGER, " Clip5 pose: %f, %f, %f", clip_poses[clip_name].position.x, clip_poses[clip_name].position.y, clip_poses[clip_name].position.z);
    // line_start_pose = poseToIsometry(clip_poses[clip_name]);
    // line_starts.push_back(line_start_pose.translation());

    // clip_name = clip_names[1];
    // clip_poses = psi.getObjectPoses(clip_names);
    // RCLCPP_INFO(LOGGER, " Clip6 pose: %f, %f, %f", clip_poses[clip_name].position.x, clip_poses[clip_name].position.y, clip_poses[clip_name].position.z);
    // line_end_pose = poseToIsometry(clip_poses[clip_name]);
    // line_ends.push_back(line_end_pose.translation());

    // clip_name = clip_names[1];
    // clip_poses = psi.getObjectPoses(clip_names);
    // line_start_pose = poseToIsometry(clip_poses[clip_name]);
    // // line_marker_start = line_start_pose.translation();
    // line_starts.push_back(line_start_pose.translation());

    // // place holder
    // line_ends.push_back(line_start_pose.translation());

    // // initialize
    // robot_state::RobotStatePtr current_state = move_group_interface.getCurrentState();

    // // Get the global transform of a specific link with respect to the world frame
    // Eigen::Isometry3d tip_pose_in_hand = Eigen::Isometry3d::Identity();
    // tip_pose_in_hand.translation().z() = 0.1034;
    //  const Eigen::Isometry3d& gripper_tip_pose = current_state->getGlobalLinkTransform("panda_1_hand")*tip_pose_in_hand;
    // line_marker_end = gripper_tip_pose.translation();

    // bool publishing = visual_tools->publishLine(line_marker_start,  line_marker_end, line_marker_color, line_marker_scale, 1);
    // visual_tools->trigger();
    // ros::Duration(0.1).sleep();
    
    rclcpp::WallRate loop_rate(10);
    // Main loop to continuously update the line marker
    while (rclcpp::ok()) {
        // Process incoming service requests
        rclcpp::spin_some(node);

        try{
            if (clip_names_changed){
                std::lock_guard<std::mutex> lock(clip_names_mutex);
                clip_names_changed = false;  // Reset the signal flag
                
                // Clear existing lines
                line_starts.clear();
                line_ends.clear();

                // Get clip poses
                std::map<std::string, geometry_msgs::msg::Pose> clip_poses = psi.getObjectPoses(clip_names);

                // Compute lines between neighboring clips
                for (size_t i = 0; i < clip_names.size(); ++i) {
                    const auto& start_clip = clip_names[i];
                    if (clip_poses.count(start_clip)){
                        line_start_pose = poseToIsometry(clip_poses[start_clip]);
                        line_starts.push_back(line_start_pose.translation());
                    }

                    // except the last clip
                    if (i < clip_names.size() - 1) {
                        const auto& end_clip = clip_names[i + 1];
                        if (clip_poses.count(end_clip)) {
                            line_end_pose = poseToIsometry(clip_poses[end_clip]);
                            line_ends.push_back(line_end_pose.translation());
                        }
                    }else{
                        // place holder, will be updated later with transform
                        line_ends.push_back(line_start_pose.translation());
                    }
                }

                clip_names_changed = false;

            }
        // Get the current state of the robot
        // moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
        // RCLCPP_INFO(LOGGER, "Waiting for robot states...");
        // while (!current_state)
        // {
        //     rclcpp::spin_some(node);
        //     rclcpp::sleep_for(std::chrono::milliseconds(100));
        // }
        // RCLCPP_INFO(LOGGER, "Robot states received.");

        // Lookup transform
        if (clip_names.size() > 0){
        geometry_msgs::msg::TransformStamped transform_stamped =
            tf_buffer->lookupTransform(reference_frame, target_frame, rclcpp::Time(0));

        // // Get the global transform of a specific link with respect to the world frame
        // Eigen::Isometry3d tip_pose_in_hand = Eigen::Isometry3d::Identity();
        // tip_pose_in_hand.translation().z() = 0.1034; // Franka TCP configuration
        // const Eigen::Isometry3d& gripper_tip_pose = current_state->getGlobalLinkTransform("right_panda_hand")*tip_pose_in_hand;
        // // line_marker_end = gripper_tip_pose.translation();
        // line_ends[1] = gripper_tip_pose.translation();

        line_ends[clip_names.size()-1] = Eigen::Vector3d(transform_stamped.transform.translation.x,
                                        transform_stamped.transform.translation.y,
                                        transform_stamped.transform.translation.z);

        // Extract the position components
        // gripper_tip_position = gripper_tip_pose.translation();
        // ROS_INFO_STREAM("Gripper tip position: " << gripper_tip_position.x() << ", "
        //                                             << gripper_tip_position.y() << ", "
        //                                             << gripper_tip_position.z());

        // bool publishing = visual_tools->publishLine(line_marker_start, line_marker_end, line_marker_color, line_marker_scale, 1);
        for (size_t i = 0; i < line_starts.size(); ++i) {
            visual_tools.publishLine(line_starts[i], line_ends[i], line_marker_color, line_marker_scale, i); // Pass 'i' as the ID
            // RCLCPP_INFO(LOGGER, "Published line marker %zu", i);
        }

        visual_tools.trigger();
        
        }   
        }catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(LOGGER, "Could not transform '%s' to '%s': %s", 
                        target_frame.c_str(), reference_frame.c_str(), ex.what());
        }

        // Optionally, add a delay to control the update frequency
        loop_rate.sleep();
    }

    // rclcpp::spin(node);
    return 0;
}

// }
// }
// }