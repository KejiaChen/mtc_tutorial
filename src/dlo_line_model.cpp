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

#include <geometry_msgs/msg/pose.hpp>

#include <rclcpp/rclcpp.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dlo_tracker");
using namespace moveit::task_constructor;

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

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
	rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("dlo_tracker");

    moveit::planning_interface::PlanningSceneInterface psi;
    moveit::planning_interface::MoveGroupInterface move_group_interface(node, "dual_arm");

    // TF buffer and listener
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // initialize rviz tool
    // rviz_visual_tools::RvizVisualToolsPtr visual_tools(new rviz_visual_tools::RvizVisualTools("world", "/interactive_robot_marray", node));
    auto visual_tools = moveit_visual_tools::MoveItVisualTools{node, "world", "/rviz_visual_tools"};
    // visual_tools->loadMarkerPub();
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    // Create a publisher for the line marker
    // auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    // ros::Subscriber gripper_tip_sub = nh.subscribe("gripper_tip_position", 10, gripperTipPositionCallback);

    // Initialize marker properties
    std::string target_frame = "right_panda_hand";  // Replace with your TF frame name
    std::string reference_frame = "world";      // Replace with your reference frame name
    std::vector<Eigen::Vector3d> line_starts;
    std::vector<Eigen::Vector3d> line_ends; 
    Eigen::Vector3d gripper_tip_position;
    rviz_visual_tools::Colors line_marker_color = rviz_visual_tools::RED;
    rviz_visual_tools::Scales line_marker_scale = rviz_visual_tools::MEDIUM;

    // Line marker starts from a certain point
    std::vector<std::string> clip_names = {"clip5", "clip6"};

    std::string clip_name = clip_names[0];
    std::map<std::string, geometry_msgs::msg::Pose> clip_poses = psi.getObjectPoses(clip_names);
    RCLCPP_INFO(LOGGER, " Clip5 pose: %f, %f, %f", clip_poses[clip_name].position.x, clip_poses[clip_name].position.y, clip_poses[clip_name].position.z);
    Eigen::Isometry3d line_start_pose = poseToIsometry(clip_poses[clip_name]);
    line_starts.push_back(line_start_pose.translation());

    clip_name = clip_names[1];
    clip_poses = psi.getObjectPoses(clip_names);
    RCLCPP_INFO(LOGGER, " Clip6 pose: %f, %f, %f", clip_poses[clip_name].position.x, clip_poses[clip_name].position.y, clip_poses[clip_name].position.z);
    Eigen::Isometry3d line_ends_pose = poseToIsometry(clip_poses[clip_name]);
    line_ends.push_back(line_ends_pose.translation());

    clip_name = clip_names[1];
    clip_poses = psi.getObjectPoses(clip_names);
    line_start_pose = poseToIsometry(clip_poses[clip_name]);
    // line_marker_start = line_start_pose.translation();
    line_starts.push_back(line_start_pose.translation());

    // place holder
    line_ends.push_back(line_start_pose.translation());

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
        try{
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
        geometry_msgs::msg::TransformStamped transform_stamgped =
            tf_buffer->lookupTransform(reference_frame, target_frame, rclcpp::Time(0));

        // // Get the global transform of a specific link with respect to the world frame
        // Eigen::Isometry3d tip_pose_in_hand = Eigen::Isometry3d::Identity();
        // tip_pose_in_hand.translation().z() = 0.1034; // Franka TCP configuration
        // const Eigen::Isometry3d& gripper_tip_pose = current_state->getGlobalLinkTransform("right_panda_hand")*tip_pose_in_hand;
        // // line_marker_end = gripper_tip_pose.translation();
        // line_ends[1] = gripper_tip_pose.translation();

        line_ends[1] = Eigen::Vector3d(transform_stamgped.transform.translation.x,
                                       transform_stamgped.transform.translation.y,
                                       transform_stamgped.transform.translation.z);

        // Extract the position components
        // gripper_tip_position = gripper_tip_pose.translation();
        // ROS_INFO_STREAM("Gripper tip position: " << gripper_tip_position.x() << ", "
        //                                             << gripper_tip_position.y() << ", "
        //                                             << gripper_tip_position.z());

        // bool publishing = visual_tools->publishLine(line_marker_start, line_marker_end, line_marker_color, line_marker_scale, 1);
        for (size_t i = 0; i < line_starts.size(); ++i) {
            visual_tools.publishLine(line_starts[i], line_ends[i], line_marker_color, line_marker_scale, i); // Pass 'i' as the ID
        }
        visual_tools.trigger();
        }catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(node->get_logger(), "Could not transform '%s' to '%s': %s", 
                        target_frame.c_str(), reference_frame.c_str(), ex.what());
        }

        // Optionally, add a delay to control the update frequency
        loop_rate.sleep();
    }

    rclcpp::spin(node);
    return 0;
}

// }
// }
// }