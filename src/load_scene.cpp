#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <moveit_visual_tools/moveit_visual_tools.h>

void loadCustomScene(const std::string &path, rclcpp::Node::SharedPtr move_group_node)
{
    // planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor);
    // if (!ps)
    // {
    //     throw std::runtime_error("Failed to lock planning scene for writing");
    // }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(move_group_node, "right_panda_arm");

    moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "right_panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                                      move_group_interface.getRobotModel());

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    std::ifstream fin(path);
    if (!fin.is_open())
    {
        throw std::runtime_error("Failed to open scene file: " + path);
    }

    std::string line;
    while (std::getline(fin, line))
    {
        // Skip invalid lines or empty lines
        if (line.empty() || line[0] == '.' || line.find("(noname)+") != std::string::npos)
            continue;

        if (line[0] == '*') // Object definition starts
        {
            std::string object_name = line.substr(2); // Extract object name after "* "
            RCLCPP_INFO(rclcpp::get_logger("load_scene"), "Loading object: %s", object_name.c_str());

            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.id = object_name;
            collision_object.header.frame_id = "world"; // Default frame

            // Read the next lines for the object block
            std::string pos_line, ori_line, shape_line, dim_line, unused_line;
            double x, y, z, qx, qy, qz, qw, dx, dy, dz;

            // Read position
            if (!std::getline(fin, pos_line) || !(std::istringstream(pos_line) >> x >> y >> z))
                throw std::runtime_error("Invalid position line for object: " + object_name);

            // Read orientation
            if (!std::getline(fin, ori_line) || !(std::istringstream(ori_line) >> qx >> qy >> qz >> qw))
                throw std::runtime_error("Invalid orientation line for object: " + object_name);

            // Skip one line (the "1" marker)
            if (!std::getline(fin, unused_line))
                throw std::runtime_error("Unexpected end of file after orientation for object: " + object_name);

            // Read shape type (assuming it's always "box")
            if (!std::getline(fin, shape_line) || shape_line != "box")
                throw std::runtime_error("Invalid or unsupported shape type for object: " + object_name);

            // Read dimensions
            if (!std::getline(fin, dim_line) || !(std::istringstream(dim_line) >> dx >> dy >> dz))
                throw std::runtime_error("Invalid dimensions line for object: " + object_name);

            // Skip the remaining unused lines in the block
            for (int i = 0; i < 4; ++i)
            {
                if (!std::getline(fin, unused_line))
                    throw std::runtime_error("Unexpected end of file in unused block for object: " + object_name);
            }
            
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] = dx;
            primitive.dimensions[primitive.BOX_Y] = dy;
            primitive.dimensions[primitive.BOX_Z] = dz;

            // Create pose
            geometry_msgs::msg::Pose pose;
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;
            pose.orientation.x = qx;
            pose.orientation.y = qy;
            pose.orientation.z = qz;
            pose.orientation.w = qw;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(pose);
            collision_object.operation = collision_object.ADD;

            collision_objects.push_back(collision_object);
        }
        
    }
    // Add object to planning scene
    
    planning_scene_interface.addCollisionObjects(collision_objects);

    // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
    visual_tools.trigger();

    RCLCPP_INFO(rclcpp::get_logger("load_scene"), "Scene loaded successfully from %s", path.c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("scene_loader");

    if (argc < 2)
    {
        RCLCPP_ERROR(node->get_logger(), "Usage: ros2 run <package_name> <executable_name> <path_to_scene_file>");
        return 1;
    }

    std::string scene_file = argv[1];
    try
    {
        loadCustomScene(scene_file, node);
        RCLCPP_INFO(node->get_logger(), "Scene loaded successfully!");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error loading scene: %s", e.what());
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
