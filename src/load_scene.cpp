#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

void loadScene(const std::string &scene_file, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, const std::string &frame_id = "world")
{
    // Read the JSON file
    std::ifstream file(scene_file);
    if (!file.is_open())
    {
        throw std::runtime_error("Could not open scene file: " + scene_file);
    }

    json scene_data;
    file >> scene_data;

    // Iterate over collision objects
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    for (const auto &obj : scene_data["collision_objects"])
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = obj["id"];
        collision_object.header.frame_id = frame_id;

        // Set pose
        geometry_msgs::msg::Pose pose;
        pose.position.x = obj["pose"]["position"]["x"];
        pose.position.y = obj["pose"]["position"]["y"];
        pose.position.z = obj["pose"]["position"]["z"];
        pose.orientation.x = obj["pose"]["orientation"]["x"];
        pose.orientation.y = obj["pose"]["orientation"]["y"];
        pose.orientation.z = obj["pose"]["orientation"]["z"];
        pose.orientation.w = obj["pose"]["orientation"]["w"];

        // Set shape and dimensions (assuming a box for simplicity)
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions = {
            obj["dimensions"][0],  // x
            obj["dimensions"][1],  // y
            obj["dimensions"][2]   // z
        };

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);
    }

    // Apply the collision objects to the scene
    planning_scene_interface.applyCollisionObjects(collision_objects);
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

    // Get the .scene file path from the command line argument
    std::string scene_file = argv[1];

    // Initialize the PlanningSceneInterface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    try
    {
        loadScene(scene_file, planning_scene_interface);
        RCLCPP_INFO(node->get_logger(), "Scene loaded successfully from %s!", scene_file.c_str());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to load scene: %s", e.what());
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

