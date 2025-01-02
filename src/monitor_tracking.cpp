#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class EEDistanceMonitor : public rclcpp::Node
{
public:
  EEDistanceMonitor()
      : Node("ee_distance_monitor"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    // Declare parameters for the frame names
    this->declare_parameter<std::string>("master_ee_frame", "right_panda_hand");
    this->declare_parameter<std::string>("follower_ee_frame", "left_panda_hand");

    // Get the frame names
    this->get_parameter("master_ee_frame", master_ee_frame_);
    this->get_parameter("follower_ee_frame", follower_ee_frame_);

    // Publisher to publish the distance
    distance_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ee_distance", 10);

    // Publisher to publish the velocity scaling factor
    leader_scaling_pub = this->create_publisher<std_msgs::msg::Float64>("/right_arm_controller/scaling_factor", 10);
    follower_scaling_pub = this->create_publisher<std_msgs::msg::Float64>("/left_arm_controller/scaling_factor", 10);

    // Timer for periodic distance computation
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                     std::bind(&EEDistanceMonitor::computeDistanceScaling, this));
  }

private:
  void computeDistanceScaling()
  {
    try
    {
      // Lookup transforms for master and follower end-effectors
      auto master_transform_msg = tf_buffer_.lookupTransform("world", master_ee_frame_, tf2::TimePointZero);
      tf2::Transform master_transform;
      tf2::fromMsg(master_transform_msg.transform, master_transform);
      auto follower_transform_msg = tf_buffer_.lookupTransform("world", follower_ee_frame_, tf2::TimePointZero);
      tf2::Transform follower_transform;
      tf2::fromMsg(follower_transform_msg.transform, follower_transform);

      // Master and follower gripper tip positions
      // Define the transform of the gripper tip in the hand frame
      tf2::Transform grasp_frame_transform;
      grasp_frame_transform.setIdentity(); // Initialize to identity
      grasp_frame_transform.setOrigin(tf2::Vector3(0.0, 0.0, 0.1034)); // Translation along Z

      tf2::Transform master_tip_transform = master_transform * grasp_frame_transform;
      tf2::Transform follower_tip_transform = follower_transform * grasp_frame_transform;

      // Compute Euclidean distance
      // distance between hand frames
      // double dx = master_transform_msg.transform.translation.x - follower_transform_msg.transform.translation.x;
      // double dy = master_transform_msg.transform.translation.y - follower_transform_msg.transform.translation.y;
      // double dz = master_transform_msg.transform.translation.z - follower_transform_msg.transform.translation.z;

      // distance between gripper tips
      double dx = master_tip_transform.getOrigin().x() - follower_tip_transform.getOrigin().x();
      double dy = master_tip_transform.getOrigin().y() - follower_tip_transform.getOrigin().y();
      double dz = master_tip_transform.getOrigin().z() - follower_tip_transform.getOrigin().z();

      // double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      double distance = std::sqrt(dx * dx + dy * dy); // Ignore z-axis

      // RCLCPP_INFO(this->get_logger(), "Master Transform: x=%.9f, y=%.9f, z=%.9f",
      //       master_transform_msg.transform.translation.x,
      //       master_transform_msg.transform.translation.y,
      //       master_transform_msg.transform.translation.z);
      // RCLCPP_INFO(this->get_logger(), "Follower Transform: x=%.9f, y=%.9f, z=%.9f",
      //             follower_transform_msg.transform.translation.x,
      //             follower_transform_msg.transform.translation.y,
      //             follower_transform_msg.transform.translation.z);
      // RCLCPP_INFO(this->get_logger(), "Distance between end-effectors: %.9f", distance);

      // Publish the distance
      std_msgs::msg::Float64 distance_msg;
      distance_msg.data = distance;
      distance_pub_->publish(distance_msg);

      // Publish the scaling factor
      std_msgs::msg::Float64 follow_scaling_msg;
      double follow_scaling_factor = EEDistanceMonitor::followerScaling(distance, 0.1);
      follow_scaling_msg.data = follow_scaling_factor;
      follower_scaling_pub->publish(follow_scaling_msg);
      RCLCPP_INFO(this->get_logger(), "Published scaling factor for follower: %.4f", follow_scaling_factor);

      std_msgs::msg::Float64 lead_scaling_msg;
      lead_scaling_msg.data = 1.0;
      leader_scaling_pub->publish(lead_scaling_msg);

      // Log the distance
      // RCLCPP_INFO(this->get_logger(), "Distance between end-effectors: %.4f", distance);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
  }

  double followerScaling(double current_distance, double desired_distance){
    // Calculate velocity scaling factor for the follower
    double scaling_factor;

    if (current_distance < desired_distance)
    {
      // If the follower is too close, slow down or stop
      scaling_factor = std::max(0.001, 1.0 - (desired_distance - current_distance) * 10.0);
    }
    else
    {
      // If the follower is too far, speed up
      scaling_factor = 1.0 + (current_distance - desired_distance) * 5.0;
    }

    // Clamp to ensure scaling factor remains within a reasonable range
    scaling_factor = std::max(0.001, std::min(scaling_factor, 2.0));
    
    return scaling_factor;
  }

  // Frame names for the master and follower end-effectors
  std::string master_ee_frame_;
  std::string follower_ee_frame_;

  // TF2 listener and buffer
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr leader_scaling_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr follower_scaling_pub;

  // Timer for periodic computation
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EEDistanceMonitor>());
  rclcpp::shutdown();
  return 0;
}
