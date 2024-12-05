#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

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

    // Timer for periodic distance computation
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&EEDistanceMonitor::computeDistance, this));
  }

private:
  void computeDistance()
  {
    try
    {
      // Lookup transforms for master and follower end-effectors
      auto master_transform = tf_buffer_.lookupTransform("world", master_ee_frame_, tf2::TimePointZero);
      auto follower_transform = tf_buffer_.lookupTransform("world", follower_ee_frame_, tf2::TimePointZero);

      // Compute Euclidean distance
      double dx = master_transform.transform.translation.x - follower_transform.transform.translation.x;
      double dy = master_transform.transform.translation.y - follower_transform.transform.translation.y;
      double dz = master_transform.transform.translation.z - follower_transform.transform.translation.z;
      double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

      // Publish the distance
      std_msgs::msg::Float64 distance_msg;
      distance_msg.data = distance;
      distance_pub_->publish(distance_msg);

      // Log the distance
      // RCLCPP_INFO(this->get_logger(), "Distance between end-effectors: %.4f", distance);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
  }

  // Frame names for the master and follower end-effectors
  std::string master_ee_frame_;
  std::string follower_ee_frame_;

  // TF2 listener and buffer
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Publisher for the distance
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;

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
