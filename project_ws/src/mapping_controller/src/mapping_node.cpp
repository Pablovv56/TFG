#include <rclcpp/rclcpp.hpp>
#include <bonxai_server.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class MappingNode : public rclcpp::Node
{
public:
  MappingNode() : Node("mapping_node")
  {
    RCLCPP_INFO(this->get_logger(), "MappingNode has been created!");

    // Load BonxaiServer as a component
    bonxai_server_ = std::make_shared<bonxai_server::BonxaiServer>(rclcpp::NodeOptions());
    RCLCPP_INFO(this->get_logger(), "BonxaiServer has been initialized!");

    // Subscription to /pointcloud_transform
    pointcloud_transform_subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "/pointcloud_transform", 10,
      [this](const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
        pointcloudTransformCallback(msg);
      });

    // Subscription to /clean_pcl
    clean_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/clean_pcl", 10,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        cleanPclCallback(msg);
      });
  }

private:
  void pointcloudTransformCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Processing TransformStamped message");
    
    //Check if the latest_pcl recieved exists
    if (latest_pcl_ == nullptr)
    {
      RCLCPP_WARN(this->get_logger(), "No PointCloud2 data available yet.");
      return;
    }

    // Check if the timestamps match
    if (latest_pcl_->header.stamp.sec == msg->header.stamp.sec)
    {
        // Transform the latest_pcl_ using the received transform
      sensor_msgs::msg::PointCloud2 transformed_pcl;
      try {
        // Use tf2 to apply the transform to the point cloud
        tf2::doTransform(*latest_pcl_, transformed_pcl, *msg);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to transform PointCloud2: %s", e.what());
        return;
      }

      // Update BonxaiServer with the transformed PCL and the transform
      if (bonxai_server_) {
        // Insert the new cloud in the map
        bonxai_server_->insertCloudCallback(std::make_shared<sensor_msgs::msg::PointCloud2>(transformed_pcl));
        RCLCPP_INFO(this->get_logger(), "BonxaiServer updated with transformed PointCloud2 and TransformStamped.");
      } else {
        RCLCPP_WARN(this->get_logger(), "BonxaiServer instance is not available.");
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Timestamps do not match: PointCloud2 timestamp = %d, TransformStamped timestamp = %d", latest_pcl_->header.stamp.sec, msg->header.stamp.sec);
    }
  }

  void cleanPclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Storing PointCloud2 message");
    // Storing the newly recieved PCL
    latest_pcl_ = msg;
  }

  // Create PCL Transformation subscription pointer
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr pointcloud_transform_subscription_;
  
  // Create PCL subscription pointer
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr clean_pcl_subscription_;

  //Variable that stores a pointer to latest PCL
  sensor_msgs::msg::PointCloud2::SharedPtr latest_pcl_;

  // BonxaiServer instance
  std::shared_ptr<bonxai_server::BonxaiServer> bonxai_server_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create an instance of MappingNode
  auto node = std::make_shared<MappingNode>();

  RCLCPP_INFO(node->get_logger(), "MappingNode is running!");

  // Spin the node
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}