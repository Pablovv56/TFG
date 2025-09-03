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

    // Subscription to /transformed_pcl
    pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/new_pcl", 10,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pclCallback(msg);
      });
  }

private:

  void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {

    // Check that the message is recieved correctly
    if (msg == NULL || msg->data.empty())
    {
      bonxai_server_->publishAll(rclcpp::Clock{}.now());
      RCLCPP_WARN(this->get_logger(), "Map not updated, received empty PointCloud2 message.");
    }else{
      RCLCPP_INFO(this->get_logger(), "Received PCL message");
      if (bonxai_server_) {
        // Insert the new cloud in the map
        bonxai_server_->insertCloudCallback(std::make_shared<sensor_msgs::msg::PointCloud2>(*msg));
        RCLCPP_INFO(this->get_logger(), "BonxaiServer updated with transformed PointCloud2 and TransformStamped.");
      }
    }
    
  }
  
  // Create PCL subscription pointer
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscription_;

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