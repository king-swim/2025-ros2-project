#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "embedded_project/srv/set_filter_config.hpp"

#include "embedded_project/sensor_utils.hpp"

class FilterServiceNode : public rclcpp::Node
{
public:
    FilterServiceNode() : Node("filter_service"), have_config_(false)
    {
        // Subscribe to incoming PointCloud2 data from LIDAR
        lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "LIDAR_TOP", 10,
            std::bind(&FilterServiceNode::lidarCallback, this, std::placeholders::_1));

        // Create a service that allows users to set camera configuration
        service_ = create_service<embedded_project::srv::SetFilterConfig>(
            "SetFilterConfig",
            std::bind(&FilterServiceNode::serviceCallback, this,
                std::placeholders::_1, std::placeholders::_2));

        // Publishers for filtered pointcloud and image result
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("selected_image", 10);
    }

private:
    void serviceCallback(
        const std::shared_ptr<embedded_project::srv::SetFilterConfig::Request> request,
        std::shared_ptr<embedded_project::srv::SetFilterConfig::Response> response)
    {
        /***************************************************************
        // TODO: Store camera name, intrinsic, and extrinsic information
        // - Set camera_name_ from request
        // - Save camera_info_ and camera_pose_ from request
        // - Mark configuration as available (have_config_ = true)
        ***************************************************************/







        // Build the topic name for the compressed image stream
        std::string image_topic = "/" + camera_name_ + "/image_rect_compressed";

        // If previously subscribed, unsubscribe before resubscribing
        if (image_sub_) {
            image_sub_.reset();
        }

        /***************************************************************
        // TODO: Subscribe to the compressed image topic
        // - Create a new subscription to image_topic (type: [sensor_msgs::msg::CompressedImage])
        // - Bind the callback to imageCallback()
        // - Set success message for the service response
        ***************************************************************/







    }

    void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        /***************************************************************
        // TODO: Convert CompressedImage to Image and publish
        // - Use sensor_utils::convert_compressed_to_image
        ***************************************************************/





    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Skip filtering if configuration has not yet been set
        /***************************************************************
        // TODO: Check if camera config is available
        // - If not, log a throttled warning and return
        ***************************************************************/






        /***************************************************************
        // TODO: Filter the incoming pointcloud using sensor_utils
        // - Call sensor_utils::filter_pointcloud with msg, camera_info_, camera_pose_
        // - Publish the result to "filtered_points" topic
        ***************************************************************/






    }

    // Configuration status and data
    bool have_config_;
    std::string camera_name_;
    sensor_msgs::msg::CameraInfo camera_info_;
    geometry_msgs::msg::Pose camera_pose_;

    // Subscriptions and publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Service<embedded_project::srv::SetFilterConfig>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
};

int main(int argc, char** argv)
{
    // Start ROS 2 and spin the node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FilterServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
