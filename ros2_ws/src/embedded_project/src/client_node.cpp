#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "embedded_project/srv/set_filter_config.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    std::string camera_name;
    // Prompt user to enter the camera name (used in topic and parameter naming)
    std::cout << "Enter camera name (e.g. CAM_FRONT): ";
    std::getline(std::cin, camera_name);

    // Create a ROS 2 node with name "filter_config_client"
    auto node = rclcpp::Node::make_shared("filter_config_client");

    // Define containers for translation and rotation values
    std::vector<double> translation, rotation;
    std::vector<double> default_trans{0.0, 0.0, 0.0};
    std::vector<double> default_rot{0.0, 0.0, 0.0, 1.0};

    /***************************************************************
    // TODO: Declare and retrieve parameters from the parameter server
    // - Declare parameters "<camera_name>.translation" and "<camera_name>.rotation"
    // - Get their values into the `translation` and `rotation` vectors
    ***************************************************************/
    // 내가 적은: eclare parameters
    node->declare_parameter<std::vector<double>>(camera_name + ".translation", default_trans);
    node->declare_parameter<std::vector<double>>(camera_name + ".rotation", default_rot);

    // 내가 적은: Retrieve parameters
    translation = node->get_parameter(camera_name + ".translation").as_double_array();
    rotation = node->get_parameter(camera_name + ".rotation").as_double_array();

    // Display the extrinsic translation and rotation values in the log
    RCLCPP_INFO(node->get_logger(), "extrinsic = [%.2f %.2f %.2f] / rot = [%.2f %.2f %.2f %.2f]",
                translation[0], translation[1], translation[2],
                rotation[0], rotation[1], rotation[2], rotation[3]);

    // CameraInfo message pointer to hold the received message
    sensor_msgs::msg::CameraInfo::SharedPtr cam_info_msg = nullptr;

    // Subscribe to the camera_info topic to receive intrinsic camera parameters
    auto sub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_name + "/camera_info", 1,
        [&cam_info_msg](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
        {
            cam_info_msg = msg;
        });

    // Wait for the camera_info message to be received
    RCLCPP_INFO(node->get_logger(), "Waiting for camera_info...");
    rclcpp::Rate rate(10);
    while (rclcpp::ok() && cam_info_msg == nullptr)
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }
    RCLCPP_INFO(node->get_logger(), "CameraInfo received.");

    // Create a client for the SetFilterConfig service
    auto client = node->create_client<embedded_project::srv::SetFilterConfig>("SetFilterConfig");

    // Wait until the service becomes available
    RCLCPP_INFO(node->get_logger(), "Waiting for service...");
    if (!client->wait_for_service(5s))
    {
        RCLCPP_ERROR(node->get_logger(), "Service not available.");
        rclcpp::shutdown();
        return 1;
    }

    // Create a request and populate its fields
    auto req = std::make_shared<embedded_project::srv::SetFilterConfig::Request>();
    req->camera_name = camera_name;
    req->camera_intrinsic = *cam_info_msg;

    /***************************************************************
    // TODO: Construct geometry_msgs::msg::Pose from translation and rotation vectors
    // - Assign the translation values to pose.position.{x, y, z}
    // - Assign the rotation values to pose.orientation.{x, y, z, w}
    // - Assign the constructed pose to req->camera_extrinsic
    ***************************************************************/
    // 내가 적은:
    geometry_msgs::msg::Pose pose;
    pose.position.x = translation[0];
    pose.position.y = translation[1];
    pose.position.z = translation[2];

    pose.orientation.x = rotation[0];
    pose.orientation.y = rotation[1];
    pose.orientation.z = rotation[2];
    pose.orientation.w = rotation[3];

    req->camera_extrinsic = pose;

    // Send the service request asynchronously
    auto future = client->async_send_request(req);

    // Wait for the result and log the response
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto res = future.get();
        RCLCPP_INFO(node->get_logger(), "Service response: %s", res->message.c_str());
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Service call failed.");
    }

    // Shutdown the ROS 2 node and clean up resources
    rclcpp::shutdown();
    return 0;
}
