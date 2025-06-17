#ifndef EMBEDDED_PROJECT__SENSOR_UTILS_HPP_
#define EMBEDDED_PROJECT__SENSOR_UTILS_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace sensor_utils
{

sensor_msgs::msg::PointCloud2 filter_pointcloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const geometry_msgs::msg::Pose & camera_pose);

sensor_msgs::msg::Image convert_compressed_to_image(
  const sensor_msgs::msg::CompressedImage & compressed_msg);

}  

#endif  
