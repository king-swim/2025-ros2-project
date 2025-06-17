#include "embedded_project/sensor_utils.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <vector>
#include <stdexcept>

namespace sensor_utils
{

sensor_msgs::msg::PointCloud2 filter_pointcloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const geometry_msgs::msg::Pose & camera_pose)
{
  int off_x = -1, off_y = -1, off_z = -1;
  for (const auto & f : cloud.fields) {
    if (f.name == "x") off_x = f.offset;
    if (f.name == "y") off_y = f.offset;
    if (f.name == "z") off_z = f.offset;
  }

  tf2::Quaternion q(
    camera_pose.orientation.x,
    camera_pose.orientation.y,
    camera_pose.orientation.z,
    camera_pose.orientation.w);
  tf2::Vector3 t(
    camera_pose.position.x,
    camera_pose.position.y,
    camera_pose.position.z);
  tf2::Quaternion q_inv = q.inverse();

  const auto & K = camera_info.k;
  float fx = static_cast<float>(K[0]);
  float fy = static_cast<float>(K[4]);
  float cx = static_cast<float>(K[2]);
  float cy = static_cast<float>(K[5]);
  uint32_t width = camera_info.width;
  uint32_t height = camera_info.height;

  const auto & in_data = cloud.data;
  size_t step = cloud.point_step;
  size_t count = cloud.width * cloud.height;

  std::vector<uint8_t> out_data;
  out_data.reserve(in_data.size());

  for (size_t i = 0; i < count; ++i) {
    const uint8_t * ptr = &in_data[i * step];

    float bx = *reinterpret_cast<const float*>(ptr + off_x);
    float by = *reinterpret_cast<const float*>(ptr + off_y);
    float bz = *reinterpret_cast<const float*>(ptr + off_z);

    tf2::Vector3 p_base(bx, by, bz);
    tf2::Vector3 p_rel = p_base - t;
    tf2::Vector3 p_cam = tf2::quatRotate(q_inv, p_rel);

    float cx_ = p_cam.x();
    float cy_ = p_cam.y();
    float cz_ = p_cam.z();

    if (cz_ <= 0.0f) continue;

    float u = fx * cx_ / cz_ + cx;
    float v = fy * cy_ / cz_ + cy;

    if (u >= 0 && u < width && v >= 0 && v < height) {
      out_data.insert(out_data.end(), ptr, ptr + step);
    }
  }

  sensor_msgs::msg::PointCloud2 output;
  output.header = cloud.header;
  output.height = 1;
  output.width = static_cast<uint32_t>(out_data.size() / step);
  output.fields = cloud.fields;
  output.is_bigendian = cloud.is_bigendian;
  output.point_step = step;
  output.row_step = output.width * step;
  output.is_dense = false;
  output.data = std::move(out_data);

  return output;
}

sensor_msgs::msg::Image convert_compressed_to_image(
  const sensor_msgs::msg::CompressedImage & compressed_msg)
{
  try {
    cv::Mat decoded = cv::imdecode(cv::Mat(compressed_msg.data), cv::IMREAD_COLOR);

    sensor_msgs::msg::Image out_msg;
    out_msg.header = compressed_msg.header;
    out_msg.height = decoded.rows;
    out_msg.width = decoded.cols;
    out_msg.encoding = "bgr8";
    out_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(decoded.cols * decoded.elemSize());
    out_msg.data.assign(decoded.data, decoded.data + decoded.total() * decoded.elemSize());
    return out_msg;
  } catch (const std::exception & e) {
    return sensor_msgs::msg::Image();
  }
}

}  
