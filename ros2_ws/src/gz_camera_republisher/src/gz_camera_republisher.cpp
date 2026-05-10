#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace
{
struct ImageFormat
{
  std::string encoding;
  std::size_t bytes_per_pixel;
};

ImageFormat ConvertPixelFormat(const gz::msgs::PixelFormatType format)
{
  switch (format) {
    case gz::msgs::L_INT8:
      return {"mono8", 1};
    case gz::msgs::L_INT16:
      return {"mono16", 2};
    case gz::msgs::RGB_INT8:
      return {"rgb8", 3};
    case gz::msgs::RGBA_INT8:
      return {"rgba8", 4};
    case gz::msgs::BGRA_INT8:
      return {"bgra8", 4};
    case gz::msgs::BGR_INT8:
      return {"bgr8", 3};
    case gz::msgs::BAYER_RGGB8:
      return {"bayer_rggb8", 1};
    case gz::msgs::BAYER_BGGR8:
      return {"bayer_bggr8", 1};
    case gz::msgs::BAYER_GBRG8:
      return {"bayer_gbrg8", 1};
    case gz::msgs::BAYER_GRBG8:
      return {"bayer_grbg8", 1};
    default:
      return {"rgb8", 3};
  }
}

std::string ConvertDistortionModel(
  const gz::msgs::CameraInfo_Distortion_DistortionModelType model)
{
  switch (model) {
    case gz::msgs::CameraInfo_Distortion_DistortionModelType_EQUIDISTANT:
      return "equidistant";
    case gz::msgs::CameraInfo_Distortion_DistortionModelType_RATIONAL_POLYNOMIAL:
      return sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
    case gz::msgs::CameraInfo_Distortion_DistortionModelType_PLUMB_BOB:
    default:
      return sensor_msgs::distortion_models::PLUMB_BOB;
  }
}
}  // namespace

class GzCameraRepublisher : public rclcpp::Node
{
public:
  GzCameraRepublisher()
  : Node("gz_camera_republisher")
  {
    const auto default_image_topic =
      "/world/baylands/model/standard_vtol_mono_cam_0/link/camera_link/sensor/camera/image";
    const auto default_camera_info_topic =
      "/world/baylands/model/standard_vtol_mono_cam_0/link/camera_link/sensor/camera/camera_info";

    gz_image_topic_ = declare_parameter<std::string>("gz_image_topic", default_image_topic);
    gz_camera_info_topic_ =
      declare_parameter<std::string>("gz_camera_info_topic", default_camera_info_topic);
    ros_image_topic_ = declare_parameter<std::string>("ros_image_topic", "/camera/image_raw");
    ros_camera_info_topic_ =
      declare_parameter<std::string>("ros_camera_info_topic", "/camera/camera_info");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_link");

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      ros_image_topic_, rclcpp::SensorDataQoS());
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      ros_camera_info_topic_, rclcpp::QoS(10));

    const bool image_ok =
      gz_node_.Subscribe(gz_image_topic_, &GzCameraRepublisher::OnImage, this);
    const bool camera_info_ok =
      gz_node_.Subscribe(gz_camera_info_topic_, &GzCameraRepublisher::OnCameraInfo, this);

    if (!image_ok) {
      RCLCPP_ERROR(get_logger(), "Failed to subscribe Gazebo image topic: %s",
        gz_image_topic_.c_str());
    }
    if (!camera_info_ok) {
      RCLCPP_ERROR(get_logger(), "Failed to subscribe Gazebo camera_info topic: %s",
        gz_camera_info_topic_.c_str());
    }

    RCLCPP_INFO(get_logger(), "Gazebo image: %s -> ROS 2: %s",
      gz_image_topic_.c_str(), ros_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Gazebo camera_info: %s -> ROS 2: %s",
      gz_camera_info_topic_.c_str(), ros_camera_info_topic_.c_str());
  }

private:
  void OnImage(const gz::msgs::Image &_msg)
  {
    sensor_msgs::msg::Image image;
    image.header.stamp = now();
    image.header.frame_id = frame_id_;
    image.height = _msg.height();
    image.width = _msg.width();

    const auto format = ConvertPixelFormat(_msg.pixel_format_type());
    image.encoding = format.encoding;
    image.is_bigendian = false;
    image.step = _msg.step() > 0 ?
      _msg.step() : static_cast<uint32_t>(image.width * format.bytes_per_pixel);

    const auto & data = _msg.data();
    image.data.resize(data.size());
    std::copy(data.begin(), data.end(), image.data.begin());

    image_pub_->publish(image);
  }

  void OnCameraInfo(const gz::msgs::CameraInfo &_msg)
  {
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header.stamp = now();
    camera_info.header.frame_id = frame_id_;
    camera_info.width = _msg.width();
    camera_info.height = _msg.height();

    if (_msg.has_intrinsics()) {
      const auto & k = _msg.intrinsics().k();
      const auto n = std::min<int>(k.size(), camera_info.k.size());
      for (int i = 0; i < n; ++i) {
        camera_info.k[i] = k.Get(i);
      }
    }

    if (_msg.has_projection()) {
      const auto & p = _msg.projection().p();
      const auto n = std::min<int>(p.size(), camera_info.p.size());
      for (int i = 0; i < n; ++i) {
        camera_info.p[i] = p.Get(i);
      }
    }

    camera_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    if (_msg.has_distortion()) {
      const auto & distortion = _msg.distortion();
      camera_info.distortion_model = ConvertDistortionModel(distortion.model());
      camera_info.d.resize(distortion.k_size());
      for (int i = 0; i < distortion.k_size(); ++i) {
        camera_info.d[i] = distortion.k(i);
      }
    } else {
      camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      camera_info.d.assign(5, 0.0);
    }

    camera_info_pub_->publish(camera_info);
  }

  gz::transport::Node gz_node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  std::string gz_image_topic_;
  std::string gz_camera_info_topic_;
  std::string ros_image_topic_;
  std::string ros_camera_info_topic_;
  std::string frame_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzCameraRepublisher>());
  rclcpp::shutdown();
  return 0;
}
