#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "aruco/aruco.h"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv4/opencv2/calib3d.hpp"
#include <aruco/cvdrawingutils.h>
#include "tf2/LinearMath/Transform.h"

cv::Mat inImage;
aruco::CameraParameters camParam;
aruco::MarkerDetector mDetector;
std::vector<aruco::Marker> markers;

rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub;
rclcpp::Node::SharedPtr node;
image_transport::Publisher image_pub;

std::string parent_name;
std::string child_name;
std::string dictionary_type;
bool useRectifiedImages;
bool cam_info_received = false;

std::unique_ptr<tf2_ros::TransformBroadcaster> br;
std::map<int, double> marker_sizes_by_id;
double default_marker_size = 0.1;

void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (!cam_info_received) return;

  rclcpp::Time curr_stamp = msg->header.stamp;

  try {
    auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    inImage = cv_ptr->image;

    markers.clear();
    mDetector.detect(inImage, markers, camParam, default_marker_size, false);

    for (auto& marker : markers) {  // <- Cambiado a mutable (no const)
      double size = default_marker_size;
      if (!marker_sizes_by_id.empty()) {
        auto it = marker_sizes_by_id.find(marker.id);
        if (it == marker_sizes_by_id.end()) continue;  // Ignorar marcadores no deseados
        size = it->second;
      }

      marker.calculateExtrinsics(size, camParam, false);

      cv::Mat rot(3, 3, CV_64FC1);
      cv::Mat Rvec64;
      marker.Rvec.convertTo(Rvec64, CV_64FC1);
      cv::Rodrigues(Rvec64, rot);
      cv::Mat tran64;
      marker.Tvec.convertTo(tran64, CV_64FC1);

      tf2::Matrix3x3 tf_rot(
        rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
        rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
        rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2)
      );

      tf2::Vector3 tf_orig(
        tran64.at<double>(0, 0),
        tran64.at<double>(1, 0),
        tran64.at<double>(2, 0)
      );

      tf2::Transform transform(tf_rot, tf_orig);

      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = curr_stamp;
      t.header.frame_id = parent_name;
      t.child_frame_id = child_name + std::to_string(marker.id);
      t.transform.translation.x = transform.getOrigin().x();
      t.transform.translation.y = transform.getOrigin().y();
      t.transform.translation.z = transform.getOrigin().z();
      t.transform.rotation.x = transform.getRotation().x();
      t.transform.rotation.y = transform.getRotation().y();
      t.transform.rotation.z = transform.getRotation().z();
      t.transform.rotation.w = transform.getRotation().w();

      br->sendTransform(t);
      aruco::CvDrawingUtils::draw3dAxis(inImage, marker, camParam);
    }

    if (image_pub.getNumSubscribers() > 0) {
      cv_bridge::CvImage out_msg;
      out_msg.header.stamp = curr_stamp;
      out_msg.encoding = sensor_msgs::image_encodings::RGB8;
      out_msg.image = inImage;
      image_pub.publish(out_msg.toImageMsg());
    }
  }
  catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void cam_info_callback(const sensor_msgs::msg::CameraInfo &cam_info)
{
  if (cam_info_received) return;

  cv::Mat cameraMatrix(3, 4, CV_64FC1, 0.0);
  cv::Mat distorsionCoeff(4, 1, CV_64FC1);
  cv::Size size(cam_info.width, cam_info.height);

  if (useRectifiedImages) {
    for (int i = 0; i < 12; ++i)
      cameraMatrix.at<double>(i / 4, i % 4) = cam_info.p[i];
    distorsionCoeff.setTo(0);
  } else {
    for (int i = 0; i < 9; ++i)
      cameraMatrix.at<double>(i / 3, i % 3) = cam_info.k[i];

    if (cam_info.d.size() == 4)
      for (int i = 0; i < 4; ++i)
        distorsionCoeff.at<double>(i, 0) = cam_info.d[i];
    else
      distorsionCoeff.setTo(0);
  }

  camParam = aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);
  cam_info_received = true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("image_publisher");
  image_transport::ImageTransport it(node);

  // Parámetros generales
  node->declare_parameter("parent_name", "camera");
  node->declare_parameter("child_name", "marker_");
  node->declare_parameter("dictionary_type", "DICT_4X4_100");
  node->declare_parameter("image_is_rectified", true);
  node->declare_parameter("marker_size", 0.1);
  node->declare_parameter("recognized_ids", std::vector<long>());  // Cambiado a long
  node->declare_parameter("recognized_sizes", std::vector<double>());

  node->get_parameter("parent_name", parent_name);
  node->get_parameter("child_name", child_name);
  node->get_parameter("dictionary_type", dictionary_type);
  node->get_parameter("image_is_rectified", useRectifiedImages);
  node->get_parameter("marker_size", default_marker_size);

  std::vector<long> recognized_ids_long;
  std::vector<double> recognized_sizes;
  node->get_parameter("recognized_ids", recognized_ids_long);
  node->get_parameter("recognized_sizes", recognized_sizes);

  std::vector<int> recognized_ids;
  recognized_ids.reserve(recognized_ids_long.size());
  for (auto id : recognized_ids_long) {
    recognized_ids.push_back(static_cast<int>(id));
  }

  if (!recognized_ids.empty()) {
    if (recognized_ids.size() != recognized_sizes.size()) {
      RCLCPP_ERROR(node->get_logger(), "recognized_ids y recognized_sizes deben tener el mismo tamaño");
      return -1;
    }
    for (size_t i = 0; i < recognized_ids.size(); ++i) {
      marker_sizes_by_id[recognized_ids[i]] = recognized_sizes[i];
    }
    RCLCPP_INFO(node->get_logger(), "Modo filtrado: detectando %lu IDs específicos.", marker_sizes_by_id.size());
  } else {
    RCLCPP_INFO(node->get_logger(), "Modo completo: detectando todos los IDs con tamaño %.2fm.", default_marker_size);
  }

  mDetector.setDictionary(dictionary_type);

  // Inicializa TransformBroadcaster solo una vez
  br = std::make_unique<tf2_ros::TransformBroadcaster>(node);

  image_transport::Subscriber image_sub = it.subscribe("/image_rect", 1, image_callback);
  cam_info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>("/camera_info", 1, cam_info_callback);
  image_pub = it.advertise("result", 10);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
