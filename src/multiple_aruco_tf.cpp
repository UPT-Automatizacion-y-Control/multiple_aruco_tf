#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h> 
#include <aruco_ros/ArucoThresholdConfig.h>

cv::Mat inImage;
aruco::CameraParameters camParam;
bool useRectifiedImages;
aruco::MarkerDetector mDetector; 
std::vector<aruco::Marker> markers;
ros::Subscriber cam_info_sub; 
bool cam_info_received;
image_transport::Publisher image_pub;
std::string parent_name;
std::string child_name; 
std::string dictionary_type;
double marker_size;

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  double ticksBefore = cv::getTickCount();
  static tf::TransformBroadcaster br;
  if (cam_info_received)
  {
    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      inImage = cv_ptr->image;

      markers.clear();
      mDetector.detect(inImage, markers, camParam, marker_size, false);
      for (unsigned int i = 0; i < markers.size(); ++i)
      {
        tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
        br.sendTransform(tf::StampedTransform(transform, curr_stamp, parent_name, child_name + std::to_string(markers[i].id)));   
        aruco::CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
      }

      if (image_pub.getNumSubscribers() > 0)
      {
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage;
        image_pub.publish(out_msg.toImageMsg());
      }

      ROS_DEBUG("runtime: %f ms", 1000 * (cv::getTickCount() - ticksBefore) / cv::getTickFrequency());
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
}

void cam_info_callback(const sensor_msgs::CameraInfo &msg)
{
  camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);
  cam_info_received = true;
  cam_info_sub.shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_simple_array");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  
  nh.param<double>("marker_size", marker_size, 0.05);
  nh.param<std::string>("parent_name", parent_name, "camera");
  nh.param<std::string>("child_name", child_name, "marker_");
  nh.param<std::string>("dictionary_type", dictionary_type, "ALL_DICTS");
  
  mDetector.setDictionary(dictionary_type);

  if (parent_name == "" || child_name == "")
  {
    ROS_ERROR("parent_name and/or child_name was not set!");
    return -1;
  }

  nh.param<bool>("image_is_rectified", useRectifiedImages, true);
  ROS_INFO_STREAM("Image is rectified: " << useRectifiedImages);

  image_transport::Subscriber image_sub = it.subscribe("/image", 1, &image_callback);
  cam_info_sub = nh.subscribe("/camera_info", 1, &cam_info_callback);

  cam_info_received = false;
  image_pub = it.advertise("result", 1);

  ROS_INFO("ArUco node started with marker size of %f meters", marker_size);
  ROS_INFO("ArUco node will publish pose array to TF with (%s, %s_i) as (parent,child).", parent_name.c_str(), child_name.c_str());

  ros::spin();
}
