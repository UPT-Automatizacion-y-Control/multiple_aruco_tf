#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>

double alpha_val = 1.0;
double beta_val = 0.0;

cv::Mat inImage;
image_transport::Publisher image_pub;
cv_bridge::CvImage out_msg;

void image_callback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        inImage = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    if (image_pub.getNumSubscribers() > 0){   
        inImage.convertTo(inImage,-1,alpha_val,beta_val);    
        out_msg.header.stamp = msg->header.stamp;
        out_msg.encoding = sensor_msgs::image_encodings::MONO8;
        out_msg.image = inImage;
        image_pub.publish(out_msg.toImageMsg());
    }  
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "imag_sat");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  
  nh.param<double>("alpha_val", alpha_val, 1.0);
  nh.param<double>("beta_val", beta_val, 0.0);
  
  image_transport::Subscriber image_sub = it.subscribe("/image_in", 1, &image_callback);
  image_pub = it.advertise("image_out", 1);

  ROS_INFO("Image saturation is running!");
  ROS_INFO("Alpha is %f",alpha_val);
  ROS_INFO("Beta is %f",beta_val);
  ros::spin();
}
