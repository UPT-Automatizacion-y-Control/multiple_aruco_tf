#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

std::string tf_begin, tf_end;
tf::StampedTransform transform;
geometry_msgs::Twist relative_pose;
ros::Publisher pub;

void timerCallback(const ros::TimerEvent&)
{  
	tf::TransformListener listener;
	try{
		listener.waitForTransform(tf_begin, tf_end, ros::Time(), ros::Duration(5.0));
		listener.lookupTransform(tf_begin,  tf_end, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("The error is %s",ex.what());
      return;
    } 
      
	relative_pose.linear.x = transform.getOrigin().x();
    relative_pose.linear.y = transform.getOrigin().y();
    relative_pose.linear.z = transform.getOrigin().z();
    
    tf::Quaternion q = transform.getRotation(); 
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
		
	relative_pose.angular.x = roll;
	relative_pose.angular.y = pitch;
	relative_pose.angular.z = yaw;
		
	pub.publish(relative_pose);       
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "relative_pose_node");
  ros::NodeHandle nh("~"); 
  ROS_INFO("Start relative pose"); 

  nh.param<std::string>("tf_begin", tf_begin, "map");
  nh.param<std::string>("tf_end"  , tf_end,   "");
  
  if (tf_end == "")
  {
    ROS_ERROR("tf_end is not defined!!");
    return -1;
  }
  else
  	ROS_INFO("Will publish tf from %s to %s",tf_begin.c_str(),tf_end.c_str());

  ros::Timer timer = nh.createTimer(ros::Duration(1.0/24.0), timerCallback);
  pub = nh.advertise<geometry_msgs::Twist>("relative_pose", 10); 
  ros::spin();
}
