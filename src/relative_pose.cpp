/*
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>

//#include <multiple_aruco_tf/CurrentPose.h>

using namespace std::chrono_literals;

class TfListenerNode : public rclcpp::Node
{
public:
    TfListenerNode() : Node("relative_pose_node")
    {

        this->declare_parameter<std::string>("child_frame", "marker_7");
        this->declare_parameter<std::string>("parent_frame", "marker_8");

        this->get_parameter("child_frame", child_frame);
        this->get_parameter("parent_frame", parent_frame);
        if (parent_frame == "")
        {
          RCLCPP_ERROR(this->get_logger(), "parent_frame is not defined !!");
          //return -1;
        }
        else
          //RCLCPP_INFO(rclcpp::get_logger("relative_pose_node"), "Will publish tf from %s to %s", child_frame.c_str(), parent_frame.c_str());
        // Set up a timer to check for transforms

        // Create the TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        publisher_ =
          this->create_publisher<geometry_msgs::msg::Twist>("relative_pose/" + child_frame + "_to_" + parent_frame, 10);

        auto timer_callback=
        [this]() -> void{
            try
            {
                // Try to get the transform from 'frame_a' to 'frame_b'
                geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
                //RCLCPP_INFO(rclcpp::get_logger("relative_pose_node"), "Will publish tf from %s to %s", child_frame.c_str(), parent_frame.c_str());
    
               /* RCLCPP_INFO(this->get_logger(), "Transform: [%f, %f, %f]",
                            t.transform.translation.x,
                            t.transform.translation.y,
                            t.transform.translation.z);
    
                relative_pose.linear.x = t.transform.translation.x;
                relative_pose.linear.y = t.transform.translation.y;
                relative_pose.linear.z = t.transform.translation.z;

                tf2::Quaternion quat;
                tf2::fromMsg(t.transform.rotation, quat);

                double roll, pitch, yaw;
                tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
                
                relative_pose.angular.x = roll;
                relative_pose.angular.y = pitch;
                relative_pose.angular.z = yaw;
                
                publisher_->publish(relative_pose);  
            }
            catch (const tf2::TransformException &ex)
            {
                //RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
            }
        };
        
        timer_ = this->create_wall_timer(41.666666667ms, timer_callback);


    }
private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string child_frame;
    std::string parent_frame;
    geometry_msgs::msg::Twist relative_pose;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfListenerNode>());

  //ros::Timer timer = nh.createTimer(ros::Duration(1.0/24.0), timerCallback);
  
  //pub = nh.advertise<geometry_msgs::Twist>("relative_pose", 10); 
  //pub = this->create_publisher<geometry_msgs::msg::Twist>("relative_pose", 10);
  
  //ros::ServiceServer service = nh.advertiseService("current_pose", srvCallback);
  
  //ROS_INFO("Relative pose is running!"); 
  //ros::spin();
  
  rclcpp::shutdown();
  return 0;
}*/
/*#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class TfListenerNode : public rclcpp::Node
{
public:
    TfListenerNode() : Node("relative_pose_node")
    {
        this->declare_parameter<std::string>("child_frame", "marker_7");
        this->declare_parameter<std::string>("parent_frame", "marker_8");

        this->get_parameter("child_frame", child_frame);
        this->get_parameter("parent_frame", parent_frame);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "relative_pose/" + child_frame + "_to_" + parent_frame, 10);

        timer_ = this->create_wall_timer(50ms, std::bind(&TfListenerNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::Twist twist_msg;

        try
        {
            auto t = tf_buffer_->lookupTransform(parent_frame, child_frame, tf2::TimePointZero);

            twist_msg.linear.x = t.transform.translation.x;
            twist_msg.linear.y = t.transform.translation.y;
            twist_msg.linear.z = t.transform.translation.z;

            tf2::Quaternion quat;
            tf2::fromMsg(t.transform.rotation, quat);

            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            twist_msg.angular.x = roll;
            twist_msg.angular.y = pitch;
            twist_msg.angular.z = yaw;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Transformación no disponible entre %s y %s: %s",
                parent_frame.c_str(), child_frame.c_str(), ex.what());

            // twist_msg se mantiene en ceros
        }

        publisher_->publish(twist_msg);
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string child_frame;
    std::string parent_frame;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfListenerNode>());
    rclcpp::shutdown();
    return 0;
}*/
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class TfListenerNode : public rclcpp::Node
{
public:
    TfListenerNode() : Node("relative_pose_node")
    {
        // 🚗 Marcos configurables
        this->declare_parameter<std::string>("child_frame", "marker_7");   // vehículo
        this->declare_parameter<std::string>("parent_frame", "marker_8");  // marco inercial

        this->get_parameter("child_frame", child_frame_);
        this->get_parameter("parent_frame", parent_frame_);

        // 📡 TF Buffer, Listener y Broadcaster
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 🌀 Publicador de Twist con nombre consistente con el TF
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "relative_pose/" + child_frame_ + "_rel_to_" + parent_frame_, 10);

        // ⏱️ Temporizador
        timer_ = this->create_wall_timer(50ms, std::bind(&TfListenerNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(),
            "📡 Nodo iniciado: publicando pose de [%s] respecto a [%s]",
            child_frame_.c_str(), parent_frame_.c_str());
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::Twist twist_msg;

        try
        {
            // 🔍 Transformación del child respecto al parent
            auto t = tf_buffer_->lookupTransform(parent_frame_, child_frame_, tf2::TimePointZero);

            // --- Publicar Twist ---
            twist_msg.linear.x = t.transform.translation.x;
            twist_msg.linear.y = t.transform.translation.y;
            twist_msg.linear.z = t.transform.translation.z;

            tf2::Quaternion quat;
            tf2::fromMsg(t.transform.rotation, quat);

            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            twist_msg.angular.x = roll;
            twist_msg.angular.y = pitch;
            twist_msg.angular.z = yaw;

            twist_pub_->publish(twist_msg);

            // --- Publicar TF relativo ---
            t.header.stamp = this->get_clock()->now();
            t.child_frame_id = child_frame_ + "_rel_to_" + parent_frame_;  // 👈 nombre único y claro
            tf_broadcaster_->sendTransform(t);

        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "⚠️ Transformación no disponible entre %s y %s: %s",
                parent_frame_.c_str(), child_frame_.c_str(), ex.what());
        }
    }

    // --- Atributos ---
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

    std::string child_frame_;
    std::string parent_frame_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfListenerNode>());
    rclcpp::shutdown();
    return 0;
}



