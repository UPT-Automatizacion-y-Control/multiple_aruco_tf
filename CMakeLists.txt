cmake_minimum_required(VERSION 3.0.2)
project(multiple_aruco_tf)

find_package(catkin REQUIRED COMPONENTS
  cv_camera
  aruco_ros
  std_msgs
  geometry_msgs
  message_generation
) 

add_service_files(
  FILES
  CurrentPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS message_runtime geometry_msgs
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_executable(multiple_aruco_tf src/multiple_aruco_tf.cpp)
target_link_libraries(multiple_aruco_tf ${catkin_LIBRARIES})

add_executable(relative_pose src/relative_pose.cpp)
target_link_libraries(relative_pose ${catkin_LIBRARIES})
add_dependencies(relative_pose multiple_aruco_tf_generate_messages_cpp)

add_executable(image_sat src/image_sat.cpp)
target_link_libraries(image_sat ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
