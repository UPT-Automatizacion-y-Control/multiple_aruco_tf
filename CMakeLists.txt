cmake_minimum_required(VERSION 3.0.2)
project(multiple_aruco_tf)

find_package(catkin REQUIRED COMPONENTS
  aruco_ros
) 

catkin_package(
  INCLUDE_DIRS 
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

 add_executable(multiple_aruco_tf src/multiple_aruco_tf.cpp)
 target_link_libraries(multiple_aruco_tf ${catkin_LIBRARIES} )
