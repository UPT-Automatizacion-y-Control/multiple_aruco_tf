cmake_minimum_required(VERSION 3.8)
project(multiple_aruco_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11

set(THIS_PACKAGE_INCLUDE_DEPENDS
    OpenCV
    cv_bridge
    geometry_msgs
    image_transport
    rclcpp
    rclpy
    tf2
    tf2_ros
    tf2_geometry_msgs
    aruco
    aruco_msgs
    sensor_msgs
    visualization_msgs
    turtlesim
    usb_cam
    yaml-cpp 
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(multiple_aruco_tf_ros2 src/multiple_aruco_tf_ros2.cpp)
target_include_directories(multiple_aruco_tf_ros2
  PUBLIC
  include)

target_include_directories(multiple_aruco_tf_ros2
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(multiple_aruco_tf_ros2 ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(multiple_aruco_tf_ros2 ${OpenCV_LIBRARIES})

add_executable(relative_pose src/relative_pose.cpp)
target_include_directories(relative_pose
  PUBLIC
  include)

target_include_directories(relative_pose
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(relative_pose ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(relative_pose ${OpenCV_LIBRARIES})

add_executable(obtener_imagen src/obtener_imagen.cpp)
target_include_directories(obtener_imagen
  PUBLIC
  include)

target_include_directories(obtener_imagen
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(obtener_imagen ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(obtener_imagen ${OpenCV_LIBRARIES})

add_executable(tf_relativo src/tf_relativo.cpp)
target_include_directories(tf_relativo
  PUBLIC
  include)

target_include_directories(tf_relativo
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(tf_relativo ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(tf_relativo ${OpenCV_LIBRARIES})

add_executable(camera_hd src/camera_hd.cpp)
target_include_directories(camera_hd
  PUBLIC
  include)

target_include_directories(camera_hd
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(camera_hd ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(camera_hd ${OpenCV_LIBRARIES})









add_executable(calibrar_camara src/calibrar_camara.cpp)
target_include_directories(calibrar_camara
  PUBLIC
  include)

target_include_directories(calibrar_camara
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(calibrar_camara ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(calibrar_camara ${OpenCV_LIBRARIES})

##################################################################################333
add_executable(video_capture_best_effort src/video_capture_best_effort.cpp)
target_include_directories(video_capture_best_effort
  PUBLIC
  include)

target_include_directories(video_capture_best_effort
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(video_capture_best_effort ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(video_capture_best_effort ${OpenCV_LIBRARIES})

##################################################################################333
add_executable(hsv_segmentacion src/hsv_segmentacion.cpp)
target_include_directories(hsv_segmentacion
  PUBLIC
  include)

target_include_directories(hsv_segmentacion
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(hsv_segmentacion ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(hsv_segmentacion ${OpenCV_LIBRARIES})

##################################################################################333
add_executable(almacenar_tf src/almacenar_tf.cpp)
target_include_directories(almacenar_tf
  PUBLIC
  include)

target_include_directories(almacenar_tf
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(almacenar_tf ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(almacenar_tf ${OpenCV_LIBRARIES})

##################################################################################333

##################################################################################333
add_executable(almacenar_tf_txt src/almacenar_tf_txt.cpp)
target_include_directories(almacenar_tf_txt
  PUBLIC
  include)

target_include_directories(almacenar_tf_txt
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(almacenar_tf_txt ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(almacenar_tf_txt ${OpenCV_LIBRARIES})

##################################################################################333
add_executable(publish_tf src/publish_tf.cpp)
target_include_directories(publish_tf
  PUBLIC
  include)

target_include_directories(publish_tf
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(publish_tf ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(publish_tf ${OpenCV_LIBRARIES})
#add_executable(listener_tf src/listener_tf.cpp)
#target_include_directories(listener_tf
#  PUBLIC
#  include)

#target_include_directories(listener_tf
#  SYSTEM PUBLIC
#  ${OpenCV_INCLUDE_DIRS}
#)
#ament_target_dependencies(listener_tf ${THIS_PACKAGE_INCLUDE_DEPENDS})
#target_link_libraries(listener_tf ${OpenCV_LIBRARIES})


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
  multiple_aruco_tf_ros2
  relative_pose
  tf_relativo
  obtener_imagen
  camera_hd
  calibrar_camara
  video_capture_best_effort
  hsv_segmentacion
  almacenar_tf
  almacenar_tf_txt
  publish_tf
#  listener_tf
  DESTINATION lib/${PROJECT_NAME})

	
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)  


ament_package()
