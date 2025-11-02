# multiple_aruco_ros2
This ROS 2 package enables the detection of multiple ArUco markers and publishes the corresponding TFs, which can be visualized in RViz.

 This tutorial was tested on ubuntu 24.04 with ROS2 jazzy.

# Dependencies
This the package depends on cv_camera, so it is recommended that you have already installed this package and follow all the instructions:  https://github.com/UPT-Automatizacion-y-Control/cv_camera.git

# Installation
1. Clone this package to your ~/ros2_ws/src:
   ```
   cd ~/ros2_ws/src
   ```
   
   ```
   git clone -b multiple_aruco_ros2 https://github.com/UPT-Automatizacion-y-Control/multiple_aruco_tf.git
   ```
2. Change the name of the package from "multiple_aruco_tf" to "multiple_aruco_rso2".

3. Resolve the package dependencies. In your workspace directory:

   ```
   rosdep install -i --from-path src --rosdistro jazzy -y
   ```

   1. If some dependencies are not resolved automatically, try installing them manually:
      ```
      sudo apt install ros-jazzy-aruco
      ```
      ```
      sudo apt install ros-jazzy-aruco-msgs
      ```


# ArUco detection
This package provides two launch files:
1. Detect all ArUco markers from a dictionary with the same size. 

You can specify the dictionary and marker size in the launch file.
```
ros2 launch multiple_aruco_ros2 all_arucos_fixed_size.launch
```   

2. Detect selected marker IDs with custom sizes for each: 

This launch file allows you to specify wich IDs you want to recognize and the custom size for each. The other IDs will be ignored.

```  
ros2 launch multiple_aruco_ros2 selected_arucos_selected_sizes.launch
``` 

Supported Dictionaries

- ARUCO

- ARUCO_MIP_16h3

- ARUCO_MIP_25h7

- ARUCO_MIP_36h12

- ARTOOLKITPLUS

- ARTOOLKITPLUSBCH

- ARTAG

- TAG16h5

- TAG25h7

- TAG25h9

- TAG36h11

- TAG36h10

- CHILITAGS

Download printable markes in: https://chev.me/arucogen/

You can also generate and use Dictionaries using as example the "generar_arucos_7_7_completo.py" node in the /src file which genererate the 7x7 Dictionary . An example of this is in the "relative_pose_example.launch".

# Relative Pose
Each launch file includes an optional feature to publish the relative pose between two TF frames via a topic.

Simply provide the parent_frame and child_frame in the launch file parameters.

The topic name will be automatically generated based on the frames provided.

Furthermore, the transformations will be generated automatically, which can be seen in rviz2.








