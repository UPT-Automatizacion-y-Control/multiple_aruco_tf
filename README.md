# multiple_aruco_tf
This ros package shows the detection of multiple aruco markers and the corresponding tf in rviz.

 This tutorial works on ubuntu 20.04 with ros noetic.
 
0. You should configure your catkin_ws before starting. 
 Follow the instructions:  http://wiki.ros.org/catkin/Tutorials/create_a_workspace

1. Clone this package to your ~/catkin_ws/src:

   cd ~/catkin_ws/src
   git clone https://github.com/UPT-Automatizacion-y-Control/multiple_aruco_tf.git

2. This package depends on aruco-ros, if you don't have it, install it to your ~/catkin_ws/src:

   sudo apt-get install ros-noetic-aruco-ros

3. The package uses cv_camera to capture video device, if you don't have it, install it to your ~/catkin_ws/src:

   sudo apt-get install ros-noetic-cv-camera

4. Compile your workspace:

   cd ~/catkin_ws

   catkin_make
   
5. If you don't have the calibration parameters of your camera follow steps 6-8, otherwise go to step 9.
   
6. run: roslaunch multiple_aruco_tf camera_calibration.launch

7. Follow the instructions: http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

8. Extract the yaml file, it is saved to /tmp/calibrationdata.tar.gz 

9. Copy the yaml file of the calibration parameters of your camera to

    ~/catkin_ws/src/multiple_aruco_tf/camera_info

10. For each launch file located in ~/catkin_ws/src/multiple_aruco_tf/launch

    Define argument fixed_tf, the name of the fixed frame.

    Define argument camera_name, the yaml file of camera configuration parameters must have the same name

11. Open the yaml file of camera configuration parameters, camera_name must be the same as the one defined in step 10.
    
12. Run in different terminals:

    roslaunch multiple_aruco_tf get_image.launch
    
    roslaunch multiple_aruco_tf get_transformation.launch
    
    roslaunch multiple_aruco_tf remote_visualization.launch 

Now you should be able to see the video stream and the aruco marker detection as well as rviz showing tf.

In case aruco markers are not detected edit dictionary_type in multiple_aruco_tf.launch.

Package aruco_ros can detect these dictionaries: 

ARUCO, ARUCO_MIP_16h3, ARUCO_MIP_16h3, ARUCO_MIP_25h7, ARUCO_MIP_36h12, ARTOOLKITPLUS, RTOOLKITPLUSBCH, ARTAG, TAG16h5, TAG25h7, TAG25h9, TAG36h11, TAG36h10, CHILITAGS

Download printable markes in: https://damianofalcioni.github.io/js-aruco2/samples/marker-creator/marker-creator.html


