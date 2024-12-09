# Experimental Robotics Lab - Assignment 1 
## Author: Marco Meschini 6273938
This repository contains the implementation of the  Assignment 1 for the Experimental Robotics Lab. The task involves detecting Aruco markers using a robot in a simulated environment. The implementation covers two scenarios:

1. **Robot Movement**: the robot moves to detect the markers.
2. **Camera Movement**: the robot remains stationary, and only the camera moves to detect the markers.

The URDF model of the robot has been modified to allow camera rotation through the controller. Also the simulation world has been updated to include five Aruco markers arranged in a circular configuration around the robot, randomly ordered.
A routine is implemented to find the Aruco markers in ascending order of their IDs. Each time a marker is detected, an image is published to a custom topic `'/aruco_markers'`, with a circle drawn around the detected marker.

## Scripts
Two scripts have been implemented:
- **`marker_detector`**: implements the behaviour where the entire robot moves to find the markers.
- **`marker_detector_cam`**: implements the behaviour where only the camera moves to find the markers.

Both scripts are located in the `ros2_aruco` package.

Is possible to launch the launch files containing the two scripts using the following commands:

1. **Robot Movement**:
 ```bash
 ros2 launch robot_urdf gazebo_aruco.launch.py
 ```
   
2. **Camera Movement**:
  ```bash
  ros2 launch robot_urdf gazebo_aruco_controller.launch.py
 ```

To visualize the topic where the images are published, I suggest to use `rqt_image_view` , selecting as topic `'/aruco_markers'`
  ```bash
ros2 run rqt_image_view rqt_image_view
 ```
