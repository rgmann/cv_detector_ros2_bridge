# cv_detector_ros2_bridge

This project implements a flatbuffers-to-ROS message conversion bridge for detection lists published by the [gst-opencv-detector-plugin](https://github.com/rgmann/gst-opencv-detector) (or anything that publishes with the same interface).

## Getting Started

### Build the node

`colcon build --packages-select cv_detector_ros2_bridge`

### Run the node

`ros2 run cv_detector_ros2_bridge detections_ros2_bridge`

### Build Dependencies

This node depends on google/flatbuffers, which is not managed by any package control system. Once you download it, the packet is automatically built together with the node.

 1. `cd dep`
 1. `wget https://github.com/google/flatbuffers/archive/refs/tags/v25.1.24.tar.gz`
 1. `tar -zxvf v25.1.24.tar.gz`

 ## Author
 Robert Vaughan
