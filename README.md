# cv_detector_ros2_bridge

This project implements a flatbuffers-to-ROS message conversion bridge for detection lists published by the [gst-opencv-detector-plugin](https://github.com/rgmann/gst-opencv-detector) (or anything that publishes with the same interface).

## Getting Started

Add this package to your ROS2 workspace (e.g. as a submodule) in the `src` directory.

### Build the node

From the workspace root directory:  
`colcon build --packages-select cv_detector_ros2_bridge`

### Run the node

From the workspace root directory:  
`ros2 run cv_detector_ros2_bridge detections_ros2_bridge`

## Parameters

This node supports the following parameters.

`host`  
The IP address or hostname of the gst_opencv_detector server instance. The node defaults to assuming that the server is running locally so `host="127.0.0.1"`.

`port`  
The port number for the gst_opencv_detector server instance. The port defaults to `port="5050"`.

`class-filter`  
A string specifying the detection class names that should be forwarded by the bridge node. Filtering is disabled by specifying an empty string (default behavior). The parameter is treated as a string of strings, so if you wish to permit multiple class to be forwarded, specify something like `"person|dog"` or `"person,dog"`. The seperator is immaterial.

`sort-by-confidence`  
If true (default), the detections in the output list will be sort by decreasing confidence.

### Build Dependencies

This node depends on google/flatbuffers, which is not managed by any package control system. Once you download it, the packet is automatically built together with the node.

 1. `cd dep`
 1. `wget https://github.com/google/flatbuffers/archive/refs/tags/v25.1.24.tar.gz`
 1. `tar -zxvf v25.1.24.tar.gz`

 ## Author
 Robert Vaughan
