# YOLOv5 ROS on Nvidia Jetson
This is a ROS interface for using YOLOv5 for real time object detection on a ROS image topic. It supports inference on multiple deep learning frameworks used in the [official YOLOv5 repository](https://github.com/ultralytics/yolov5).

## Installation

### Dependencies
This package is built and tested on Ubuntu 20.04 LTS and ROS Noetic with Python 3.8.

* Clone the packages to ROS workspace and install requirement for YOLOv5 submodule:
```bash
cd <ros_workspace>/src
git clone https://github.com/mats-robotics/detection_msgs.git
git clone --recurse-submodules https://github.com/mats-robotics/yolov5_ros.git 
cd yolov5_ros/src/yolov5
pip install -r requirements.txt # install the requirements for yolov5
```
* Build the ROS package:
```bash
cd <ros_workspace>
catkin build yolov5_ros # build the ROS package
```
* Make the Python script executable 
```bash
cd <ros_workspace>/src/yolov5_ros/src
chmod +x detect.py
```

## Basic usage
Change the parameter for `input_image_topic` in launch/yolov5.launch to any ROS topic with message type of `sensor_msgs/Image` or `sensor_msgs/CompressedImage`. Other parameters can be modified or used as is.

* Launch the node with **default** parameters:
```bash
roslaunch yolov5_ros yolov5.launch
```

* Launch the node with **custom** parameters:
```bash
roslaunch yolov5_ros yolov5.launch \
    weights:=/path/to/your/custom_weights.pt \
    data:=/path/to/your/custom_data.yaml \
    confidence_threshold:=0.8 \
    iou_threshold:=0.5 \
    maximum_detections:=500 \
    device:=gpu \
    agnostic_nms:=false \
    line_thickness:=5 \
    dnn:=false \
    half:=true \
    inference_size_h:=720 \
    inference_size_w:=1280 \
    view_image:=false \
    input_image_topic:=/your/custom/image/topic \
    output_topic:=/your/custom/output/topic \
    publish_image:=true \
    output_image_topic:=/your/custom/output/image/topic
```

## Using custom weights and dataset (Working)
* Put your weights into `yolov5_ros/src/yolov5`
* Put the yaml file for your dataset classes into `yolov5_ros/src/yolov5/data`
* Change related ROS parameters in yolov5.launch: `weights`,  `data`

## Reference
* YOLOv5 official repository: https://github.com/ultralytics/yolov5
* YOLOv3 ROS PyTorch: https://github.com/eriklindernoren/PyTorch-YOLOv3
* Darknet ROS: https://github.com/leggedrobotics/darknet_ros
