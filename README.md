# Active SLAM with Cartograher

This work contains a modified version of [cartographer_frontier_detection](https://github.com/larics/cartographer_frontier_detection) and [rrt_exploration](https://github.com/hasauino/rrt_exploration). We implement an active exploration process and improve its robustness and performance. More details are described in the paper "Frontier Detection and Reachability Analysis for Efficient 2D Graph-SLAM Based Active Exploration" (IROS2020).

[![Active Exploration with Lazy and Reachable Frontier Detection](https://res.cloudinary.com/marcomontalbano/image/upload/v1595410567/video_to_markdown/images/youtube--o2mwIeMHkOo-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=o2mwIeMHkOo&t=49s "Active Exploration with Lazy and Reachable Frontier Detection")

## 1. Requirements

The package has been tested on Ubuntu18.04 with ROS Melodic, it should work on Ubuntu16.04 with ROS Kinetic. The following requirements are needed before installing the package:

(1) You should have installed a ROS distribution.

(2) Created a workspace.

(3) Installed the "Frontier detection for Google Cartographer" package. You can follow the intructions for compiling [Frontier detection for Google Cartographer](https://github.com/larics/cartographer_frontier_detection). However, instead of cloning the official version of cartographer_ros, clone this repository into your catkin workspace instead.

(4) Install ROS navigation stack. You can do that with the following command (assuming Ubuntu, ROS Melodic):

`$ sudo apt-get install ros-melodic-navigation`

(5) You should have Python 2.7 (it was not tested on Python 3) and have/install the following python modules:

Numpy

`$ sudo apt-get install python-numpy`

## 2. Usage

### 2.1 Public dataset

We have tested on [Deutsches Museum Dataset](https://google-cartographer-ros.readthedocs.io/en/latest/data.html). 

The data used in the paper is [here](https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag).

Remember to turn off `#define E2_ROBOT` in the file: cartographer_ros/cartographer_ros/cartographer_ros/sensor_bridge.cc

Run the launch file with:

`roslaunch rrt_exploration carto_offline_museum.launch bag_filename:=${PATH}/cartographer_paper_deutsches_museum.bag`

### 2.2 Our dataset

Remember to turn on `#define E2_ROBOT` in the file: cartographer_ros/cartographer_ros/cartographer_ros/sensor_bridge.cc

You can run our test dataset with:

`roslaunch rrt_exploration carto_offline_office.launch bag_filename:=${PATH}/horizon.bag`

## 3. Citing this work

The paper has not been published yet, the reference link will be given later.

## 4. Lisence

The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Zezhou Sun (sun_zezhou@njust.edu.cn) or Banghe Wu (916103860134@njust.edu.cn).
