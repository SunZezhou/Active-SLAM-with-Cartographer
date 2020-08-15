# Active SLAM with Cartograher

This work contains a modified version of [cartographer_frontier_detection](https://github.com/larics/cartographer_frontier_detection) and [rrt_exploration](https://github.com/hasauino/rrt_exploration). We implement an active exploration process and improve its robustness and performance. More details are described in the paper "Frontier Detection and Reachability Analysis for Efficient 2D Graph-SLAM Based Active Exploration" (IROS2020).

[![Active Exploration with Lazy and Reachable Frontier Detection](https://res.cloudinary.com/marcomontalbano/image/upload/v1595410567/video_to_markdown/images/youtube--o2mwIeMHkOo-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=o2mwIeMHkOo&t=49s "Active Exploration with Lazy and Reachable Frontier Detection")

## 1. Requirements

The package has been tested on Ubuntu16.04 with ROS Kinetic. It is known that apt-get install kuboki will cause errors in ROS Melodic. 

I provide a detailed docker installation procedure in [my-docke-env](https://github.com/SunZezhou/My-docker-env).

## 2. Usage

I have provided several available launch files:

`roslaunch cartographer_ros playbag.launch`  (Note that the default lidar topic is ‘/scan’)

`roslaunch cartographer_ros carto_gazebo.launch`  (The robot explores in the simulation environment。 TODO: adjust the move_base parameters)

`roslaunch cartographer_ros carto_movebase.launch`  (Standard SLAM with navigation)

`roslaunch cartographer_ros simple_move.launch`  (Send a virtual void map, only call move_base)

`roslaunch cartographer_ros turtlebot_hokuyo.launch` (The robot explores in the real environment。 TODO: adjust the move_base parameters)

## 3. Updates

master: Migrate code to turtlebot2.  (2020.08.15)

branch: Paper Version == tags v1.0   

## 4. Lisence

The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Zezhou Sun (sun_zezhou@njust.edu.cn) or Banghe Wu (916103860134@njust.edu.cn).
