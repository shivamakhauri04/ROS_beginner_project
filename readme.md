[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://github.com/shivamakhauri04/beginner_tutorials/blob/master/LICENSE.txt)

# ROS TF, Level 2 Tests with Ros and Ros Recording and playback

## Overview

This project demonstrates the TF concepts in ROS. TF calculates the frame transformations which are boon to robot software developers for the ease it provides when calculating frame transformations of large robots with multiple frame of references. This project explores the basics of TF broadcaster and listener concepts.
The project further explores the concept of rosbag which allows recording and playing back Ros topics. The project involves recording the tf broadcaster topic and read back the broadcaster topic details through he rosbag, when the listener is running.
The project also involves testing the ros broadcaster topic based on the concepts of level2 testing.


## Demo Instructions

To run the tf broadcaster and the listener
```
-git clone --recursive "https://github.com/shivamakhauri04/ROS_beginner_project.git"
-cp <path to repository> <catkin_workspace/src/>
-cd <catkin_workspace>
-catkin_make
-source ./devel/setup.bash
-roslaunch beginner_tutorials service.launch record_flag:="<argument>"
Note: The <argument> is "false" by default. If one wants to record the tf_broadcaster again, use "true".
- rosrun beginner_tutorials listener
-press Ctrl+C to end roslaunch

```
To playback rosbag data and execute listener 

```
(terminal 1)
git clone "https://github.com/shivamakhauri04/ROS_beginner_project.git"
cp <path to repository> <catkin_workspace/src/>
cd <catkin_workspace>
catkin_make
roscore
(terminal 2)
cd <catkin_workspace>
source ./devel/setup.bash
rosrun beginner_tutorials listener 
(terminal 3)
cd <catkin_workspace/src/beginner_tutorials/results/>
rosbag play talkerData.bag 

```
To Test the setup

```
git clone "https://github.com/shivamakhauri04/ROS_beginner_project.git"
cp <path to repository> <catkin_workspace/src/>
cd <catkin_workspace>
catkin_make tests
catkin_make test

```

## Assumption:
1. The Ubuntu system is setup with ROS Kinetics.
2. The catkin workspace has been setup.


## Copyright

Copyright (C) 2019 Shivam Akhauri
For license information, see [LICENSE.txt](LICENSE.txt).