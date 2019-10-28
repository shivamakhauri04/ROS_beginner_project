# ROS Tutorial on Publishers and Subscribers

## Overview

This project demonstrates the publishers and suscribers concept in ROS. The project involves development of communation between a talker and a listner. The talker publishes a text and the listener suscribes to the text. The project is developed in C++.


## Standard install via command-line

```
git clone --recursive "https://github.com/shivamakhauri04/ROS_beginner_project.git"
cp <path to repository> <catkin_workspace/src/>
cd <catkin_workspace/build>
make
cd ..
roscore (in terminal 1)
rosrun beginner_tutorial talker (in terminal 2)
rosrun beginner_tutorial listener (in terminal 3)

```
