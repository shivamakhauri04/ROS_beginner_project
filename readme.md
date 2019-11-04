[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://github.com/shivamakhauri04/beginner_tutorials/blob/master/LICENSE.txt)

# ROS service to look up for a subject information from a subject database of the University of Maryland.

## Overview

This project demonstrates the service and client concept in ROS. The project involves a service which checks for a subject information from the subject database.

Tha database stores subject information like the name of the subject, the number of credits associated with the subject, and information if the suject is a robotics subject or CMSC subject. The information in the database is stored using Subject Code as the key.

The client sends the request to this service in form of the subject code. The request is received by the service which then looks up in the database for the subject code.
If the subject code is in the database, the service retrives the details from the database. If the subject code is not in the database, the user is told that the subject informtaion is not available.



## Standard install via command-line

```
-git clone --recursive "https://github.com/shivamakhauri04/ROS_beginner_project.git"
-cp <path to repository> <catkin_workspace/src/>
-cd <catkin_workspace>
-catkin_make
-roslaunch --screen beginner_tutorials service.launch args1:="<argument>"
Note: The <argument> is "slow" for slower service rate or "fast" for faster service.
-Enter the Subject code you want to search about. 
-Enter "x" if you want to exit and press Ctrl+C to end roslaunch

```
## Limitation:
- Conflict with ROS and C++ 11 standard : The .srv files for the ros service automatically creates header files, which have template of accepting address by reference using the '&' operator. The cpplint requires this refernce to be made as "const" but the ros package doesn't accept "const" to be specified before the refernce. Hence this leads to cpplint errors to comply with ROS templates. 

## Assumption:
1. The Ubuntu system is setup with ROS Kinetics.
2. The catkin workspace has been setup.

Note : Curently the database has information about only 3 subjects: ENPM808X, ENPM662, CMSC440  


## Copyright

Copyright (C) 2019 Shivam Akhauri
For license information, see [LICENSE.txt](LICENSE.txt).