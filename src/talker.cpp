/**
Copyright [MIT] 2019 Shivam Akhauri

Permission is hereby granted, free of charge, to any person obtaining a copy of 
this software and associated documentation files (the "Software"), to deal in 
the Software without restriction, including without limitation the rights to 
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 the Software, and to permit persons to whom the Software is furnished to do so,
 subject to the following conditions:

The above copyright notice and this permission notice shall be included in all 
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR 
 COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER 
 IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
 CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

/**
* @file talker.cpp
* @author Shivam Akhauri 
* @date 7 November 2019
* @copyright 2019 Shivam Akhauri
* @brief Contains the service to check for the details of a subject 
* in the subject database of University of Maryland
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

/**
* @brief function poseCallback
* @param const turtlesim::PoseConstPtr&
* @return none
* calculates the robot frame transformations 
* between the robot frames and world frame 
*/

void  poseCallback(const turtlesim::PoseConstPtr& msg){
  // create transform broadcaster to publish the transformations
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  // set the origin
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, msg->theta);
  // set the rotations
  transform.setRotation(q);
  // send the calculated transform
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}


int main(int argc, char** argv){
  // create the node
  ros::init(argc, argv, "talkWithParentWorld");
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("The Ros node for tf broadcaster not initialized");
  }
  turtle_name = argv[1];
  // create the node handle
  ros::NodeHandle node;
  // call the broadcaster to calculate transform between the frames
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);
  ros::spin();
  return 0;
};

