//example ROS service:
// run this as: rosrun example_ROS_service example_ROS_service
// in another window, tickle it manually with (e.g.): 
//    rosservice call lookup_by_name 'Ted'


#include <ros/ros.h>
#include <beginner_tutorials/exampleServiceMessage.h>
#include <iostream>
#include <string>
using namespace std;

bool callback(beginner_tutorials :: exampleServiceMessageRequest& request, beginner_tutorials :: exampleServiceMessageResponse& response) {
  ROS_INFO("callback activated");
  //let's convert this to a C++-class string, so can use member funcs
  string in_name(request.subject); 
  //cout<<"in_name:"<<in_name<<endl;
  response.onList=false;
  // access a small database..
  if (in_name.compare("ENPM808X") == 0) {
    ROS_INFO("asked about 808X");
    response.credits = 3;
    response.roboticsCourse = true;
    response.onList=true;
    response.subjectName="Robotics SOftware Development";
  } 

  if (in_name.compare("ENPM662") == 0) {
    ROS_INFO("asked about Robot Modelling");
    response.credits = 3;
    response.roboticsCourse = true;
    response.onList = true;
    response.subjectName="Robot Modelling";
  }

  if (in_name.compare("CMSC440") == 0) {
    ROS_INFO("asked about DataStructures");
    response.credits = 2;
    response.roboticsCourse = false;
    response.onList = true;
    response.subjectName = "Datastructures and Algorithms";
  }     

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "beginner_tutorials");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("lookup_by_subject_code", callback);
  ROS_INFO("Ready to check databse for the subjects");
  ros::spin();

  return 0;
}