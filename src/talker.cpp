/**
* @file talker.cpp
* @author Shivam Akhauri 
* @date 31 October 2019
* @copyright 2019 Shivam Akhauri
* @brief Contains the service to check for the details of a subject 
* in the subject database of University of Maryland
*/

#include <ros/ros.h>
#include <beginner_tutorials/exampleServiceMessage.h>
#include <iostream>
#include <string>


/**
* @brief function to provide the service to check 
* for a subject from the subject database. It returns the subject name,
* Number of credits associated with the subject, if the suject is robotics
* subject or CMSC subject.
* The client sends the request to this service in form of a subject code
* The service analyses the request to check if it is in the database
* @params beginner_tutorials :: exampleServiceMessageRequest& request 
* <generated in the exampleServiceMessage.h in devel/include 
* upon adding .srv to the package 
* @params beginner_tutorials :: exampleServiceMessageResponse& response
* <generated in the exampleServiceMessage.h in devel/include 
* upon adding .srv to the package 
* @return bool: if the service responses
*/
bool callback(beginner_tutorials :: exampleServiceMessageRequest& request, beginner_tutorials :: exampleServiceMessageResponse& response) {
  ROS_INFO("callback activated");
  // convert the client data to a C++-class string, to use member funcs
  std::string in_name(request.subject); 
  response.onList=false;
  // create a small database.
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


/**
* @brief main function. creates a service node
* creates a service and advertises it over ROS
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "beginner_tutorials_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("lookup_by_subject_code", callback);
  ROS_INFO("Ready to check databse for the subjects");
  int rate = 10;
  if (argv[1]=="slow") {
    rate = 20;
  }
  ros::Rate loop_rate(rate);
  loop_rate.sleep();
  ros::spin();

  return 0;
}