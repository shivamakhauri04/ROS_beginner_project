/**
* @file listener.cpp
* @author Shivam Akhauri 
* @date 31 October 2019
* @copyright 2019 Shivam Akhauri
* @brief Contains the client to check for the details of a subject 
* in the subject database of University of Maryland
*/

#include <ros/ros.h>
#include <beginner_tutorials/exampleServiceMessage.h>
#include <iostream>
#include <string>

/**
* @brief main function. creates a client 
* generates a call to the service 
*/
int main(int argc, char **argv) {
    // initialize the node. Necessary for ros program 
    ros::init(argc, argv, "beginner_tutorials_client");
    // the main access point to communicate with ros systems. 
    // fully initializes the node
    ros::NodeHandle n;
    // this creates the client
    ros::ServiceClient client = n.serviceClient<beginner_tutorials::exampleServiceMessage>("lookup_by_subject_code");
    beginner_tutorials::exampleServiceMessage srv;
    std::string subject_code;
    while (ros::ok()) {
        // following print statements are for UI
        std::cout<<std::endl;
        std::cout << "Enter a subject code (x to quit): ";
        std::cout << "Note Current database has info on ENPM808X, ENPM662, CMSC440.";
        // Enter the subject code to lookup database
        std::cin >> subject_code;
        // If pressed 'x' exit
        if (subject_code.compare("x")==0)
            return 0;
        // assign values into the request member
        srv.request.subject = subject_code;
        std::cout << subject_code;
        // calling of the service.
        if (client.call(srv)) {
            // check for the responses and accordingly print the results
            if (srv.response.onList) {
                std::cout << srv.request.subject << " is known as " << srv.response.subjectName << std::endl;
                std::cout << " It has " << srv.response.credits << " credits associated" << std::endl;
                if (srv.response.roboticsCourse)
                    std::cout << " This subject is a Robotics engineering course" << std::endl;
                else
                    std::cout << " This subject is a CMSC course. Not a ENPM course" << std::endl;
            } else {
                std::cout << srv.request.subject << " is not in my subject list databse. Current database has info on ENPM808X, ENPM662, CMSC 440. " << std::endl;
            }

        } else {
            // if the service call fails
            ROS_ERROR(" Failed to call service lookup_by_subject_code. ");
            return 1;
        }
    }
    return 0;
}