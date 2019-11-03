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
    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM("The Ros node for service not initialized");
    }
    // this creates the client
    ros::ServiceClient client = n.serviceClient<beginner_tutorials::exampleServiceMessage>("lookup_by_subject_code");
    beginner_tutorials::exampleServiceMessage srv;
    std::string subject_code;
    while (ros::ok()) {
        // following print statements are for UI
        std::cout<<std::endl;
        std::cout << "Enter a subject code (x to quit): ";
        ROS_INFO_STREAM("Note Current database has info on ENPM808X, ENPM662, CMSC440.");
        // Enter the subject code to lookup database
        std::cin >> subject_code;
        // If pressed 'x' exit
        if (subject_code.compare("x")==0){
            ROS_WARN_STREAM("you did not enter any subject code.");
            return 0;
        }
        // assign values into the request member
        srv.request.subject = subject_code;
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
            ROS_ERROR_STREAM(" Failed to call service lookup_by_subject_code. ");
            return 1;
        }
    }
    return 0;
}