#include <ros/ros.h>
#include <beginner_tutorials/exampleServiceMessage.h> // this message type is defined in the current package
#include <iostream>
#include <string>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "beginner_tutorials");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<beginner_tutorials::exampleServiceMessage>("lookup_by_subject_code");
    beginner_tutorials::exampleServiceMessage srv;
    bool found_on_list = false;
    string subject_code;
    while (ros::ok()) {
        cout<<endl;
        cout << "enter a subject code (x to quit): ";
        cin>>subject_code;
        if (subject_code.compare("x")==0)
            cout<< "exit with X";
            return 0;
        //cout<<"you entered "<<in_name<<endl;
        srv.request.subject = subject_code; //"ENPM808X";
        if (client.call(srv)) {
            if (srv.response.onList) {
                cout << srv.request.subject << " is known as " << srv.response.subjectName << endl;
                cout << "It is" << srv.response.credits << " credits associated" << endl;
                if (srv.response.roboticsCourse)
                    cout << "This subject is a Robotics engineering course" << endl;
                else
                    cout << "This subject is a CMSC course. Not a ENPM course" << endl;
            } else {
                cout << srv.request.subject << " is not in my subject list databse. Current database has info on ENPM808X, ENPM662, CMSC 440." << endl;
            }

        } else {
            ROS_ERROR("Failed to call service lookup_by_subject_code.");
            return 1;
        }
    }
    return 0;
}