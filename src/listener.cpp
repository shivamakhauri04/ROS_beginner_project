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
**/

#include <tf/transform_listener.h>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "tf_listener");


    tf::TransformListener listener;
    ros::NodeHandle node;
    ros::Rate rate(10);
    while (ros::ok()) {
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("/world", "/talk",
                                   ros::Time(0), transform);
            std::cout << "x: " << transform.getOrigin().x() << " y: "
                            << transform.getOrigin().y() << " z: "
                            << transform.getOrigin().z() << std::endl;
            std::cout << "Quaternion: " << transform.getRotation().x() << ", "
                                    << transform.getRotation().y() << ", "
                                    << transform.getRotation().z() << ", "
                                    << transform.getRotation().w() << std::endl;
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
