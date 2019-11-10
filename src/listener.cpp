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