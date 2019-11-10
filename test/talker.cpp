#include <ros/ros.h>
#include <gtest/gtest.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"

void testCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Level 2 test hearing: [%s].Test successful", msg->data.c_str());
}

TEST(TESTSuite, testTFbroadcasterTransformationExistance)
{
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("chatter", 1000, testCallback);
  bool exists(sub);
  EXPECT_TRUE(exists);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tf_talker");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

