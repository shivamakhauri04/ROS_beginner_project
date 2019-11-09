#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <tf/transform_broadcaster.h>
#include "/home/shivam/catkin_ws/src/beginner_tutorials/src/talker.cpp"


TEST(TESTSuite, testTFbroadcaster)
{
  EXPECT_TRUE(&poseCallback);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "talkWithParentWorldClient");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

