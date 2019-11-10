#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <tf/transform_broadcaster.h>
#include "../src/pose.cpp"


TEST(TESTSuite, testTFbroadcaster)
{
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/turtle1/pose", 10, &poseCallback);
  bool exists(sub);
  EXPECT_TRUE(exists);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "talkWithParentWorldClient");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

