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


#include <ros/ros.h>
#include <gtest/gtest.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"

/**
* @file test/talker.cpp
* @author Shivam Akhauri 
* @date 7 November 2019
* @copyright 2019 Shivam Akhauri
* @brief Creates a callback function for unit 
* testing TF_broadcaster topic 
*/
void testCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Level 2 test hearing: [%s].Test successful", msg->data.c_str());
}

/**
* @file test/talker.cpp
* @author Shivam Akhauri 
* @date 7 November 2019
* @copyright 2019 Shivam Akhauri
* @brief Contains the unit test for tf 
* broadcaster Topic existance 
*/
TEST(TESTSuite, testTFTopicTransformationExistance) {
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("chatter", 1000, testCallback);
  bool exists(sub);
  EXPECT_TRUE(exists);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tf_talker");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

