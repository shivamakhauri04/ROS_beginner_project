cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS tf roscpp rospy std_msgs genmsg message_generation)

## add_service_files(FILES talkWithParentWorld.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_gencpp)

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener beginner_tutorials_gencpp)

## Testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
  talkWithParentWorldClient service.launch test/talker.cpp)
  target_link_libraries(talkWithParentWorldClient ${catkin_LIBRARIES})
  add_dependencies(talkWithParentWorldClient talkWithParentWorld ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
endif()
