cmake_minimum_required(VERSION 2.8.3)
project(odev)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  message_filters 
  tf
  nav_msgs 
  geometry_msgs
)


add_message_files(
   FILES
   floatStamped.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   nav_msgs
   tf
   geometry_msgs
 )

catkin_package(CATKIN_DEPENDS message_runtime message_filters)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
)



add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
