cmake_minimum_required(VERSION 2.8.3)
project(omnicam)
find_package(catkin REQUIRED COMPONENTS
	rospy
	sensor_msgs
	message_generation
	cv_bridge
)
 add_message_files(
   FILES
   Omni.msg
 )
 generate_messages(
   DEPENDENCIES
   std_msgs 
 )

catkin_package(
   CATKIN_DEPENDS message_runtime
)
include_directories(
 include ${catkin_INCLUDE_DIRS}
)
