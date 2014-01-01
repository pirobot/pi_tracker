cmake_minimum_required(VERSION 2.8.3)
project(pi_tracker)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	nav_msgs
	message_generation)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(${catkin_INCLUDE_DIRS})

add_service_files(DIRECTORY srv
                  FILES
                  SetCommand.srv)

generate_messages(
 	DEPENDENCIES geometry_msgs std_msgs nav_msgs
)

catkin_package(
	DEPENDS #
    CATKIN DEPENDS message_runtime
	rospy
	roscpp
)

