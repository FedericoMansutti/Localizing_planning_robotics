cmake_minimum_required(VERSION 3.0.2)
project(second_project)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	nav_msgs
	move_base_msgs
	move_base
	actionlib
	tf
)

catkin_package(
	CATKIN_DEPENDS
        roscpp
        nav_msgs
        move_base_msgs
	move_base
        actionlib
	tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_publisher src/tf_publisher.cpp)
add_executable(base_to_lidar src/base_to_lidar.cpp)
add_executable(navigation src/navigation.cpp)

target_link_libraries(tf_publisher
  ${catkin_LIBRARIES}
)
target_link_libraries(base_to_lidar
  ${catkin_LIBRARIES}
)
target_link_libraries(navigation
  ${catkin_LIBRARIES}
)
