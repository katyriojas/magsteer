cmake_minimum_required(VERSION 2.8.3)
project(magsteering_test)

find_package(catkin REQUIRED COMPONENTS
  message_runtime
  message_generation
  roscpp
  std_msgs 
  ati_ft_sensor #including force sensor package nodelet
  smaract #include smaract package
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
 ${catkin_INCLUDE_DIRS} # to include files like ros.h
 include
)

link_directories(
 /usr/lib # location of external libraries
)

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
