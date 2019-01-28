cmake_minimum_required(VERSION 2.8.3)
project(smaract)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  std_msgs
  ati_ft_sensor #needs the ati force sensor data
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  act_data.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  setZeroActs.srv
  moveAbsolute.srv
  moveRelative.srv
  movetoLimit.srv
)

###########
## Build ##
###########

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include #specify location of exported headers
  #LIBRARIES //List the exported libraries from the project
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime  #other catkin projects that this package depends on
  #DEPENDS non-catkin cmake projects that this project depends on 
)

include_directories(
 ${catkin_INCLUDE_DIRS} # to include files like ros.h
 include # to include headers created in this package
 /usr/include # location of headers for external libraries (not necessary)
)

link_directories(
 /usr/lib # location of external libraries
)

add_executable(smaract_control src/ActsAsynch.cpp src/smaract_control.cpp)
target_link_libraries(smaract_control ${catkin_LIBRARIES} mcscontrol ftchipid ftd2xx) 
add_dependencies(smaract_control smaract_generate_messages_cpp)

add_executable(test_smaract src/test_smaract.cpp)
target_link_libraries(test_smaract ${catkin_LIBRARIES}) 
add_dependencies(test_smaract smaract_generate_messages_cpp)
