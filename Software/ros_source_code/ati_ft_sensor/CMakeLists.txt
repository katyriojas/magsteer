cmake_minimum_required(VERSION 2.8.3)
project(ati_ft_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
)

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   force_data.msg
 )
	
# Generate services in the 'srv' folder
 add_service_files(
   FILES
   setZeroFT.srv
  )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS} # to include files like ros.h
  include # to include headers created in this package
  /usr/include # location of headers for external libraries (not necessary)
)

link_directories(
  /usr/lib # location of external libraries
)

add_executable(read_ft_sensor src/read_ft_sensor.cpp src/ForceSensor.cpp)
target_link_libraries(read_ft_sensor ${catkin_LIBRARIES} comedi armadillo)
add_dependencies(read_ft_sensor ati_ft_sensor_generate_messages_cpp) # may not need

add_executable(test_ft_sensor src/test_ft_sensor.cpp)
target_link_libraries(test_ft_sensor ${catkin_LIBRARIES})
add_dependencies(test_ft_sensor ati_ft_sensor_generate_messages_cpp)
