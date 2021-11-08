cmake_minimum_required(VERSION 2.8.3)
project(rfid_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  std_msgs
  geometry_msgs
  message_generation
)

find_package(gazebo REQUIRED)

add_message_files(
  FILES
  Tag.msg
  TagArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
	roscpp
	gazebo_ros
	std_msgs
  	geometry_msgs
  	message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(gazeboRfidAntennaPlugin SHARED src/rfidAntenna.cpp)
target_link_libraries(gazeboRfidAntennaPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazeboRfidAntennaPlugin ${PROJECT_NAME}_generate_messages)

add_library(gazeboRfidTagPlugin SHARED src/rfidTag.cpp)
target_link_libraries(gazeboRfidTagPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazeboRfidTagPlugin ${PROJECT_NAME}_generate_messages)



