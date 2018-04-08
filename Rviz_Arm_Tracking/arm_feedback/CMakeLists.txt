cmake_minimum_required(VERSION 2.8.3)
project(arm_feedback)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  message_generation
	visualization_msgs
	tf
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

SET (CMAKE_CXX_STANDARD 11)
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET (CMAKE_CXX_STANDARD_REQUIRED ON)
SET (SONAME "hebi")
SET (MAJOR_VERSION "0") # C MAJOR VERSION
SET (MINOR_VERSION "12") # C MINOR VERSION
SET (CPP_VERSION "0") # CPP VERSION
SET (PKG_DIR ${CMAKE_BINARY_DIR}/${MAJOR_VERSION}.${MINOR_VERSION}.${CPP_VERSION})

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  CommandML.msg
  FeedbackML.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime visualization_msgs moveit_core 
  moveit_ros_planning_interface
  DEPENDS system_lib
	EXPORTED_TARGETS arm_feedback_generate_messages_cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories (
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/src      # CPP wrapper classes
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include  # Underlying C headers
  ${Eigen_INCLUDE_DIRS}
  )

link_directories (
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
  )



add_executable(kdc_arm_feedback
# hebi_listener 
#   src/hebi_listener.cpp
  src/kdc_arm_feedback.cpp
  src/mac_address.cpp
  src/command.cpp
  src/feedback.cpp
  src/info.cpp
  src/group_command.cpp
  src/group_feedback.cpp
  src/group_info.cpp
  src/module.cpp
  src/group.cpp
  src/lookup.cpp
  src/arm_dynamics.cpp
  src/arm_kinematics.cpp
)
add_dependencies(kdc_arm_feedback arm_feedback_generate_messages_cpp)
ADD_DEPENDENCIES(kdc_arm_feedback ${catkin_EXPORTED_TARGETS})
TARGET_LINK_LIBRARIES(kdc_arm_feedback ${catkin_LIBRARIES})


target_link_libraries(kdc_arm_feedback ${catkin_LIBRARIES} hebi m pthread)

add_executable(arm_feedback
# hebi_listener 
#   src/hebi_listener.cpp
	src/arm_feedback.cpp
  src/mac_address.cpp
  src/command.cpp
  src/feedback.cpp
  src/info.cpp
  src/group_command.cpp
  src/group_feedback.cpp
  src/group_info.cpp
  src/module.cpp
  src/group.cpp
  src/lookup.cpp
  src/arm_dynamics.cpp
)

add_dependencies(arm_feedback arm_feedback_generate_messages_cpp)
ADD_DEPENDENCIES(arm_feedback ${catkin_EXPORTED_TARGETS})
TARGET_LINK_LIBRARIES(arm_feedback ${catkin_LIBRARIES})

add_library(arm_feedback_files OBJECT
  src/mac_address.cpp
  src/command.cpp
  src/feedback.cpp
  src/info.cpp
  src/group_command.cpp
  src/group_feedback.cpp
  src/group_info.cpp
  src/module.cpp
  src/group.cpp
  src/lookup.cpp
)

# target_link_libraries(hebi_listener ${catkin_LIBRARIES} hebi m pthread )
target_link_libraries(arm_feedback ${catkin_LIBRARIES} hebi m pthread)

