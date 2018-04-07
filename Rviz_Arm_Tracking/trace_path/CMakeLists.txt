cmake_minimum_required(VERSION 2.8.3)
project(trace_path)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
	tf
	geometry_msgs
	visualization_msgs
)

# The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS roscpp rospy tf2 tf2_ros tf geometry_msgs visualization_msgs
)

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
# With catkin_make all packages are built within a single CMake context
add_executable(trace_path src/trace_path.cpp)
# Specify libraries to link a library or executable target against
target_link_libraries(trace_path
   ${catkin_LIBRARIES}
)

add_executable(trace_path2 src/trace_path2.cpp)
target_link_libraries(trace_path2
	${catkin_LIBRARIES}
)

add_executable(ref_path src/ref_path.cpp)
target_link_libraries(ref_path
  ${catkin_LIBRARIES}
)

add_executable(trace_path_interval src/trace_path_interval.cpp)
target_link_libraries(trace_path_interval
  ${catkin_LIBRARIES}
)

