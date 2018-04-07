cmake_minimum_required(VERSION 2.8.3)
project(arm_rviz)

find_package(catkin REQUIRED COMPONENTS
	roslaunch
	xacro
)

roslaunch_add_file_check(launch)

install(DIRECTORY launch/ rviz urdf
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY meshes/
  		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(DIRECTORY urdf/
  		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

catkin_package()
