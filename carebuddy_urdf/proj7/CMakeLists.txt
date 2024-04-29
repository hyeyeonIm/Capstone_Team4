cmake_minimum_required(VERSION 2.8.3)

project(proj7)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
  std_msgs
  tf
  urdf
  xacro
)
catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
