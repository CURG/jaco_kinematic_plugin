cmake_minimum_required(VERSION 2.8.3)
project(jaco_kinematics_plugin)

 find_package(catkin REQUIRED
    COMPONENTS message_generation  
    moveit_core
    moveit_ros_perception
    pluginlib
    actionlib
    dynamic_reconfigure
    angles
    roscpp
)
find_package(Eigen REQUIRED)
include_directories(SYSTEM
                    ${EIGEN_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
     moveit_core moveit_ros_perception pluginlib actionlib dynamic_reconfigure angles roscpp 
)

install(
  FILES
  kdl_kinematics_plugin_description.xml
  DESTINATION
      ${CATKIN_PACKAGE_SHARE_DESTINATION})

#add_dependencies(planning ~/jaco_moveit_ws/src/moveit_ros/planning)

add_library(jaco_kinematics_plugin src/jaco_kinematics_plugin.cpp)
