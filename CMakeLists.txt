cmake_minimum_required(VERSION 2.8.3)
project(jaco_kinematics_plugin)
find_package(PCL REQUIRED COMPONENTS common registration)

 find_package(catkin REQUIRED
    geometry_msgs
    moveit_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_perception
    pcl_conversions
)
#find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS include    
    ${PCL_COMMON_INCLUDE_DIRS}
    ${PCL_REGISTRATION_INCLUDE_DIRS}
  CATKIN_DEPENDS     
         geometry_msgs
         moveit_ros_planning
         moveit_ros_perception
         moveit_msgs
         moveit_core
         moveit_ros         

  LIBRARIES
         )




#no messages, services, or action generations (add messages)
#no invoke message/service/action generations (generate messages)

include_directories(include                     
		    ${catkin_INCLUDE_DIRS}
                    )
link_directories(${catkin_LIBRARY_DIRS})


#OMPL stuff
#find_package(OMPL REQUIRED)
#include_directories( ${OMPL_INCLUDE_DIRS})
#link_directories(${OMPL_LIBRARY_DIRS})



add_library(jaco_kinematics_plugin src/jaco_kinematics_nn_cache_plugin.cpp)
target_link_libraries(jaco_kinematics_plugin moveit_kdl_kinematics_plugin ${catkin_LIBRARIES})

add_library(jaco_planning_adapter_plugin src/cache_planning_request_adapter.cpp)
target_link_libraries(jaco_planning_adapter_plugin ${catkin_LIBRARIES})

#add_library(moveit_pcl_registration src/moveit_pcl_registraction.cpp)
#target_link_libraries(moveit_pcl_registration $(catkin_LIBRARIES))
install(
  FILES
  jaco_kinematics_plugin_description.xml
  DESTINATION
      ${CATKIN_PACKAGE_SHARE_DESTINATION})

#add_dependencies(planning ~/jaco_moveit_ws/src/moveit_ros/planning)
