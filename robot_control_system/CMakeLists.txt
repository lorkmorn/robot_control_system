cmake_minimum_required(VERSION 2.8.3)
project(robot_control_system)

########## load variables in catkin and add dependencies         
find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs nav_msgs rosconsole roscpp_serialization roslib rostime tf)

########## add path to header files
include_directories(
 include
 ${catkin_INCLUDE_DIRS}   ####this is a path variable 
 /usr/local/include/yaml-cpp
)

link_directories(
 ${catkin_LIBRARY_DIRS} 
 /usr/local/lib 
 /usr/include
)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS geometry_msgs std_msgs std_srvs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations

add_executable(controller src/controller.cpp src/dwa_nav/dwa_nav.cpp)
##add_dependencies(controller ${nav_msgs_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES})

add_executable(motion_model_simulation src/motion_model_simulation.cpp src/dwa_nav/dwa_nav.cpp)
##add_dependencies(controller ${nav_msgs_EXPORTED_TARGETS})
target_link_libraries(motion_model_simulation ${catkin_LIBRARIES} yaml-cpp)

add_executable(drawer src/drawer.cpp)
##add_dependencies(controller ${nav_msgs_EXPORTED_TARGETS})
target_link_libraries(drawer ${catkin_LIBRARIES})
