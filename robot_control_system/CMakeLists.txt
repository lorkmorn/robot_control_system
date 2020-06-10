cmake_minimum_required(VERSION 2.8.3)
project(robot_control_system)


find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs nav_msgs rosconsole roscpp_serialization roslib rostime tf)
find_package(Qt5Widgets REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
 /usr/local/include/yaml-cpp
)

link_directories(${catkin_LIBRARY_DIRS} /usr/local/lib /usr/include)

catkin_package(CATKIN_DEPENDS geometry_msgs std_msgs std_srvs)


## Specify additional locations of header files
## Your package locations should be listed before other locations


add_executable(controller src/controller.cpp)
##add_dependencies(controller ${nav_msgs_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES})

add_executable(motion_model_simulation src/motion_model_simulation.cpp)
##add_dependencies(controller ${nav_msgs_EXPORTED_TARGETS})
target_link_libraries(motion_model_simulation ${catkin_LIBRARIES} yaml-cpp)

add_executable(drawer src/drawer.cpp)
##add_dependencies(controller ${nav_msgs_EXPORTED_TARGETS})
target_link_libraries(drawer ${catkin_LIBRARIES})
