This is a demo use dynamic window approach to navigate from one place to another in two-dimension.

My environment: ubuntu18.04, ros melodic

Before navigation, initial position [x,y,yaw] and goal position[x,y,yaw] can be set in robotconfig.yaml. Also, [max acceleration] can be set in robotconfig.yaml, start position is now useless. Then compile the pkg, and [roslaunch robot_control_system robot_full.launch] and you will see the navigation.
