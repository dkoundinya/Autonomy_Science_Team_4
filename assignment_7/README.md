# Assignment 7 : SLAM Mapping and Autonomous Navigation

### Part 1 : SLAM and Autonomous Navigation on Actual turtlebot

1. Launching of slam mapping node and teleop node

`roslaunch assignment_7 turtlebot3_bringup.launch `

2. Save the map generated and yaml files for using in autonomous navigation
 
`rosrun map_server map_saver -f ~/gmapping`
`rosrun map_server map_saver -f ~/karto`


3. Run the navigation Launch file and select the end goal using 2D Nav goal (the yaml files name need to updated depending on method)

`roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/gmapping.yaml'
`roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/karto.yaml'

### Part 2 : Comparison of Hokuyo Lidar and LDS sensor in Gazebo

For default LDS sensor:

1. Use the command for launching slam in gazebo world along with teleop node :

`roslaunch assignment_7 turtlebot3_world.launch `

2. Save the map generated and yaml files for using in autonomous navigation
 
`rosrun map_server map_saver -f ~/slamwithlds`

3. Run the navigation Launch file and select the end goal using 2D Nav goal

`roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/slamwithlds.yaml'

For Hokuyo Sensor :


1. Modify the urdf and xacro files in the gazebo /opt/ros/noetic/share/turtlebot3_description/urdf
2. add the stl mesh file of the corresponding sensor in the /opt/ros/noetic/share/turtlebot3_description/meshes

3. Use the command for launching slam in gazebo world along with teleop node :

`roslaunch assignment_7 turtlebot3_world_hokuyo.launch `

4. Save the map generated and yaml files for using in autonomous navigation
 
`rosrun map_server map_saver -f ~/slammapwithokuyo`

The brief report on our findings and videos of different sensors  is stored in report folder and videos folder respectively



