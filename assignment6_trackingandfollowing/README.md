
# Assignment 6 : Turtlebot following the April Tag 

### Git clone the repository :

`git clone https://github.com/dkoundinya/Autonomy_Science_Team_4.git `


This assignment consists of 3 parts:

  - Part 1: Turtlebot burger following the track based on color
  - Part 2: Turtlebot burger following the april tag 
 

## Part 1: Track Following

Run the task1.launch file in the launch folder using the following command:

`roslaunch assignment6_trackingandfollowing task1.launch`


## Part 2: April Tag Following

  To implement april tag following in real world use the following command:
  
`rosrun image_transport republish compressed in:=camera/image raw out:=camera/image_raw`

`roslaunch apriltag_ros continuous_detection.launch`

`rosrun assignment6_trackingandfollowing Tagfollowing.py`



## Videos of simulations and real-world implementation  are located in the videos folder

## Turtlebot Bringup commands:

`Run roscore at remote pc`

`roslaunch turtlebot3_bringup turtlebot3_robot.launch`

`roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch`

![April Tag Follow](https://github.com/dkoundinya/Autonomy_Science_Team_4/blob/main/assignment6_trackingandfollowing/src/Videos/Realworld.gif)

