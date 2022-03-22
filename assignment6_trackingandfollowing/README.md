#Run roscore at remote pc

roslaunch turtlebot3_bringup turtlebot3_robot.launch
roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch
rosrun image_transport republish compressed in:=camera/image raw out:=camera/image_raw
roslaunch apriltag_ros continuous_detection.launch
rosrun assignment6_trackingandfollowing Tagfollowing.py
