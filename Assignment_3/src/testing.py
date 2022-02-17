#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def move(dist):
    while True:
        if dist<= 2.000000000:
            stop()
        else:
         vel.linear.x=0.4
         pub.publish(vel)
         print(vel)
         scan_callback(sub)   

def stop():
    vel.linear.x= 0
    pub.publish(vel)


def scan_callback(msg):
    range = msg.ranges
    print(msg.ranges)
    dist=min(min(range),10)
    print(dist)
    move(dist)

if __name__ == '__main__':
     try:
         while not rospy.is_shutdown():
            rospy.init_node('group4',anonymous=True)
            pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
            vel=Twist()
            scanpub= rospy.Publisher('/revised_scan',LaserScan,queue_size=10)
            scann=LaserScan()
            sub=rospy.Subscriber('/scan',LaserScan,scan_callback)
            rate=rospy.Rate(10)
            print(vel)
            rospy.spin() 
     except rospy.ROSInterruptException:
        pass