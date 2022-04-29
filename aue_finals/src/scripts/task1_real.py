#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import math
from sensor_msgs.msg import LaserScan

def callback(msg):
    print('callback')

    ranges=list(msg.ranges)
    pub= rospy.Publisher('/cmd_vel',Twist,queue_size=30)
    vel=Twist()
    k=0.3
    omega=0.05

    # front=min(min(i for i in range[330:360] if i>0),min(i for i in range[0:30] if i>0))
    for i in range(len(ranges)):
        if ranges[i]==0:
            ranges[i]=3
    
    # print(ranges)
    # print(range)
    front=min(min(i for i in ranges[330:360] if i>0),min(i for i in ranges[0:30] if i>0))
    # fr=np.mean(range[30:70])
    # fl=np.mean(range[290:330])
    # left=min(i for i in range[30:85] if i>0)
    # right=min(i for i in range[275:330] if i>0)
    left=min(i for i in ranges[30:90] if i>0)
    right=min(i for i in ranges[270:330] if i>0)
    rmin=(right)*(-1)
    lmin=(left)
    if front==0:
        front=3
    if rmin == 0:
        rmin=-3
    if lmin == 0:
        lmin=3
    side=rmin+lmin
    # ef=front-0.3


    

    print(front,side,lmin,rmin)  
    # or lmin<0.3 or rmin>-0.3: 
    if front<1.2:
        vel.linear.x=k*front*0.5
        vel.angular.z=omega*side*(1/(front))*8
        print('turn')
        print(vel)
 
    else:
        vel.linear.x=k*front
        vel.angular.z=omega*side*(1/(front))
        # vel.angular.z=0
        print('normal')

    pub.publish(vel)   
    
def main():
    rospy.init_node('turtlebot3', anonymous=True)
    print('calling')
    scan=rospy.Subscriber('/scan',LaserScan,callback)

    


if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            print('main')
            main()

            rospy.spin()

        except rospy.ROSInterruptException: 
            pass



