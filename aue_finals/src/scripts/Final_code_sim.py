#!/usr/bin/env python3
#Improved
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3
import math
from sensor_msgs.msg import LaserScan
from darknet_ros_msgs.msg import BoundingBoxes
from apriltag_ros.msg import AprilTagDetectionArray
import time



class LineFollower(object):
    # print("NO.2")

    def __init__(self):
        self.frame=False
        self.error=0
        self.sign=1
        self.bridge_object = CvBridge()
        self.stopsign=False
        self.mover=True
        self.stop_active=False

        self.image_sub = rospy.Subscriber("tb_1/camera/rgb/image_raw/",Image,self.camera_callback)
        self.scan=rospy.Subscriber('tb_1/scan',LaserScan,self.laser_callback) 
        self.stop=rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.stop_sign_callback)  
        self.apriltag=rospy.Subscriber('/tag_detections_image',AprilTagDetectionArray,self.april_tag_callback)
        self.moveTurtlebot3_object = MoveTurtlebot3()
        self.twist_object = Twist()



    def stop_sign_callback(self,data):
        if self.stopsign==False:
            print('stop sign')
            x=data
            self.stopsign=False
            for box in x.bounding_boxes:
                print(box.Class, box.id)
                if box.id == 11 and self.stopsign==False:
                    self.frame==False
                    # self.stop_active=True
                    self.twist_object.linear.x=0
                    self.twist_object.angular.z=0
                    # start=time.time()
                    # stopper=time.time()


                    #######check stop sign distance########
                    min_x=box.xmin
                    max_x=box.xmax
                    if (min_x<=320):
                        scan_start= int(305+((min_x/320)*55))
                    else:
                        scan_start=int(0+(min_x/639)*55)

                    if (max_x<=320):
                        scan_end= int(305+((max_x/320)*55))
                    else:
                        scan_end=int(0+(max_x/639)*55)
                    
        
                    print(range)

                    stop_dist=min((range[scan_start]),(range[scan_end]))
                    print(min_x,max_x,scan_start,scan_end,stop_dist)
                    
                    # if stop_dist<=0.7:
                    print('STOPPING')
                    self.stop_active=True
                    self.moveTurtlebot3_object.move_robot(self.twist_object,delay=0)
                    rospy.sleep(10)
                    self.stop_active=False
                    self.stopsign=True
            
            print('moving')
    


    def camera_callback(self, data):
        if self.stop_active==False:
            try:
                # We select bgr8 because its the OpneCV encoding by default
                cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
            except CvBridgeError as e:
                print(e)

            # We get image dimensions and crop the parts of the image we dont need
            height, width, channels = cv_image.shape
            crop_img = cv_image[int((height/2)+200):int((height/2)+220)][1:int(width)]   
            # crop_img = cv_image[340:360][1:640]

            # Convert from RGB to HSV
            # hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
           

            # Define the Yellow Colour in HSV

            """
            To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
            """

            # Threshold the HSV image to get only yellow colors
            s = 50
            lower_yellow = np.array([0, 100, 100]) #[0, 0, 40]
            upper_yellow = np.array([50, 255, 255])# [37, 161, 167]) 60, 78, 98]
            # lower_yellow = np.array([20, 100, 100]) #([0, 0, 43])
            # upper_yellow = np.array([50,255,255])# ([40, 51, 80])
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
            # Calculate centroid of the blob of binary image using ImageMoments
            m = cv2.moments(mask, True)
            # print(m['m00'])

            if m['m00'] >0:
                
                self.frame=True
                print("followingline")

                cx= m['m10']/m['m00']
                cy=m['m01']/m['m00']
                # global error
                self.error=cx-(width/2)
                self.twist_object.angular.z = np.clip((-float(self.error/1000)), -0.1, 0.1)

                temp = np.clip((-float(self.error/1000)), -0.1, 0.1)
                self.twist_object.linear.x = np.clip(0.2*(1-abs(temp)/0.2), 0, 0.08)
                # self.twist_object.linear.x =  0.08
                self.moveTurtlebot3_object.move_robot(self.twist_object)
                print(self.twist_object)

                if self.twist_object.angular.z <0:
                    self.sign=-1
                else:
                    self.sign=1
                
            if m['m00']==0:
                cx, cy = height/2, width/2
                if self.frame:
                    print("followingline")
                    # print('go straight')
                    self.twist_object.linear.x = 0
                    self.twist_object.angular.z = 0.08*self.sign
                    self.moveTurtlebot3_object.move_robot(self.twist_object)
                    print(self.twist_object)
                    
            
            # Draw the centroid in the resultut image
            # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
            cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
            cv2.imshow("hsv_img", hsv)
            cv2.imshow("cv img", cv_image)
            cv2.imshow("MASK", mask)
            cv2.imshow("cropped image",crop_img)
            cv2.waitKey(1)

    def laser_callback(self,msg):
        # print(self.stopsign)
        # print(self.stop_active)
    #     global range
        range_laser=list(msg.ranges)
        # print(range)
        self.range_laser=msg.ranges
        

        # print(self.error)
        if self.error == 0 and self.frame==False and self.stop_active==False:
            print("laser")

            # print(range)


            k=0.3
            omega=0.05

      
            for i in range(len(range_laser)):
                if range_laser[i]==0:
                    range_laser[i]=3
            front=min(min(i for i in range_laser[330:360] if i>0),min(i for i in range_laser[0:30] if i>0))

            left=min(i for i in range_laser[30:90] if i>0)
            right=min(i for i in range_laser[270:330] if i>0)
            rmin=(right)*(-1)
            lmin=(left)
            if front==math.inf:
                front=3
            if rmin == -math.inf:
                rmin=-3
            if lmin == math.inf:
                lmin=3
            side=rmin+lmin
            ef=front-0.3

            if front<1.2:
                self.twist_object.linear.x=k*front*0.5
                self.twist_object.angular.z=omega*side*(1/(front))*8

            else:
                self.twist_object.linear.x=k*front
                self.twist_object.angular.z=omega*side*(1/(front))

            print("laser velocity")
            print(self.twist_object)  
            self.moveTurtlebot3_object.move_robot(self.twist_object) 

    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()

    def april_tag_callback(self,data):
        print('apriltag')

        self.x = data.detections[0].pose.pose.pose.position.x
        self.z = data.detections[0].pose.pose.pose.position.z

        K_linear = 2
        K_angular = 1
        #PID control of the velocities of turtlebot 
        self.twist_object.linear.x =self.x*K_linear # Linear velocity
        self.twist_object.angular.z = self.z*K_angular #ANgular Velocity
        self.moveTurtlebot3_object.move_robot(self.twist_object) 
                    
    





def main():

    rospy.init_node('turtlebot3_node', anonymous=True)
    rate = rospy.Rate(5)
    line_follower_object = LineFollower()    
    ctrl_c = False
    def shutdownhook():
        # Works better than rospy.is_shutdown()
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()

if __name__ == '__main__':

        main()
