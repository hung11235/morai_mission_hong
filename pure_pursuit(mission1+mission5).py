#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, os
import rospkg
import cv2
import numpy as np
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd
from sensor_msgs.msg import CompressedImage
from tf.transformations import euler_from_quaternion,quaternion_from_euler


class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback)
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd',CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg=CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=2

        self.is_path=False
        self.is_odom=False
        self.is_image=False
        self.img_bgr=None

        self.forward_point=Point()
        self.current_postion=Point()S
        self.is_look_forward_point=False
        self.vehicle_length=3
        self.lfd=3

        rate = rospy.Rate(15) # 15hz
        while not rospy.is_shutdown():
            if self.is_path ==True and self.is_odom==True and self.is_image==True:
                vehicle_position=self.current_postion
                self.is_look_forward_point= False
                translation=[vehicle_position.x, vehicle_position.y]
                t=np.array([
                        [cos(self.vehicle_yaw), -sin(self.vehicle_yaw),translation[0]],
                        [sin(self.vehicle_yaw),cos(self.vehicle_yaw),translation[1]],
                        [0                    ,0                    ,1            ]])
                det_t=np.array([
                       [t[0][0],t[1][0],-(t[0][0]*translation[0]+t[1][0]*translation[1])],
                       [t[0][1],t[1][1],-(t[0][1]*translation[0]+t[1][1]*translation[1])],
                       [0      ,0      ,1                                               ]])
                for num,i in enumerate(self.path.poses) :
                    path_point=i.pose.position
                    global_path_point=[path_point.x,path_point.y,1]
                    local_path_point=det_t.dot(global_path_point)          
                    if local_path_point[0]>0 :
                        dis=sqrt(pow(local_path_point[0],2)+pow(local_path_point[1],2))
                        if dis>= self.lfd :
                            self.forward_point=path_point
                            self.is_look_forward_point=True
                            break
                theta=atan2(local_path_point[1],local_path_point[0])

                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)
                else :
                    print("no found forward point")
                    self.ctrl_cmd_msg.steering=0.0
                    self.ctrl_cmd_msg.velocity=0.0
               
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            else:
                os.system('clear')
                if not self.is_path:
                    print("[1] can't subscribe '/local_path' topic...")
                if not self.is_odom:
                    print("[2] can't subscribe '/odom' topic...")
                if not self.is_image:
                    print("[3] can't subscribe '/image_jpeg/compressed' topic...")
           
            self.is_path = self.is_odom = self.is_image = False
            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def img_callback(self, msg):
        self.is_image=True
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img_hsv = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2HSV)
        lower_wlane = np.array([170,120,70])
        upper_wlane = np.array([180,255,255])
        self.img_bgr = cv2.inRange(img_hsv, lower_wlane, upper_wlane)
        img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)

        img_wlane = cv2.cvtColor(img_wlane, cv2.COLOR_GRAY2BGR)
        img_concat = np.concatenate([img_wlane], axis=1)
        img_concat = img_concat[200:460,0:300,:]
        cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        
        
        

        cv2.imshow("Image window", img_concat)  
        cv2.waitKey(1) 
        
        

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass