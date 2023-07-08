!/usr/bin/env python
 
import rospy
import cv2
import numpy as np
import os
from morai_msgs.msg import CtrlCmd
from sensor_msgs.msg import CompressedImage
from enum import Enum
from morai_msgs.msg import CtrlCmd, CollisionData, EgoVehicleStatus, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv

class Gear(Enum):
    P = 1
    R = 2
    N = 3
    D = 4

class Lane_binarize:
    def __init__(self):
        rospy.init_node('lane_binarize', anonymous=True)

        self.cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.is_image = False
        
       

        self.img_bgr = None
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)
        self.is_ego = False

        rospy.wait_for_service('/Service_MoraiEventCmd', timeout= 5)

        self.event_cmd_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)

        self.rate = rospy.Rate(10)
        self.ego_status = EgoVehicleStatus()
        

        self.send_gear_cmd(Gear.D.value)
        while not rospy.is_shutdown():
            os.system('clear')
            if not self.is_image:
                print("[1] can't subscribe '/image_jpeg/compressed' topic... \n    please check your Camera sensor connection")
            else:
                print(f"Caemra sensor was connected !")

            if not self.is_ego:
                print("[2] can't subscribe '/Ego_topic' topic... \n    please check connection")

           
            ####rate.sleep()
        

    def ego_callback(self, data):
        self.is_ego = True
        self.ego_status = data

    # 기어 변경 이벤트 메시지 세팅 함수
    def send_gear_cmd(self, gear_mode):
        
        # 기어 변경이 제대로 되기 위해서는 차량 속도가 약 0 이어야함
        while( abs(self.ego_status.velocity.x) > 0.1):
            self.send_ctrl_cmd(0,0)
            self.rate.sleep()
        
        gear_cmd = EventInfo()
        gear_cmd.option = 3
        gear_cmd.ctrl_mode = 3
        gear_cmd.gear = gear_mode
        gear_cmd_resp = self.event_cmd_srv(gear_cmd)
        rospy.loginfo(gear_cmd)

    # ctrl_cmd 메시지 세팅 함수
    def send_ctrl_cmd(self, steering ,velocity):
        cmd = CtrlCmd()
        if(velocity > 0):
            cmd.longlCmdType = 2
            cmd.velocity = velocity
            cmd.steering = steering
        else:
            cmd.longlCmdType = 1
            cmd.brake = 1
            cmd.steering = 0
        self.cmd_pub.publish(cmd)


    def callback(self, msg):
        self.is_image = True
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


        img_hsv = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2HSV)

        lower_wlane = np.array([0,0,125])
        upper_wlane = np.array([50,80,255])

        img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)

        img_wlane = cv2.cvtColor(img_wlane, cv2.COLOR_GRAY2BGR)

        #img_concat = np.concatenate([self.img_bgr, img_hsv, img_wlane], axis=1)
        img_concat = np.concatenate([img_wlane], axis=1)
        img_concat = img_concat[260:460,:,:]
        

        cv2.imshow("Image window", img_concat)  
        cv2.waitKey(1) 
        #print(img_concat.shape)
        #nonzero = img_concat[np.nonzero(img_concat)]
        #print(nonzero)
        Left = img_concat[170:200,0:50,:]
        Right = img_concat[170:200,590:640,:]
        Front = img_concat[170:200,:,:]
        
    
        cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)

        cmd = CtrlCmd()
        cmd.longlCmdType = 2
        cmd.velocity = 6
        steering_cmd = [ 0.47, -0.47,0,1,-1]
        cmd_cnts = 50

        if np.sum(Front) > 1500000:
            cmd.velocity = 0
            self.send_gear_cmd(Gear.R.value)
            for _ in range(cmd_cnts):
                cmd_pub.publish(cmd)
            cmd.longlCmdType = 2
            print(np.sum(Front),"Xxxxxxxxxxxxxxxxxxxxxxx")
                
            if np.sum(Left) > np.sum(Right):
                cmd.velocity = 10
                cmd.steering = steering_cmd[3]
                for _ in range(2000):
                    cmd_pub.publish(cmd)
                self.send_gear_cmd(Gear.D.value)
            else:
                cmd.velocity = 10
                cmd.steering = steering_cmd[4]
                for _ in range(2000):
                    cmd_pub.publish(cmd)
                self.send_gear_cmd(Gear.D.value)
            
            
            '''print(np.sum(Front),"Xxxxxxxxxxxxxxxxxxxxxxx")
            
            cmd.steering = steering_cmd[2]
            rospy.loginfo(cmd)
            print("front")
            for _ in range(cmd_cnts):
                cmd_pub.publish(cmd)'''
        
        elif (np.sum(Left) - np.sum(Right))/10000000 > 1:
           
            cmd.steering = steering_cmd[0]
            rospy.loginfo(cmd)
            print("left",(np.sum(Left) - np.sum(Right))/10000000)
            for _ in range(cmd_cnts):
                cmd_pub.publish(cmd)
       

        elif (np.sum(Right) - np.sum(Left))/10000000 > 1:
        
            cmd.steering = steering_cmd[1]
            rospy.loginfo(cmd)
            print("right",(np.sum(Right) - np.sum(Left))/10000000)
            for _ in range(cmd_cnts):
                cmd_pub.publish(cmd)
             
        else:
            self.send_gear_cmd(Gear.D.value)
            cmd.steering = steering_cmd[2]
            rospy.loginfo(cmd)
            
            for _ in range(cmd_cnts):
                cmd_pub.publish(cmd)

        """ if np.sum(Front) > 10000:
                send_gear_cmd(Gear.R.value)
               
                print("Xxxxxxxxxxxxxxxxxxxxxxx")
                
                cmd.steering = steering_cmd[2]
                rospy.loginfo(cmd)
                print("front")
                for _ in range(cmd_cnts):
                    cmd_pub.publish(cmd)
             """
    



if __name__ == '__main__':
    try:
      
        Lane_binarize = Lane_binarize()

        
    except rospy.ROSInterruptException:
        pass