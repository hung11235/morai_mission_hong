#!/usr/bin/env python
 
import rospy
import cv2
import numpy as np
import os
from morai_msgs.msg import CtrlCmd
from sensor_msgs.msg import CompressedImage


class Lane_binarize:
    def __init__(self):
        rospy.init_node('lane_binarize', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.is_image = False
        
        self.img_bgr = None

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            os.system('clear')
            if not self.is_image:
                print("[1] can't subscribe '/image_jpeg/compressed' topic... \n    please check your Camera sensor connection")
            else:
                print(f"Caemra sensor was connected !")

            self.is_image = False
            rate.sleep()


    def callback(self, msg):
        self.is_image = True
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


        img_hsv = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2HSV)

        lower_wlane = np.array([0,0,135])
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
        print(img_concat.shape)
        Left = img_concat[170:200,0:50,:]
        Right = img_concat[170:200,590:640,:]
        print("11111111111111111111111111111111111")
    
        cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)

        cmd = CtrlCmd()
        cmd.longlCmdType = 2
        cmd.velocity = 7
        steering_cmd = [ 0.45, -0.45,0]
        cmd_cnts = 100

        if (np.sum(Left) - np.sum(Right)) > 18400000000000000000:
            cmd.steering = steering_cmd[0]
            rospy.loginfo(cmd)
            print("left",np.sum(Left) - np.sum(Right))
            for _ in range(cmd_cnts):
                cmd_pub.publish(cmd)
       

        elif (np.sum(Right) - np.sum(Left)) > 18400000000000000000:
            cmd.steering = steering_cmd[1]
            rospy.loginfo(cmd)
            print("right",np.sum(Right) - np.sum(Left))
            for _ in range(cmd_cnts):
                cmd_pub.publish(cmd)
             
        else:
            cmd.steering = steering_cmd[2]
            rospy.loginfo(cmd)
            print("front")
            for _ in range(cmd_cnts):
                cmd_pub.publish(cmd)
             



if __name__ == '__main__':
    try:
      
        Lane_binarize = Lane_binarize()

        
    except rospy.ROSInterruptException:
        pass