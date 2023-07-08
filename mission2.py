#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, os
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import Float32

class auto_drive :
    def __init__(self):
        rospy.init_node('auto_drive', anonymous=True)
        rospy.Subscriber("/dist_forward", Float32, self.dist_callback)
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd',CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg=CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=2
        self.dist = 0

        self.is_short_distance = False

        rate = rospy.Rate(5) # 5hz
        while not rospy.is_shutdown():
            if self.is_short_distance == True:
                self.ctrl_cmd_msg.velocity = 0.0
                os.system('clear')
                print(" distance = ", self.dist, " m")

            else :
                self.ctrl_cmd_msg.velocity = 10.0
                self.ctrl_cmd_msg.steering = 0.0
                os.system('clear')           
                print(" distance = ", self.dist, " m")

            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            self.is_short_distance = False
            rate.sleep()

    def dist_callback(self,msg):
        self.dist = msg.data
        if self.dist < 5:
            self.is_short_distance = True
        else:
            self.is_short_distance = False

if __name__ == '__main__':
    try:
        a_d=auto_drive()
    except rospy.ROSInterruptException:
        pass
