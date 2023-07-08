#!/usr/bin/env python
# -*- coding: utf-8 -*-
from re import I
import rospy
import rospkg
from math import sqrt
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry

class pathMaker :
    
    def __init__(self, pkg_name, path_name):
        rospy.init_node('path_maker', anonymous=True)
        
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        # 초기화
        self.prev_x = 0
        self.prev_y = 0
        self.prev_z = 0
        self.prev_orientation_x = 0
        self.prev_orientation_y = 0
        self.prev_orientation_z = 0
        self.prev_orientation_w = 0
        self.is_status=False
        # 패키지 경로 로드 & 파일 쓰기 모드
        rospack = rospkg.RosPack()
        pkg_path=rospack.get_path(pkg_name)
        full_path=pkg_path + '/'+path_name+'.txt'
        self.f=open(full_path, 'w')

        while not rospy.is_shutdown():
            if self.is_status==True :
                # turtle 위치 기록
                self.path_make()
        self.f.close()

    def path_make(self):
        x=self.status_msg.position.x
        y=self.status_msg.position.y
        z=self.status_msg.position.z
        orientation_x=self.status_msg.orientation.x
        orientation_y=self.status_msg.orientation.y
        orientation_z=self.status_msg.orientation.z
        orientation_w=self.status_msg.orientation.w
        distance=sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))
        # 이전 waypoint와의 거리가 0.3 이상이어야 기록
        if distance >0.3:
            data='{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\n'.format(x,y,z,orientation_x,orientation_y,
            orientation_z,orientation_w)
            self.f.write(data)
            self.prev_x= x
            self.prev_y= y
            self.prev_z = z
            self.prev_orientation_x = orientation_x
            self.prev_orientation_y = orientation_y
            self.prev_orientation_z = orientation_z
            self.prev_orientation_w = orientation_w
            print("write : ", x,y)
    
    def odom_callback(self,msg):
        self.is_status=True
        self.status_msg=msg.pose.pose

if __name__ == '__main__' :
    try:
        p_m=pathMaker("beginner_tutorials", "mission1_path")
    except rospy.ROSInternalException:
        pass
            
