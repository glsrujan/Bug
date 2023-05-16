#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
from scipy.spatial.transform import Rotation as R



class Mybug(object):
    def __init__(self):
        self.bpgt = rospy.Subscriber("/base_pose_ground_truth",Odometry,self.callback_bpgt)
        self.bs = rospy.Subscriber("/base_scan",LaserScan,self.callback_bs)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.odo = Odometry()
        self.ls = LaserScan()
        self.G_bug = np.zeros((3,1))
        self.ori = np.zeros((3,3))
        self.bug_msg = Twist()
        self.heading = 0
        self.bearing =0
        self.goal = [0,0,0,1]
        
        pass
    def callback_tf(self, data):
        self.tf_data = data
        print(self.tf_data)
        pass
    def callback_bpgt(self,data):
        self.odo = data
        # if(self.ls.ranges!=None):
        #     self.bug_controller()   

    def callback_bs(self,data):
        self.ls = data

    def bug_controller(self):

        global Orientaion_Flag

        self.G_bug[0,0] = self.odo.pose.pose.position.x
        self.G_bug[1,0] = self.odo.pose.pose.position.y
        self.G_bug[2,0] = self.odo.pose.pose.position.z

        range = self.ls.ranges

        quat = [self.odo.pose.pose.orientation.x,self.odo.pose.pose.orientation.y,self.odo.pose.pose.orientation.z,self.odo.pose.pose.orientation.w]
        G_rot_R = R.as_dcm(R.from_quat(quat))
        G_T_R = np.zeros((4,4))
        G_T_R[0] = G_rot_R[0,0], G_rot_R[0,1], G_rot_R[0,2] ,self.G_bug[0,0]
        G_T_R[1] = G_rot_R[1,0], G_rot_R[1,1], G_rot_R[1,2] ,self.G_bug[1,0]
        G_T_R[2] = G_rot_R[2,0], G_rot_R[2,1], G_rot_R[2,2] ,self.G_bug[2,0]
        G_T_R[3] = 0, 0, 0, 1

        R_T_G = np.linalg.inv(G_T_R)
        G_point = np.zeros((4,1))
        G_point[0],G_point[1],G_point[2],G_point[3], = self.goal[0],self.goal[1],0,1
        print("Robot Position\r\n ",self.G_bug) 
        print("Goal Position\r\n",self.goal)
        R_point = R_T_G@G_point
        bearing = np.degrees(np.arctan2(R_point[1,0],R_point[0,0]))
        vel = 0

        if Orientaion_Flag:
            if np.abs(bearing)<0.5:
                Orientaion_Flag = False
            else:
                self.heading = bearing
                if self.heading >10:
                    self.bug_msg.angular.z = 0.5
                elif self.heading <-10:
                    self.bug_msg.angular.z = -0.5
                else:
                    self.bug_msg.angular.z = np.radians(self.heading)
                print("Robot Orienting")
            return

        if np.abs(bearing)<135:
            if range[540]>1.5:
                if ((range[540+int(bearing/0.25)]>1) and (range[540+218]>1) and (range[540-218]>1)):
                    self.heading = bearing
                    vel = np.abs(1*np.cos(np.radians(self.heading)))
                elif ((range[540+218]>0.6) and (range[540-218]>0.6)):
                    self.heading = 0
                    vel = 0.5
                else:
                    if range[180]>range[900]:
                        self.heading = -5
                        
                    else:
                        self.heading =5
                    vel = 0
            else:
                if range[180]>range[900]:
                    self.heading = -5
                else:
                    self.heading =5
                    pass
                vel = 0
        else:
            if range[540]>1.5:
                if ((range[540+218]>1) and (range[540-218]>1)):
                    self.heading = bearing
                    vel = np.abs(1*np.cos(np.radians(self.heading)))
                elif ((range[540+218]>0.6) and (range[540-218]>0.6)):
                    self.heading = 0
                    vel = 0.5
                else:
                    if range[180]>range[900]:
                        self.heading = -5
                        
                    else:
                        self.heading =5
                        vel = 0.05
                        pass
                    vel = 0
            else:
                if range[180]>range[900]:
                    self.heading = -5
                else:
                    self.heading =5
                    pass
                vel = 0

        if self.heading >10:
            self.bug_msg.angular.z = 0.5
        elif self.heading <-10:
            self.bug_msg.angular.z = -0.5
        else:
            self.bug_msg.angular.z = np.radians(self.heading)
        self.bug_msg.linear.x = vel

        if np.linalg.norm(R_point)<1.05:
            self.bug_msg.angular.z = 0
            self.bug_msg.linear.x = 0
            print("Congrats!! - Robot has reached the Destination!!")
            print("Shoutout to Jordan and Ben!!")
            quit()

        print(bearing, self.heading, vel ,range[540],range[540+218],range[540-218])




if __name__ == "__main__":
    print("bug0_ver_2.py Started")
    rospy.init_node('bug0_subscriber',anonymous=False)
    try:
        bug0 = Mybug()
        rate = rospy.Rate(10)
        x_inp = input("Provide x co-ordinater of the Destination: ")
        y_inp = input("Provide y co-ordinater of the Destination: ")
        bug0.goal = [float(x_inp),float(y_inp),0,1]
        Orientaion_Flag = True
        while not rospy.is_shutdown():
            print("------------")
            bug0.bug_controller() 
            bug0.pub.publish(bug0.bug_msg)
            rate.sleep()
            pass
    except rospy.ROSInterruptException:
        pass
