#!/usr/bin/env python3
"""
 * ROSCPP demo publisher.
 * Sends twist messages for controlling a robot base.
 * 
 * Junaed Sattar <junaed@umn.edu>
 * October 2018.
 *
 * Adapted to Python by Alex Overman <ayres036@umn.edu>
 * April 2021.
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

"""
 *  
 * This tutorial demonstrates simple sending of messages over the ROS system
 * and controlling a robot.
 *   
"""

def demo():
    # Name your node
    rospy.init_node("stage_mover", anonymous=True)

    # Publisher object that decides what kind of topic to publish and how fast.
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # We will be sending commands of type "twist"
    com = Twist();

    print("Type a command and then press enter.")
    print("Use 'f' to move forward, 'l' to turn left, ")
    print("'r' to turn right, '.' to exit.\n")

    # The main loop will run at a rate of 10Hz, i.e., 10 times per second.
    rate = rospy.Rate(10)

    # Standard way to run ros code. Will quit if ROS is not OK, that is, the master is dead.
    while not rospy.is_shutdown():
        inp = input("command: ")
        if (inp[0] != 'f') and (inp[0] != 'l') and (inp[0] != 'r') and (inp[0] != "."):
            print("unknown command:", inp)
            continue
        # move forward
        elif inp[0] == 'f': 
            com.linear.x = 0.25
            com.angular.z = 0
        # turn left (yaw) and drive forward at the same time    
        elif inp[0] == 'l':
            com.angular.z = 0.75
            com.linear.x = 0.25
        # turn right (yaw) and drive forward at the same time
        elif inp[0] == 'r':
            com.angular.z = -0.75
            com.linear.x = 0.25
        # quit
        elif inp[0] == '.':
            break

        # Here's where we publish the actual message.
        cmd_vel_pub.publish(com)
        
        # Sleep for as long as needed to achieve the loop rate.
        rate.sleep()

if __name__ == "__main__":
    demo()
