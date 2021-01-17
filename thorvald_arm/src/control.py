#!/usr/bin/env python

import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
import rospy

PI = 3.14159

class kinematics :
    def __init__(self) :
        """ Init node """
        self.joint1_pub = rospy.Publisher('/thorvald_001/joint1_position_controller/command', Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher('/thorvald_001/joint2_position_controller/command', Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher('/thorvald_001/joint3_position_controller/command', Float64, queue_size=10)
        self.goal_sub = rospy.Subscriber('/arm_goal', Point, self.goal_callback)

        rospy.init_node('arm_controller', anonymous=True)
        self.rate = rospy.Rate(10)
        self.rate.sleep()

        # Move to inital position
        theta_1,theta_2,theta_3 = self.inverseKinematics(0,0)
        self.publish_angles(theta_1,theta_2,theta_3)
        
    def goal_callback(self,data) :
        """ Gets goal point and sets angles """
        x = data.x
        y = data.y
        theta_1,theta_2,theta_3 = self.inverseKinematics(x,y)
        self.publish_angles(theta_1,theta_2,theta_3)
        self.rate.sleep()

    def publish_angles(self,theta_1,theta_2,theta_3) :
        self.joint1_pub.publish(theta_1)
        self.joint2_pub.publish(theta_2)
        self.joint3_pub.publish(theta_3)

    def inverseKinematics(self,x,y,L1 = 0.32,L2= 0.32,L3 = 0.08):
        """ Calculate inverse kinematic solution """
        # https://www.daslhub.org/unlv/wiki/doku.php?id=2_link_kinematics
        y = -y
        cos_theta_2 = (x**2 + y**2 - L1**2 - L2**2) / (2*L1*L2)
        theta_2 = np.arccos(cos_theta_2)
        theta_1 = np.arctan2(y,x)  - np.arctan2(L2*np.sin(theta_2), (L1 + L2*np.cos(theta_2)) ) 
        # Theta 3 keeps the nozzle in the same pose as the base frame
        theta_1 = -theta_1
        theta_3 = -theta_1 + theta_2
        return theta_1, theta_2, theta_3

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        k = kinematics()
        k.run()
    except rospy.ROSInterruptException: 
        pass