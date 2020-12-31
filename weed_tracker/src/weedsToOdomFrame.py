#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose,PoseArray,PoseStamped
import tf

class weedsToOdomFrame :
    def __init__(self) :
        rospy.init_node('weedToOdomFrame')
        self.weedArraySub = rospy.Subscriber("/weeds",PoseArray,self.callback)
        self.pub = rospy.Publisher('/weedsToSpray', PoseArray, queue_size=10)
        self.tran = tf.TransformListener()
        self.weedArray = []

    def callback(self,data) :
        self.weedArray = data

    def calculateSquaredDifference(self,pose1,pose2) :
        return (pose1.x - pose2.x)**2 + (pose1.y - pose2.y)**2 + (pose1.z - pose2.z)**2

    def run(self) :
        rate = rospy.Rate(1)
        rate.sleep()
        weedsToSpray = PoseArray()
        weedsToSpray.header.frame_id = "/thorvald_001/odom"
        while not rospy.is_shutdown():
            if self.weedArray :
                for weed in self.weedArray.poses:
                    weedPose= PoseStamped()
                    weedPose.header.frame_id = self.weedArray.header.frame_id
                    weedPose.header.stamp = rospy.Time()
                    weedPose.pose.position = weed.position

                    weed_in_odom = self.tran.transformPose("/thorvald_001/odom",weedPose)
                    point_is_distant = True
                    
                    for i in weedsToSpray.poses :
                        if self.calculateSquaredDifference(weed_in_odom.pose.position,i.position) < 0.04 :
                            point_is_distant = False

                    if point_is_distant :
                        weedsToSpray.poses.append(weed_in_odom.pose)

            self.pub.publish(weedsToSpray)
        rate.sleep()

if __name__ == '__main__':
    try:
        we = weedsToOdomFrame()
        we.run()
    except rospy.ROSInterruptException:
        pass