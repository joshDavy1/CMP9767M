#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Twist,Point,PoseArray,PoseStamped 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

import numpy as np
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion

PI = 3.1416

class Mover :
    def __init__(self) :
        """ Init node """
        rospy.init_node('mover', anonymous=True)
        self.rate = rospy.Rate(10)
        # Transform listener
        self.listener = tf.TransformListener()
        # Move base client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        # Wait for sprayer server
        rospy.wait_for_service("/thorvald_001/spray")
        self.spray = rospy.ServiceProxy("/thorvald_001/spray",Empty)

        # Velocity and Arm publishers
        self.cmd_vel_pub = rospy.Publisher("/thorvald_001/teleop_joy/cmd_vel",Twist,queue_size=10)
        self.arm_pose_pub = rospy.Publisher("/arm_goal",Point,queue_size=10)
        # Get pose of weeds
        self.weedPoseSubscriber = rospy.Subscriber("/weedsToSpray",PoseArray,self.weedPoseCallback)
        self.weedPoses = PoseArray()

    def weedPoseCallback(self,data) :
        self.weedPoses = data

    def move_to_coord(self,coord) :
        """ Moves to coord and waits till arrived at goal """
        # https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
        goal = MoveBaseGoal()
        goal.target_pose.header.seq = 1
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = coord['x']
        goal.target_pose.pose.position.y = coord['y']
        goal.target_pose.pose.orientation.z = np.sin(coord['theta'])
        goal.target_pose.pose.orientation.w = np.cos(coord['theta'])

        self.client.send_goal(goal)
        # Wait for result.
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def set_arm_pose(self,x,y) :
        """ Sets arm pose """
        point = Point()
        point.x = x
        point.y = y
        # Publish Point
        self.arm_pose_pub.publish(point)

    def get_weed(self,i) :
        """ Gets weed of index i from the weedPoses """
        weed = PoseStamped()
        weed.header.frame_id = self.weedPoses.header.frame_id
        weed.pose = self.weedPoses.poses[i]
        return weed

    def at_weed(self,weed,tolerance = 0.08) :
        """ Calculates if arm is over a weed """
        weed_in_nozzle = self.listener.transformPose("/thorvald_001/arm/nozzle",weed)
        distance = weed_in_nozzle.pose.position.z
        return abs(distance) < tolerance
        
    def at_goal(self,goal,tolerance) :
        """ Calculates if robot is at the row end """
        coord = self.get_current_coords()
        if ((abs(coord['x'] - goal['x']) < tolerance) and (abs(coord['y'] - goal['y']) < tolerance)) :
            return True
        else :
            return False
        
    def get_current_coords(self) :
        """ Gets current robot coord """
        trans,rot = self.listener.lookupTransform("map","thorvald_001/base_link",rospy.Time(0))
        return {'x': trans[0],
                'y': trans[1],
                'theta': euler_from_quaternion(rot)[2]}
    
    def angle_between_two_points(self,x1,y1,x2,y2) :
        """ Calculates the angle between two points """
        theta = np.arctan2(y2-y1,x2-x1)
        return theta

    def move_straight_to(self,goal,velocity) :
        """ Moves one step towards the goal at a certain velocity """
        cmd_vel = Twist()
        cmd_vel.linear.x = velocity
        coords = self.get_current_coords()

        desired_angle =  self.angle_between_two_points(coords['x'],coords['y'],goal['x'],goal['y'])
        actual_angle = coords['theta']
        diff = desired_angle-actual_angle
        
        # If the angle will be shorter to rotate the other way"""
        if (desired_angle > PI/2 and actual_angle < -PI/2) or (desired_angle < -PI/2 and actual_angle > PI/2) :
            diff = -diff/8
        cmd_vel.angular.z = diff/4
        self.cmd_vel_pub.publish(cmd_vel)
        
    def spray_weed(self,weed,arm_depth = -0.38) :
        """ Sprays a weed if in range of the arm """
        weed_in_base = self.listener.transformPose("/thorvald_001/base_link",weed)
        # if weed is not in in the arm workspace
        if weed_in_base.pose.position.y > 0.2 or weed_in_base.pose.position.y < -0.2 :
            return
        self.set_arm_pose(weed_in_base.pose.position.y,arm_depth)
        rospy.sleep(1.5)
        self.spray()   
    
    def weedClose(self,weed1,weed2,tolerance = 0.03) :
        """ Checks how close two weeds are """
        x = abs(weed1.pose.position.x - weed2.pose.position.x)
        y = abs(weed1.pose.position.y - weed2.pose.position.y)
        z = abs(weed1.pose.position.z - weed2.pose.position.z)
        return x+y+z < tolerance
       

    def move_along_row_and_spray(self,goal,velocity = 0.15, goal_tolerance = 0.15) :
        """ Move alongs a row and stops and spray weeds if seen and within range. """
        weedsSprayed = []
        # While not at goal
        while not (self.at_goal(goal,goal_tolerance) or rospy.is_shutdown()) :
            #  For each weed
            for i in range(len(self.weedPoses.poses)) :
                weed = self.get_weed(i)
                # If close to weed
                if self.at_weed(weed) :
                    # and not already sprayerd
                    close_to_already_sprayed_weed = False
                    for i in weedsSprayed :
                        if self.weedClose(weed,i) :
                            close_to_already_sprayed_weed = True
                    # spray
                    if not close_to_already_sprayed_weed :
                        self.spray_weed(weed)
                        weedsSprayed.append(weed)
            # else carry on moving towards goal
            self.move_straight_to(goal,velocity)
            self.rate.sleep()     
            
    def run(self) :
        """ Main procedure. Moves across row1 and row2 spraying weeds accordingly"""
        rospy.sleep(1)
        row_1_start = rospy.get_param("row_1_start")
        row_1_end =   rospy.get_param("row_1_end")
        row_2_start = rospy.get_param("row_2_start")
        row_2_end =   rospy.get_param("row_2_end")

        print("Setting Arm Pose to Neutral")
        self.set_arm_pose(0,0)
        print("Move to start of first row.")
        self.move_to_coord(row_1_start)
        print("Moving along row.")
        self.move_along_row_and_spray(row_1_end)
        print("Setting Arm Pose to Neutral")
        self.set_arm_pose(0,0)
        print("Moving to start of the second row.")
        self.move_to_coord(row_2_start)
        print("Moving along row.")
        self.move_along_row_and_spray(row_2_end)
        print("Setting Arm Pose to Neutral")
        self.set_arm_pose(0,0)
        print("END")

if __name__ == '__main__':
    try:
        move = Mover()
        move.run()
    except rospy.ROSInterruptException:
        pass