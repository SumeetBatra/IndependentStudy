#!/usr/bin/env python
from CCHTN import *
import sys
import networkx as nx
import roslib
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt


class TurtleBot(object):
    def __init__(self):
        rospy.init_node('turtlebot', anonymous=False)

        self.k_linear = 1.0
        self.k_angular = 10.0
        self.dist_thresh = 0.2

        self.pose = Pose()
        self.rate = rospy.Rate(10)

        self.turtle_sub = rospy.Subscriber("/turtle1/pose", Pose, self.callback)
        self.vel_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)

    def callback(self, data):
        self.pose = data


    def moveToGoal(self, goal_pose):
        while self.dist(goal_pose) >= self.dist_thresh:
            vel_msg = Twist()
            #linear velocity
            vel_msg.linear.x = self.k_linear * self.dist(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            #angular velocity
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.k_angular * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)
            self.vel_pub.publish(vel_msg)
            print(self.pose)
            self.rate.sleep()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)


    def dist(self, goal_pose):
        #distance = np.sqrt( (goal_pose.x - self.pose.x)**2 + (goal_pose.y - self.pose.y)**2 )
        distance = sqrt( pow(goal_pose.x - self.pose.x, 2) + pow(goal_pose.y - self.pose.y, 2) )
        return distance

    def newGoal(self, x_pose, y_pose):
        goal = Pose()
        goal.x = x_pose
        goal.y = y_pose
        return goal


def main():
    parent_node = 'task_graph'
    goal_ids = ['goal_1','goal_2','goal_3','goal_4']
    goal_poses = [{'pos':(9,9)}, {'pos':(4,7)}, {'pos':(1,3)}, {'pos':(7,2)}]

    TURTLE_CCHTN = CCHTN(parent_node)
    print(TURTLE_CCHTN.get_root_node()) #prints {'uid': '0', 'parent': None, 'skill': '0', 'completed': False, 'skillType': 'skill'}
    TURTLE_CCHTN.add_chain(goal_ids, TURTLE_CCHTN.get_root_node(), data=goal_poses)


if __name__ == "__main__":
    main()
