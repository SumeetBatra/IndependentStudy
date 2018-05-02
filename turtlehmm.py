#!/usr/bin/env python
from CCHTN import *
import sys
import networkx as nx
import roslib
import rospy
import numpy as np
import scipy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

class TurtleHMM(object):
    def __init__(self):
        rospy.init_node('hmm', anonymous=False)

        self.turtle_sub = rospy.Subscriber("/turtle1/pose", Pose, self.callback)

        self.pos = Pose()

    def callback(self, data):
        self.pos = data

    def classify(self, goal_pos):
        angular_error = abs((atan2(goal_pos.y - self.pos.y, goal_pos.x - self.pos.x) - self.pos.theta))
        dist_error = self.dist(goal_pose)
        score = 1 / (2 * angular_error + dist_error)
        #sigmoid squash b/w 0 and 1
        p = scipy.special.expit(score)
        return p

    def dist(self, goal_pose):
        #distance = np.sqrt( (goal_pose.x - self.pose.x)**2 + (goal_pose.y - self.pose.y)**2 )
        distance = sqrt( pow(goal_pose.x - self.pose.x, 2) + pow(goal_pose.y - self.pose.y, 2) )
        return distance






def main():
    parent_node = 'task_graph'
    goal_ids = ['goal_1','goal_2','goal_3','goal_4']
    goal_poses = [{'pos':(9,9)}, {'pos':(4,7)}, {'pos':(1,3)}, {'pos':(7,2)}]

    TURTLE_CCHTN = CCHTN(parent_node)
    print(TURTLE_CCHTN.get_root_node()) #prints {'uid': '0', 'parent': None, 'skill': '0', 'completed': False, 'skillType': 'skill'}
    TURTLE_CCHTN.add_chain(goal_ids, TURTLE_CCHTN.get_root_node(), data=goal_poses)

if __name__ == "__main__":
    main()
