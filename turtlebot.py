#!/usr/bin/env python
import roslib
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

class TurtleBot(object):

    def __init__(self):
        rospy.init_node('turtlebot', anonymous=True)

        self.k_linear = 2
        self.k_angular = 3.0
        self.dist_thresh = 0.5

        self.pose = Pose()
        self.rate = rospy.Rate(10)

        self.turtle_sub = rospy.Subscriber("/turtle1/pose", Pose, self.callback)
        self.vel_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

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
            print(self.dist(goal_pose))
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

def main(args):
    turtle1 = TurtleBot()

    #random goal x,y positions
    goal1 = turtle1.newGoal(7, 7)
    goal2 = turtle1.newGoal(3, 7)
    goal3 = turtle1.newGoal(2, 3)
    goal4 = turtle1.newGoal(7, 2)

    turtle1.moveToGoal(goal1)
    turtle1.moveToGoal(goal2)
    turtle1.moveToGoal(goal3)
    turtle1.moveToGoal(goal4)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__=='__main__':
    main(sys.argv)
