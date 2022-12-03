#!/usr/bin/env python

import matplotlib.pyplot as plt
import math
import sys
import pathlib
import rospy
import numpy as np
import cubic_spline_planner
import cv2

from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

class PathFollowing:
    def __init__(self):
        self.goal_sub = rospy.Subscriber('goal_status', GoalStatusArray, self.call_back, queue_size=1)
        self.cmd_goal = rospy.Publisher('goal_cmd', PoseStamped, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.status = 0
        self.state = "START"
        self.pos = "END_POINT"

    def set_goal(self, posx, posy, orx, ory, orz, w):
        msg_goal = PoseStamped()
        now = rospy.get_rostime()

        msg_goal.header.stamp.secs = now.secs
        msg_goal.header.stamp.nsecs = now.nsecs
        msg_goal.header.frame_id = "racecar/map"

        msg_goal.pose.position.x = posx
        msg_goal.pose.position.y = posy
        msg_goal.pose.position.z = 0.0

        msg_goal.pose.orientation.x = orx
        msg_goal.pose.orientation.y = ory
        msg_goal.pose.orientation.z = orz
        msg_goal.pose.orientation.w = w

        # w = 1.0 => theta = 0
        # z = 1.0 => theta = 180

        self.cmd_goal.publish(msg_goal)
        print("GOAL SENT")

    def call_back(self, msg):
        if msg.status_list:
            index = len(msg.status_list)-1

            if msg.status_list[index].status == 1:
                self.status = 1
            if msg.status_list[index].status == 3:
                self.status = 3

    def timer_callback(self, event):

        if self.state == "START":
            # print("FIRST GOAL")
            self.set_goal(13.5, 2.1, 0.0, 0.0, 1.0, 0.0)
            if self.status == 1 or self.status == 3:
                self.state = "ON_ROUTE"
                self.previous = "START"

        elif self.state == "ON_ROUTE":
            if self.status == 3:
                if self.previous == "START":
                    self.pos == "END_POINT"
                if self.previous == "AT_GOAL":
                    self.pos = "START_POINT"
                self.state = "AT_GOAL"
                self.previous = "ON_ROUTE"

        elif self.state == "AT_GOAL":
            
            print("previous = ", self.previous, "pos = ", self.pos)
            if self.previous == "ON_ROUTE" and self.pos == "END_POINT":
                print("NEW GOAL")
                self.set_goal(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

                if self.status == 1:
                    self.state = "ON_ROUTE"
                    self.previous = "AT_GOAL"
            else:
                rospy.loginfo("RETURN TO ORIGIN")


                

        # if self.status != 3 and self.status != 1:
        #     self.set_goal(13.5, 2.1, 0.0, 0.0, 1.0, 0.0)

        # elif self.status == 3:
        #     if self.status != 3 and self.status != 1: 
        #         self.set_goal(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        #         if self.status == 1:
        #             self.status = 0

        # if self.status == 3:
        #     self.set_goal(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def main():
    print(__file__ + " start!!")

    rospy.init_node('path_planning_py')

    path = PathFollowing()

    rospy.spin()


if __name__ == '__main__':
    main()
    
