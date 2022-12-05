#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import message_filters
import tf
import tf2_ros
from racecar_behaviors.cfg import BlobDetectorConfig
from dynamic_reconfigure.server import Server
from std_msgs.msg import String, ColorRGBA
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Quaternion
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from libbehaviors import *
import math
import time




class VelBall:
    def __init__(self):

        self.goal = rospy.Subscriber('/goal', Float64MultiArray, self.goal_callback, queue_size=1)
        self.object = rospy.Subscriber('object_detected', String, queue_size=1)
        self.vel_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.goal_target = True


    def goal_callback(self, goal_ball):
        if(self.goal_target):
            if(0 > goal_ball.data[1] > 0):
                rospy.loginfo('allo')
                self.twist = Twist()
                self.twist.linear.x = 1
                self.twist.angular.z = goal_ball.data[1]
                self.vel_cmd.publish(self.twist)
                if(0.75 < goal_ball.data[0] < 1.8):
                    self.goal_target = False

            else:
                self.twist = Twist()
                self.twist.linear.x = 1
                self.twist.angular.z = 0
                self.vel_cmd.publish(self.twist)
                if(0.75 < goal_ball.data[0] < 1.8):
                    self.goal_target = False


        else:
            self.twist = Twist()
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.vel_cmd.publish(self.twist)
            time.sleep(5)

            

def main():
    rospy.init_node('vel_ball')
    velBall = VelBall()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

