#!/usr/bin/env python

import rospy
import math 
import numpy as np
from std_msgs.msg import String, ColorRGBA
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

class BalloonDetector:
    def __init__(self):
        self.distance = rospy.get_param('~distance', 0.75)
        self.object_detected_sub = rospy.Subscriber('object_detected', self.scan_callback, String, queue_size=1)
        self.image_detections_sub = rospy.Subscriber('image_detections', Image, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def scan_callback(self, msg):
    
        twist = Twist()

        if msg.data == 0 :
            twist.linear.x = 0
            twist.angular.z = 0
           
        self.cmd_vel_pub.publish(twist);    

def main():
    rospy.init_node('ballon_detector')
    balloondetector = BalloonDetector()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

