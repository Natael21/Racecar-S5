#!/usr/bin/env python

import rospy
import socket
import threading

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from struct import*
import struct

HOST = '127.0.0.1'
PORT = 65432

class ROSMonitor:
    def __init__(self):
        self.lidar = rospy.Subscriber("/scan", LaserScan, self.getDistance)
        self.odometrie = rospy.Subscriber("/odometry/filtered", Odometry, self.getOdometry)

        # Current robot state:
        # put the 10.0.1.31 for the id !
        self.id = 3 
        # *************************
        self.pos = (0,0,0)
        self.obstacle = 0
        self.format = "fffxxxx" # 3 float32 et 1 uint32
        self.format2 = "Ixxxxxxxxxxxx" 

        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)

        # Thread for RemoteRequest handling:
        self.rr_thread = threading.Thread(target=self.rr_loop)

        print("ROSMonitor started.")

    def rr_loop(self):
        # Init your socket here :
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Socket UDP
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) # Mode broadcast
        
        while True:
            #enc = pack(format, self.pos[0], self.pos[1], self.pos[2], self.id)
            #s.send(enc)
            print("allo")
            # Modifiy to send the data at a rate of 1 Hz
            #pass

    def getInfoClient(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((HOST, PORT))
        s.listen(1)
        while True:
            (conn, addr) = s.accept()
            data = conn.recv(16)

            data = (unpack(">4s", data)[0]).decode('utf-8')
        
            if data == "RPOS":
                self.info = struct.pack(self.format, self.pos[0], self.pos[1], self.pos[2])
            elif data == "OBSF":
                self.info = struct.pack(self.format2, self.obstacle)
            elif data == "RBID":
                self.info = struct.pack(self.format2, self.id)

            if not data: 
                break
            conn.send(self.info)
        conn.close()
        s.close()
            
    
    def getOdometry(self, message):
        #self.pos = (message.pos.pos.x, message.pos.pos.y, message.pos.pos.z)
        print(message)
    
    def getDistance(self, distance):
        #self.obstacle = distance.ranges
        print(distance)
            
if __name__=="__main__":
    rospy.init_node("ros_monitor")

    node = ROSMonitor()
    node.getInfoClient()

    rospy.spin()



