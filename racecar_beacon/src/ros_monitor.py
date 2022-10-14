#!/usr/bin/env python

#from turtle import pos
import rospy
import socket
import threading
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from struct import*
import struct

HOST_CAR = '10.0.1.31'
PORT = 65432

def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw


class ROSMonitor:

    def __init__(self):
        self.lidar = rospy.Subscriber("/racecar/scan", LaserScan, self.getDistance)
        self.odometrie = rospy.Subscriber("/racecar/odometry/filtered", Odometry, self.getOdometry)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_cb)

        HOST = '10.0.1.31'
        #HOST = '127.0.0.1'
        PORT_remote_client = 65432
        self.s_remote_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s_remote_client.settimeout(1800)
        self.s_remote_client.bind((HOST, PORT_remote_client))
        self.s_remote_client.listen(1)

        PORT_vehicle_tracker = 65431
        self.s_vehicle_tracker = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) # Socket UDP
        self.s_vehicle_tracker.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.s_vehicle_tracker.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.s_vehicle_tracker.bind((HOST, PORT_vehicle_tracker))

        # Current robot state:
        self.id = 0x0A00011F
        # *************************
        self.pos = (0,0,0)
        self.obstacle = 0
        self.format = ">fffxxxx" # 3 float32 et 1 uint32
        self.format2 = ">Ixxxxxxxxxxxx" 

        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)

        # Thread for RemoteRequest handling:
        #self.rr_thread = threading.Thread(target=self.rr_loop)

        print("ROSMonitor started.")
  

    def getInfoClient(self):

        #s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #s.bind((HOST_CAR, PORT))
        #self.s_remote_client.listen(1)

        while True:
            (conn, addr) = self.s_remote_client.accept()
            data = conn.recv(16)
            data = (unpack(">4s", data)[0]).decode('utf-8')
      
            if data == "RPOS":
                self.info = struct.pack(self.format, self.pos[0], self.pos[1], self.pos[2])
            elif data == "OBSF":
                self.info = struct.pack(self.format2, self.obstacle)
            elif data == "RBID":
                self.info = struct.pack(self.format2, self.id)
            else:
                self.info = struct.pack(self.format2, 1)

            if not data: 
                break
            conn.send(self.info)
        conn.close()
        self.s_remote_client.close()
          
    
    def getOdometry(self, message):
        yaw = quaternion_to_yaw(message.pose.pose.orientation)
        self.pos = (message.pose.pose.position.x, message.pose.pose.position.y, yaw)
        #print(message)

    def getDistance(self, distance):

        for i in distance.ranges:
            #print(i, "\n")
            if i > distance.range_min and i < distance.range_max:
                self.obstacle = 1
                #print("1")
            else:
                self.obstacle = 0       

        #print(distance)

    def timer_cb(self, msg):

        format1 = ">fffI"
        PORT_vehicle_tracker = 65431
        #print(self.pos[0])
        enc = pack(format1, self.pos[0], self.pos[1], self.pos[2], self.id)
        #print(self.pos[0], " ", self.pos[1], " ", self.pos[2], " ", self.id)
        #print(self.id)
        self.s_vehicle_tracker.sendto(enc, ('10.0.1.255', PORT_vehicle_tracker))


if __name__=="__main__":
    rospy.init_node("ros_monitor")
    node = ROSMonitor()
    node.getInfoClient()
    rospy.spin()