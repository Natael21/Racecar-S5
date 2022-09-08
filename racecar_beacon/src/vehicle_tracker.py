#!/usr/bin/env python

import socket
from struct import *


HOST = '127.0.0.255'
# This process should listen to a different port than the RemoteRequest client.
PORT = 65431

class PositionBroadcast:
    def __init__(self):

        print("allo")

    def start(self):
        while True:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            s.connect((HOST, PORT))
            #s.listen(1)
            self.data = s.recvfrom(16)
            format = "fffI"
            cmd = unpack(format, self.data)
            print(cmd)
            #s.send(bytearray(self.ipAdress))
            #self.msg_ROS = s.recv(1024)
            #print(self.msg_ROS)
            s.close()


if __name__ == "__main__":
    user = PositionBroadcast()

    user.start()
