#!/usr/bin/env python

import socket
from struct import *
import struct
from time import sleep


HOST = '127.0.0.1'
# This process should listen to a different port than the RemoteRequest client.
PORT = 65431

class PositionBroadcast:
    def __init__(self):

        print("allo")

    def start(self):


        while True:

            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            s.bind(("", PORT))
            
            data = s.recv(16)
            format = "fffI"
            
            cmd = struct.unpack(format, data)
            
            print(cmd)
            s.close()


if __name__ == "__main__":
    user = PositionBroadcast()

    user.start()
