#!/usr/bin/env python
 
from ast import IsNot
from os import kill
import socket
import time 
import sys
import struct
from struct import*
 
# On donne adresse du pi qui va prendre la position du RemoteClient
HOST = '127.0.0.1'
# This process should listen to a different port than the PositionBroadcast client.
# Ca ca veut dire quil faut quon soit sur un meme pi, mais que le port va changer pour les deux
PORT = 65432
 
class userScreen:
    def __init__(self):
    # variables of interest
        self.ipAdress = 0
        self.info = ''
        self.exit = ""
        self.format = "fffI"

    def userInfo(self):
        while True:
            self.exit = input("Do you want continue(0) or exit(1): ")
            if self.exit == str(1):
                sys.exit()

            self.ipAdress = input("IP Adress you search for (XX.XX.XX.XX): ")
            time.sleep(0.5)

            self.info = input("What info do you you want to acces (RPOS, OBSF, RBID): ")
            # print(self.info)
            if self.info == "RPOS":
                self.info = pack(">4s", b"RPOS")
                break
            elif self.info == "OBSF":
                self.info = pack(">4s", b"RPOS")
                break
            elif self.info == "RBID":
                self.info = pack(">4s", b"RPOS")
                break
            else :  
                print("This is not an option")
                continue

        # self.info = pack(">4s", bytearray(str(self.info)))

    def start_stop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        s.send(self.info)
        self.rosInfo = s.recv(128)
        print(self.rosInfo)
        # self.msg_ROS = struct.unpack(self.format, self.rosInfo)
        # print(self.msg_ROS)
        print(struct.unpack(self.format, self.rosInfo))
        s.close()

    def getIP(self):
        print(self.ipAdress)
    
    def getInfo(self):
        infoList = self.info
        print(infoList) 
 
if __name__ == "__main__":
    user = userScreen()

    while True:
        user.userInfo()
        user.start_stop()
