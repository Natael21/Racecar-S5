#!/usr/bin/env python
 
from ast import IsNot
from os import kill
import os
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
        self.ipAdress = 'sfsdgdsg'
        self.info = ''
        self.exit = ""
        self.format = "fffxxxx"

    def userInfo(self):
            self.exit = input("Do you want continue(0) or exit(1): ")
            if self.exit == str(1):
                sys.exit()

            self.ipAdress = input("IP Adress you search for (XX.XX.XX.XX): ")

            while True:
                self.info = input("What info do you you want to acces (RPOS, OBSF, RBID): ")

                if self.info == "RPOS":
                    self.info = pack(">4s", b"RPOS")
                    self.format = "fffxxxx"
                    break
                elif self.info == "OBSF":
                    self.info = pack(">4s", b"OBSF")
                    self.format = "Ixxxxxxxxxxxx"
                    break
                elif self.info == "RBID":
                    self.info = pack(">4s", b"RBID")
                    self.format = "Ixxxxxxxxxxxx"
                    break
                else :  
                    print("This is not an option")
                    continue
    
    def test_connection(self):
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((self.ipAdress, PORT))
            self.s.send(self.info)
        except:
            print("The IP address is wrong")
            sys.exit()

    def receive_data(self):
        self.rosInfo = self.s.recv(16)
        self.info = (unpack(">4s", self.info)[0]).decode('utf-8')

        self.msg_ROS = struct.unpack(self.format, self.rosInfo)

        if self.info == "RPOS":
                print(" Postion x : ", self.msg_ROS[0], "\n", "Position y : ", self.msg_ROS[1], "\n", "Position z : ", self.msg_ROS[2], "\n")

        elif self.info == "OBSF":
            if self.msg_ROS == 1:
                print("Obstacle : oui \n")
            else :
                print("Obstacle : non \n")

        elif self.info == "RBID":
                print("ID véhicule: ", self.msg_ROS[0], "\n")

        self.s.close()

    def getIP(self):
        print(self.ipAdress)
    
    def getInfo(self):
        infoList = self.info
        print(infoList) 
 
if __name__ == "__main__":
    user = userScreen()

    while True:
        user.userInfo()
        user.test_connection()
        user.receive_data()
