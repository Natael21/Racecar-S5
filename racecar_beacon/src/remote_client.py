#!/usr/bin/env python
 
from ast import IsNot
import socket
import time 

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
        self.msg_ROS = ""

    def userInfo(self):
        while True:
            self.ipAdress = input("IP Adress you search for (XX.XX.XX.XX): ")
            break

        time.sleep(0.5)

        while True:
            self.info = input("What info do you you want to acces (RPOS, OBSF, RBID): ")
            print(self.info)
            if self.info != "RPOS":
                print("This is not an option")
                continue
            # elif self.info != "OBSF":
            #     print("This is not an option")
            #     continue
            # elif self.info != "RBID":
            #     print("This is not an option")
            #     continue
            else :  
                break

        time.sleep(0.5)

    def start(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        s.send(bytearray(self.ipAdress))
        self.msg_ROS = s.recv(1024)
        print(self.msg_ROS)
        s.close()
     
    # def clientStart(self): 
    #     s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #     s.connect((HOST, PORT))
    #     s.send(bytearray(self.getIP()))
    #     data = s.recv(1024)
    #     #verifie si on a envoyer une adresse ip et une information valide
    #     while not data:
    #         print("Where is the data bro?")
    #     if data : 
    #         print(data)
    #         s.close() 
        #apres avoir envoyer un message qui marche, il faut donner la possibilite au client de fermer la connexion
    def stop (self):
        print("Dont stop, believing")

    def getIP(self):
        print(self.ipAdress)
    
    def getInfo(self):
        infoList = self.info
        print(infoList) 
 
if __name__ == "__main__":
    user = userScreen()
    user.userInfo()
    user.start()
    # user.getIP()
    # user.getInfo()

    #A des fins de test seulement quand on utiliser adresse 127.0.0.1
    # s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # s.connect((HOST, PORT))
    # s.send(b"hello")
    # data = s.recv(1024)
    # print(data)
    # s.close()