# #!/usr/bin/env python
 
import socket
import time 

from struct import*
 
# On donne adresse du pi qui va prendre la position du RemoteClient
HOST = '10.0.1.31'
# This process should listen to a different port than the PositionBroadcast client.
# Ca ca veut dire quil faut quon soit sur un meme pi, mais que le port va changer pour les deux
PORT = 65432
 
class userScreen:
 def __init__(self):
 
 # variables of interest
     self.ipAdress = 0
     self.info = ''
     
     while True:
        self.ipAdress = input("IP Adress you search for (XX.XX.XX.XX): ")

        if not self.ipAdress:
            continue
        else:
            break

     time.sleep(0.5)

     while True:
        self.info2 = input("What info do you you want to acces (RPOS, OBSF, RBID): ")

        #if self.info
        break

     time.sleep(0.5)
     
     def clientStart(self): 
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        s.send(bytearray(self.getIP()))
        data = s.recv(1024)
        #verifie si on a envoyer une adresse ip et une information valide
        while not data:
            print("Where is the data bro?")
        if data : 
            print(data)
            s.close() 
        #apres avoir envoyer un message qui marche, il faut donner la possibilite au client de fermer la connexion
 
 
 def getIP(self):
    print(self.ipAdress)
 
 def getInfo(self):
    infoList = [self.info1, self.info2]
    print(infoList) 
 
if __name__ == "__main__":
 user = userScreen()
 user.getIP()
 user.getInfo()