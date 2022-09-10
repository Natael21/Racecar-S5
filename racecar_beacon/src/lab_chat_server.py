#!/usr/bin/env python

import socket

HOST = '10.0.1.31'
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)

while True:
    (conn, addr) = s.accept()
    data =conn.recv(1024)
    if not data: break
    conn.send(data+b"*")
conn.close()