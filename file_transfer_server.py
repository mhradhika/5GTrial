import socket
from _thread import *
import os

host=socket.gethostbyname(socket.gethostname())
port=6666

server_socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server_socket.bind((host,port))

server_socket.listen(5)
print('Server has started')
count=0
done=0
count=1

def add_client(client_socket,client_addr):
    print(client_socket)
    print(client_addr)
    client_socket.send("Connected to server".encode())
    global count
    while(1):
        msg=client_socket.recv(1024).decode()
        if(msg=='bye'):
            exit()
        # print("here")
        filename=str(count)+".txt"
        file=open(filename,"w")
        count+=1
        file.write(msg)
        print(file)
        client_socket.send("done".encode())
    global done
    done=1
    client_socket.close()

while(1):
    client_socket,client_addr=server_socket.accept()
    start_new_thread(add_client,(client_socket,client_addr,))
    if(done):
        break

server_socket.close()



