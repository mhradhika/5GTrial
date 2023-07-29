import socket
import os
import ifcfg
 
 
name=socket.gethostname()
server_ip='<server_ip_addr'
ip_addr=0
port=6666
 
for interface in ifcfg.interfaces().items():
	if(interface[0]=='uesimtun0'):
		print(interface[1]['inet'])
		ip_addr=interface[1]['inet']
 
client_socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
client_socket.bind((ip_addr,port))
client_socket.connect((server_ip,port))

 
path='home/isfcr/text_files'
files_list=[]
for root,dirs,files in os.walk(path):
	for file in files:
		print(files)
		files_list.append(root+'\\'[0]+file)
 
print(files_list)
for file in files_list:
	with open(file) as my_file:
		client_socket.send(my_file.read().encode())
		while(1):
			ack=client_socket.recv(1024).decode()
			print(ack)
			if(ack=="done"):
				break
 
client_socket.send("bye".encode())
 
 
