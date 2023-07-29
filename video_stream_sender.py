import socket, cv2, pickle,struct,imutils
import ifcfg

server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
host_ip=0
client_addr='client_socker_address'

for interface in ifcfg.interfaces().items():
	if(interface[0]=='uesimtun0'):
		print(interface[1]['inet'])
		host_ip=interface[1]['inet']

port = 9999

server_socket.bind((host_ip,port))
client_socket.connect((client_addr,port))

vid = cv2.VideoCapture(0)

while(vid.isOpened()):
	img,frame = vid.read()
	frame = imutils.resize(frame,width=320)
	a = pickle.dumps(frame)
	message = struct.pack("Q",len(a))+a
	client_socket.sendall(message)
	
	cv2.imshow('TRANSMITTING VIDEO',frame)
	if cv2.waitKey(1) == '13':
		client_socket.close()
