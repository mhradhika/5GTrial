import socket,cv2, pickle,struct
from _thread import * 


server_ip = socket.gethostbyname(socket.gethostname())
port = 9999

server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server_socket.bind((server_ip,port))
server_socket.listen(5)
data = b""
payload_size = struct.calcsize("Q")
print(payload_size)

def add_client(client_socket,client_addr):
	print(client_addr)
	global data
	while True:
		while len(data) < payload_size:
			packet = client_socket.recv(4*1024) 
			while(not packet):
				continue
			if not packet: 
				break
			data+=packet
		packed_msg_size = data[:payload_size]
		data = data[payload_size:]
		msg_size = struct.unpack("Q",packed_msg_size)[0]
		
		while len(data) < msg_size:
			data += client_socket.recv(4*1024)
		frame_data = data[:msg_size]
		data  = data[msg_size:]
		frame = pickle.loads(frame_data)
		cv2.imshow("RECEIVING VIDEO",frame)
		if cv2.waitKey(1) == '13':
			break

while(1):
	client_socket,client_addr=server_socket.accept()
	start_new_thread(add_client,(client_socket,client_addr,))
server_socket.close()
