import socket
from threading import Thread

def thr_callback(sockd):
	data = sockd.recv(1024);

	type=int.from_bytes(data[0:1],"big");

	if(type!=0x02):
		print("Error. Received reply has an invalid type:",data[0:1]);
	
	# Bytes 4 to 7 contain the Free RAM data (network byte order is always big endian)
	free_ram=type=int.from_bytes(data[4:8],"big");

	# Bytes 8 and 9 contain the CPU usage data in 0.01%
	cpu_usage=type=int.from_bytes(data[8:10],"big")/100.0;

	# Bytes 10 and 11 contain the GPU usage data in 0.01%
	gpu_usage=type=int.from_bytes(data[10:12],"big")/100.0;

	print("Free RAM (MB):",free_ram);
	print("CPU Usage (%):",cpu_usage);
	print("GPU Usage (%):",gpu_usage);
	

IP_ADDR="127.0.0.1"
EDCP_PORT=48888
# Only the first 2 bits should be set to 0x01 to send a request to the EDCP collector
# 0x01 means 'REQUEST', 0x02 means 'REPLY', 0x00 and 0x03 shall not be used as they are reserved
REQUEST_MSG=b'\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'

def main():
	udp_sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
	
	# Spawning a thread can be useful to be sure to enter the blocking recv()
	# before "triggering" the EDCP Collector with sendto()
	rx_thread = Thread(target = thr_callback, args = (udp_sock, ));
	rx_thread.start();

	udp_sock.sendto(REQUEST_MSG,(IP_ADDR,EDCP_PORT));

	rx_thread.join();
	

if __name__ == "__main__":
	main();
