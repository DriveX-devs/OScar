import socket
import sys
import argparse
import json
import time

def main():
	parser = argparse.ArgumentParser(description='Sample JSON TCP client for AIM')
	parser.add_argument('--server_ip_address', nargs='?', default="127.0.0.1", help='AIM JSON server address')
	parser.add_argument('--server_port', nargs='?', default=49000, help='AIM JSON server port')
	parser.add_argument('latitude', help='Center latitude around which the data should be gathered')
	parser.add_argument('longitude', help='Center longitude around which the data should be gathered')
	parser = argparse.ArgumentParser(description='Sample JSON TCP client for OScar')
	parser.add_argument('--server_ip_address', nargs='?', default="127.0.0.1", help='OScar JSON server address')
	parser.add_argument('--server_port', nargs='?', default=49000, help='OScar JSON server port')
	parser.add_argument('latitude', help='Center latitude used for computing the relative distance of each vehicle w.r.t. the specified point')
	parser.add_argument('longitude', help='Center longitude used for computing the relative distance of each vehicle w.r.t. the specified poin')
	args = parser.parse_args()

	# Create a new TCP socket
	tcp_sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)

	# Connect to the server (i.e., to AIM), using default port 8080 and considering a localhost instance of AIM
	# Connect to the server (i.e., to the OScar JSON server), using the specified port (default: 49000) and IP address (default: local instance of OScar)
	tcp_sock.connect((args.server_ip_address, int(args.server_port)))

	# Wait for a proper "welcome message" from OScar (used to verify if the API connection works)
	welcome_msg=tcp_sock.recv(1024);

	if(welcome_msg.decode("utf-8")=="Connection: confirmed"):
		# Prepare request, specifying the reference lat and lon
		# OScar will return the content of the LDM with relative distances computed with respect to the specified latitude and longitude
		request={"lat": float(args.latitude), "lon": float(args.longitude)}

		# Send the JSON-over-TCP request to OScar
		tcp_sock.sendall(bytes(json.dumps(request)+"\0",encoding="utf-8"))

		# Recive and parse reply
		# Recive and parse the reply from OScar
		rx_data_str=tcp_sock.recv(4096).decode("utf-8")

		# In this simple sample client, we just print the received information
		print(rx_data_str);

		rx_data=json.loads(rx_data_str)

		print("Received JSON file:")
		print(json.dumps(rx_data,indent=2,sort_keys=False));
	else:
		print("Error: received a wrong confirmation message:",welcome_msg.decode("utf-8"));
		sys.exit(1);

if __name__ == "__main__":
	main()