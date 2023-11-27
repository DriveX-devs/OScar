#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "vehicle-visualizer.h"

vehicleVisualizer::vehicleVisualizer()
{
	// Set default ip and port
	m_ip="127.0.0.1";
	m_port=48110;

	// Set the default HTTP Node.js server port
	m_httpport=8080;

	m_is_connected=false;
	m_is_map_sent=false;
	m_is_server_active=false;
	m_serverpath=DEFAULT_NODEJS_SERVER_PATH;
}

vehicleVisualizer::vehicleVisualizer(int port)
{
  // Set default ip
  m_ip="127.0.0.1";
  m_port=port;

  // Set the default HTTP Node.js server port
  m_httpport=8080;

  m_is_connected=false;
  m_is_map_sent=false;
  m_is_server_active=false;
  m_serverpath=DEFAULT_NODEJS_SERVER_PATH;
}

vehicleVisualizer::vehicleVisualizer(int port,std::string ipv4)
{
  m_ip=ipv4;
  m_port=port;

  m_httpport=8080;

  m_is_connected=false;
  m_is_map_sent=false;
  m_is_server_active=false;
  m_serverpath=DEFAULT_NODEJS_SERVER_PATH;
}

int
vehicleVisualizer::connectToServer()
{
	m_sockfd=socketOpen();

	if(m_sockfd<0) {
		return -1;
	}

	m_is_connected=true;

	return 0;
}

vehicleVisualizer::~vehicleVisualizer()
{
	// The destructor will attempt to send a termination message to the Node.js server only if a server
	// was successfully started with startServer()
	if(m_is_server_active) {
		terminateServer ();

		// Close the UDP socket, if it was opened before
		if(m_sockfd!=-1) {
			close(m_sockfd);
			m_sockfd=-1;
		}
	}
}

int
vehicleVisualizer::sendMapDraw(double lat, double lon)
{
	if(m_is_connected==false) {
		std::cerr<<"Error: attempted to use a non-connected vehicle visualizer client."<<std::endl;
		exit(EXIT_FAILURE);
	}

	if(m_is_map_sent==true) {
		std::cerr<<"Error in vehicle visualizer client: attempted to send twice a map draw message. This is not allowed."<<std::endl;
		exit(EXIT_FAILURE);
	}

	std::ostringstream oss;
	int send_rval=-1;

	oss.precision(7);
	oss<<"map,"<<lat<<","<<lon;

	std::string msg_string = oss.str();
	char *msg_buffer = new char[msg_string.length() + 1];
	std::copy(msg_string.c_str(), msg_string.c_str() + msg_string.length() + 1, msg_buffer);

	send_rval=send(m_sockfd,msg_buffer,msg_string.length()+1,0);

	delete[] msg_buffer;

	m_is_map_sent=true;

	return send_rval;
}

int 
vehicleVisualizer::sendMapDraw(double lat, double lon, double minlat, double minlon, double maxlat, double maxlon, double lat_ext_factor, double lon_ext_factor)
{
	if(m_is_connected==false) {
		std::cerr<<"Error: attempted to use a non-connected vehicle visualizer client."<<std::endl;
		exit(EXIT_FAILURE);
	}

	if(m_is_map_sent==true) {
		std::cerr<<"Error in vehicle visualizer client: attempted to send twice a map draw message. This is not allowed."<<std::endl;
		exit(EXIT_FAILURE);
	}

	std::ostringstream oss;
	int send_rval=-1;

	oss.precision(7);
	oss<<"map_areas,"<<lat<<","<<lon<<","<<minlat<<","<<minlon<<","<<maxlat<<","<<maxlon<<","<<lat_ext_factor<<","<<lon_ext_factor;

	std::string msg_string = oss.str();
	char *msg_buffer = new char[msg_string.length() + 1];
	std::copy(msg_string.c_str(), msg_string.c_str() + msg_string.length() + 1, msg_buffer);

	send_rval=send(m_sockfd,msg_buffer,msg_string.length()+1,0);

	delete[] msg_buffer;

	m_is_map_sent=true;

	return send_rval;
}

int
vehicleVisualizer::sendObjectUpdate(std::string objID, double lat, double lon, int stationType, double heading)
{
	if(m_is_connected==false) {
		std::cerr << "Error: attempted to use a non-connected vehicle visualizer client." << std::endl;
		exit(EXIT_FAILURE);
	}

	if(m_is_map_sent==false) {
		std::cerr << "Error in vehicle visualizer client: attempted to send an object update before sending the map draw message." << std::endl;
		exit(EXIT_FAILURE);
	}

	std::ostringstream oss;
	int send_rval=-1;

	oss.precision(7);
	oss<<"object,"<<objID<<","<<lat<<","<<lon<<","<<stationType<<",";
	oss.precision(3);
	oss<<heading;

	std::string msg_string = oss.str();
	char *msg_buffer = new char[msg_string.length()+1];
	std::copy(msg_string.c_str(), msg_string.c_str() + msg_string.length() + 1, msg_buffer);

	send_rval=send(m_sockfd,msg_buffer,msg_string.length()+1,0);

	delete[] msg_buffer;

	return send_rval;
}

int
vehicleVisualizer::sendObjectUpdate(std::string objID, double lat, double lon, int stationType)
{
	return sendObjectUpdate (objID,lat,lon,stationType,VIS_HEADING_INVALID);
}

int
vehicleVisualizer::sendObjectClean(std::string objID)
{
	if(m_is_connected==false) {
		std::cerr << "Error: attempted to use a non-connected vehicle visualizer client." << std::endl;
		exit(EXIT_FAILURE);
	}

	if(m_is_map_sent==false) {
		std::cerr << "Error in vehicle visualizer client: attempted to send an object clean message before sending the map draw message." << std::endl;
		exit(EXIT_FAILURE);
	}

	std::ostringstream oss;
	int send_rval=-1;

	oss<<"objclean,"<<objID;

	std::string msg_string = oss.str();
	char *msg_buffer = new char[msg_string.length()+1];
	std::copy(msg_string.c_str(), msg_string.c_str() + msg_string.length() + 1, msg_buffer);

	send_rval=send(m_sockfd,msg_buffer,msg_string.length()+1,0);

	delete[] msg_buffer;

	return send_rval;	
}

int
vehicleVisualizer::startServer()
{
	std::string servercmd;

	// Create a FIFO special file (see https://linux.die.net/man/3/mkfifo) for the "startup communication"
	// between the S-LDM and the node.js server we are going to start
	// Basically this file will be used as a pipe to tell the S-LDM when the node.js server UDP socket
	// creation is complete, so that the S-LDM can start sending information to the server
	// The special FIFO file will be named "vehvizfifo<PID of this S-LDM instance>"
	std::string fifofile = "/tmp/vehvizfifo" + std::to_string(getpid());
	int fifofd=-1;

	// Print an error and exit if mkfifo() failed due to any reason different than an already existing file (errno==EEXIST)
	// If the FIFO special file already exists, for any reason, we should be able to simply re-use it
	if(mkfifo(fifofile.c_str(), 0660) < 0) {
		if(errno!=EEXIST) {
			std::cerr << "Cannot start the Node.js server. mkfifo() communication pipe creation error." << std::endl;
			perror("Error details:");
			exit(EXIT_FAILURE);
		}
	} else {
		std::cout << "VehicleVisualizer startup: using temporary FIFO special file: " << fifofile << std::endl;
	}

	// Open the FIFO file (O_RDWR is needed, instead of O_RDONLY, to avoid blocking on this open() due to the absence of a writer.
	// Indeed, the writer will be the node.js server, which hasn't started yet)
	fifofd = open(fifofile.c_str(),O_RDWR);

	if(fifofd < 0) {
		std::cerr << "Cannot start the Node.js server. mkfifo() communication pipe error." << std::endl;
		exit(EXIT_FAILURE);
	}

	// Start the node.js server managing the web-based vehicle visualizer S-LDM GUI
	int nodeCheckRval = std::system("command -v node > /dev/null");

	if(nodeCheckRval != 0) {
		std::cerr << "Error. Node.js does not seem to be installed. Please install it before using the S-LDM vehicle visualizer." << std::endl;
		exit(EXIT_FAILURE);
	}

	// servercmd = "node " + m_serverpath + " " + std::to_string(m_httpport) + " &";

	// node.js server (server.js) command line parameters: HTTP web interface port, UDP socket bind address, UDP socket port, PID of this S-LDM instance
	servercmd = "node " + m_serverpath + " " + std::to_string(m_httpport) + " " + m_ip + " " + std::to_string(m_port) + " " + std::to_string(getpid()) + " &";

	int startCmdRval = std::system(servercmd.c_str());

	// If the result is 0, system() was able to successfully launch the command (which may fail afterwards, though)
	if (startCmdRval == 0) {
		std::cout<<"Used the following command to start up the vehicle visualizer Node.js server: " << servercmd << std::endl;
		m_is_server_active = true;
	} else {
		std::cerr << "Cannot send the command for starting the Node.js server." << std::endl;
		exit(EXIT_FAILURE);
	}

	std::cout << "VehicleVisualizer startup: Waiting for the VehicleVisualizer node.js server to come up..." << std::endl;

	// Wait 1 second for the server to come up (no more needed since the FIFO special file mechanism is in place)
	// sleep(1);

	// Blocking read on the FIFO special file
	// This read() will block until the node.js server writes a string on the same file, to signal that its internal UDP socket is ready to receive
	// data from the S-LDM
	// The expected string from the node.js server, which will be stored inside "buf", is "STARTED" (so, we need 7 characters + '\0')
	char buf[8] = {0};
	if(read(fifofd,buf,8)<=0) {
		std::cout << "Error. It seems that the node.js server could not come up." << std::endl;
	} else {
		std::cout << "VehicleVisualizer node.js server succesfully started. Confirmation message content: " << std::string(buf) << std::endl;
	}

	// Close and delete (with unlink(), as there should be no other process referring to the same file) the FIFO special file which was used as 
	// a "synchronization mean" between the S-LDM and the node.js server
	close(fifofd);
	unlink(fifofile.c_str());

	return startCmdRval;
}

int
vehicleVisualizer::terminateServer()
{
	if(m_is_connected==false) {
		std::cerr << "Error: attempted to use a non-connected vehicle visualizer client." << std::endl;
		exit(EXIT_FAILURE);
	}

	// Just send a message with the "terminate" string
	char terminatebuf[10]="terminate";

	int retval = send(m_sockfd,terminatebuf,9,0);

	if (retval==9) {
		m_is_connected=false;
		m_is_server_active=false;
	}

	return retval;
}

int
vehicleVisualizer::socketOpen(void)
{
	struct sockaddr_in saddr={};
	struct in_addr destIPaddr;
	int sockfd;

	sockfd=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);

	if(sockfd<0) {
		perror("socket() error:");
		std::cerr << "Error! Cannot open the UDP socket for the vehicle visualizer." << std::endl;
		exit(EXIT_FAILURE);
	}

	// Bind structure
	saddr.sin_family=AF_INET;
	saddr.sin_port=0;
	saddr.sin_addr.s_addr=INADDR_ANY;

	if(bind(sockfd,(struct sockaddr *) &(saddr),sizeof(saddr))<0) {
		perror("Cannot bind socket: bind() error");
		std::cerr << "Error! Cannot bind the UDP socket for the vehicle visualizer." << std::endl;
		exit(EXIT_FAILURE);
	}

	// Get struct in_addr corresponding to the destination IP address
	if(inet_pton(AF_INET,m_ip.c_str(),&(destIPaddr))!=1) {
		fprintf(stderr,"Error in parsing the destination IP address.\n");
		std::cerr << "Error! Cannot parse the destination IP for the UDP socket for the vehicle visualizer." << std::endl;
		exit(EXIT_FAILURE);
	}

	// Connect structure (re-using the same structure - sin_family should remain the same)
	saddr.sin_port=htons(m_port);
	saddr.sin_addr=destIPaddr;

	if(connect(sockfd,(struct sockaddr *) &(saddr),sizeof(saddr))<0) {
		perror("Cannot connect socket: connect() error");
		std::cerr << "Error! Cannot connect the UDP socket for the vehicle visualizer." << std::endl;
		exit(EXIT_FAILURE);
	}

	return sockfd;
}

void
vehicleVisualizer::setPort(int port)
{
	// Set the default port when an invalid port is specified
	if(port>=1 && port<=65535) {
		m_port=port;
	} else {
		std::cerr << "Error: called setPort for vehicleVisualizer with an invalid port number. Using the default port." << std::endl;
		m_port=48110;
	}
}

void
vehicleVisualizer::setHTTPPort(int port)
{
	// Set the default port when an invalid port is specified
	if(port>=1 && port<=65535) {
		m_httpport=port;
	} else {
		std::cerr << "Error: called setPort for vehicleVisualizer with an invalid port number. Using the default port." << std::endl;
		m_httpport=8080;
	}
}

