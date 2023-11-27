#ifndef VEHICLE_VISUALIZER_H
#define VEHICLE_VISUALIZER_H

#define VIS_HEADING_INVALID 361
#define DEFAULT_NODEJS_SERVER_PATH "./js/server.js"

#include <string>

class vehicleVisualizer
{
	public:
		// Create a new vehicleVisualizer object with the default IP and port for the connection to the Node.js server
		vehicleVisualizer();

		// Create a new vehicleVisualizer object with the default IP for the connection to the Node.js server
		vehicleVisualizer(int port);

		// Create a new vehicleVisualizer object for the connection to the Node.js server
		vehicleVisualizer(int port,std::string ipv4);

		// Object destructor, taking also care of sending the "terminate" message to the Node.js server to gracefully terminate it
		virtual ~vehicleVisualizer();

		// Setters to set an IPv4 or port, different than the default ones, for the UDP connection to the Node.js server
		void setIPv4(std::string ipv4) {m_ip=ipv4;}
		void setPort(int port);

		// Setter to set a different HTTP port at which the web visualizer will be available
		void setHTTPPort(int port);

		// This function must be called just after creating a new vehicleVisualizer object
		// It will start a Node.js server receiving data from ms-van3t via UDP and making a web GUI available to the user on loopback, port "m_httpport"
		// This function will return only when the Node.js server is ready
		int startServer();

		// This function must be called after calling startServer()
		// It will set up the UDP socket for the connection between ms-van3t and the server
		int connectToServer();

		// These functions should be automatically called by the TraCI client o by the GPS-tc client
		// The user, however, can also call sendObjectUpdate() to add and move an external object on the map, not managed by SUMO or GPS-tc

		// Send a "map draw" message to tell the server where the client (i.e. the web browser) should center the map at
		// This function should always be called once before calling sendObjectUpdate()
		int sendMapDraw(double lat, double lon);
		int sendMapDraw(double lat, double lon, double minlat, double minlon, double maxlat, double maxlon, double lat_ext_factor, double lon_ext_factor);

		// These functions will update the position on the map of the object with unique id "objID" (the vehicle ID can be used here, for instance)
		// If the object does not exist yet on the map, it will be first added
		// If no "heading" is specified, VIS_HEADING_INVALID (i.e. no heading available) will be sent to the server
		int sendObjectUpdate(std::string objID, double lat, double lon, int stationType);
		int sendObjectUpdate(std::string objID, double lat, double lon, int stationType, double heading);

		// This function will remove an object from the map, given its unique "objID"
		int sendObjectClean(std::string objID);

		// This function should be called to terminate the execution of the Node.js server
		// Normally, the user should not call it, as it is automatically called by the destructor of the vehicleVisualizer object
		int terminateServer();

		// This setter can be used to set a different path for the Node.js server.js file location
		void setServerPath(std::string srvpath) {m_serverpath = srvpath;}

		// These methods allows the user to check whether the UDP socket creation was successfully executed or whether the "map draw" message was successfully sent via UDP
		bool isConnected() {return m_is_connected;}
		bool mapDrawSent() {return m_is_map_sent;}
	private:
		int m_port;
		int m_httpport;
		std::string m_ip;
		int m_sockfd = -1;
		bool m_is_connected;
		bool m_is_map_sent;
		bool m_is_server_active;
		std::string m_serverpath;

		// Internal (private) function to open the UDP socket for the communication with the Node.js server
		int socketOpen(void);
};

#endif /* VEHICLE_VISUALIZER_H */

