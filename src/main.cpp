#include <iostream>
#include <string>
#include <climits>
#include <cfloat>
#include <unistd.h>
// TCLAP headers
#include "tclap/CmdLine.h"
// VDP GPS Client
#include "gpsc.h"
// CA Basic Service (TX only, for the time being)
#include "caBasicService.h"
// Linux net includes
#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>

#define INVALID_LONLAT -DBL_MAX
#define GNSS_DEFAULT_PORT 3000

int main (int argc, char *argv[]) {
	std::string dissem_vif = "wlan0";
	std::string gnss_device = "localhost";
	long gnss_port = 3000; // Using 3000 as default port, in our case
	double fixed_lat = -DBL_MAX; // [deg]
	double fixed_lon = -DBL_MAX; // [deg]
	double fixed_speed_ms = -DBL_MAX; // [m/s]
	double fixed_heading_deg = -DBL_MAX; // [deg]

	// Parse the command line options with the TCLAP library
	try {
		TCLAP::CmdLine cmd("The Open CA Basic Service implementatiuon", ' ', "0.1");

		// Arguments: short option, long option, description, is it mandatory?, default value, type indication (just a string to help the user)
		TCLAP::ValueArg<std::string> vifName("I","interface","Broadcast dissemination interface",false,"wlan0","string");
		cmd.add(vifName);

		TCLAP::ValueArg<std::string> GNSSDevArg("D","gnss-device","GNSS device to be used",false,"localhost","string");
		cmd.add(GNSSDevArg);

		TCLAP::ValueArg<long> GNSSPortArg("P","gnss-port","Port to be used to connect to the GNSS device",false,GNSS_DEFAULT_PORT,"integer");
		cmd.add(GNSSPortArg);

		TCLAP::ValueArg<double> fixedLat("L","fixed-lat","Force a fixed latitude value when testing without a GNSS device",false,-DBL_MAX,"float (deg)");
		cmd.add(fixedLat);

		TCLAP::ValueArg<double> fixedLon("l","fixed-lon","Force a fixed longitude value when testing without a GNSS device",false,-DBL_MAX,"float (deg)");
		cmd.add(fixedLon);

		TCLAP::ValueArg<double> fixedSpeed("s","fixed-speed","Force a fixed speed value when testing without a GNSS device",false,-DBL_MAX,"float (m/s)");
		cmd.add(fixedSpeed);

		//TCLAP::ValueArg<double> fixedHeading("h","fixed-heading","Force a fixed heading value when testing without a GNSS device",false,-DBL_MAX,"float (deg)");
		//cmd.add(fixedHeading);

		// TCLAP::SwitchArg skipGNArg("S","skip-gn","Specify this option to send only Facilities Layer messages, instead of full ITS messages (Facilities layer + GeoNetworking + BTP). Warning! Experimental feature (it will work when only CAMs are sent)!");
		// cmd.add(skipGNArg);

		cmd.parse(argc,argv);

		dissem_vif=vifName.getValue();

		gnss_device=GNSSDevArg.getValue();
		gnss_port=GNSSPortArg.getValue();

		fixed_lat=fixedLat.getValue();
		fixed_lon=fixedLon.getValue();
		fixed_speed_ms=fixedSpeed.getValue();
		//fixed_heading_deg=fixedHeading.getValue();

		std::cout << "[INFO] CAM dissemination interface: " << dissem_vif << std::endl;
	} catch (TCLAP::ArgException &tclape) { 
		std::cerr << "TCLAP error: " << tclape.error() << " for argument " << tclape.argId() << std::endl;

		return 1;
	}

	// Create the UDP socket for the transmission of CAMs
	int sockfd=-1;
	sockfd=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);

	if(sockfd<0) {
		std::cerr << "Critical error: cannot open UDP socket for CAM dissemination.\n" << std::endl;
		exit(EXIT_FAILURE);
	}

	// Get the IP address of the dissemination interface
	struct ifreq ifreq;
	struct in_addr dissem_vif_addr;
	strncpy(ifreq.ifr_name,dissem_vif.c_str(),IFNAMSIZ);

	ifreq.ifr_addr.sa_family=AF_INET;

	if(ioctl(sockfd,SIOCGIFADDR,&ifreq)!=-1) {
		dissem_vif_addr=((struct sockaddr_in*)&ifreq.ifr_addr)->sin_addr;
	} else {
		std::cerr << "Critical error: cannot find an IP address for interface: " << dissem_vif << std::endl;
		exit(EXIT_FAILURE);
	}

	// Enable broadcast on the UDP socket
	int enableBcast=1;
	if(setsockopt(sockfd,SOL_SOCKET,SO_BROADCAST,&enableBcast,sizeof(enableBcast))<0) {
		std::cerr << "Critical error: cannot set broadcast permission on UDP socket for CAM dissemination.\n" << std::endl;
		exit(EXIT_FAILURE);
	}

	// Bind UDP socket
	struct sockaddr_in bindSockAddr;
	memset(&bindSockAddr,0,sizeof(struct sockaddr_in));
	bindSockAddr.sin_family=AF_INET;
	bindSockAddr.sin_port=htons(0);
	bindSockAddr.sin_addr.s_addr=dissem_vif_addr.s_addr;

	errno=0;
	if(bind(sockfd,(struct sockaddr *) &bindSockAddr,sizeof(struct sockaddr_in))<0) {
		std::cerr << "Critical error: cannot bind the UDP slave discovery socket to the '" << dissem_vif << "' interface. IP: " << inet_ntoa(dissem_vif_addr) << "." << std::endl
			<< "Socket: " << sockfd << ". Error: " << strerror(errno) << "." << std::endl;
		exit(EXIT_FAILURE);
	}

	// VDP (Vehicle Data Provider) GPS Client object test
	int cnt = 0;
	VDPGPSClient vdpgpsc(gnss_device,gnss_port);
	try {
		vdpgpsc.openConnection();

		while (cnt<5) {
			VDPGPSClient::CAM_mandatory_data_t CAMdata;

			std::cout << "[INFO] VDP GPS Client test: getting GNSS data..." << std::endl;

			CAMdata = vdpgpsc.getCAMMandatoryData();

			std::cout << "[INFO] [" << cnt << "] VDP GPS Client test result: Lat: " << CAMdata.latitude << " deg - Lon: " << CAMdata.longitude << " deg - Heading: " << CAMdata.heading.getValue() << std::endl;

			sleep(1);
			cnt++;
		}

		CABasicService CABS;
		CABS.setStationProperties(2398471,StationType_passengerCar);
		CABS.setSocketTx(sockfd);
		CABS.setVDP(&vdpgpsc);
		CABS.startCamDissemination();
	} catch(const std::exception& e) {
		std::cerr << "Error in creating a new VDP GPS Client connection: " << e.what() << std::endl;
	}

	while(1);

	vdpgpsc.closeConnection();
	close(sockfd);

	return 0;
}