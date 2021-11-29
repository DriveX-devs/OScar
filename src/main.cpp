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

#include <sys/socket.h>
#include <net/ethernet.h>
#include <linux/if_packet.h>

#define INVALID_LONLAT -DBL_MAX
#define GNSS_DEFAULT_PORT 3000

int main (int argc, char *argv[]) {
	std::string dissem_vif = "wlan0";
	std::string gnss_device = "localhost";
	long gnss_port = 3000; // Using 3000 as default port, in our case
	unsigned long vehicleID = 0; // Vehicle ID (mandatory when starting OCABS)

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

		TCLAP::ValueArg<unsigned long> VehicleIDArg("v","vehicle-id","CA Basic Service Station ID",true,0,"unsigned integer");
                cmd.add(VehicleIDArg);

		cmd.parse(argc,argv);

		dissem_vif=vifName.getValue();

		gnss_device=GNSSDevArg.getValue();
		gnss_port=GNSSPortArg.getValue();

		vehicleID=VehicleIDArg.getValue();

		std::cout << "[INFO] CAM dissemination interface: " << dissem_vif << std::endl;
	} catch (TCLAP::ArgException &tclape) { 
		std::cerr << "TCLAP error: " << tclape.error() << " for argument " << tclape.argId() << std::endl;

		return 1;
	}

	// Create the raw socket for the transmission of CAMs, encapsulated inside GeoNetworking and BTP (in user space) 
	int sockfd=-1;
	// sockfd=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP); // Old UDP socket, no more used
	sockfd=socket(AF_PACKET,SOCK_RAW,htons(ETH_P_ALL));

	if(sockfd<0) {
		std::cerr << "Critical error: cannot open UDP socket for CAM dissemination.\n" << std::endl;
		exit(EXIT_FAILURE);
	}

	// Get the index of the dissemination interface
	int ifindex=if_nametoindex(dissem_vif.c_str());
	struct ifreq ifreq;
	struct in_addr dissem_vif_addr;
	strncpy(ifreq.ifr_name,dissem_vif.c_str(),IFNAMSIZ);

	ifreq.ifr_addr.sa_family=AF_INET;

	if(ifindex<1) {
		std::cerr << "Critical error: cannot find an interface index for interface: " << dissem_vif << std::endl;
		exit(EXIT_FAILURE);
	}

	// Get the MAC address of the dissemination interface and store it inside "srcmac"
	uint8_t srcmac[6]={0};
	strncpy(ifreq.ifr_name,dissem_vif.c_str(),IFNAMSIZ); 
	if(ioctl(sockfd,SIOCGIFHWADDR,&ifreq)!=-1) {
		memcpy(srcmac,ifreq.ifr_hwaddr.sa_data,6);
	} else {
		std::cerr << "Critical error: cannot find a MAC address for interface: " << dissem_vif << std::endl;
		exit(EXIT_FAILURE);
	}

	// Enable broadcast on the socket (is it really needed? To be double-checked!)
	int enableBcast=1;
	if(setsockopt(sockfd,SOL_SOCKET,SO_BROADCAST,&enableBcast,sizeof(enableBcast))<0) {
		std::cerr << "Critical error: cannot set broadcast permission on UDP socket for CAM dissemination.\n" << std::endl;
		exit(EXIT_FAILURE);
	}

	// Bind raw socket
	struct sockaddr_ll addrll;
	memset(&addrll,0,sizeof(addrll));
	addrll.sll_ifindex=ifindex;
	addrll.sll_family=AF_PACKET;
	addrll.sll_protocol=htons(ETH_P_ALL);

	errno=0;
	if(bind(sockfd,(struct sockaddr *) &addrll,sizeof(addrll))<0) {
		std::cerr << "Critical error: cannot bind the raw socket to the '" << dissem_vif << "' interface. Ifindex: " << ifindex << "." << std::endl
			<< "Socket: " << sockfd << ". Error: " << strerror(errno) << "." << std::endl;
		exit(EXIT_FAILURE);
	}

	// VDP (Vehicle Data Provider) GPS Client object test
	int cnt = 0;
	VDPGPSClient vdpgpsc(gnss_device,gnss_port);

	bool m_retry_flag=false;

	do {
		m_retry_flag=false;

		try {
			vdpgpsc.openConnection();

			while (cnt<10) {
				VDPGPSClient::CAM_mandatory_data_t CAMdata;

				std::cout << "[INFO] VDP GPS Client test: getting GNSS data..." << std::endl;

				CAMdata = vdpgpsc.getCAMMandatoryData();

				std::cout << "[INFO] [" << cnt << "] VDP GPS Client test result: Lat: " << CAMdata.latitude << " deg - Lon: " << CAMdata.longitude << " deg - Heading: " << CAMdata.heading.getValue() << std::endl;

				sleep(1);
				cnt++;
			}

			std::cout << "[INFO] Dissemination started!" << std::endl;

			CABasicService CABS;
			GeoNet GN;
			GN.setVDP(&vdpgpsc);
			GN.setSocketTx(sockfd,ifindex,srcmac);
			GN.setStationProperties(vehicleID,StationType_passengerCar);

			btp BTP;
			BTP.setGeoNet(&GN);

			CABS.setBTP(&BTP);
			CABS.setStationProperties(vehicleID,StationType_passengerCar);
			CABS.setVDP(&vdpgpsc);
			CABS.startCamDissemination();
		} catch(const std::exception& e) {
			std::cerr << "Error in creating a new VDP GPS Client connection: " << e.what() << std::endl;
			sleep(5);
			m_retry_flag=true;
		}
	} while(m_retry_flag==true);

	vdpgpsc.closeConnection();
	close(sockfd);

	return 0;
}
