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
	std::string log_filename = "dis";
	std::string gnss_device = "localhost";
	std::string aux_device_IP = "dis";
	long gnss_port = 3000; // Using 3000 as default port, in our case
	unsigned long vehicleID = 0; // Vehicle ID (mandatory when starting OCABS)
	bool enable_enhanced_CAMs = false;
	std::string extra_computation_device_IP = "dis";

	std::string udp_sock_addr = "dis";
	std::string udp_bind_ip = "0.0.0.0";

	// Enhanced CAMs-only options
	double rssi_aux_update_interval_msec=-1;

	// Future work: automatically get these values given an interface name
	std::string own_private_IP = "dis";
	std::string own_public_IP = "dis";

	// Parse the command line options with the TCLAP library
	try {
		TCLAP::CmdLine cmd("The Open CA Basic Service implementatiuon", ' ', "0.3");

		// Arguments: short option, long option, description, is it mandatory?, default value, type indication (just a string to help the user)
		TCLAP::ValueArg<std::string> vifName("I","interface","Broadcast dissemination interface. Default: wlan0.",false,"wlan0","string");
		cmd.add(vifName);
		
		TCLAP::ValueArg<std::string> Logfile("L","log-file","Print on file the log for the CAM condition checks. Default: (disabled).",false,"dis","string");
		cmd.add(Logfile);

		TCLAP::ValueArg<std::string> GNSSDevArg("D","gnss-device","GNSS device to be used (i.e., where gpsd is currently running - this is not the /dev/ttyACM* device, which is already being used by gpsd, which in turn can provide the GNSS data to OCABS). Default: localhost.",false,"localhost","string");
		cmd.add(GNSSDevArg);

		TCLAP::ValueArg<long> GNSSPortArg("P","gnss-port","Port to be used to connect to the GNSS device. It should correspond to the port used by gpsd for the desired receiver. Warning! The default port for gpsd is 2947, while the default for OCABS is 3000.",false,GNSS_DEFAULT_PORT,"integer");
		cmd.add(GNSSPortArg);

		TCLAP::ValueArg<unsigned long> VehicleIDArg("v","vehicle-id","CA Basic Service Station ID",true,0,"unsigned integer");
		cmd.add(VehicleIDArg);

		TCLAP::SwitchArg enchancedCAMsArg("E","enable-enhanced-CAMs","Enable the dissemination of experimental enhanced CAMs",false);
		cmd.add(enchancedCAMsArg);

		TCLAP::ValueArg<std::string> AuxDevIPArg("A","aux-dev-ip","IP of a possible auxiliary device to use for wireless communication. Writing 'dis' instead of an IP address disables support to additional devices. The device must run RouterOS as main OS.",false,"dis","string");
		cmd.add(AuxDevIPArg);

		TCLAP::ValueArg<double> rssiAuxUpdateIntervalArg("p","rssi-aux-update-interval","RSSI retrieval update interval for a connected auxiliary communication device based on RouterOS. Setting this to any value <=0 will disable the auxiliary RSSI retrieval.",false,-1.0,"float");
		cmd.add(rssiAuxUpdateIntervalArg);

		TCLAP::ValueArg<std::string> ExtraDevIPArg("x","extra-comp-dev-ip","IP of a possible extra computation device to use for offloading computation tasks from the device running OCABS. Writing 'dis' instead of an IP address disables support to additional device. The device must run an information provider compliant with the EDCP custom protocol. This option has an effect only when sending enhanced CAMs.",false,"dis","string");
		cmd.add(ExtraDevIPArg);
		
		TCLAP::ValueArg<std::string> OwnPrivateIPArg("Z","own-private-ip","Specify the own IP address to be disseminated through enhanced CAMs, if the devices supports IP-based communication. Writing 'dis' will disable the dissemination of this information.",false,"dis","string");
		cmd.add(OwnPrivateIPArg);

		TCLAP::ValueArg<std::string> OwnPublicIPArg("z","own-public-ip","Specify the own public IP address (if the device can be reached from the outside world, with or without port forwarding) to be disseminated through enhanced CAMs, if the devices supports IP-based communication. Writing 'dis' will disable the dissemination of this information.",false,"dis","string");
		cmd.add(OwnPublicIPArg);

		TCLAP::ValueArg<std::string> UDPSockAddrArg("u","udp-sock-addr","If specified, OCABS, in addition to the standard-compliant CAM dissemination, will also encapsulate each CAM inside UDP, and send these messages to the address (in the form <IP:port>) specified after this options.",false,"dis","string");
		cmd.add(UDPSockAddrArg);

		TCLAP::ValueArg<std::string> UDPBindIPArg("U","udp-bind-ip","This options is valid only if --udp-sock-addr/-u has been specified. It can be used to set an interface/address to bind the UDP socket to. By default, no specific address is used for binding (i.e., binding to any address/interface).",false,"0.0.0.0","string");
		cmd.add(UDPBindIPArg);

		cmd.parse(argc,argv);

		dissem_vif=vifName.getValue();
		
		log_filename=Logfile.getValue();

		gnss_device=GNSSDevArg.getValue();
		gnss_port=GNSSPortArg.getValue();

		vehicleID=VehicleIDArg.getValue();

		enable_enhanced_CAMs=enchancedCAMsArg.getValue();

		aux_device_IP=AuxDevIPArg.getValue();
		rssi_aux_update_interval_msec=rssiAuxUpdateIntervalArg.getValue();

		extra_computation_device_IP=ExtraDevIPArg.getValue();

		own_private_IP=OwnPrivateIPArg.getValue();
		own_public_IP=OwnPublicIPArg.getValue();

		udp_sock_addr=UDPSockAddrArg.getValue();

		udp_bind_ip=UDPBindIPArg.getValue();

		std::cout << "[INFO] CAM dissemination interface: " << dissem_vif << std::endl;
	} catch (TCLAP::ArgException &tclape) { 
		std::cerr << "TCLAP error: " << tclape.error() << " for argument " << tclape.argId() << std::endl;

		return 1;
	}

	if(udp_bind_ip!="0.0.0.0" && udp_sock_addr=="dis") {
		std::cerr << "Error. --udp-bind-ip/-U can only be specified when --udp-sock-addr/-u is specified too." << std::endl;

		return 1;
	}

	// Try to get the MAC address of the auxiliary device, if requested and if enhanced CAMs are enabled
	std::string aux_device_MAC="unavailable";
	if(enable_enhanced_CAMs==true && aux_device_IP!="dis") {
		char aux_dev_MAC_buf[18];
		std::string ssh_command = "stdbuf -o L ssh -o ConnectTimeout=5 admin@" + aux_device_IP + " interface w60g print | stdbuf -o L grep \"mac\" | stdbuf -o L cut -d\"=\" -f5";
		FILE *ssh = popen(ssh_command.c_str(),"r");

		if(ssh!=NULL) {
			if(fgets(aux_dev_MAC_buf,18,ssh)!=NULL) {
				aux_device_MAC=std::string(aux_dev_MAC_buf);
				std::cerr << "[INFO] Succesfully retrieved the auxiliary device MAC address at " << aux_device_IP << ": " << aux_device_MAC << std::endl;
			} else {
				std::cerr << "[WARN] Unable to retrieve the auxiliary device MAC address at " << aux_device_IP << ". This information will not be disseminated." << std::endl;
			}
		}
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
	if(ifindex<1) {
		std::cerr << "Critical error: cannot find an interface index for interface: " << dissem_vif << std::endl;
		exit(EXIT_FAILURE);
	}

	// Get the MAC address of the dissemination interface and store it inside "srcmac"
	uint8_t srcmac[6]={0};
	struct ifreq ifreq;
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

			if(udp_sock_addr!="dis") {
				GN.openUDPsocket(udp_sock_addr,udp_bind_ip);
			}

			btp BTP;
			BTP.setGeoNet(&GN);

			if(enable_enhanced_CAMs==true) {
				CABS.enableEnhancedCAMs();

				if(aux_device_MAC!="unavailable") {
					CABS.setEnhancedCAMAuxiliaryMAC(aux_device_MAC);
				}

				if(aux_device_IP!="dis" && rssi_aux_update_interval_msec>0) {
					CABS.enableAuxRSSIRetrieval(rssi_aux_update_interval_msec,aux_device_IP);
				}

				// Set the IP of the extra computation device to retrieve the CPU/GPU/RAM usage from, if enhanced CAMs are enabled and an IP address is specified
				if(extra_computation_device_IP!="dis") {
					CABS.setExtraComputationDeviceIP(extra_computation_device_IP);
				}

				if(own_private_IP!="dis" && own_private_IP!="0.0.0.0") {
					CABS.setOwnPrivateIP(own_private_IP);
				}

				if(own_public_IP!="dis" && own_public_IP!="0.0.0.0") {
					CABS.setOwnPublicIP(own_public_IP);
				}
			}
			if(log_filename!="dis" && log_filename!="") {
				//FILE* f_out;
			
				//char filename[strlen(log_filename.c_str())+1];
				//snprintf(filename,sizeof(filename),"%s",log_filename.c_str());
				
				//f_out=fopen(filename,"w");
				//fclose(f_out);
			
				CABS.setLogfile(log_filename);
			}
			
			/* CAM print
			* Unused - left here just for future reference (this comment will be removed in the final deployed version)
			*
			if(cam_sent!="dis" && cam_sent!="") {
				char camFile[strlen(cam_sent.c_str())+1];
				snprintf(camFile,sizeof(camFile),"%s",cam_sent.c_str());
				
				f_out=fopen(camFile,"w");
				fclose(f_out);
			
				CABS.setCAMsent(cam_sent);
				GN.setCAMsentFile(cam_sent);
				BTP.setCAMfile(cam_sent);
			}
			*/
			
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
