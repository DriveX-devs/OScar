#include <iostream>
#include <string>
#include <climits>
#include <cfloat>
#include <unistd.h>
#include <condition_variable>
#include <thread>
#include <atomic>
// TCLAP headers
#include "tclap/CmdLine.h"
// VDP GPS Client
#include "gpsc.h"
// VRUdp
#include "VRUdp.h"
// LDM
#include "LDMmap.h"
// CA Basic Service (TX only)
#include "caBasicService.h"
// VRU Basic service (TX only)
#include "VRUBasicService.h"
// Rx of CAMs and VAMs
#include "SocketClient.h"
// Linux net includes
#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>

#include <sys/socket.h>
#include <net/ethernet.h>
#include <linux/if_packet.h>

#include "etsiDecoderFrontend.h"

#include "JSONserver.h"
#include "vehicle-visualizer.h"

#define DB_CLEANER_INTERVAL_SECONDS 5
#define DB_DELETE_OLDER_THAN_SECONDS 7 // This value should NEVER be set greater than (5-DB_CLEANER_INTERVAL_SECONDS/60) minutes or (300-DB_CLEANER_INTERVAL_SECONDS) seconds - doing so may break the database age check functionality!

#define INVALID_LONLAT -DBL_MAX
#define GNSS_DEFAULT_PORT 3000

// Vehicle visualizer default options
#define DEFAULT_VEHVIZ_NODEJS_UDP_ADDR "127.0.0.1"
#define DEFAULT_VEHVIZ_NODEJS_UDP_PORT 48110
#define DEFAULT_VEHVIZ_WEB_PORT 8080
// Default Vehicle Visualizer web-based GUI update rate, in seconds
#define DEFAULT_VEHVIZ_UPDATE_INTERVAL_SECONDS 0.5

#define DEFAULT_JSON_OVER_TCP_PORT 49000

// Global atomic flag to terminate all the threads in case of errors
std::atomic<bool> terminatorFlag;

// Global pointer to a visualizer object (to be accessed by both DBcleaner_callback() and VehVizUpdater_callback())
vehicleVisualizer* globVehVizPtr=nullptr;

// Global mutex (plus condition variable) to synchronize the threads using the object pointer defined above
std::mutex syncmtx;
std::condition_variable synccv;

// Global flag to tell if the HMI has been enabled
bool enable_hmi = false;

typedef struct vizOptions {
	ldmmap::LDMMap *db_ptr;
	int vehviz_nodejs_port;
	int vehviz_web_interface_port;
	double vehviz_update_interval_sec;
	std::string vehviz_nodejs_addr;
} vizOptions_t;

void clearVisualizerObject(uint64_t id,void *vizObjVoidPtr) {
	vehicleVisualizer *vizObjPtr = static_cast<vehicleVisualizer *>(vizObjVoidPtr);

	vizObjPtr->sendObjectClean(std::to_string(id));
}

void updateVisualizer(ldmmap::vehicleData_t vehdata,void *vizObjVoidPtr) {
	vehicleVisualizer *vizObjPtr = static_cast<vehicleVisualizer *>(vizObjVoidPtr);

	vizObjPtr->sendObjectUpdate(std::to_string(vehdata.stationID),vehdata.lat,vehdata.lon,static_cast<int>(vehdata.stationType),vehdata.heading);
}

void *VehVizUpdater_callback(void *arg) {
	// Get the pointer to the visualizer options/parameters
	vizOptions_t *vizopts_ptr = static_cast<vizOptions_t *>(arg);
	// Get a direct pointer to the database
	ldmmap::LDMMap *db_ptr = vizopts_ptr->db_ptr;

	// Get the central lat and lon values stored in the DB
	std::pair<double,double> centralLatLon= db_ptr->getCentralLatLon();

	// Create a new veheicle visualizer object reading the (IPv4) address and port from the options (the default values are set as a macro in options/options.h)
	vehicleVisualizer vehicleVisObj(vizopts_ptr->vehviz_nodejs_port,vizopts_ptr->vehviz_nodejs_addr);

	// Start the node.js server and perform an initial connection with it
	vehicleVisObj.setHTTPPort(vizopts_ptr->vehviz_web_interface_port);
	vehicleVisObj.startServer();
	vehicleVisObj.connectToServer ();
	vehicleVisObj.sendMapDraw(centralLatLon.first, centralLatLon.second);

	globVehVizPtr=&vehicleVisObj;

	synccv.notify_all();

	// Create a new timer
	struct pollfd pollfddata;
	int clockFd;

	std::cout << "[INFO] Vehicle visualizer updater started. Updated every " << vizopts_ptr->vehviz_update_interval_sec << " seconds." << std::endl;

	if(timer_fd_create(pollfddata, clockFd, vizopts_ptr->vehviz_update_interval_sec*double(1e6))<0) {
		std::cerr << "[ERROR] Fatal error! Cannot create timer for the Vehicle Visualizer update thread!" << std::endl;
		terminatorFlag = true;
		pthread_exit(nullptr);
	}

	POLL_DEFINE_JUNK_VARIABLE();

	while(terminatorFlag == false) {
		if(poll(&pollfddata,1,0)>0) {
			POLL_CLEAR_EVENT(clockFd);

			// ---- These operations will be performed periodically ----

			db_ptr->executeOnAllContents(&updateVisualizer, static_cast<void *>(&vehicleVisObj));

			// --------

		}
	}

	if(terminatorFlag == true) {
		std::cerr << "[WARN] Vehicle visualizer updater terminated due to error." << std::endl;
	}

	close(clockFd);

	pthread_exit(nullptr);
}

void *DBcleaner_callback(void *arg) {
	// Get the pointer to the database
	ldmmap::LDMMap *db_ptr = static_cast<ldmmap::LDMMap *>(arg);

	// Create a new timer
	struct pollfd pollfddata;
	int clockFd;

	std::cout << "[INFO] Database cleaner started. The DB will be garbage collected every " << DB_CLEANER_INTERVAL_SECONDS << " seconds." << std::endl;

	if(timer_fd_create(pollfddata, clockFd, DB_CLEANER_INTERVAL_SECONDS*1e6)<0) {
		std::cerr << "[ERROR] Fatal error! Cannot create timer for the DB cleaner thread!" << std::endl;
		terminatorFlag = true;
		pthread_exit(nullptr);
	}

	std::unique_lock<std::mutex> synclck(syncmtx);

	if(enable_hmi==true) {
		synccv.wait(synclck);
	}

	POLL_DEFINE_JUNK_VARIABLE();

	while(terminatorFlag == false) {
		if(poll(&pollfddata,1,0)>0) {
			POLL_CLEAR_EVENT(clockFd);

			// ---- These operations will be performed periodically ----

			db_ptr->deleteOlderThan(DB_DELETE_OLDER_THAN_SECONDS*1e3);
			//db_ptr->deleteOlderThanAndExecute(DB_DELETE_OLDER_THAN_SECONDS*1e3,clearVisualizerObject,static_cast<void *>(globVehVizPtr));

			// --------
		}
	}

	if(terminatorFlag == true) {
		std::cerr << "[WARN] Database cleaner terminated due to error." << std::endl;
	}

	close(clockFd);

	pthread_exit(nullptr);
}

void txThr(std::string gnss_device,
	int gnss_port,
	bool enable_CAM_dissemination,
	bool enable_VAM_dissemination,
	int sockfd,
	int ifindex,
	uint8_t srcmac[6],
	int VRUID,
	int vehicleID,
	std::string udp_sock_addr,
	std::string udp_bind_ip,
	bool extra_position_udp,
	std::string log_filename_CAM,
	ldmmap::LDMMap *db_ptr,
	std::string log_filename_VAM,
	double pos_th,
	double speed_th,
	double head_th
	) {
	bool m_retry_flag=false;

	// VDP (Vehicle Data Provider) GPS Client object test
	int cnt_CAM = 0;
	VDPGPSClient vdpgpsc(gnss_device,gnss_port);
	
	// VRUdp
	int cnt_VAM = 0;
	VRUdp vrudp(gnss_device,gnss_port);
	
	do {
		m_retry_flag=false;
		
		if(enable_CAM_dissemination){
			try {
				vdpgpsc.openConnection();

				while (cnt_CAM<10) {
					VDPGPSClient::CAM_mandatory_data_t CAMdata;

					std::cout << "[INFO] VDP GPS Client test: getting GNSS data..." << std::endl;

					CAMdata = vdpgpsc.getCAMMandatoryData();

					std::cout << "[INFO] [" << cnt_CAM << "] VDP GPS Client test result: Lat: " << CAMdata.latitude << " deg - Lon: " << CAMdata.longitude << " deg - Heading: " << CAMdata.heading.getValue() << std::endl;

					sleep(1);
					cnt_CAM++;
				}

				std::cout << "[INFO] Dissemination started!" << std::endl;

				CABasicService CABS;
				GeoNet GN;
			
				GN.setVDP(&vdpgpsc);
				GN.setSocketTx(sockfd,ifindex,srcmac);
				GN.setStationProperties(vehicleID,StationType_passengerCar);

				if(udp_sock_addr!="dis") {
					int rval;

					rval=GN.openUDPsocket(udp_sock_addr,udp_bind_ip,extra_position_udp);

					if(rval<0) {
						std::cerr << "Error. Cannot create UDP socket for additional packet transmission. Internal code: " << rval << ". Additional details: " << (errno==0 ? "None" : strerror(errno)) << std::endl;
						close(sockfd);
						exit(EXIT_FAILURE);
					}
				}

				btp BTP;
				BTP.setGeoNet(&GN);

				if(log_filename_CAM!="dis" && log_filename_CAM!="") {
					//FILE* f_out;
			
					//char filename[strlen(log_filename_CAM.c_str())+1];
					//snprintf(filename,sizeof(filename),"%s",log_filename_CAM.c_str());
					
					//f_out=fopen(filename,"w");
					//fclose(f_out);
			
					CABS.setLogfile(log_filename_CAM);
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
				CABS.setLDM(db_ptr);
				CABS.startCamDissemination();
			} catch(const std::exception& e) {
				std::cerr << "Error in creating a new VDP GPS Client connection: " << e.what() << std::endl;
				sleep(5);
				m_retry_flag=true;
			}
		} else if(enable_VAM_dissemination){
			try {
				vrudp.openConnection();

				while (cnt_VAM<10) {
					VAM_mandatory_data_t VAMdata;

					std::cout << "[INFO] VRUdp test: getting GNSS data..." << std::endl;

					VAMdata = vrudp.getVAMMandatoryData();

					std::cout << "[INFO] [" << cnt_VAM << "] VRUdp test results: Lat: " << VAMdata.latitude << " deg - Lon: " << VAMdata.longitude << " deg - Heading: " << VAMdata.heading.getValue() << std::endl;

					sleep(1);
					cnt_VAM++;
				}

				std::cout << "[INFO] Dissemination started!" << std::endl;

				VRUBasicService VBS;
				GeoNet GN;
			
				GN.setVRUdp(&vrudp);
				GN.setSocketTx(sockfd,ifindex,srcmac);
				GN.setStationProperties(VRUID,StationType_pedestrian);

				if(udp_sock_addr!="dis") {
					int rval;

					rval=GN.openUDPsocket(udp_sock_addr,udp_bind_ip,extra_position_udp);

					if(rval<0) {
						std::cerr << "Error. Cannot create UDP socket for additional packet transmission. Internal code: " << rval << ". Additional details: " << (errno==0 ? "None" : strerror(errno)) << std::endl;
						close(sockfd);
						exit(EXIT_FAILURE);
					}
				}

				btp BTP;
				BTP.setGeoNet(&GN);
				
				if(log_filename_VAM!="dis" && log_filename_VAM!="") {
					//FILE* f_out;
			
					//char filename[strlen(log_filename_VAM.c_str())+1];
					//snprintf(filename,sizeof(filename),"%s",log_filename_VAM.c_str());
					
					//f_out=fopen(filename,"w");
					//fclose(f_out);
			
					VBS.setLogfile(log_filename_VAM);
				}
			
				VBS.setBTP(&BTP);
				VBS.setStationProperties(VRUID,StationType_pedestrian);
				VBS.setVRUdp(&vrudp);
				VBS.setLDM(db_ptr);

				// Set triggering conditions threshold
				if(pos_th != -1){
					VBS.setPositionThreshold(pos_th);
				}
				if(speed_th != -1){
					VBS.setSpeedThreshold(speed_th);
				}
				if(head_th != -1){
					VBS.setHeadingThreshold(head_th);
				}

				VBS.startVamDissemination();
			} catch(const std::exception& e) {
				std::cerr << "Error in creating a new VRUdp connection: " << e.what() << std::endl;
				sleep(5);
				m_retry_flag=true;
			}
		}
	} while(m_retry_flag==true && terminatorFlag==false);

	vdpgpsc.closeConnection();
	vrudp.closeConnection();
	terminatorFlag=true;
}

int main (int argc, char *argv[]) {
	std::string dissem_vif = "wlan0";
	std::string log_filename_CAM = "dis";
	std::string log_filename_VAM = "dis";
	std::string log_filename_rcv = "dis";
	std::string gnss_device = "localhost";
	long gnss_port = 3000; // Using 3000 as default port, in our case
	unsigned long vehicleID = 0; // Vehicle ID
	unsigned long VRUID = 0; // VRU ID
	bool enable_CAM_dissemination = false;
	bool enable_VAM_dissemination = false;
	bool enable_DENM_decoding = false;
	bool enable_reception = false;
	bool disable_selfMAC_check = false;
	int json_over_tcp_port = 49000;

	std::string udp_sock_addr = "dis";
	std::string udp_bind_ip = "0.0.0.0";
	bool extra_position_udp = false;

	// VAM triggering conditions thresholds
	double pos_th = -1;
	double speed_th = -1;
	double head_th = -1;
	
	// Options structure for the receiver thread
	options_t rx_opts;
	
	terminatorFlag = false;

	// DB cleaner thread ID
	pthread_t dbcleaner_tid;
	// Vehicle visualizer update thread ID
	pthread_t vehviz_tid;

	// Vehicle Visualizer options
	vizOptions_t vizOpts;

	// Parse the command line options with the TCLAP library
	try {
		TCLAP::CmdLine cmd("The Open CA Basic Service implementatiuon", ' ', "0.3");

		// Arguments: short option, long option, description, is it mandatory?, default value, type indication (just a string to help the user)
		TCLAP::ValueArg<std::string> vifName("I","interface","Broadcast dissemination interface. Default: wlan0.",false,"wlan0","string");
		cmd.add(vifName);
		
		TCLAP::ValueArg<std::string> LogfileCAM("L","log-file-CAM","Print on file the log for the CAM condition checks. Default: (disabled).",false,"dis","string");
		cmd.add(LogfileCAM);
		
		TCLAP::ValueArg<std::string> LogfileVAM("F","log-file-VAM","Print on file the log for the VAM condition checks. Default: (disabled).",false,"dis","string");
		cmd.add(LogfileVAM);
		
		TCLAP::ValueArg<std::string> LogfileReception("R","log-file-Reception","Print on file the data retrieved from CAM/VAM/DENM reception. Default: (disabled).",false,"dis","string");
		cmd.add(LogfileReception);

		TCLAP::ValueArg<std::string> GNSSDevArg("D","gnss-device","GNSS device to be used (i.e., where gpsd is currently running - this is not the /dev/ttyACM* device, which is already being used by gpsd, which in turn can provide the GNSS data to OCABS). Default: localhost.",false,"localhost","string");
		cmd.add(GNSSDevArg);

		TCLAP::ValueArg<long> GNSSPortArg("P","gnss-port","Port to be used to connect to the GNSS device. It should correspond to the port used by gpsd for the desired receiver. Warning! The default port for gpsd is 2947, while the default for OCABS is 3000.",false,GNSS_DEFAULT_PORT,"integer");
		cmd.add(GNSSPortArg);

		TCLAP::ValueArg<unsigned long> VehicleIDArg("v","vehicle-id","CA Basic Service Station ID",false,0,"unsigned integer");
		cmd.add(VehicleIDArg);
		
		TCLAP::ValueArg<unsigned long> VRUIDArg("r","VRU-id","VRU Basic Service Station ID",false,1000,"unsigned integer");
		cmd.add(VRUIDArg);
		
		TCLAP::SwitchArg CAMsDissArg("C","enable-CAMs-dissemination","Enable the dissemination of standard CAMs",false);
		cmd.add(CAMsDissArg);
		
		TCLAP::SwitchArg VAMsDissArg("V","enable-VAMs-dissemination","Enable the dissemination of VAMs",false);
		cmd.add(VAMsDissArg);
		
		TCLAP::SwitchArg DENMsDecArg("d","enable-DENMs-decoding","Enable the decoding of DENMs",false);
		cmd.add(DENMsDecArg);

		TCLAP::ValueArg<std::string> UDPSockAddrArg("u","udp-sock-addr","If specified, OCABS, in addition to the standard-compliant CAM dissemination, will also encapsulate each CAM inside UDP, and send these messages to the address (in the form <IP:port>) specified after this options.",false,"dis","string");
		cmd.add(UDPSockAddrArg);

		TCLAP::ValueArg<std::string> UDPBindIPArg("U","udp-bind-ip","This options is valid only if --udp-sock-addr/-u has been specified. It can be used to set an interface/address to bind the UDP socket to. By default, no specific address is used for binding (i.e., binding to any address/interface).",false,"0.0.0.0","string");
		cmd.add(UDPBindIPArg);

		TCLAP::SwitchArg ExtraPosUDPArg("X","add-extra-position-udp","This options is valid only if --udp-sock-addr/-u has been specified. If specified, this option will make OCABS add, before the actual CAM payload of each UDP packets, 64 extra bits, contatining the current latitude and longitude (32 bits each), in network byte order and stored as degrees*1e7.",false);
		cmd.add(ExtraPosUDPArg);

		TCLAP::ValueArg<double> POS_threshold("p","Position-threshold","VAM position triggering condition threshold",false,-1,"double");
		cmd.add(POS_threshold);

		TCLAP::ValueArg<double> SPEED_threshold("s","Speed-threshold","VAM speed triggering condition threshold",false,-1,"double");
		cmd.add(SPEED_threshold);

		TCLAP::ValueArg<double> HEAD_threshold("e","Heading-threshold","VAM heading triggering condition threshold",false,-1,"double");
		cmd.add(HEAD_threshold);

		TCLAP::SwitchArg EnableRxArg("x","enable-reception","Enable the reception of messages and the LDM",false);
		cmd.add(EnableRxArg);

		// Vehicle Visualizer options
		TCLAP::ValueArg<long> VV_NodejsPortArg("1","vehviz-nodejs-port","Advanced option: set the port number for the UDP connection to the Vehicle Visualizer Node.js server",false,DEFAULT_VEHVIZ_NODEJS_UDP_PORT,"integer");
		cmd.add(VV_NodejsPortArg);

		TCLAP::ValueArg<long> VV_WebInterfacePortArg("2","vehviz-web-interface-port","set the port at which the web interface of the Vehicle Visualizer will be available",false,DEFAULT_VEHVIZ_WEB_PORT,"integer");
		cmd.add(VV_WebInterfacePortArg);

		TCLAP::ValueArg<double> VV_UpdateIntervalArg("3","vehviz-update-interval-sec",
			"Advanced option: this option can be used to modify the update rate of the web-based GUI. "
			"Warning: decreasing too much this value will affect the LDM database performance!"
			"This value cannot be less than 0.05 s and more than 1 s.",
			false,DEFAULT_VEHVIZ_UPDATE_INTERVAL_SECONDS,"double");
		cmd.add(VV_UpdateIntervalArg);

		TCLAP::ValueArg<std::string> VV_NodejsAddrArg("4","vehviz-nodejs-addr",
			"Advanced option: set the IPv4 address for the UDP connection to the Vehicle Visualizer Node.js server (excluding the port number). "
			"This is the address without port number.",
			false,DEFAULT_VEHVIZ_NODEJS_UDP_ADDR,"IPv4 address string");
		cmd.add(VV_NodejsAddrArg);

		TCLAP::SwitchArg EnableHMIArg("m","enable-HMI","Enable the OScar HMI",false);
		cmd.add(EnableHMIArg);

		TCLAP::SwitchArg DisableSelfMACArg("T","disable-self-MAC-check","Debugging option: disable the self MAC check: if this option is set, "
			"OScar will likely receive also the messages sent by itself (i.e., it will receive messages with a MAC address equal to its own too). Useful for debugging.",false);
		cmd.add(DisableSelfMACArg);

		TCLAP::ValueArg<long> JSONserverPortArg("j","ldm-json-server-port","Set the port for on-demand JSON-over-TCP requests to the LDM.",false,DEFAULT_JSON_OVER_TCP_PORT,"integer");
		cmd.add(JSONserverPortArg);

		cmd.parse(argc,argv);

		dissem_vif=vifName.getValue();
		
		log_filename_CAM=LogfileCAM.getValue();
		log_filename_VAM=LogfileVAM.getValue();
		log_filename_rcv=LogfileReception.getValue();

		gnss_device=GNSSDevArg.getValue();
		gnss_port=GNSSPortArg.getValue();

		vehicleID=VehicleIDArg.getValue();
		VRUID=VRUIDArg.getValue();

		enable_CAM_dissemination=CAMsDissArg.getValue();
		enable_VAM_dissemination=VAMsDissArg.getValue();
		enable_DENM_decoding=DENMsDecArg.getValue();

		udp_sock_addr=UDPSockAddrArg.getValue();
		udp_bind_ip=UDPBindIPArg.getValue();
		extra_position_udp=ExtraPosUDPArg.getValue();
		
		rx_opts.gnss_device = gnss_device;
		rx_opts.dissemination_device = dissem_vif;

		pos_th = POS_threshold.getValue();
		speed_th = SPEED_threshold.getValue();
		head_th = HEAD_threshold.getValue();

		enable_reception = EnableRxArg.getValue();

		vizOpts.vehviz_nodejs_port=VV_NodejsPortArg.getValue();
		vizOpts.vehviz_web_interface_port=VV_WebInterfacePortArg.getValue();
		vizOpts.vehviz_update_interval_sec=VV_UpdateIntervalArg.getValue();
		vizOpts.vehviz_nodejs_addr=VV_NodejsAddrArg.getValue();

		if(vizOpts.vehviz_update_interval_sec<0.05 || vizOpts.vehviz_update_interval_sec>1) {
			std::cerr << "[Error] The Vehicle Visualizer update interval cannot be lower than 0.05 s or grater than 1 second." << std::endl;
			return 1;
		}

		enable_hmi=EnableHMIArg.getValue();
		disable_selfMAC_check=DisableSelfMACArg.getValue();
		json_over_tcp_port=JSONserverPortArg.getValue();

		std::cout << "[INFO] CAM/VAM dissemination interface: " << dissem_vif << std::endl;
	} catch (TCLAP::ArgException &tclape) { 
		std::cerr << "TCLAP error: " << tclape.error() << " for argument " << tclape.argId() << std::endl;

		return 1;
	}

	if(udp_bind_ip!="0.0.0.0" && udp_sock_addr=="dis") {
		std::cerr << "Error. --udp-bind-ip/-U can only be specified when --udp-sock-addr/-u is specified too." << std::endl;

		return 1;
	}

	if(extra_position_udp==true && udp_sock_addr=="dis") {
		std::cerr << "Error. --add-extra-position-udp/-X can only be specified when --udp-sock-addr/-u is specified too." << std::endl;

		return 1;
	}

	// Create the raw socket for the transmission of CAMs/VAMs, encapsulated inside GeoNetworking and BTP (in user space) 
	int sockfd=-1;
	// sockfd=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP); // Old UDP socket, no more used
	sockfd=socket(AF_PACKET,SOCK_RAW,htons(ETH_P_ALL));

	if(sockfd<0) {
		std::cerr << "Critical error: cannot open raw socket for CAM dissemination. Details: " << strerror(errno) << "\n" << std::endl;
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
	
	// Create a new DB object
	ldmmap::LDMMap *db_ptr = new ldmmap::LDMMap();
	
	// We have to create a thread reading periodically (e.g. every 5 s) the database through the pointer "db_ptr" and "cleaning" the entries
	// which are too old
	pthread_create(&dbcleaner_tid,NULL,DBcleaner_callback,(void *) db_ptr);

	if(enable_hmi==true) {
		// If the HMI has been enabled, we should also start here a second parallel thread, reading periodically the database (e.g. every 500 ms) and sending the vehicle data to
		// the vehicleVisualizer
		// pthread_attr_init(&tattr);
		// pthread_attr_setdetachstate(&tattr,PTHREAD_CREATE_DETACHED);
		vizOpts.db_ptr=db_ptr;
		pthread_create(&vehviz_tid,NULL,VehVizUpdater_callback,(void *) &vizOpts);
		// pthread_attr_destroy(&tattr);
	}

	// Transmission loop
	std::thread txThrObj(txThr,
		gnss_device,
		gnss_port,
		enable_CAM_dissemination,
		enable_VAM_dissemination,
		sockfd,
		ifindex,
		srcmac,
		VRUID,
		vehicleID,
		udp_sock_addr,
		udp_bind_ip,
		extra_position_udp,
		log_filename_CAM,
		db_ptr,
		log_filename_VAM,
		pos_th,
		speed_th,
		head_th);
	
	// Reception loop (using the main thread)
	if(enable_reception==true) {
		fprintf(stdout,"Configuring socket for reception. Descriptor: %d\n",sockfd);

		if(terminatorFlag==false) {
			// Create the main SocketClient object for the reception of the V2X messages
			SocketClient mainRecvClient(sockfd,&rx_opts, db_ptr, log_filename_rcv);
			
			if(enable_DENM_decoding) {
				mainRecvClient.enableDENMdecoding();
			}

			// Set the "self" MAC address, so that all the messages coming from this address will be discarded
			if(disable_selfMAC_check==false) {
				mainRecvClient.setSelfMAC(srcmac);
			}

			// Create an additional 
			VDPGPSClient logginggpsc(gnss_device,gnss_port);
			logginggpsc.openConnection();
			mainRecvClient.setLoggingGNSSClient(&logginggpsc);

			// Before starting the data reception, create a new JSONserver object for client to retrieve the DB data
			JSONserver jsonsrv(db_ptr);
			jsonsrv.setServerPort(json_over_tcp_port);
			if(jsonsrv.startServer()!=true) {
				fprintf(stderr,"Critical error: cannot start the JSON server for the client data retrieval.\n");
				terminatorFlag=true;
				goto exit_failure;
			}

			fprintf(stdout,"Reception is going to start very soon...\n");

			// Start the reception of V2X messages
			mainRecvClient.startReception();

			jsonsrv.stopServer();
			logginggpsc.closeConnection();
		}
	}
	
	exit_failure:

	terminatorFlag=true;

	txThrObj.join();
	
	pthread_join(dbcleaner_tid,nullptr);

	if(enable_hmi==true) {
		pthread_join(vehviz_tid,nullptr);
	}

	db_ptr->clear();

	// Close the socket
	close(sockfd);

	return 0;
}
