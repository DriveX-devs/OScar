#include <iostream>
#include <string>
#include <cfloat>
#include <unistd.h>
#include <condition_variable>
#include <thread>
#include <atomic>
// TCLAP headers
#include "tclap/CmdLine.h"
// VDP GPS Client + VRUdp
#include "gpsc.h"
// LDM
#include "LDMmap.h"
// CA Basic Service (TX only)
#include "caBasicService.h"
// VRU Basic service (TX only)
#include "VRUBasicService.h"
// CP Basic service (TX only)
#include "cpBasicService.h"
// Rx of CAMs and VAMs
#include "SocketClient.h"
// Sensor reader
#include "basicSensorReader.h"
// CAN database reader
#include "dbcReader.h"
// Certificate manager
#include <ECManager.h>
#include <ATManager.h>

// Linux net includes
#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>

#include <sys/socket.h>
#include <net/ethernet.h>
#include <linux/if_packet.h>

// Linux net include for the vehicle data tx thread
#include <netdb.h>

#include "JSONserver.h"
#include "vehicle-visualizer.h"

#include "DCC.h"

#include "MetricSupervisor.h"

#define DB_CLEANER_INTERVAL_SECONDS 5
#define DB_DELETE_OLDER_THAN_SECONDS 2 // This value should NEVER be set greater than (5-DB_CLEANER_INTERVAL_SECONDS/60) minutes or (300-DB_CLEANER_INTERVAL_SECONDS) seconds - doing so may break the database age check functionality!

#define INVALID_LONLAT -DBL_MAX
#define GNSS_DEFAULT_PORT 3000

// Vehicle visualizer default options
#define DEFAULT_VEHVIZ_NODEJS_UDP_ADDR "127.0.0.1"
#define DEFAULT_VEHVIZ_NODEJS_UDP_PORT 48110
#define DEFAULT_VEHVIZ_WEB_PORT 8080
// Default Vehicle Visualizer web-based GUI update rate, in seconds
#define DEFAULT_VEHVIZ_UPDATE_INTERVAL_SECONDS 0.5

#define DEFAULT_JSON_OVER_TCP_PORT 49000

#define MAXIMUM_TIME_WINDOW_DCC 2000

#define BITRATE 3e6

// Global atomic flag to terminate all the threads in case of errors
std::atomic<bool> terminatorFlag;

// Global serial parser object
UBXNMEAParserSingleThread serialParser;

// Global pointer to a visualizer object (to be accessed by both DBcleaner_callback() and VehVizUpdater_callback())
vehicleVisualizer* globVehVizPtr=nullptr;

// Global mutex (plus condition variable) to synchronize the threads using the object pointer defined above
std::mutex syncmtx;
std::condition_variable synccv;

// Global flag to tell if the HMI has been enabled
bool enable_hmi = true;

//CABasicService* cabs;
//CPBasicService* cpbs;
//VRUBasicService* vrubs;

// Structure to store the options for the vehicle visualizer
typedef struct vizOptions {
	ldmmap::LDMMap *db_ptr;
	int vehviz_nodejs_port;
	int vehviz_web_interface_port;
	double vehviz_update_interval_sec;
	std::string vehviz_nodejs_addr;
    StationType_t ego_station_type;
} vizOptions_t;

void clearVisualizerObject(uint64_t id,void *vizObjVoidPtr) {
	vehicleVisualizer *vizObjPtr = static_cast<vehicleVisualizer *>(vizObjVoidPtr);

    // As we are dereferencing vizObjPtr, sendObjectClean() must be executed only when the HMI is active and vizObjPtr is not a nullptr
    if(vizObjVoidPtr!=nullptr) {
        vizObjPtr->sendObjectClean(std::to_string(id));
    }
}

void updateVisualizer(ldmmap::vehicleData_t vehdata,void *vizObjVoidPtr) {
	vehicleVisualizer *vizObjPtr = static_cast<vehicleVisualizer *>(vizObjVoidPtr);

	vizObjPtr->sendObjectUpdate(std::to_string(vehdata.stationID),vehdata.lat,vehdata.lon,static_cast<int>(vehdata.stationType),vehdata.heading);
}

// Thread function for updating the objects displayed in the web-based vehicle visualizer GUI of OScar
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
            auto db_retval = db_ptr->updateEgoPosition(vizopts_ptr->ego_station_type);
            if(db_retval == ldmmap::LDMMap::LDMMAP_NO_VDP) {
                std::cerr << "[ERROR] Cannot update the ego position in the LDM: VDP not set." << std::endl;
                terminatorFlag = true;
                break;
            } else if(db_retval == ldmmap::LDMMap::LDMMAP_INVALID_STATIONTYPE) {
                std::cerr << "[ERROR] Cannot update the ego position in the LDM: invalid station type. This is a bug. Please inform the developers." << std::endl;
                terminatorFlag = true;
                break;
            }

            // Dirty workaround to 'flush' gps data TODO: remove this
//            for (int i = 0; i < 20; i++) {
//                db_ptr->updateEgoPosition();
//            }
            db_ptr->updateEgoPosition(vizopts_ptr->ego_station_type);
            ////////////////////////////////////////

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

// Thread function that periodically checks the LDM database content and clears outdated entries
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

            // Beware! This function may be called with a nullptr as globVehVizPtr if the reception is enabled but the HMI is disabled; clearVisualizerObject() must handle this case!
			db_ptr->deleteOlderThanAndExecute(DB_DELETE_OLDER_THAN_SECONDS*1e3,clearVisualizerObject,static_cast<void *>(globVehVizPtr));

			// --------
		}
	}

	if(terminatorFlag == true) {
		std::cerr << "[WARN] Database cleaner terminated due to error." << std::endl;
	}

	close(clockFd);

	pthread_exit(nullptr);
}

// Main CAM transmission thread
void CAMtxThr(std::string gnss_device,
        int gnss_port,
        int sockfd,
        int ifindex,
        uint8_t srcmac[6],
        int vehicleID,
        std::string udp_sock_addr,
        std::string udp_bind_ip,
        bool extra_position_udp,
        std::string log_filename_CAM,
        std::string log_filename_GNsecurity,
        ldmmap::LDMMap *db_ptr,
        ATManager *atManager,
        double pos_th,
        double speed_th,
        double head_th,
        bool rx_enabled,
        bool use_gpsd,
        bool enable_security,
        bool force_20Hz_freq,
        CABasicService* cabs,
        DCC *dcc
    ) {
    bool m_retry_flag=false;

    // VDP (Vehicle Data Provider) GPS Client object test
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-variable"
    int cnt_CAM = 0;
    int cnt_CPM = 0;
    #pragma GCC diagnostic pop

    VDPGPSClient vdpgpsc(gnss_device,gnss_port);
    vdpgpsc.selectGPSD(use_gpsd);
    if(!use_gpsd) {
        vdpgpsc.setSerialParser(&serialParser);
    }

    GeoNet GN;
    btp BTP;

    double test_lat, test_lon;

    do {
        try {
            std::cout << "[INFO] CAM VDP GPS Client: opening connection..." << std::endl;
            vdpgpsc.openConnection();

            GN.setVDP(&vdpgpsc);
            GN.setSocketTx(sockfd, ifindex, srcmac);
            GN.setStationProperties(vehicleID, StationType_passengerCar);
            GN.setSecurity(enable_security);
            const int message_type = 1;
            GN.setMessageType(message_type);
            if (enable_security && atManager != nullptr) {
                GN.setATmanager(atManager);
            }
            if(enable_security && log_filename_GNsecurity != "dis" && !log_filename_GNsecurity.empty()){
                GN.setLogFile2(log_filename_GNsecurity);
            }
            GN.setDCC(dcc);
            GN.attachDCC();
            BTP.setGeoNet(&GN);
            
            while (true) {
                VDPGPSClient::CAM_mandatory_data_t CAMdata;

                std::cout << "[INFO] VDP GPS Client test: getting GNSS data..." << std::endl;

                CAMdata = vdpgpsc.getCAMMandatoryData();
                test_lat = CAMdata.latitude/1e7;
                test_lon = CAMdata.longitude/1e7;

                if(test_lat>=-90.0 && test_lat<=90.0 && test_lon>=-180.0 && test_lon<=180.0) {
                    std::cout << "[INFO] Position available after roughly " << cnt_CAM << " seconds: latitude: " << test_lat << " - longitude: " << test_lon << std::endl;
                    break;
                } else {
                    std::cout << "[INFO] Position not yet available. Unavail. value: " << CAMdata.latitude << "," << CAMdata.longitude << ". Waiting 1 second and trying again..." << std::endl;
                }

                std::cout << "[INFO] Waiting for VDP to provide the position (a fix may not be yet available)..." << std::endl;
                
                sleep(1);
                cnt_CAM++;
            }

            std::cout << "[INFO] CAM Dissemination started!" << std::endl;

            if (udp_sock_addr != "dis") {
                int rval;

                rval = GN.openUDPsocket(udp_sock_addr, udp_bind_ip, extra_position_udp);

                if (rval < 0) {
                    std::cerr << "Error. Cannot create UDP socket for additional packet transmission. Internal code: "
                              << rval << ". Additional details: " << (errno == 0 ? "None" : strerror(errno))
                              << std::endl;
                    close(sockfd);
                    exit(EXIT_FAILURE);
                }
            }
        
            cabs->setBTP(&BTP);
            cabs->setStationProperties(vehicleID, StationType_passengerCar);
            cabs->setVDP(&vdpgpsc);
            cabs->setLDM(db_ptr);

            if(force_20Hz_freq) {
                cabs->force20HzFreq();
            }

            if (log_filename_CAM != "dis" && log_filename_CAM != "") {
                cabs->setLogfile(log_filename_CAM);
            }
            // Start the CAM dissemination
            // TODO: stop dissemination when the terminator flag becomes true
            // TODO: we should call CABS.terminateDissemination() or pass directly a pointer to the global atomic terminatorFlag to the CA Basic Service
            cabs->startCamDissemination();

        } catch (const std::exception &e) {
            std::cerr << "Error in creating a new VDP GPS Client connection: " << e.what() << std::endl;
            sleep(5);
            m_retry_flag = true;

        }
    }while(m_retry_flag==true && terminatorFlag==false);

    vdpgpsc.closeConnection();
    terminatorFlag=true;

}

// Main CPM transmission thread
void CPMtxThr(std::string gnss_device,
            int gnss_port,
            int sockfd,
            int ifindex,
            uint8_t srcmac[6],
            int vehicleID,
            std::string udp_sock_addr,
            std::string udp_bind_ip,
            bool extra_position_udp,
            std::string log_filename_CPM,
            std::string log_filename_GNsecurity,
            ldmmap::LDMMap *db_ptr,
            bool use_gpsd,
            double check_faulty_object_acceleration,
            bool disable_cpm_speed_triggering,
            bool verbose,
            bool enable_security,
            ATManager *atManager,
            CPBasicService* cpbs,
            DCC *dcc
        ) {
    bool m_retry_flag=false;

    VDPGPSClient vdpgpsc(gnss_device,gnss_port);
    vdpgpsc.selectGPSD(use_gpsd);

    if(!use_gpsd) {
        vdpgpsc.setSerialParser(&serialParser);
    }

    std::cout << use_gpsd << std::endl;

    VDPGPSClient vrudp(gnss_device,gnss_port);
    GeoNet GN;
    btp BTP;

    do {
        try {
            std::cout << "[INFO] CAM VDP GPS Client: opening connection..." << std::endl;
            vdpgpsc.openConnection();

            GN.setVDP(&vdpgpsc);
            GN.setSocketTx(sockfd, ifindex, srcmac);
            GN.setStationProperties(vehicleID, StationType_passengerCar);
            GN.setSecurity(enable_security);
            const int message_type = 2;
            GN.setMessageType(message_type);
            if (enable_security && atManager != nullptr) {
                GN.setATmanager(atManager);
            }
            if(enable_security && log_filename_GNsecurity != "dis" && !log_filename_GNsecurity.empty()){
                GN.setLogFile2(log_filename_GNsecurity);
            }
            GN.setDCC(dcc);
            GN.attachDCC();
            BTP.setGeoNet(&GN);


            std::cout << "[INFO] CPM Dissemination started!" << std::endl;

            if (udp_sock_addr != "dis") {
                int rval;

                rval = GN.openUDPsocket(udp_sock_addr, udp_bind_ip, extra_position_udp);

                if (rval < 0) {
                    std::cerr << "Error. Cannot create UDP socket for additional packet transmission. Internal code: "
                              << rval << ". Additional details: " << (errno == 0 ? "None" : strerror(errno))
                              << std::endl;
                    close(sockfd);
                    exit(EXIT_FAILURE);
                }
            }
            
            cpbs->setBTP(&BTP);
            cpbs->setStationProperties(vehicleID, StationType_passengerCar);
            cpbs->setVDP(&vdpgpsc);
            cpbs->setLDM(db_ptr);
            cpbs->setVerbose(verbose);
            if (check_faulty_object_acceleration != 0.0)
                cpbs->setFaultyAccelerationCheck(check_faulty_object_acceleration);
            cpbs->setSpeedTriggering(!disable_cpm_speed_triggering);
            if (log_filename_CPM != "dis" && log_filename_CPM != "") {
                cpbs->setLogfile(log_filename_CPM);
            }
            // Start the CAM dissemination
            cpbs->initDissemination();

        } catch (const std::exception &e) {
            std::cerr << "Error in creating a new VDP GPS Client connection: " << e.what() << std::endl;
            sleep(5);
            m_retry_flag = true;

        }
    }while(m_retry_flag==true && terminatorFlag==false);

    vdpgpsc.closeConnection();
    vrudp.closeConnection();
    terminatorFlag=true;
}

// Main VAM transmission thread
void VAMtxThr(std::string gnss_device,
            int gnss_port,
            int sockfd,
            int ifindex,
            uint8_t srcmac[6],
            int VRUID,
            std::string udp_sock_addr,
            std::string udp_bind_ip,
            bool extra_position_udp,
            std::string log_filename_VAM,
            std::string log_filename_GNsecurity,
            ldmmap::LDMMap *db_ptr,
            double pos_th,
            double speed_th,
            double head_th,
            bool use_gpsd,
            bool enable_security,
            ATManager *atManager,
            VRUBasicService* vrubs,
            DCC *dcc
        ) {
    bool m_retry_flag=false;

    VDPGPSClient vrudp(gnss_device,gnss_port);
    vrudp.selectGPSD(use_gpsd);

    if(!use_gpsd) {
        vrudp.setSerialParser(&serialParser);
    }

    do {
        try {
            vrudp.openConnection();

            int vam_cnt_test=0;
            while (true) {
                VDPGPSClient::VRU_position_latlon_t pos;

                pos=vrudp.getPedPosition();

                if(pos.lat>=-90.0 && pos.lat<=90.0 && pos.lon>=-180.0 && pos.lon <= 180.0) {
                    std::cout << "[INFO] Position available after roughly " << vam_cnt_test << " seconds: latitude: " << pos.lat << " - longitude: " << pos.lon << std::endl;
                    break;
                } else {
                    std::cout << "[INFO] Position not yet available. Unavail. value: " << pos.lat << ". Waiting 1 second and trying again..." << std::endl;
                }

                std::cout << "[INFO] Waiting for the VRU Data Provider to provide the position (a fix may not be yet available)..." << std::endl;

                sleep(1);
                vam_cnt_test++;
            }

            std::cout << "[INFO] VAM Dissemination started!" << std::endl;

            GeoNet GN;

            GN.setVRUdp(&vrudp);
            GN.setSocketTx(sockfd,ifindex,srcmac);
            GN.setStationProperties(VRUID,StationType_pedestrian);
            GN.setSecurity(enable_security);
            const int message_type = 3;
            GN.setMessageType(message_type);
            if (enable_security && atManager != nullptr) {
                GN.setATmanager(atManager);
            }
            if(enable_security && log_filename_GNsecurity!="dis" && !log_filename_GNsecurity.empty()) {
                GN.setLogFile2(log_filename_GNsecurity);
            }
            GN.setDCC(dcc);
            GN.attachDCC();

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
                vrubs->setLogfile(log_filename_VAM);
            }

            vrubs->setBTP(&BTP);
            vrubs->setStationProperties(VRUID,StationType_pedestrian);
            vrubs->setVRUdp(&vrudp);
            vrubs->setLDM(db_ptr);

            // Set triggering conditions threshold
            if(pos_th != -1){
                vrubs->setPositionThreshold(pos_th);
            }
            if(speed_th != -1){
                vrubs->setSpeedThreshold(speed_th);
            }
            if(head_th != -1){
                vrubs->setHeadingThreshold(head_th);
            }

            vrubs->startVamDissemination();

        } catch(const std::exception& e) {
            std::cerr << "Error in creating a new VRUdp connection: " << e.what() << std::endl;
            sleep(5);
            m_retry_flag=true;
        }
} while(m_retry_flag==true && terminatorFlag==false);

    vrudp.closeConnection();
    terminatorFlag=true;
}

// Main sensor reader thread
void radarReaderThr(std::string gnss_device,
                    long gnss_port,
                    std::string can_device,
                    ldmmap::LDMMap *db_ptr,
                    uint64_t vehicleID,
                    bool use_gpsd,
                    bool enable_sensor_classification,
                    bool verbose,
                    CAN_SENSOR_SIGNAL_INFO_t can_db_sensor_info,
                    std::vector<uint32_t> can_db_id_info) {
    bool m_retry_flag=false;

    do {
        try {
            VDPGPSClient sensorgpsc(gnss_device,gnss_port);
            sensorgpsc.selectGPSD(use_gpsd);

            if(!use_gpsd) {
                sensorgpsc.setSerialParser(&serialParser);
            }

            sensorgpsc.openConnection();
            BasicSensorReader sensorReader(can_device, db_ptr, &sensorgpsc, vehicleID);
            sensorReader.setCANdbInfo(can_db_sensor_info,can_db_id_info);
            sensorReader.setEnableClassification(enable_sensor_classification);
            sensorReader.setVerbose(verbose);
            sensorReader.startReader();
        } catch(const std::exception& e) {
            std::cerr << "Error in creating a new VDP GPS Client connection: " << e.what() << std::endl;
            sleep(5);
            m_retry_flag=true;
        }
    } while(m_retry_flag==true && terminatorFlag==false);

    terminatorFlag=true;
}

void vehdataTxThread(std::string udp_sock_addr,
              double periodicity_s,
              std::string gnss_device,
             int gnss_port,
             bool use_gpsd) {
    VDPGPSClient vdpgpsc(gnss_device,gnss_port);
    vdpgpsc.selectGPSD(use_gpsd);
    if(!use_gpsd) {
        vdpgpsc.setSerialParser(&serialParser);
    }

    // Open UDP socket
    size_t delimiter_pos=udp_sock_addr.find(":");
    std::string dest_ip=udp_sock_addr.substr(0, delimiter_pos);
    udp_sock_addr.erase(0,delimiter_pos+1);
    long dest_port=strtol(udp_sock_addr.c_str(),nullptr,0);

    if(udp_sock_addr=="dis") {
        std::cerr << "[ERROR] Fatal error! Cannot start the vehicle data transmission thread: no destination address specified!" << std::endl;
        terminatorFlag = true;
        return;
    }

    int udp_sockfd=socket(AF_INET,SOCK_DGRAM,0);

    if(udp_sockfd<0) {
        std::cerr << "[ERROR] Fatal error! Cannot create UDP socket for the vehicle data transmission thread!" << std::endl;
        terminatorFlag = true;
        return;
    }

    // Generic size of a struct sockaddr_in (used multiple times below)
    socklen_t addrlen = sizeof(struct sockaddr_in);

    struct sockaddr_in bind_address;
    memset(&bind_address,0,addrlen);
    bind_address.sin_family = AF_INET;
    // By default, do not bind to any specific address/interface
    bind_address.sin_addr.s_addr = INADDR_ANY;
    bind_address.sin_port = htons(0);

    if(bind(udp_sockfd,(struct sockaddr*) &bind_address,addrlen)<0) {
        std::cerr << "[ERROR] Fatal error! Cannot bind UDP socket for the vehicle data transmission thread!" << std::endl;
        terminatorFlag = true;
        close(udp_sockfd);
        return;
    }

    // "connect" the UDP socket (i.e., set the default destination address and port)
    struct sockaddr_in dest_address;
    memset(&dest_address,0,addrlen);
    dest_address.sin_family = AF_INET;

    if(dest_ip=="0.0.0.0" || inet_pton(AF_INET,dest_ip.c_str(),&dest_address.sin_addr)<1) {
        std::cout << "[ERROR] " << dest_ip << " does not appear to be a valid destination IP address for the UDP packet. Attempting address resolution..." << std::endl;
        // Attempt to resolve host name if inet_pton fails (maybe the user specified a name and not an IP address?)
        if(dest_ip!="0.0.0.0") {
            struct hostent *hostaddrs;
            struct in_addr **addr_list;

            hostaddrs=gethostbyname(dest_ip.c_str());

            if(hostaddrs==nullptr) {
                herror("[ERROR] Address resolution failed for UDP destination address. Details");
                terminatorFlag = true;
                close(udp_sockfd);
                return;
            }

            addr_list=(struct in_addr **)hostaddrs->h_addr_list;

            // Gather the first address corresponding to the given name
            if(addr_list[0]==nullptr) {
                std::cerr << "[ERROR] Address resolution failed for " << dest_ip << ". No IP addresses found for given host name." << std::endl;
                terminatorFlag = true;
                close(udp_sockfd);
                return;
            } else {
                dest_address.sin_addr=*addr_list[0];
            }
        } else {
            std::cerr << "[ERROR] Generic address resolution error for " << dest_ip << "." << std::endl;
            terminatorFlag = true;
            close(udp_sockfd);
            return;
        }
    }

    dest_address.sin_port = htons(dest_port);

    if(connect(udp_sockfd,(struct sockaddr*) &dest_address,addrlen)<0) {
        std::cerr << "[ERROR] Fatal error! Cannot set destination address for UDP socket for the vehicle data transmission thread!" << std::endl;
        terminatorFlag = true;
        close(udp_sockfd);
        return;
    }

    // Create a new timer
    struct pollfd pollfddata;
    int clockFd;

    std::cout << "[INFO] Vehicle data transmission thread started. The vehicle data will be sent to " << dest_ip << ":" << dest_port <<  " every " << periodicity_s << " seconds." << std::endl;

    if(timer_fd_create(pollfddata, clockFd, periodicity_s*1e6)<0) {
        std::cerr << "[ERROR] Fatal error! Cannot create timer for the vehicle data transmission thread!" << std::endl;
        terminatorFlag = true;
        return;
    }

    POLL_DEFINE_JUNK_VARIABLE();

    while(terminatorFlag == false) {
        if(poll(&pollfddata,1,0)>0) {
            POLL_CLEAR_EVENT(clockFd);

            auto vehdata = vdpgpsc.getCAMMandatoryData();
            double lat = vehdata.latitude/1e7; // [deg]
            double lon = vehdata.longitude/1e7; // [deg]
            double altitude = vehdata.altitude.getValue()/100.0; // [m]
            double speed = vehdata.speed.getValue()/100.0; // [m/s]
            double heading = vehdata.heading.getValue()/10.0; // [deg]
            double acceleration = vehdata.longAcceleration.getValue()/10.0; // [m/s^2]
            double yawRate = vehdata.yawRate.getValue()/100.0; // [deg/s]

            // Check if at least the position is available; if not, do not send any message and iterate again
            time_t seconds;
            uint64_t microseconds;
            struct timespec now;
            double rt_tstamp_ms;

            if(clock_gettime(CLOCK_REALTIME, &now) == -1) {
                perror("[WARN] Cannot get the current microseconds UTC timestamp");
                rt_tstamp_ms = 0;
            }

            seconds=now.tv_sec;
            microseconds=round(now.tv_nsec/1e3);

            // milliseconds, due to the rounding operation, shall not exceed 999999
            if(microseconds > 999999) {
                seconds++;
                microseconds=0;
            }

            rt_tstamp_ms = (seconds*1000000.0+microseconds)/1000.0;

            if(lat>=-90.0 && lat<=90.0 && lon>=-180.0 && lon<=180.0) {
                // Save all the gathered data in a human-readable string to be sent via UDP socket
                std::string veh_data = "timestamp_monotonic=" + std::to_string(static_cast<double>(get_timestamp_us())/1000.0) +
                                       ",timestamp_realtime=" + std::to_string(rt_tstamp_ms) +
                                       ",lat=" + std::to_string(lat) +
                                       ",lon=" + std::to_string(lon) +
                                       ",speed=" + std::to_string(speed) +
                                       ",heading=" + std::to_string(heading) +
                                       ",acceleration=" + std::to_string(acceleration) +
                                       ",yawRate=" + std::to_string(yawRate);

                // Send the data via UDP socket
                ssize_t sent_bytes = send(udp_sockfd, veh_data.c_str(), veh_data.size(), 0);
                if(sent_bytes < 0) {
                    std::cerr << "[WARN] Cannot send vehicle data via UDP socket. Error: " << (errno == 0 ? "None" : strerror(errno)) << std::endl;
                }
            }
        }
    }

    if(terminatorFlag == true) {
        std::cerr << "[WARN] Vehicle data transmission thread terminated due to error." << std::endl;
    }

    close(udp_sockfd);
}


int main (int argc, char *argv[]) {
    std::string dissem_vif = "wlan0";

    std::string log_filename_CAM = "dis";
    std::string log_filename_GNsecurity = "dis";
    std::string log_filename_VAM = "dis";
    std::string log_filename_rcv = "dis";
    std::string log_filename_CPM = "cps_dis";

    // CAN database file name for CPM transmission
    std::string can_db = "";

    // INI file for configuring the CAN database parsing (i.e., which CAN messages carry information about sensed objects and which are the names of the related signals)
    std::string can_db_param_ini = "dis";

    // gpsd options
    std::string gnss_device = "localhost";
    long gnss_port = 3000; // Using 3000 as default port, in our case

    // serial parser options
    std::string serial_device = "/dev/ttyACM0";
    int serial_device_baudrate = 115200;
    double serial_device_validity_thr = 1;

    unsigned long vehicleID = 0; // Vehicle ID
    unsigned long VRUID = 0; // VRU ID
    bool enable_CAM_dissemination = false;
    bool enable_VAM_dissemination = false;
    bool enable_CPM_dissemination = false;
    bool enable_DENM_decoding = false;
    bool enable_reception = false;
    bool disable_selfMAC_check = false;
    bool enable_sensor_classification = false;
    bool verbose = false;

    int json_over_tcp_port = 49000;
    bool enable_security = false;

    double check_faulty_object_acceleration = 0.0;
    bool disable_cpm_speed_triggering = false;

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

    // Wrong input threshold
    long wrong_input_threshold = 1000;

    std::string can_device = "none";

    // true if gpsd is used to gather positioning data, false if the internal NMEA+UBX parser is used
    bool use_gpsd = false;

    int debug_age_info_rate_ms = 0;

    bool force_20Hz_freq = false;

    // Ego station type; it is set to passenger car if CAMs are activated, and to pedestrian if VAMs are activated
    StationType_t ego_station_type = StationType_passengerCar;

    // Flag that is set to true when using a pre-recorded JSON trace, recorded with TRACEN-X (https://github.com/DriveX-devs/TRACEN-X)
    // instead of reading from a real serial device, when the serial parser is used to get positioning+IMU data via NMEA+UBX
    bool use_json_trace = false;

    int bitrate;

    std::string veh_data_sock_addr = "dis"; // Address for the vehicle data socket, used to send the vehicle data to a remote server (e.g., for data analysis purposes)
    double veh_data_periodicity_s = 1.0; // Periodicity of the vehicle data transmission, in seconds

    bool enable_DCC = false;
    int time_window_DCC = 0;
    std::string modality_DCC = "";
    bool verbose_DCC = false;
    std::string log_filename_DCC = "";

    bool enable_metric_supervisor = false;
    uint64_t time_window_met_sup = 0;
    std::string log_filename_met_sup = "";
    float cbr_target;
    int queue_length;
    int queue_lifetime;

    // Parse the command line options with the TCLAP library
    try {
        TCLAP::CmdLine cmd("OScar: the open ETSI C-ITS implementation", ' ', "7.9-development");

        // TCLAP arguments: short option (can be left empty for long-only options), long option, description, is it mandatory?, default value, type indication (just a string to help the user)
        // All options should be added here in alphabetical order. Long-only options should be added after the sequence of short+long options.
        TCLAP::ValueArg<int> SerialDeviceBaudrate("b", "serial-device-baudrate",
                                                  "[Considered only if -g is not specified] Serial device baudrate for the GNSS receiver. Default: 115200",
                                                  false, 115200, "positive integer");
        cmd.add(SerialDeviceBaudrate);

        TCLAP::ValueArg<std::string> CANDeviceArg("c", "can-device",
                                                  "Optional CAN interface: CAN device to be used (i.e., where can-utils is currently running). Default: none (CAN bus interface disabled).",
                                                  false, "none", "string");
        cmd.add(CANDeviceArg);

        TCLAP::SwitchArg CAMsDissArg("C", "enable-CAMs-dissemination", "Enable the dissemination of standard CAMs",
                                     false);
        cmd.add(CAMsDissArg);

        TCLAP::SwitchArg DENMsDecArg("d", "enable-DENMs-decoding", "Enable the decoding of DENMs", false);
        cmd.add(DENMsDecArg);

        TCLAP::ValueArg<std::string> GNSSDevArg("D", "gnss-device",
                                                "[Considered only if -g is specified] GNSS device to be used (i.e., where gpsd is currently running - this is not the /dev/ttyACM* device, which is already being used by gpsd, which in turn can provide the GNSS data to OCABS). Default: localhost.",
                                                false, "localhost", "string");
        cmd.add(GNSSDevArg);

        TCLAP::ValueArg<double> HEAD_threshold("e", "Heading-threshold", "VAM heading triggering condition threshold",
                                               false, -1, "double");
        cmd.add(HEAD_threshold);

        TCLAP::ValueArg<std::string> LogfileGNsecurity("f", "log-file-GNsecurity",
                                                       "Print on file the log for the GN security checks. Default: (disabled).",
                                                       false, "dis", "string");
        cmd.add(LogfileGNsecurity);

        TCLAP::ValueArg<std::string> LogfileVAM("F", "log-file-VAM",
                                                "[Considered only if security is enabled] Print on file the log for the VAM condition checks. Default: (disabled).",
                                                false, "dis", "string");
        cmd.add(LogfileVAM);

        TCLAP::SwitchArg UseGPSD("g", "use-gpsd",
                                 "Use GPSD instead of the serial NMEA+UBX parser that is normally used by default.",
                                 false);
        cmd.add(UseGPSD);

        // -h / --help is reserved for the help and should not be added as an option

        TCLAP::ValueArg<std::string> vifName("I", "interface", "Broadcast dissemination interface. Default: wlan0.",
                                             false, "wlan0", "string");
        cmd.add(vifName);

        TCLAP::ValueArg<long> JSONserverPortArg("j", "ldm-json-server-port",
                                                "Set the port for on-demand JSON-over-TCP requests to the LDM.", false,
                                                DEFAULT_JSON_OVER_TCP_PORT, "integer");
        cmd.add(JSONserverPortArg);

        TCLAP::ValueArg<std::string> LogfileCAM("L", "log-file-CAM",
                                                "Print on file the log for the CAM condition checks. Default: (disabled).",
                                                false, "dis", "string");
        cmd.add(LogfileCAM);

        TCLAP::SwitchArg EnableHMIArg("m", "enable-HMI", "Enable the OScar HMI", false);
        cmd.add(EnableHMIArg);

        TCLAP::SwitchArg CPMsDissArg("M", "enable-CPMs-dissemination", "Enable the dissemination of CPMs", false);
        cmd.add(CPMsDissArg);

        TCLAP::ValueArg<double> POS_threshold("p", "Position-threshold", "VAM position triggering condition threshold",
                                              false, -1, "double");
        cmd.add(POS_threshold);

        TCLAP::ValueArg<long> GNSSPortArg("P", "gnss-port",
                                          "[Considered only if -g is specified] Port to be used to connect to the GNSS device. It should correspond to the port used by gpsd for the desired receiver. Warning! The default port for gpsd is 2947, while the default for OCABS is 3000.",
                                          false, GNSS_DEFAULT_PORT, "integer");
        cmd.add(GNSSPortArg);

        TCLAP::ValueArg<unsigned long> VRUIDArg("r", "VRU-id", "VRU Basic Service Station ID", false, 1000,
                                                "unsigned integer");
        cmd.add(VRUIDArg);

        TCLAP::ValueArg<std::string> LogfileReception("R", "log-file-Reception",
                                                      "Print on file the data retrieved from CAM/VAM/DENM reception. Default: (disabled).",
                                                      false, "dis", "string");
        cmd.add(LogfileReception);

        TCLAP::ValueArg<std::string> SerialDevice("s", "serial-device",
                                                  "[Considered only if -g is not specified] Serial device path for the GNSS receiver. Default: /dev/ttyACM0",
                                                  false, "/dev/ttyACM0", "string");
        cmd.add(SerialDevice);

        TCLAP::ValueArg<double> SPEED_threshold("S", "Speed-threshold", "VAM speed triggering condition threshold",
                                                false, -1, "double");
        cmd.add(SPEED_threshold);

        TCLAP::SwitchArg DisableSelfMACArg("T", "disable-self-MAC-check",
                                           "Debugging option: disable the self MAC check: if this option is set, "
                                           "OScar will likely receive also the messages sent by itself (i.e., it will receive messages with a MAC address equal to its own too). Useful for debugging.",
                                           false);
        cmd.add(DisableSelfMACArg);

        TCLAP::ValueArg<std::string> UDPSockAddrArg("u", "udp-sock-addr",
                                                    "If specified, OCABS, in addition to the standard-compliant CAM dissemination, will also encapsulate each CAM inside UDP, and send these messages to the address (in the form <IP:port>) specified after this options.",
                                                    false, "dis", "string");
        cmd.add(UDPSockAddrArg);

        TCLAP::ValueArg<std::string> UDPBindIPArg("U", "udp-bind-ip",
                                                  "This options is valid only if --udp-sock-addr/-u has been specified. It can be used to set an interface/address to bind the UDP socket to. By default, no specific address is used for binding (i.e., binding to any address/interface).",
                                                  false, "0.0.0.0", "string");
        cmd.add(UDPBindIPArg);

        TCLAP::ValueArg<unsigned long> VehicleIDArg("v", "vehicle-id", "CA Basic Service Station ID", false, 0,
                                                    "unsigned integer");
        cmd.add(VehicleIDArg);

        TCLAP::SwitchArg VAMsDissArg("V", "enable-VAMs-dissemination", "Enable the dissemination of VAMs", false);
        cmd.add(VAMsDissArg);

        TCLAP::SwitchArg SecurityArg("w", "enable-security",
                                     "Enable the the transmission and reception of secured messages (tested on CAMs and CPMs)",
                                     false);
        cmd.add(SecurityArg);

        TCLAP::SwitchArg EnableRxArg("x", "enable-reception", "Enable the reception of messages and the LDM", false);
        cmd.add(EnableRxArg);

        TCLAP::SwitchArg ExtraPosUDPArg("X", "add-extra-position-udp",
                                        "This options is valid only if --udp-sock-addr/-u has been specified. If specified, this option will make OCABS add, before the actual CAM payload of each UDP packets, 64 extra bits, contatining the current latitude and longitude (32 bits each), in network byte order and stored as degrees*1e7.",
                                        false);
        cmd.add(ExtraPosUDPArg);

        TCLAP::ValueArg<double> SerialDeviceValidityThr("y", "serial-device-validity-threshold",
                                                        "[Considered only if -g is not specified] Advanced options (set only if you know what you are doing): serial device data validity time threshold for the GNSS receiver. Default: 1 sec",
                                                        false, 1, "positive double");
        cmd.add(SerialDeviceValidityThr);

        TCLAP::SwitchArg Force20HzFreq("Y", "force-CAM-20Hz",
                                       "Advanced options: force the CAM transmission frequency to a fixed 20 Hz (in line with the 5G-CARMEN project experimentation)",
                                       false);
        cmd.add(Force20HzFreq);

        TCLAP::ValueArg<std::string> LogfileCPM("z", "log-file-CPM",
                                                "Print on file the log for the CPM condition checks. Default: (disabled).",
                                                false, "cps_dis", "string");
        cmd.add(LogfileCPM);

        // Vehicle Visualizer options
        TCLAP::ValueArg<long> VV_NodejsPortArg("1", "vehviz-nodejs-port",
                                               "Advanced option: set the port number for the UDP connection to the Vehicle Visualizer Node.js server",
                                               false, DEFAULT_VEHVIZ_NODEJS_UDP_PORT, "integer");
        cmd.add(VV_NodejsPortArg);

        TCLAP::ValueArg<long> VV_WebInterfacePortArg("2", "vehviz-web-interface-port",
                                                     "set the port at which the web interface of the Vehicle Visualizer will be available",
                                                     false, DEFAULT_VEHVIZ_WEB_PORT, "integer");
        cmd.add(VV_WebInterfacePortArg);

        TCLAP::ValueArg<double> VV_UpdateIntervalArg("3", "vehviz-update-interval-sec",
                                                     "Advanced option: this option can be used to modify the update rate of the web-based GUI. "
                                                     "Warning: decreasing too much this value will affect the LDM database performance!"
                                                     "This value cannot be less than 0.05 s and more than 1 s.",
                                                     false, DEFAULT_VEHVIZ_UPDATE_INTERVAL_SECONDS, "double");
        cmd.add(VV_UpdateIntervalArg);

        TCLAP::ValueArg<std::string> VV_NodejsAddrArg("4", "vehviz-nodejs-addr",
                                                      "Advanced option: set the IPv4 address for the UDP connection to the Vehicle Visualizer Node.js server (excluding the port number). "
                                                      "This is the address without port number.",
                                                      false, DEFAULT_VEHVIZ_NODEJS_UDP_ADDR, "IPv4 address string");
        cmd.add(VV_NodejsAddrArg);

        TCLAP::ValueArg<long> WrongInputTsholdArg("5", "set-wrong-input-threshold",
                                                  "[Considered only if -g is not specified] Advanced option: set the number of unrecognized bytes after which the parser should stop its execution and return an error.",
                                                  false, 1000, "integer");
        cmd.add(WrongInputTsholdArg);

        TCLAP::ValueArg<double> CheckFaultyObjectAcceleration("6", "faulty-object-acceleration-threshold",
                                                              "Advanced option: enable the check of faulty acceleration values of the objects in the CPMs by setting a custom threshold.",
                                                              false, 0.0, "double");
        cmd.add(CheckFaultyObjectAcceleration);

        TCLAP::SwitchArg DisableCPMSpeedTriggering("7", "disable-cpm-speed-triggering",
                                                   "Advanced option: disable the triggering of the CPMs based on speed variation.",
                                                   false);
        cmd.add(DisableCPMSpeedTriggering);

        TCLAP::SwitchArg EnableSensorClassification("8", "classification",
                                                    "Advanced option: enable the usage of sensor output classification.",
                                                    false);
        cmd.add(EnableSensorClassification);

        TCLAP::SwitchArg EnableVerbose("9", "vv", "Enable verbose output.", false);
        cmd.add(EnableVerbose);

        TCLAP::ValueArg<std::string> CANdb("", "can-db",
                                           "CAN database file for CPM dissemination. A CAN db telling OSCar how to parse data about sensed objects is mandatory for the dissemination of CPMs.",
                                           false, "", "string");
        cmd.add(CANdb);

        TCLAP::ValueArg<std::string> CANdbParamINI("", "can-db-param-file",
                                                   "INI file for specifying which CAN messages should be considered for the parsing of the sensor data for CPM dissemination, and the names of the CAN signals corresponding to object classification (classification), "
                                                   "angle between the sensor and the left-most edge of the object (phi_left), angle between the sensor and the right-most edge of the object (phi_right), distance between sensor and object (distance). "
                                                   "The name of the CAN messages containing such signals (sensor_message_regex in the INI file) should be specified through a regular expression, in case more than one message should be parsed (e.g., if different CAN frames carry information about "
                                                   "different sensed objects at the same time). In any case, OScar assumes that all the CAN messages matching the regular expression contain the same signals related to detected objects. "
                                                   "If this option is not specified, default values will be used, i.e., a CAN message name regex equal to \"^Video_Object_\\d{2}_B$\", and signal names set to, respectively, \"classification\", \"phi_left\", \"phi_right\" and \"dx_v\".",
                                                   false, "dis", "string");
        cmd.add(CANdbParamINI);

        TCLAP::ValueArg<int> ShowDebugAgeInfo("", "show-live-data",
                                              "[Considered only if -g is not specified] When activated, OScar will show, while it is running, the live data obtained from the GNSS system, together with the age of each piece of information (how old it is with respect to when it was last retrieved). After the option, an update interval in milliseconds should be specified. This will be the frequency at which the live data will be displayed.",
                                              false, 0, "integer");
        cmd.add(ShowDebugAgeInfo);

        TCLAP::SwitchArg UseJsonTrace("", "use-tracenx-json-trace",
                                      "[Considered only if -g is not specified] Instead of reading from a real serial device, when the serial parser is used, use a JSON pre-recorded trace for the provision of positioning data. The path to the .json file should be specified after -s as if it was the path to a serial device. OScar supports JSON trace files recorded with TRACEN-X (https://github.com/DriveX-devs/TRACEN-X).",
                                      false);
        cmd.add(UseJsonTrace);

        TCLAP::ValueArg<std::string> SendVehdataSocket("", "send-vehdata-socket",
                                                       "Send vehicle data to external applications via a dedicated UDP socket. After this option, the IP and port of the external application should be specified in the format <IP>:<port>.",
                                                       false, "dis", "string");
        cmd.add(SendVehdataSocket);

        TCLAP::ValueArg<double> SendVehdataPeriod("", "send-vehdata-period",
                                                  "Periodicity, in seconds, for sending the vehicle data to external applications. Can be specified only if --send-vehdata-socket has been specified too. Default: 1 second.",
                                                  false, 1.0, "double");
        cmd.add(SendVehdataPeriod);

        TCLAP::ValueArg<int> Bitrate("", "bitrate", "Bitrate used in Mbit/s. Default: 3Mb/s", false, 3, "int");

        cmd.add(Bitrate);

        TCLAP::SwitchArg EnableDCC("", "enable-DCC",
                                   "Activate the Decentralized Congestion Control (DCC) for the CAM, VAM and CPM messages. Remember to specify also the other DCC arguments to guarantee a correct usage of this feature.",
                                   false);
        cmd.add(EnableDCC);

        std::string helpText_dcc =
                "Time window for DCC Channel State check (Channel Busy Ratio). "
                "It must be a strictly positive integer value, expressed in milliseconds, "
                "and it has to be lower than " + std::to_string(MAXIMUM_TIME_WINDOW_DCC) + "ms.";

        TCLAP::ValueArg<int> TimeWindowDCC("", "time-window-DCC", helpText_dcc, false, 0, "integer");

        cmd.add(TimeWindowDCC);

        TCLAP::ValueArg<std::string> ModalityDCC("", "modality-DCC",
                                                 "Select the DCC modality, it could be Reactive or Adaptive. The strings to be used to indicate the modality are: ['reactive', 'adaptive'].",
                                                 false, "", "string");
        
        cmd.add(ModalityDCC);
        
        TCLAP::ValueArg<float> CBRTarget("", "CBR-target", "For the Adaptive DCC, the CBR we would the environment reach. Default: 0.68", false, 0.68, "float");

        cmd.add(CBRTarget);


        TCLAP::SwitchArg VerboseDCC("", "verbose-DCC",
                                    "If set to 1, this argument provides a verbose description of the Channel State during the DCC checks.",
                                    false);
        cmd.add(VerboseDCC);

        TCLAP::ValueArg<std::string> LogfileDCC("", "log-file-DCC",
                                                "Print on file the log for the DCC metrics. Default: (disabled).",
                                                false, "", "string");
        cmd.add(LogfileDCC);

        TCLAP::ValueArg<int> QueueLengthDCC("", "queue-length-DCC",
                                                "Length of the priority queue for DCC. Default: 0 (no queue).",
                                                false, 0, "int");
        cmd.add(QueueLengthDCC);

        TCLAP::ValueArg<int> QueueLifetimeDCC("", "queue-lifetime-DCC",
                                                "Lifetime for packet queued by DCC in ms. Default: 300.",
                                                false, 300, "int");
        cmd.add(QueueLifetimeDCC);

        TCLAP::SwitchArg EnableMetricSupervisor("", "enable-metric-supervisor",
                                                "Activate the Metric Supervisor to collect information and metrics about V2X messages sent and received.",
                                                false);
        cmd.add(EnableMetricSupervisor);

        std::string helpText_met_sup = "Time window for Metric Supervisor check. It must be a strictly positive integer value, expressed in milliseconds";

        TCLAP::ValueArg<int> TimeWindowMetricSupervisor("", "time-window-metric-supervisor", helpText_met_sup, false, 0, "integer");

        cmd.add(TimeWindowMetricSupervisor);

        TCLAP::ValueArg<std::string> LogfileMetricSupervisor("", "log-file-metric-supervisor",
                                                             "Print on file the log for the Metric Supervisor measured metrics. Default: (disabled).",
                                                             false, "", "string");
        cmd.add(LogfileMetricSupervisor);

        cmd.parse(argc, argv);

        dissem_vif = vifName.getValue();

        log_filename_CAM = LogfileCAM.getValue();
        log_filename_GNsecurity = LogfileGNsecurity.getValue();
        log_filename_VAM = LogfileVAM.getValue();
        log_filename_rcv = LogfileReception.getValue();
        log_filename_CPM = LogfileCPM.getValue();

        can_db = CANdb.getValue();
        can_db_param_ini = CANdbParamINI.getValue();

        gnss_device = GNSSDevArg.getValue();
        gnss_port = GNSSPortArg.getValue();

        vehicleID = VehicleIDArg.getValue();
        VRUID = VRUIDArg.getValue();

        enable_CAM_dissemination = CAMsDissArg.getValue();
        enable_VAM_dissemination = VAMsDissArg.getValue();
        enable_CPM_dissemination = CPMsDissArg.getValue();
        enable_DENM_decoding = DENMsDecArg.getValue();
        enable_security = SecurityArg.getValue();

        check_faulty_object_acceleration = CheckFaultyObjectAcceleration.getValue();
        disable_cpm_speed_triggering = DisableCPMSpeedTriggering.getValue();

        enable_sensor_classification = EnableSensorClassification.getValue();
        verbose = EnableVerbose.getValue();

        udp_sock_addr = UDPSockAddrArg.getValue();
        udp_bind_ip = UDPBindIPArg.getValue();
        extra_position_udp = ExtraPosUDPArg.getValue();

        rx_opts.gnss_device = gnss_device;
        rx_opts.dissemination_device = dissem_vif;

        pos_th = POS_threshold.getValue();
        speed_th = SPEED_threshold.getValue();
        head_th = HEAD_threshold.getValue();

        enable_reception = EnableRxArg.getValue();

        vizOpts.vehviz_nodejs_port = VV_NodejsPortArg.getValue();
        vizOpts.vehviz_web_interface_port = VV_WebInterfacePortArg.getValue();
        vizOpts.vehviz_update_interval_sec = VV_UpdateIntervalArg.getValue();
        vizOpts.vehviz_nodejs_addr = VV_NodejsAddrArg.getValue();

        veh_data_sock_addr = SendVehdataSocket.getValue();
        veh_data_periodicity_s = SendVehdataPeriod.getValue();

        if (vizOpts.vehviz_update_interval_sec < 0.05 || vizOpts.vehviz_update_interval_sec > 1) {
            std::cerr
                    << "[ERROR] The Vehicle Visualizer update interval cannot be lower than 0.05 s or grater than 1 second."
                    << std::endl;
            return 1;
        }

        if (veh_data_periodicity_s < 0.01 || veh_data_periodicity_s > 60.0) {
            std::cerr
                    << "[ERROR] The vehicle data transmission periodicity cannot be lower than 100 ms or greater than 60 seconds."
                    << std::endl;
            return 1;
        }

        // Set the ego station type -> if CAMs are enabled, set it to passenger car, otherwise to pedestrian
        if (enable_CAM_dissemination) {
            ego_station_type = StationType_passengerCar;
        } else {
            ego_station_type = StationType_pedestrian;
        }

        vizOpts.ego_station_type = ego_station_type;

        enable_hmi = EnableHMIArg.getValue();
        disable_selfMAC_check = DisableSelfMACArg.getValue();
        json_over_tcp_port = JSONserverPortArg.getValue();

        can_device = CANDeviceArg.getValue();

        use_gpsd = UseGPSD.getValue();
        serial_device = SerialDevice.getValue();
        serial_device_baudrate = SerialDeviceBaudrate.getValue();
        serial_device_validity_thr = SerialDeviceValidityThr.getValue();
        debug_age_info_rate_ms = ShowDebugAgeInfo.getValue();

        wrong_input_threshold = WrongInputTsholdArg.getValue();

        force_20Hz_freq = Force20HzFreq.getValue();

        use_json_trace = UseJsonTrace.getValue();

        bitrate = Bitrate.getValue();

        enable_DCC = EnableDCC.getValue();
        time_window_DCC = TimeWindowDCC.getValue();
        modality_DCC = ModalityDCC.getValue();
        verbose_DCC = VerboseDCC.getValue();
        log_filename_DCC = LogfileDCC.getValue();
        cbr_target = CBRTarget.getValue();
        queue_length = QueueLengthDCC.getValue();
        queue_lifetime = QueueLifetimeDCC.getValue();

        enable_metric_supervisor = EnableMetricSupervisor.getValue();
        time_window_met_sup = TimeWindowMetricSupervisor.getValue();
        log_filename_met_sup = LogfileMetricSupervisor.getValue();

        if (can_db == "") {
            if (can_db_param_ini != "dis" && can_db_param_ini != "") {
                std::cerr
                        << "[ERROR] Specified an INI file for CAN database parsing but no .dbc file was specified with --can-db!"
                        << std::endl;
                return 1;
            }
        }

        if (enable_CPM_dissemination == true && can_db == "") {
            std::cerr
                    << "[ERROR] A CAN database file must be specified with --can-db when CPM dissemination is enabled."
                    << std::endl;
            return 1;
        }

        if (enable_CPM_dissemination == false && can_db != "") {
            std::cerr
                    << "[WARN] A CAN databae file was specified even if CPM dissemination is disabled. The --can-db option will be ignored."
                    << std::endl;
        }

        if (use_gpsd == true && use_json_trace == true) {
            std::cerr
                    << "[ERROR] --use-tracenx-json-trace can only be used when --use-gpsd is not specified and the serial parser is used."
                    << std::endl;
            return 1;
        }

        if (enable_reception == false && enable_hmi == true) {
            // TODO: support the usage of the HMI just to show the ego vehicle/road user, letting the user enable the HMI without reception
            std::cerr
                    << "[ERROR] Reception must be enabled to use the HMI, for the time being. This limitation is going to be solved in the future."
                    << std::endl;
            return 1;
        }

        if (enable_reception == true && enable_hmi == false) {
            std::cerr
                    << "[WARN] Enabling reception without HMI is not recommended! However, it is possible and everything should just work."
                    << std::endl;
        }

        if (enable_DCC == true) {
            if (time_window_DCC <= 0 || time_window_DCC >= MAXIMUM_TIME_WINDOW_DCC) {
                std::cerr
                        << "[ERROR] Time window for DCC was not correctly set. Remember that it must be an integer value greater than 0 and lower than "
                        << MAXIMUM_TIME_WINDOW_DCC << "ms, please check the helper." << std::endl;
                return 1;
            }

            if (modality_DCC != "reactive" && modality_DCC != "adaptive") {
                std::cerr
                        << "[ERROR] Modality for DCC was not correctly set. Remember that it must be a string of value 'reactive' or 'adaptive', please check the helper."
                        << std::endl;
                return 1;
            }

            std::cout << "[INFO] DCC enabled correctly in " << modality_DCC << " modality and with a time window of "
                      << std::to_string(time_window_DCC) << std::endl;
        }

        if (enable_metric_supervisor == true) {
            if (time_window_met_sup <= 0) {
                std::cerr
                        << "[ERROR] Time window for DCC was not correctly set. Remember that it must be an integer value greater than 0ms, please check the helper."
                        << std::endl;
                return 1;
            }

            if (log_filename_met_sup == "") {
                std::cerr
                        << "[ERROR] Log filename for Metric Supervisor was not correctly set. Remember that it must be a non-empty string, please check the helper."
                        << std::endl;
                return 1;
            }

            std::cout << "[INFO] Metric Supervisor enabled with a time window of "
                      << std::to_string(time_window_met_sup) << std::endl;
        }

        std::cout << "[INFO] CAM/CPM/VAM dissemination interface: " << dissem_vif << std::endl;
    } catch (TCLAP::ArgException &tclape) {
        std::cerr << "TCLAP error: " << tclape.error() << " for argument " << tclape.argId() << std::endl;

        return 1;
    }

    if (udp_bind_ip != "0.0.0.0" && udp_sock_addr == "dis") {
        std::cerr << "Error. --udp-bind-ip/-U can only be specified when --udp-sock-addr/-u is specified too."
                  << std::endl;

        return 1;
    }

    if (extra_position_udp == true && udp_sock_addr == "dis") {
        std::cerr
                << "Error. --add-extra-position-udp/-X can only be specified when --udp-sock-addr/-u is specified too."
                << std::endl;

        return 1;
    }

    // Read the CAN db if CPM dissemination is enabled
    std::vector<uint32_t> CPM_CAN_ids;
    CAN_SENSOR_SIGNAL_INFO_t CPM_CAN_signals;

    if (can_db != "" && enable_CPM_dissemination == true) {
        dbcReader dbr;

        if (can_db_param_ini != "dis") {
            if (!dbr.setUserParamsIni(can_db_param_ini)) {
                std::cerr
                        << "[ERROR] Cannot open the INI file with the parameters for parsing the CAN database. CPM dissemination will be disabled."
                        << std::endl;
                enable_CPM_dissemination = false;
                // Disable also the basic sensor reader
                can_device = "none";
            }
        }

        // If no error occurred, the CPM dissemination should be still enabled at this point
        if (enable_CPM_dissemination == true) {
            if (!dbr.parseDBC(can_db)) {
                std::cerr << "[ERROR] Cannot parse the specified CAN database file: " << can_db
                          << ". CPM dissemination will be disabled." << std::endl;
                enable_CPM_dissemination = false;
                // Disable also the basic sensor reader
                can_device = "none";
            } else {
                bool error = false;
                CPM_CAN_ids = dbr.getSensorObjectCANIDs(error);
                if (error) {
                    std::cerr
                            << "[ERROR] Cannot get the required CAN message IDs from the specified CAN database file: "
                            << can_db << ". CPM dissemination will be disabled." << std::endl;
                    enable_CPM_dissemination = false;
                    // Disable also the basic sensor reader
                    can_device = "none";
                } else {
                    CPM_CAN_signals = dbr.getSignalInfoSensorObject(error);
                    if (error) {
                        std::cerr
                                << "[ERROR] Cannot get the required signal information from the specified CAN database file: "
                                << can_db << ". CPM dissemination will be disabled." << std::endl;
                        enable_CPM_dissemination = false;
                        // Disable also the basic sensor reader
                        can_device = "none";
                    } else {
                        std::cout << "[INFO] CAN database successfully parsed." << std::endl;
                        if (verbose) {
                            dbr.printCANdb(error);
                        }
                    }
                }
            }
        }
    }

    // Create the raw socket for the transmission of CAMs/VAMs, encapsulated inside GeoNetworking and BTP (in user space)
    int sockfd = -1;
    sockfd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));

    if (sockfd < 0) {
        std::cerr << "Critical error: cannot open raw socket for CAM dissemination. Details: " << strerror(errno)
                  << "\n" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Get the index of the dissemination interface
    int ifindex = if_nametoindex(dissem_vif.c_str());
    if (ifindex < 1) {
        std::cerr << "Critical error: cannot find an interface index for interface: " << dissem_vif << std::endl;
        exit(EXIT_FAILURE);
    }

    // Get the MAC address of the dissemination interface and store it inside "srcmac"
    uint8_t srcmac[6] = {0};
    struct ifreq ifreq;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
    strncpy(ifreq.ifr_name, dissem_vif.c_str(), IFNAMSIZ);
#pragma GCC diagnostic pop

    if (ioctl(sockfd, SIOCGIFHWADDR, &ifreq) != -1) {
        memcpy(srcmac, ifreq.ifr_hwaddr.sa_data, 6);
    } else {
        std::cerr << "Critical error: cannot find a MAC address for interface: " << dissem_vif << std::endl;
        exit(EXIT_FAILURE);
    }

    // Enable broadcast on the socket (is it really needed? To be double-checked!)
    int enableBcast = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &enableBcast, sizeof(enableBcast)) < 0) {
        std::cerr << "Critical error: cannot set broadcast permission on UDP socket for CAM dissemination.\n"
                  << std::endl;
        exit(EXIT_FAILURE);
    }

    // Bind raw socket
    struct sockaddr_ll addrll;
    memset(&addrll, 0, sizeof(addrll));
    addrll.sll_ifindex = ifindex;
    addrll.sll_family = AF_PACKET;
    addrll.sll_protocol = htons(ETH_P_ALL);

    errno = 0;
    if (bind(sockfd, (struct sockaddr *) &addrll, sizeof(addrll)) < 0) {
        std::cerr << "Critical error: cannot bind the raw socket to the '" << dissem_vif << "' interface. Ifindex: "
                  << ifindex << "." << std::endl
                  << "Socket: " << sockfd << ". Error: " << strerror(errno) << "." << std::endl;
        exit(EXIT_FAILURE);
    }

    ECManager ecManager;
    ATManager atManager(&terminatorFlag);
    ATManager *atManager_ptr = nullptr;
    if (enable_security) {
        if (!ecManager.manageRequest()) {
            std::cerr << "Error in managing the EC request" << std::endl;
            exit(EXIT_FAILURE);
        }
        std::string ecBytes = ecManager.getECBytes();
        atManager.setECHex(ecBytes);
        if (!atManager.manageRequest()) {
            std::cerr << "Error in managing the AT request" << std::endl;
            exit(EXIT_FAILURE);
        }
        atManager_ptr = &atManager;
    }

    // Create a new DB object
    ldmmap::LDMMap *db_ptr = new ldmmap::LDMMap();
    if (ego_station_type == StationType_passengerCar) {
        db_ptr->setStationID(vehicleID);
    } else if (ego_station_type == StationType_pedestrian) {
        db_ptr->setStationID(VRUID);
    } else {
        // For the time being, use the vehicle ID as a fallback in case an unexpected station type is encountered
        db_ptr->setStationID(vehicleID);
    }

    // Configure the global serial parser if gpsd is not used
    if (!use_gpsd) {
        serialParser.setValidityThreshold(serial_device_validity_thr);
        serialParser.setWrongInputThreshold(wrong_input_threshold);
        if (debug_age_info_rate_ms) {
            serialParser.setDebugAgeInfo(debug_age_info_rate_ms);
        }
        serialParser.startUBXNMEAParser(serial_device, serial_device_baudrate, 8, 'N', 1, &terminatorFlag,
                                        use_json_trace);
    }

    // Create a new VDP GPS Client object for the LDM
    VDPGPSClient ldmgpsc(gnss_device, gnss_port);
    ldmgpsc.selectGPSD(use_gpsd);
    if (!use_gpsd) {
        ldmgpsc.setSerialParser(&serialParser);
    }
    ldmgpsc.openConnection();
    db_ptr->setLoggingGNSSClient(&ldmgpsc);

    // Take the current position (if available) from the positioning provider (either gpsd or serial parser)
    int pos_avail_cnt = 0;
    double test_lat, test_lon;
    while (!terminatorFlag) {
        std::pair<double, double> curr_pos = ldmgpsc.getCurrentPositionDbl();
        test_lat = curr_pos.first;
        test_lon = curr_pos.second;

        if (test_lat >= -90.0 && test_lat <= 90.0 && test_lon >= -180.0 && test_lon <= 180.0) {
            std::cout << "[INFO] Position available after roughly " << pos_avail_cnt << " seconds: latitude: "
                      << test_lat << " - longitude: " << test_lon << std::endl;
            break;
        } else {
            std::cout << "[INFO] Position not yet available. Waiting 1 second and trying again..." << std::endl;
        }

        std::cout
                << "[INFO] Waiting for the positioning provider to provide the position (a fix may not be yet available)..."
                << std::endl;

        sleep(1);
        pos_avail_cnt++;

        if (pos_avail_cnt > 20) {
            std::cerr << "[WARN] Position not available after 20 seconds. Using default position for the LDM and GUI."
                      << std::endl;
            break;
        }
    }

    if (pos_avail_cnt > 20) {
        db_ptr->setCentralLatLon(45.014570, 7.568314); // Sample default lat and lon values, centered near Turin, Italy
    } else {
        db_ptr->setCentralLatLon(test_lat, test_lon);
    }

    // We have to create a thread reading periodically (e.g. every 5 s) the database through the pointer "db_ptr" and "cleaning" the entries
    // which are too old
    pthread_create(&dbcleaner_tid, NULL, DBcleaner_callback, (void *) db_ptr);

    if (enable_hmi == true) {
        // If the HMI has been enabled, we should also start here a second parallel thread, reading periodically the database (e.g. every 500 ms) and sending the vehicle data to
        // the vehicleVisualizer
        // pthread_attr_init(&tattr);
        // pthread_attr_setdetachstate(&tattr,PTHREAD_CREATE_DETACHED);
        vizOpts.db_ptr = db_ptr;
        pthread_create(&vehviz_tid, NULL, VehVizUpdater_callback, (void *) &vizOpts);
        // pthread_attr_destroy(&tattr);
    }

    // Transmission threads creation
    std::vector<std::thread> txThreads;

    // Create a new Metric Supervisor and configure it if enabled
    MetricSupervisor metric_supervisor;
    if (enable_metric_supervisor) {
        metric_supervisor.setupMetricSupervisor(log_filename_met_sup, time_window_met_sup, dissem_vif);
        metric_supervisor.start();
    }

    CABasicService cabs;
    CABasicService *cabs_ptr = &cabs;
    if (enable_metric_supervisor) {
        cabs_ptr->setMetricSupervisor(&metric_supervisor);
    }

    CPBasicService cpbs;
    CPBasicService *cpbs_ptr = &cpbs;
    if (enable_metric_supervisor) {
        cpbs_ptr->setMetricSupervisor(&metric_supervisor);
    }

    VRUBasicService vrubs;
    VRUBasicService* vrubs_ptr = &vrubs;
    if(enable_metric_supervisor) {
        vrubs_ptr->setMetricSupervisor(&metric_supervisor);
    }

    DCC *dcc = new DCC();
    dcc->setBitRate(bitrate * 1e6);
    if (enable_DCC)
    {
        dcc->setupDCC(time_window_DCC, modality_DCC, dissem_vif, cbr_target, 0.01f, verbose_DCC, queue_length, queue_lifetime, log_filename_DCC);
        dcc->setMetricSupervisor(&metric_supervisor);
    }

    // This must be defined here, otherwise the goto will jump over its definition
    SocketClient *mainRecvClient = nullptr;

    if(terminatorFlag) {
        goto exit_failure;
    }

    if(enable_CAM_dissemination) {
        txThreads.emplace_back(CAMtxThr,
                                        gnss_device,
                                        gnss_port,
                                        sockfd,
                                        ifindex,
                                        srcmac,
                                        vehicleID,
                                        udp_sock_addr,
                                        udp_bind_ip,
                                        extra_position_udp,
                                        log_filename_CAM,
                                        log_filename_GNsecurity,
                                        db_ptr,
                                        atManager_ptr,
                                        pos_th,
                                        speed_th,
                                        head_th,
                                        enable_reception,
                                        use_gpsd,
                                        enable_security,
                                        force_20Hz_freq,
                                        cabs_ptr,
                                        dcc
                                    );
    }
    if(enable_VAM_dissemination) {
        txThreads.emplace_back(VAMtxThr,
                                        gnss_device,
                                        gnss_port,
                                        sockfd,
                                        ifindex,
                                        srcmac,
                                        VRUID,
                                        udp_sock_addr,
                                        udp_bind_ip,
                                        extra_position_udp,
                                        log_filename_VAM,
                                        log_filename_GNsecurity,
                                        db_ptr,
                                        pos_th,
                                        speed_th,
                                        head_th,
                                        use_gpsd,
                                        enable_security,
                                        atManager_ptr,
                                        vrubs_ptr,
                                        dcc
                                    );
    }
    if(enable_CPM_dissemination) {
        txThreads.emplace_back(CPMtxThr,
                                        gnss_device,
                                        gnss_port,
                                        sockfd,
                                        ifindex,
                                        srcmac,
                                        vehicleID,
                                        udp_sock_addr,
                                        udp_bind_ip,
                                        extra_position_udp,
                                        log_filename_CPM,
                                        log_filename_GNsecurity,
                                        db_ptr,
                                        use_gpsd,
                                        check_faulty_object_acceleration,
                                        disable_cpm_speed_triggering,
                                        verbose,
                                        enable_security,
                                        atManager_ptr,
                                        cpbs_ptr,
                                        dcc
                                    );
    }
    if (can_device != "none") {
        txThreads.emplace_back(radarReaderThr,
                               gnss_device,
                               gnss_port,
                               can_device,
                               db_ptr,
                               vehicleID,
                               use_gpsd,
                               enable_sensor_classification,
                               verbose,
                               CPM_CAN_signals,
                               CPM_CAN_ids);
    }

    if(veh_data_sock_addr!="dis") {
        // If the user has specified a socket address for the vehicle data transmission, start the thread
        txThreads.emplace_back(vehdataTxThread,
                               veh_data_sock_addr,
                               veh_data_periodicity_s,
                               gnss_device,
                               gnss_port,
                               use_gpsd);
    }

	// Enable debug age of information, if option has been specified
    if (serialParser.getDebugAgeInfo()) {
        serialParser.showDebugAgeInfo();
    }

    if(enable_DCC)
    {
        dcc->startDCC();
    }

	// Reception loop (using the main thread)
	if(enable_reception==true) {
		fprintf(stdout,"Configuring socket for reception. Descriptor: %d\n",sockfd);

		if(terminatorFlag==false) {
			// Create the main SocketClient object for the reception of the V2X messages
            mainRecvClient = new SocketClient(sockfd,&rx_opts, db_ptr, log_filename_rcv, enable_security, log_filename_GNsecurity);

			if(enable_DENM_decoding) {
				mainRecvClient->enableDENMdecoding();
			}

            if(enable_metric_supervisor) {
                mainRecvClient->setMetricSupervisor(&metric_supervisor);
            }

			// Set the "self" MAC address, so that all the messages coming from this address will be discarded
			if(disable_selfMAC_check==false) {
				mainRecvClient->setSelfMAC(srcmac);
			}

			// Create an additional VDP GPS Client object for logging the GNSS data
			VDPGPSClient logginggpsc(gnss_device,gnss_port);
            logginggpsc.selectGPSD(use_gpsd);
            if(!use_gpsd) {
                logginggpsc.setSerialParser(&serialParser);
            }

			logginggpsc.openConnection();
			mainRecvClient->setLoggingGNSSClient(&logginggpsc);

			// Before starting the data reception, create a new JSONserver object for client to retrieve the DB data
			JSONserver jsonsrv(db_ptr);
			jsonsrv.setServerPort(json_over_tcp_port);
			if(jsonsrv.startServer()!=true) {
				fprintf(stderr,"[ERROR] Critical error: cannot start the JSON server for the client data retrieval.\n");
				terminatorFlag=true;
			}

            if(!terminatorFlag) {
                fprintf(stdout, "Reception is going to start very soon...\n");

                // Start the reception of V2X messages
                mainRecvClient->startReception();

                jsonsrv.stopServer();
                logginggpsc.closeConnection();
            }
		}
	}
	
	exit_failure:

    for(auto& t : txThreads) {
        t.join();
    }

	terminatorFlag=true;
	
	pthread_join(dbcleaner_tid,nullptr);

	if(enable_hmi==true) {
		pthread_join(vehviz_tid,nullptr);
	}

	db_ptr->clear();

	// Close the socket
	close(sockfd);

	return 0;
}
