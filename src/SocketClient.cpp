#include "SocketClient.h"
#include "utils.h"
#include "Seq.hpp"
#include "SequenceOf.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <iostream>
#include <poll.h>
#include <unistd.h>
#include <linux/if_packet.h> // Needed for "struct sockaddr_ll"
#include <net/ethernet.h> // Needed for "struct ether_header"
#include <linux/wireless.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <set>
#include "utmuts.h"

#define RADIUS_EARTH 6371000.0

#define lowFreqContainerManager(ptr_name,stationID) \
		if(ptr_name->cam.camParameters.lowFrequencyContainer!=NULL) { \
		      if(ptr_name->cam.camParameters.lowFrequencyContainer->choice.basicVehicleContainerLowFrequency.exteriorLights.buf!=NULL) { \
		              return ldmmap::OptionalDataItem<uint8_t>(ptr_name->cam.camParameters.lowFrequencyContainer->choice.basicVehicleContainerLowFrequency.exteriorLights.buf[0]); \
		      } else { \
		              return ldmmap::OptionalDataItem<uint8_t>(false); \
		      } \
		} else { \
		      ldmmap::LDMMap::returnedVehicleData_t retveh; \
		      if(m_db_ptr->lookup(stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) { \
		              return retveh.vehData.exteriorLights; \
		      } else { \
		              return ldmmap::OptionalDataItem<uint8_t>(false); \
		      } \
		}

// If this is a full ITS message manage the low frequency container data
inline ldmmap::OptionalDataItem<uint8_t>
SocketClient::manage_LowfreqContainer(void *decoded_cam_void,uint32_t stationID) {
	CAM_t *decoded_cam = (CAM_t *) decoded_cam_void;
	lowFreqContainerManager(decoded_cam,stationID);
}

inline bool compare_mac(uint8_t mac_a[6],uint8_t mac_b[6]) {
	return (mac_a[0]==mac_b[0] && 
		mac_a[1]==mac_b[1] && 
		mac_a[2]==mac_b[2] && 
		mac_a[3]==mac_b[3] && 
		mac_a[4]==mac_b[4] && 
		mac_a[5]==mac_b[5]);
}

SocketClient::SocketClient(const int &raw_rx_sock,options_t* opts_ptr, ldmmap::LDMMap *db_ptr, std::string logfile_name,bool enable_security, std::string logfile_security, GeoNet* gn)
{
	m_raw_rx_sock = raw_rx_sock;
	m_opts_ptr = opts_ptr;
	m_db_ptr = db_ptr;
	m_logfile_name = logfile_name;
    m_decodeFrontend.setEnableSecurity(enable_security);
	m_decodeFrontend.setGeoNet(gn);
    m_enable_security = enable_security;
	m_logfile_security = logfile_security;
	m_client_id="unset";
	m_logfile_file=nullptr;
	m_printMsg=false;
	m_stopflg=false;
	m_receptionInProgress=false;
	m_unlock_pd_rd=-1;
	m_unlock_pd_wr=-1;
	m_self_mac_set=false;
	memset(m_self_mac,0,6);
	denm_decoding_enabled=false;
	m_gpsc_ptr=nullptr;
	// m_routeros_rssi={};
	// m_terminate_routeros_rssi_flag=false;
}

void
SocketClient::rxThr(void) {
    pthread_setname_np(pthread_self(), "rx_thr");

	size_t msglen=-1;
	uint8_t msgbuf[MSGBUF_MAXSIZ];

	struct sockaddr_ll addrll;
	socklen_t addrLen=sizeof(addrll);

	m_receptionInProgress=true;

	struct pollfd socketMon[2];

	// Monitor the socket
	socketMon[0].fd=m_raw_rx_sock;
	socketMon[0].revents=0;
	socketMon[0].events=POLLIN;

	// Monitor the "unlock pipe"
	socketMon[1].fd=m_unlock_pd_rd;
	socketMon[1].revents=0;
	socketMon[1].events=POLLIN;

	fprintf(stdout,"[INFO] Message reception started on interface: %s. Socket: %d.\n",m_opts_ptr->dissemination_device.c_str(),m_raw_rx_sock);

	struct ether_header* etherHeaderPtr;

	while(m_stopflg==false) {
		if(poll(socketMon,2,-1)>0) {
			if(socketMon[0].revents>0) {
				msglen=recvfrom(m_raw_rx_sock,msgbuf,sizeof(msgbuf),0,(struct sockaddr *)&addrll,&addrLen);

				etherHeaderPtr=reinterpret_cast<struct ether_header *>(msgbuf);

				// Discard any non-GN packet
				if (ntohs(etherHeaderPtr->ether_type)!=GN_ETHERTYPE) { 
					continue;
				}

				// A message from myself has been received -> this message should be discarded
				if(m_self_mac_set==true && compare_mac(etherHeaderPtr->ether_shost,m_self_mac)==true) {
					continue;
				}

				if(msglen<0) {
					fprintf(stderr,"[ERROR] Unable to receive a message from the specified socket.\n");
				} else {
					// Process the received message, after removing the "Ethernet" header with the EtherType, and the source and destination MAC addresses
					m_received_msg++;
					manageMessage(msgbuf+sizeof(struct ether_header),msglen-sizeof(struct ether_header));
				}
			} else if(socketMon[1].revents>0) {
				break;
			}
		}
	}

	fprintf(stdout,"[INFO] Message reception terminated on interface: %s\n",m_opts_ptr->dissemination_device.c_str());

	m_unlock_pd_rd=-1;
	m_unlock_pd_wr=-1;
	m_receptionInProgress=false;
}

void 
SocketClient::startReception(void) {
	int m_unlock_pd[2]={-1,-1};

	// Interpret "dis" as "no log file should be generated"
	if(m_logfile_name=="dis") {
		m_logfile_name="";
	}

	if(m_logfile_name!="") {
		if(m_logfile_name=="stdout") {
			m_logfile_file=stdout;
		} else {
			// Opening the output file in write + append mode just to be safe in case the user does not change the file name
			// between different executions of the S-LDM
			m_logfile_file=fopen(m_logfile_name.c_str(),"wa");
		}
	}

	if(m_receptionInProgress==true) {
		fprintf(stderr,"[WARNING] Attempted to start a V2X message reception while the reception is already in progress.\n"
			"Nothing will be done.\n");
	} else {
		if(pipe(m_unlock_pd)>=0) {
			m_unlock_pd_rd=m_unlock_pd[0];
			m_unlock_pd_wr=m_unlock_pd[1];
		}

        // Create nl80211 socket for RSSI retrieval
        m_nl_sock_info = open_nl_socket(m_opts_ptr->dissemination_device);
		
		std::thread runningRxThr(&SocketClient::rxThr,this);
		// if(m_opts_ptr->rssi_aux_update_interval_msec>0) {
		// 	std::thread runningRouterOSRSSIThr(&SocketClient::routerOS_RSSI_retriever,this);
		// 	runningRouterOSRSSIThr.join();
		// }

		runningRxThr.join();

        free_nl_socket(m_nl_sock_info);
	}
}

void 
SocketClient::stopReception(void) {
	// m_terminate_routeros_rssi_flag=true;

	if(m_unlock_pd_rd!=-1 && m_unlock_pd_wr!=-1) {
		if(write(m_unlock_pd_wr,"\0",1)<0) {
			fprintf(stderr,"[ERROR] Failure to successfully execute stopReception(). The program behaviour may be unexpected from now on.\n");
		}
	}

	if(m_logfile_name!="" && m_logfile_name!="stdout") {
		fclose(m_logfile_file);
	}
}

void 
SocketClient::manageMessage(uint8_t *message_bin_buf,size_t bufsize) {
	etsiDecoder::etsiDecodedData_t decodedData;

	uint64_t bf = 0.0,af = 0.0;
	uint64_t main_bf = 0.0,main_af = 0.0;

	if(m_logfile_name!="") {
		main_bf=get_timestamp_ns();

		// This additional log line has been commented out to avoid being too verbose
		// fprintf(m_logfile_file,"[NEW MESSAGE RX]\n");
	}

	if(m_printMsg == true) {
		fprintf(stdout,"Rx msg: ");
		for(size_t sti=0;sti<bufsize;sti++) {
			fprintf(stdout,"%02X",message_bin_buf[sti]);
		}
		fprintf(stdout,"\n");
	}

	if(m_logfile_name!="") {
		bf=get_timestamp_ns();
	}

	// Decode the content of the message, using the decoder-module frontend class
	// m_decodeFrontend.setPrintPacket(true); // <- uncomment to print the bytes of each received message. Should be used for debug only, and should be kept disabled when deploying the S-LDM.
	if(m_decodeFrontend.decodeEtsi(message_bin_buf, bufsize, decodedData, etsiDecoder::decoderFrontend::MSGTYPE_AUTO)!=ETSI_DECODER_OK) {
		std::cerr << "Error! Cannot decode ETSI packet!" << std::endl;
		return;
	}

	if(m_logfile_name!="") {
		af=get_timestamp_ns();

		fprintf(m_logfile_file,"[LOG - MESSAGE DECODER (Client %s)] ProcTimeMilliseconds=%.6lf\n",m_client_id.c_str(),(af-bf)/1000000.0);
	}

	// If a CAM has been received, it should be used to update the internal in-memory database
	if(decodedData.type == etsiDecoder::ETSI_DECODED_CAM || decodedData.type == etsiDecoder::ETSI_DECODED_CAM_NOGN) {
		CAM_t *decoded_cam = (CAM_t *) decodedData.decoded_msg;

		double lat, lon;
		uint64_t stationID;

        // Signal to the Metric Supervisor that a CAM has been received
        if(m_met_sup_ptr!=nullptr) {
            m_met_sup_ptr->signalReceivedPacket(MessageId_cam);
        }
		
		lat = decoded_cam->cam.camParameters.basicContainer.referencePosition.latitude/10000000.0;
		lon = decoded_cam->cam.camParameters.basicContainer.referencePosition.longitude/10000000.0;
		stationID = decoded_cam->header.stationId;
		
		double l_inst_period=0.0;

		if(m_logfile_name!="") {
			bf=get_timestamp_ns();
		}

		// Update the database
		ldmmap::vehicleData_t vehdata;
		ldmmap::LDMMap::LDMMap_error_t db_retval;

		vehdata.exteriorLights = manage_LowfreqContainer(decodedData.decoded_msg,stationID);

		uint64_t gn_timestamp;
		if(decodedData.type == etsiDecoder::ETSI_DECODED_CAM) {
		    gn_timestamp = decodedData.gnTimestamp;
			// There is no need for an else if(), as we can enter here only if the decoded message type is either ETSI_DECODED_CAM or ETSI_DECODED_CAM_NOGN
		} else {
			gn_timestamp=UINT64_MAX;
            fprintf(stdout,"[WARNING] Current message contains no GN timestamp, ageCheck disabled.\n");
		}

		// Check the age of the data store inside the database (if the age check is enabled / -g option not specified)
		// before updating it with the new receive data
		if(gn_timestamp != UINT64_MAX) {
			ldmmap::LDMMap::returnedVehicleData_t retveh;

			if(m_db_ptr->lookup(stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
				// According to the standard: GNTimestamp = (TAI timestamp since 2004-01-01 00:00:00) % 4294967296
				// Due to the modulo operation, it is not enough to consider the difference between the received GNTimestamp and the one
				// stored in the database, as this may cause issues when receiving data and the GNTimestamp values are cyclically reset
				// at the same time
				// We thus check the "gap" between the received numbers. Let's consider for instance: stored=4294967291, rx=3
				// In this case the "rx" data is the most up-to-date, but a cyclical reset occurred
				// We can then compute gap = 3 - 4294967291 = -429467288 < -300000 (-5 minutes) - ok! We keep this data even if 3 < 4294967291
				// Let's consider instead:
				// stored=3, rx=4294967291
				// In this case 'rx' is not the most up to date data (it is impossible to have '3' stored in the database and then receive
				// '4294967291', unless clock jumps occur in the car, bacause after all that time the data corresponding to '3' would have already
				// been garbage cleaned from the database)
				// We can then compute gap = 4294967291 - 3 = 429467288 > 300000 (5 minutes) - no! We should dicard the data we just received
				// Let's consider now a "normal" scenario:
				// stored=3, rx=114
				// gap = 114 - 3 = 111 < 300000 - ok! The data is kept (it would be discarded only if gap > 300000)
				// Finally, let's briefly analyze a final scenario:
				// stored=4294967292, rx=4294967291
				// It is evident how the rx data should be discarded because older than the stored one
				// gap = rx - stored = 4294967291 - 4294967292 = -1 > -300000 (-5 minutes) - The data is correctly discarded due to the second
				// condition in the if() clause
				long long int gap = static_cast<long long int>(gn_timestamp)-static_cast<long long int>(retveh.vehData.gnTimestamp);

				if((gn_timestamp>retveh.vehData.gnTimestamp && gap>300000) ||
					(gn_timestamp<retveh.vehData.gnTimestamp && gap>-300000)) {
					if(m_logfile_name!="") {
						fprintf(m_logfile_file,"[LOG - DATABASE UPDATE (Client %s)] Message discarded (data is too old). Rx = %lu, Stored = %lu, Gap = %lld\n",
							m_client_id.c_str(),
							gn_timestamp,retveh.vehData.gnTimestamp,gap);
						return;
					}
				}
			}
		}

		vehdata.lon = lon;
		vehdata.lat = lat;
		vehdata.timestamp_us = get_timestamp_us();

		vehdata.elevation = decoded_cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue/100.0;

		if(decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue==HeadingValue_unavailable) {
			vehdata.heading=LDM_HEADING_UNAVAILABLE;
		} else {
			vehdata.heading = decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue/10.0;
		}

		vehdata.speed_ms = decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue/100.0;
		vehdata.camTimestamp = static_cast<long>(decoded_cam->cam.generationDeltaTime);
		vehdata.stationType = static_cast<ldmmap::e_StationTypeLDM>(decoded_cam->cam.camParameters.basicContainer.stationType);

        vehdata.stationType = ldmmap::StationType_LDM_passengerCar;
        vehdata.detected = false;

		vehdata.gnTimestamp = gn_timestamp;
		vehdata.stationID = stationID; // It is very important to save also the stationID
		memcpy(vehdata.macaddr,&(decodedData.GNaddress[0])+2,6); // Save the vehicle MAC address into the database (the MAC address is stored in the last 6 Bytes of the GN Address)

		// Retrieve, if available, the information on the RSSI for the vehicle corresponding to the MAC address of the sender
		// This is the RSSI on the CAM dissemination interface
        // Uncomment this to retrieve the RSSI via iw, instead of using nl80211 (requires iw to be available in the system)
		// vehdata.rssi_dBm=get_rssi_from_iw(vehdata.macaddr,std::string(m_opts_ptr->dissemination_device));
        if(m_nl_sock_info.sock_valid==true) {
            vehdata.rssi_dBm = get_rssi_from_netlink(vehdata.macaddr, m_nl_sock_info);
        } else {
            vehdata.rssi_dBm = RSSI_UNAVAILABLE;
        }

		if(decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth != VehicleWidth_unavailable) {
			vehdata.vehicleWidth = ldmmap::OptionalDataItem<long>(decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth*100);
		} else {
			vehdata.vehicleWidth = ldmmap::OptionalDataItem<long>(false);
		}

		if(decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue != VehicleLengthValue_unavailable) {
			vehdata.vehicleLength = ldmmap::OptionalDataItem<long>(decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue*100);
		} else {
			vehdata.vehicleLength = ldmmap::OptionalDataItem<long>(false);
		}

		// If logging is enabled, compute also an "instantaneous update period" metric (i.e., how much time has passed between two consecutive vehicle updates)
		if(m_logfile_name!="") {
			ldmmap::LDMMap::returnedVehicleData_t retveh;

			if(m_db_ptr->lookup(stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
				l_inst_period=(get_timestamp_us()-retveh.vehData.timestamp_us)/1000.0;
			} else {
				l_inst_period=-1.0;
			}
		}

		// std::cout << "[DEBUG] Updating vehicle with stationID: " << vehdata.stationID << std::endl;

		db_retval=m_db_ptr->insert(vehdata);

		if(db_retval!=ldmmap::LDMMap::LDMMAP_OK && db_retval!=ldmmap::LDMMap::LDMMAP_UPDATED) {
			std::cerr << "Warning! Insert on the database for vehicle " << (int) stationID << "failed!" << std::endl;
		}

		if(m_logfile_name!="") {
			af=get_timestamp_ns();

			fprintf(m_logfile_file,"[LOG - DATABASE UPDATE (Client %s)] LowFrequencyContainerAvail=%d InsertReturnValue=%d ProcTimeMilliseconds=%.6lf\n",
				m_client_id.c_str(),
					decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth != VehicleWidth_unavailable,
				db_retval,
				(af-bf)/1000000.0);
		}

		if(m_logfile_name!="") {
			main_af=get_timestamp_ns();

            // If a pointer to a VDPGPSClient has been specified, log also the current position of the receiver
            std::pair<double,double> latlon;
            if(m_gpsc_ptr!=nullptr) {
                latlon = m_gpsc_ptr->getCurrentPositionDbl();
            }

			logfprintf(m_logfile_file,std::string("FULL CAM PROCESSING (Client ") + m_client_id + std::string(")"),"StationID=%u Coordinates=%.7lf:%.7lf Heading=%.1lf InstUpdatePeriod=%.3lf"
				" CAMTimestamp=%ld GNTimestamp=%lu CAMTimestampDiff=%ld GNTimestampDiff=%ld"
				" ProcTimeMilliseconds=%.6lf MAC_Addr=%02X:%02X:%02X:%02X:%02X:%02X"
				" RSSI=%.2lf "
				" IPAddress=%s PublicIPAddress=%s",
				stationID,lat,lon,
				vehdata.heading,
				l_inst_period,
				vehdata.camTimestamp,vehdata.gnTimestamp,get_timestamp_ms_cam()-vehdata.camTimestamp,get_timestamp_ms_gn()-vehdata.gnTimestamp,
				(main_af-main_bf)/1000000.0,
				vehdata.macaddr[0],vehdata.macaddr[1],vehdata.macaddr[2],vehdata.macaddr[3],vehdata.macaddr[4],vehdata.macaddr[5],
				vehdata.rssi_dBm,
				vehdata.ipaddr.c_str(),
				vehdata.publicipaddr.c_str());

            if(m_gpsc_ptr!=nullptr) {
                fprintf(m_logfile_file," ReceiverCoordinates=%.7lf:%.7lf\n",latlon.first,latlon.second);
            } else {
                fprintf(m_logfile_file,"\n");
            }
			
			// fprintf(m_logfile_file,"[LOG - FULL CAM PROCESSING] StationID=%u Coordinates=%.7lf:%.7lf InstUpdatePeriod=%.3lf"
			// 	" CAMTimestamp=%ld GNTimestamp=%lu CAMTimestampDiff=%ld GNTimestampDiff=%ld"
			// 	" ProcTimeMilliseconds=%.6lf\n",
			// 	stationID,lat,lon,
			// 	l_inst_period,
			// 	vehdata.camTimestamp,vehdata.gnTimestamp,get_timestamp_ms_cam()-vehdata.camTimestamp,get_timestamp_ms_gn()-vehdata.gnTimestamp,
			// 	(main_af-main_bf)/1000000.0);	
		}
	} else if(denm_decoding_enabled==true && (decodedData.type == etsiDecoder::ETSI_DECODED_DENM || decodedData.type == etsiDecoder::ETSI_DECODED_DENM_NOGN)) {
		// For DENMs, if reception is enabled, print just some basic information, for the time being
		DENM_t *decoded_denm = (DENM_t *) decodedData.decoded_msg;

		// Buffer for the MAC address of the sending vehicle or RSU (in general, of the sending ITS-S)
		uint8_t sender_mac[6];

		if(decoded_denm!=nullptr) {
            // Signal to the Metric Supervisor that a DENM has been received
            if(m_met_sup_ptr!=nullptr) {
                m_met_sup_ptr->signalReceivedPacket(MessageId_denm);
            }

			// Get the stationID from the DENM
			long stationID = decoded_denm->header.stationId;

			// Get the event reference latitude
			double reference_latitude = decoded_denm->denm.management.eventPosition.latitude/1e7;

			// Get the event reference longitude
			double reference_longitude = decoded_denm->denm.management.eventPosition.longitude/1e7;

			// Get the GN timestamp
			uint32_t gn_ts = decodedData.gnTimestamp;

			// Get the MAC address of the sending ITS-S (the MAC address is stored in the last 6 Bytes of the GN Address)
			memcpy(sender_mac,&(decodedData.GNaddress[0])+2,6);

			// // Use the MAC address to get the signal level through iw
			// // A return value of RSSI_UNAVAILABLE (-80000) means that no RSSI value could be retrieved at present time
            // Uncomment this to retrieve the RSSI via iw, instead of using nl80211 (requires iw to be available in the system)
            // double rssi = get_rssi_from_iw(sender_mac,std::string(m_opts_ptr->dissemination_device));
            double rssi = RSSI_UNAVAILABLE;
            if(m_nl_sock_info.sock_valid==true) {
                rssi = get_rssi_from_netlink(sender_mac, m_nl_sock_info);
            }

			if(m_logfile_name!="") {
				fprintf(m_logfile_file,"[DENM] Current_timestamp=%" PRIu64 " StationID=%ld RSSI_dbm=%.1lf GN_ts=%" PRIu32 " Ref_lat=%.7lf Ref_lon=%-7lf Sender_MAC=%02X:%02X:%02X:%02X:%02X:%02X",
					get_timestamp_us(),
					stationID,
					rssi,
					gn_ts,
					reference_latitude,
					reference_longitude,
					sender_mac[0],sender_mac[1],sender_mac[2],sender_mac[3],sender_mac[4],sender_mac[5]);
				fflush(m_logfile_file);
			} else {
				printf("[DENM] Current_timestamp=%" PRIu64 " StationID=%ld RSSI_dbm=%.1lf GN_ts=%" PRIu32 " Ref_lat=%.7lf Ref_lon=%-7lf\n Sender_MAC=%02X:%02X:%02X:%02X:%02X:%02X",
					get_timestamp_us(),
					stationID,
					rssi,
					gn_ts,
					reference_latitude,
					reference_longitude,
					sender_mac[0],sender_mac[1],sender_mac[2],sender_mac[3],sender_mac[4],sender_mac[5]);
			}

			if(m_logfile_name!="") {
				fprintf(m_logfile_file,"\n");
				fflush(m_logfile_file);
			} else {
				printf("\n");
			}

			ASN_STRUCT_FREE(asn_DEF_DENM,decoded_denm);
		}
	} else if(decodedData.type == etsiDecoder::ETSI_DECODED_VAM || decodedData.type == etsiDecoder::ETSI_DECODED_VAM_NOGN) {
		VAM_t *decoded_vam = (VAM_t *) decodedData.decoded_msg;

		double lat, lon;
		uint64_t stationID;

        // Signal to the Metric Supervisor that a VAM has been received
        if(m_met_sup_ptr!=nullptr) {
            m_met_sup_ptr->signalReceivedPacket(MessageId_vam);
        }
		
		lat = decoded_vam->vam.vamParameters.basicContainer.referencePosition.latitude/10000000.0;
		lon = decoded_vam->vam.vamParameters.basicContainer.referencePosition.longitude/10000000.0;
		stationID = decoded_vam->header.stationId;
		
		double l_inst_period=0.0;

		if(m_logfile_name!="") {
			bf=get_timestamp_ns();
		}

		// Update the database
		ldmmap::vehicleData_t vehdata;
		ldmmap::LDMMap::LDMMap_error_t db_retval;
		
		uint64_t gn_timestamp;
		if(decodedData.type == etsiDecoder::ETSI_DECODED_VAM) {
		    gn_timestamp = decodedData.gnTimestamp;
		    // There is no need for an else if(), as we can enter here only if the decoded message type is either ETSI_DECODED_VAM or ETSI_DECODED_VAM_NOGN
		} else {
			gn_timestamp=UINT64_MAX;
            fprintf(stdout,"[WARNING] Current message contains no GN timestamp, ageCheck disabled.\n");
		}
		
		// Check the age of the data store inside the database (if the age check is enabled / -g option not specified)
		// before updating it with the new receive data
		if(gn_timestamp != UINT64_MAX){
			ldmmap::LDMMap::returnedVehicleData_t retveh;

			if(m_db_ptr->lookup(stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
				long long int gap = static_cast<long long int>(gn_timestamp)-static_cast<long long int>(retveh.vehData.gnTimestamp);

				if((gn_timestamp>retveh.vehData.gnTimestamp && gap>300000) ||
					(gn_timestamp<retveh.vehData.gnTimestamp && gap>-300000)) {
					if(m_logfile_name!="") {
						fprintf(m_logfile_file,"[LOG - DATABASE UPDATE (Client %s)] Message discarded (data is too old). Rx = %lu, Stored = %lu, Gap = %lld\n",
							m_client_id.c_str(),
							gn_timestamp,retveh.vehData.gnTimestamp,gap);
						return;
					}
				}
			}
		}
		
		vehdata.lon = lon;
		vehdata.lat = lat;
		vehdata.timestamp_us = get_timestamp_us();
		
		vehdata.elevation = decoded_vam->vam.vamParameters.basicContainer.referencePosition.altitude.altitudeValue/100.0;

		if(decoded_vam->vam.vamParameters.vruHighFrequencyContainer.heading.value==Wgs84AngleValue_unavailable) {
			vehdata.heading = LDM_HEADING_UNAVAILABLE;
		} else {
			vehdata.heading = decoded_vam->vam.vamParameters.vruHighFrequencyContainer.heading.value/10.0;
		}
		

		vehdata.speed_ms = decoded_vam->vam.vamParameters.vruHighFrequencyContainer.speed.speedValue/100.0;
		vehdata.camTimestamp = static_cast<long>(decoded_vam->vam.generationDeltaTime);
		vehdata.stationType = static_cast<ldmmap::e_StationTypeLDM>(decoded_vam->vam.vamParameters.basicContainer.stationType);
		
		vehdata.gnTimestamp = gn_timestamp;
		vehdata.stationID = stationID; // It is very important to save also the stationID
		memcpy(vehdata.macaddr,&(decodedData.GNaddress[0])+2,6); // Save the vehicle MAC address into the database (the MAC address is stored in the last 6 Bytes of the GN Address)
		
		// Retrieve, if available, the information on the RSSI for the vehicle corresponding to the MAC address of the sender
		// This is the RSSI on the VAM dissemination interface
        // Uncomment this to retrieve the RSSI via iw, instead of using nl80211 (requires iw to be available in the system)
        // vehdata.rssi_dBm=get_rssi_from_iw(vehdata.macaddr,std::string(m_opts_ptr->dissemination_device.c_str()));
        if(m_nl_sock_info.sock_valid==true) {
            vehdata.rssi_dBm = get_rssi_from_netlink(vehdata.macaddr, m_nl_sock_info);
        } else {
            vehdata.rssi_dBm = RSSI_UNAVAILABLE;
        }
		
		vehdata.vehicleWidth = ldmmap::OptionalDataItem<long>(false);
		vehdata.vehicleLength = ldmmap::OptionalDataItem<long>(false);
		
		// If logging is enabled, compute also an "instantaneous update period" metric (i.e., how much time has passed between two consecutive vehicle updates)
		if(m_logfile_name!="") {
			ldmmap::LDMMap::returnedVehicleData_t retveh;

			if(m_db_ptr->lookup(stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
				l_inst_period=(get_timestamp_us()-retveh.vehData.timestamp_us)/1000.0;
			} else {
				l_inst_period=-1.0;
			}

			// If a pointer to a VDPGPSClient has been specified, log also the current position of the receiver
			std::pair<double,double> latlon;
			if(m_gpsc_ptr!=nullptr) {
				latlon = m_gpsc_ptr->getCurrentPositionDbl();
			}

			if(m_logfile_name!="") {
				logfprintf(m_logfile_file,std::string("FULL VAM PROCESSING (Client ") + m_client_id + std::string(")"),"StationID=%u Coordinates=%.7lf:%.7lf Heading=%.1lf InstUpdatePeriod=%.3lf"
					" MAC_Addr=%02X:%02X:%02X:%02X:%02X:%02X"
					" RSSI=%.2lf",
					stationID,lat,lon,
					vehdata.heading,
					l_inst_period,
					vehdata.macaddr[0],vehdata.macaddr[1],vehdata.macaddr[2],vehdata.macaddr[3],vehdata.macaddr[4],vehdata.macaddr[5],
					vehdata.rssi_dBm);
			}

			if(m_gpsc_ptr!=nullptr) {
				fprintf(m_logfile_file," ReceiverCoordinates=%.7lf:%.7lf\n",latlon.first,latlon.second);
			} else {
				fprintf(m_logfile_file,"\n");
			}
		}
		
		db_retval=m_db_ptr->insert(vehdata);

		if(db_retval!=ldmmap::LDMMap::LDMMAP_OK && db_retval!=ldmmap::LDMMap::LDMMAP_UPDATED) {
			std::cerr << "Warning! Insert on the database for VRU " << (int) stationID << "failed!" << std::endl;
		}
		
		ASN_STRUCT_FREE(asn_DEF_VAM,decoded_vam);
	} else if(decodedData.type == etsiDecoder::ETSI_DECODED_CPM || decodedData.type == etsiDecoder::ETSI_DECODED_CPM_NOGN)   {
        uint64_t fromStationID;

        // Signal to the Metric Supervisor that a CPM has been received
        if(m_met_sup_ptr!=nullptr) {
            m_met_sup_ptr->signalReceivedPacket(MessageId_cpm);
        }

        double fromLat = asn1cpp::getField (decodedData.decoded_cpm->payload.managementContainer.referencePosition.latitude, double) / 10000000.0;
        double fromLon = asn1cpp::getField (decodedData.decoded_cpm->payload.managementContainer.referencePosition.longitude, double) / 10000000.0;

        fromStationID = asn1cpp::getField (decodedData.decoded_cpm->header.stationId, uint64_t);

        double l_inst_period=0.0;

        if(m_logfile_name!="") {
            bf=get_timestamp_ns();
        }

        int wrappedContainer_size = asn1cpp::sequenceof::getSize(decodedData.decoded_cpm->payload.cpmContainers);
        for (int i=0; i<wrappedContainer_size; i++)
        {
            auto wrappedContainer = asn1cpp::sequenceof::getSeq(decodedData.decoded_cpm->payload.cpmContainers,WrappedCpmContainer,i);
            WrappedCpmContainer__containerData_PR present = asn1cpp::getField(wrappedContainer->containerData.present,WrappedCpmContainer__containerData_PR);
            if(present == WrappedCpmContainer__containerData_PR_PerceivedObjectContainer)
            {
                auto POcontainer = asn1cpp::getSeq(wrappedContainer->containerData.choice.PerceivedObjectContainer,PerceivedObjectContainer);
                int PObjects_size = asn1cpp::sequenceof::getSize(POcontainer->perceivedObjects);
                for(int j=0; j<PObjects_size;j++)
                {
                    ldmmap::LDMMap::returnedVehicleData_t PO_ret_data;
                    auto PO_seq = asn1cpp::makeSeq(PerceivedObject);
                    PO_seq = asn1cpp::sequenceof::getSeq(POcontainer->perceivedObjects,PerceivedObject,j);

                    //Translate to ego vehicle coordinates
                    ldmmap::vehicleData_t PO_data;
                    PO_data.detected = true;
                    PO_data.vehicleLength = asn1cpp::getField(PO_seq->objectDimensionX->value,long);
                    PO_data.vehicleWidth = asn1cpp::getField(PO_seq->objectDimensionY->value,long);
                    PO_data.heading = asn1cpp::getField(PO_seq->angles->zAngle.value,double) / DECI;
                    PO_data.xSpeed = asn1cpp::getField(PO_seq->velocity->choice.cartesianVelocity.xVelocity.value,long);
                    PO_data.xSpeed = asn1cpp::getField(PO_seq->velocity->choice.cartesianVelocity.yVelocity.value,long);
                    PO_data.speed_ms = (sqrt (pow(PO_data.xSpeed,2) +
                                             pow(PO_data.ySpeed,2)))/CENTI;

                    double lonPO, latPO, from_x,from_y,xDistance, yDistance;
                    double gammar=0;
                    double kr=0;
                    xDistance = asn1cpp::getField(PO_seq->position.xCoordinate.value,double)/100;
                    yDistance = asn1cpp::getField(PO_seq->position.yCoordinate.value,double)/100;
                    transverse_mercator_t tmerc = UTMUPS_init_UTM_TransverseMercator();
                    TransverseMercator_Forward(&tmerc, fromLon, fromLat, fromLon, &from_x, &from_y, &gammar, &kr);
                    from_x += xDistance;
                    from_y += yDistance;
                    TransverseMercator_Reverse(&tmerc, fromLon, from_x, from_y, &latPO, &lonPO, &gammar, &kr);
                    PO_data.lat = latPO;
                    PO_data.lon = lonPO;
                    PO_data.camTimestamp = static_cast<long>(asn1cpp::getField(decodedData.decoded_cpm->payload.managementContainer.referenceTime,long)) - static_cast<long>(asn1cpp::getField(PO_seq->measurementDeltaTime,long));
                    PO_data.perceivedBy = asn1cpp::getField(decodedData.decoded_cpm->header.stationId,long);
                    PO_data.stationType = ldmmap::StationType_LDM_detectedPassengerCar;

                    if(m_recvCPMmap[fromStationID].find(asn1cpp::getField(PO_seq->objectId,long)) == m_recvCPMmap[fromStationID].end()){
                        // First time we have received this object from this vehicle
                        //If PO id is already in local copy of LDM
                        if(m_db_ptr->lookup(asn1cpp::getField(PO_seq->objectId,long),PO_ret_data) == ldmmap::LDMMap::LDMMAP_OK)
                        {
                            // We need a new ID for object
                            std::set<uint64_t> IDs;
                            m_db_ptr->getAllIDs (IDs);
                            int newID = 1;
                            for (int num : IDs) {
                                if (num == newID) {
                                    ++newID;
                                } else if (num > newID) {
                                    break;
                                }
                            }
                            //Update recvCPMmap
                            m_recvCPMmap[fromStationID][asn1cpp::getField(PO_seq->objectId,long)] = newID;
                            PO_data.stationID = newID;
                        }
                        else
                        {
                            //Update recvCPMmap
                            m_recvCPMmap[fromStationID][asn1cpp::getField(PO_seq->objectId,long)] = asn1cpp::getField(PO_seq->objectId,long);
                            PO_data.stationID = asn1cpp::getField(PO_seq->objectId,long);
                        }
                    }
                    else
                    {
                        PO_data.stationID = m_recvCPMmap[fromStationID][asn1cpp::getField(PO_seq->objectId,long)];
                    }

                    ldmmap::LDMMap::LDMMap_error_t db_retval;

                    uint64_t gn_timestamp;
                    if(decodedData.type == etsiDecoder::ETSI_DECODED_CPM) {
                        gn_timestamp = decodedData.gnTimestamp;
                        // There is no need for an else if(), as we can enter here only if the decoded message type is either ETSI_DECODED_CAM or ETSI_DECODED_CAM_NOGN
                    } else {
                        gn_timestamp=UINT64_MAX;
                        fprintf(stdout,"[WARNING] Current message contains no GN timestamp, ageCheck disabled.\n");
                    }

                    // Check the age of the data store inside the database (if the age check is enabled / -g option not specified)
                    // before updating it with the new receive data
                    if(gn_timestamp != UINT64_MAX) {
                        ldmmap::LDMMap::returnedVehicleData_t retveh;

                        if(m_db_ptr->lookup(PO_data.stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
                            /* According to the standard: GNTimestamp = (TAI timestamp since 2004-01-01 00:00:00) % 4294967296
                            // Due to the modulo operation, it is not enough to consider the difference between the received GNTimestamp and the one
                            // stored in the database, as this may cause issues when receiving data and the GNTimestamp values are cyclically reset
                            // at the same time
                            // We thus check the "gap" between the received numbers. Let's consider for instance: stored=4294967291, rx=3
                            // In this case the "rx" data is the most up-to-date, but a cyclical reset occurred
                            // We can then compute gap = 3 - 4294967291 = -429467288 < -300000 (-5 minutes) - ok! We keep this data even if 3 < 4294967291
                            // Let's consider instead:
                            // stored=3, rx=4294967291
                            // In this case 'rx' is not the most up to date data (it is impossible to have '3' stored in the database and then receive
                            // '4294967291', unless clock jumps occur in the car, bacause after all that time the data corresponding to '3' would have already
                            // been garbage cleaned from the database)
                            // We can then compute gap = 4294967291 - 3 = 429467288 > 300000 (5 minutes) - no! We should dicard the data we just received
                            // Let's consider now a "normal" scenario:
                            // stored=3, rx=114
                            // gap = 114 - 3 = 111 < 300000 - ok! The data is kept (it would be discarded only if gap > 300000)
                            // Finally, let's briefly analyze a final scenario:
                            // stored=4294967292, rx=4294967291
                            // It is evident how the rx data should be discarded because older than the stored one
                            // gap = rx - stored = 4294967291 - 4294967292 = -1 > -300000 (-5 minutes) - The data is correctly discarded due to the second
                            // condition in the if() clause
                             */
                            long long int gap = static_cast<long long int>(gn_timestamp)-static_cast<long long int>(retveh.vehData.gnTimestamp);

                            if((gn_timestamp>retveh.vehData.gnTimestamp && gap>300000) ||
                               (gn_timestamp<retveh.vehData.gnTimestamp && gap>-300000)) {
                                if(m_logfile_name!="") {
                                    fprintf(m_logfile_file,"[LOG - DATABASE UPDATE (Client %s)] Message discarded (data is too old). Rx = %lu, Stored = %lu, Gap = %lld\n",
                                            m_client_id.c_str(),
                                            gn_timestamp,retveh.vehData.gnTimestamp,gap);
                                    return;
                                }
                            }
                        }
                    }

                    PO_data.timestamp_us = get_timestamp_us();
                    memcpy(PO_data.macaddr,&(decodedData.GNaddress[0])+2,6); // Save the vehicle MAC address into the database (the MAC address is stored in the last 6 Bytes of the GN Address)

                    // Retrieve, if available, the information on the RSSI for the vehicle corresponding to the MAC address of the sender
                    // This is the RSSI on the CPM dissemination interface
                    // Uncomment this to retrieve the RSSI via iw, instead of using nl80211 (requires iw to be available in the system)
                    // PO_data.rssi_dBm=get_rssi_from_iw(PO_data.macaddr,std::string(m_opts_ptr->dissemination_device.c_str()));
                    if(m_nl_sock_info.sock_valid==true) {
                        PO_data.rssi_dBm = get_rssi_from_netlink(PO_data.macaddr, m_nl_sock_info);
                    } else {
                        PO_data.rssi_dBm = RSSI_UNAVAILABLE;
                    }

                    if(m_logfile_name!="") {
                        ldmmap::LDMMap::returnedVehicleData_t retveh;

                        if(m_db_ptr->lookup(PO_data.stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
                            l_inst_period=(get_timestamp_us()-retveh.vehData.timestamp_us)/1000.0;
                        } else {
                            l_inst_period=-1.0;
                        }

                        // If a pointer to a VDPGPSClient has been specified, log also the current position of the receiver
                        std::pair<double,double> latlon;
                        if(m_gpsc_ptr!=nullptr) {
                            latlon = m_gpsc_ptr->getCurrentPositionDbl();
                        }

                        if(m_logfile_name!="") {
                            logfprintf(m_logfile_file,std::string("FULL CPM Perceived Object PROCESSING (Client ") + m_client_id + std::string(")"),"StationID=%u Coordinates=%.7lf:%.7lf Heading=%.1lf InstUpdatePeriod=%.3lf"
                                                                                                                                   " MAC_Addr=%02X:%02X:%02X:%02X:%02X:%02X"
                                                                                                                                   " RSSI=%.2lf",
                                       PO_data.stationID,PO_data.lat,PO_data.lon,
                                       PO_data.heading,
                                       l_inst_period,
                                       PO_data.macaddr[0],PO_data.macaddr[1],PO_data.macaddr[2],PO_data.macaddr[3],PO_data.macaddr[4],PO_data.macaddr[5],
                                       PO_data.rssi_dBm);
                        }

                        if(m_gpsc_ptr!=nullptr) {
                            fprintf(m_logfile_file," ReceiverCoordinates=%.7lf:%.7lf\n",latlon.first,latlon.second);
                        } else {
                            fprintf(m_logfile_file,"\n");
                        }
                    }
                    db_retval=m_db_ptr->insert(PO_data);

                    if(db_retval!=ldmmap::LDMMap::LDMMAP_OK && db_retval!=ldmmap::LDMMap::LDMMAP_UPDATED) {
                        std::cerr << "[WARN] Warning! Insert on the database for Perceived Object " << (int) PO_data.stationID << "failed!" << std::endl;
                    }
                }
            }
        }
    } else {
		std::cerr << "[WARN] Warning! Only CAM, CPM and VAM messages (and, optionally, DENMs) are supported for the time being!" << std::endl;
		return;
	}
}
