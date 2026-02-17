//
// Created by Carlos Mateo Risma Carletti on 06/05/21.
// Updated by Francesco Raviglione in November 2021
//

#ifndef OSCAR_GEONET_H
#define OSCAR_GEONET_H

#include <cstdint>
#include <map>
#include <set>
#include <mutex>
#include <shared_mutex>
#include "gpsc.h"
#include "security.h"
#include "basicHeader.h"
#include "commonHeader.h"
#include "shbHeader.h"
#include "gbcHeader.h"
#include "DCC.h"
#include "ATManager.h"



#define GN_ETHERTYPE 0x8947

class GeoNet {
	public:

        typedef struct LocationTableExtension{
            std::vector<std::tuple<double , double>> CBR_R0_Hop;
            std::vector<std::tuple<double , double>> CBR_R1_Hop;
        } LocationTableExtension;

        typedef struct _LocTableEntry {
            /**
            *   ETSI EN 302 636-4-1 [8.1.2]
            */
            char GN_ADDR [8];
            char LL_ADDR [6];
            uint8_t type;
            uint8_t version;
            GNlpv_t lpv; //! long position vector
            bool LS_PENDING;
            bool IS_NEIGHBOUR;
            std::set<uint16_t> DPL; //! Duplicate packet list
            long timestamp;
            uint32_t PDR;
            LocationTableExtension cbr_extension;
        } GNLocTE;

        typedef struct _egoPositionVector {
            /**
            *   ETSI EN 302 636-4-1 [8.2.2]
            */
            VDPGPSClient::VRU_position_latlon POS_EPV;
            double S_EPV;
            double H_EPV;
            long TST_EPV;
            uint32_t PAI_EPV;
        }GNegoPV;

		GeoNet();

		~GeoNet();

		void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype);
		void setStationID(unsigned long fixed_stationid);
		void setStationType(long fixed_stationtype);
		void setVDP(VDPGPSClient* vdp);
		void setVRUdp(VDPGPSClient* vrudp);
		void setSocketTx(int socket_tx_descr,int ifindex,uint8_t srcmac[6]);
		std::tuple<GNDataConfirm_t, MessageId_t> sendGN(GNDataRequest_t dataRequest, int priority, MessageId_t message_id);
		
		gnError_e decodeGN(unsigned char * packet, GNDataIndication_t* dataIndication);

		void setLogFile(std::string msgfile) {m_log_filename=msgfile;}
		void setLogFile2(const std::string &filename);
		int openUDPsocket(std::string udp_sock_addr,std::string interface_ip,bool extra_position_udp=false);
		void closeUDPsocket();
        void setSecurity(bool security){
            enableSecurity = security;
            m_security.setMessageType(m_messageType);
        }
        void setMessageType(int type){m_messageType = type; m_security.setMessageType(type);}
		void setATmanager(ATManager *atm){m_atmanager = atm;};


		void setDCC(DCC *dcc) {m_dcc = dcc; attachSendFromDCCQueue(); attachGlobalCBRCheck();}
        void attachSendFromDCCQueue();
        void attachGlobalCBRCheck();
		void writeEndRowLogRX (MessageId_t msg_id);
	private:
		typedef struct _extralatlon_t {
			int32_t lat;
			int32_t lon;
		} extralatlon_t;

		GNDataConfirm_t sendSHB(GNDataRequest_t dataRequest,commonHeader commonHeader,basicHeader basicHeader,GNlpv_t longPV);
		GNDataConfirm_t sendGBC(GNDataRequest_t dataRequest,commonHeader commonHeader, basicHeader basicHeader,GNlpv_t longPV);
		GNDataIndication_t* processSHB(GNDataIndication_t* dataIndication);
		GNDataIndication_t* processGBC(GNDataIndication_t* dataIndication, uint8_t shape);
		
		uint8_t encodeLT (double seconds);
		bool decodeLT(uint8_t lifeTime, double * seconds);
		bool isInsideGeoArea(GeoArea_t geoArea);
		void MakeManagedconfiguredAddress (uint8_t addr[8], uint8_t ITSType, uint8_t out_addr[8]);

		uint16_t m_seqNumber; //! ETSI EN 302 636-4-1 [8.3]

		uint8_t m_GNAddress[8];

		int m_socket_tx=-1;

        Security m_security;
		ATManager *m_atmanager;

        bool enableSecurity;
		bool isCertificate;
		int m_messageType;


        FILE* f_out = nullptr; // Log file pointer
        std::string m_log_filename2 = "dis";

		VDPGPSClient* m_vdp;
        VDPGPSClient* m_vrudp;
		StationId_t m_station_id;
		StationType_t m_stationtype;

		// OScar-specific attributes
		uint8_t m_mac_src[6] = {0};
		int m_ifindex=-1;

		//ETSI 302 636-4-1 ANNEX H: GeoNetworking protocol constans
		uint8_t m_GnLocalGnAddr[6] = {0};
		uint8_t m_GnLocalAddrCongMethod = 1; //! MANAGED
		uint8_t m_GnPtotocolVersion = 1;
		bool m_GnIsMobile=true; //!To set wether if Mobile(1) or Stationary(0)
		uint8_t m_GnIfType = 1;
		double m_GnMinUpdateFrequencyEPV = 1000;
		uint32_t m_GnPaiInterval = 80;
		uint32_t m_GnMaxSduSize = 1398;
		uint8_t m_GnMaxGeoNetworkingHeaderSize = 88;
		uint8_t m_GnLifeTimeLocTE = 20; //! seconds
		uint8_t m_GnSecurity = 0; //!Disabled
		uint8_t m_GnSnDecapResultHandling = 0; //!STRICT
		uint8_t m_GnLocationServiceMaxRetrans = 10;
		uint16_t m_GnLocationServiceRetransmitTimer = 1000;
		uint16_t m_GnLocationServicePacketBufferSize = 1024;
		uint16_t m_GnBeaconServiceRetransmitTimer = 3000;
		uint16_t m_GnBeaconServiceMaxJItter = m_GnBeaconServiceRetransmitTimer/4;
		uint8_t m_GnDefaultHopLimit = 10;
		uint8_t m_GnDPLLength = 8;
		uint16_t m_GNMaxPacketLifetime = 600;
		uint8_t m_GnDefaultPacketLifetime = 60 ; // seconds (0xf2)
		uint16_t m_GNMaxPacketDataRate = 100;
		uint16_t m_GNMaxPacketDataRateEmaBeta = 90;
		uint16_t m_GNMaxGeoAreaSize = 10;
		uint16_t m_GNMinPacketRepetitionInterval = 100;
		uint16_t m_GNNonAreaForwardingAlgorithm = 1; //GREEDY
		uint16_t m_GNAreaForwardingAlgorithm = 1;
		uint16_t m_GNCbfMinTime = 1;
		uint16_t m_GNCbfMaxTime = 100;
		uint16_t m_GnDefaultMaxCommunicationRange = 1000;
		uint16_t m_GnBroadcastCBFDefSectorAngle = 30;
		uint16_t m_GnUcForwardingPacketBufferSize = 256;
		uint16_t m_GnBcForwardingPacketBufferSize = 1024;
		uint16_t m_FnCbfPacketBufferSize = 256;
		uint16_t m_GnDefaultTrafficClass = 0;
		bool m_RSU_epv_set = false;
		
		std::string m_log_filename = "dis";

		int m_udp_sockfd = -1;

		// If this flag is set to "true" and UDP packets are sent by the GeoNetworking layer, in addition to the
		// standard-compliant message dissemination, each UDP packet will contain an extra information at the
		// beginning of the payload, before the actual message
		// This extra information is represented by 64 additional bits, containing respectively the current
		// latitude (32 bits) and longitude (32 bits) of the vehicle, as degrees*1e7 and in network byte order
		bool m_extra_position_udp = false;

		DCC *m_dcc = nullptr;
		// CBRReader m_cbr_reader;
        double m_GNLocTTimerCBR_ms = 1000.0;

        std::shared_mutex m_LocT_Mutex;
        std::map<uint64_t, GNLocTE> m_GNLocT;

		std::string m_GN_log_file_tx = "GN_log_TX.csv";
		std::string m_GN_log_file_rx = "GN_log_RX.csv";
		std::mutex m_mutex_log_rx; 
};

#endif // OSCAR_GEONET_H
