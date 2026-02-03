//
// Created by Carlos Mateo Risma Carletti on 06/05/21.
// Updated by Francesco Raviglione in November 2021
//

#include <iostream>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <iomanip>
#include "functional"
#include "geonet.h"

#define IEEE80211_DATA_PKT_HDR_LEN 24
#ifndef IEEE80211_FCS_LEN 
#define IEEE80211_FCS_LEN 4
#endif

#define SN_MAX 65536

// This funciton fills the out_addr 8-byte array with a GN address, starting from the local MAC address specified as "addr", and from the current stationType
void 
GeoNet::MakeManagedconfiguredAddress (uint8_t addr[6], uint8_t ITSType, uint8_t out_addr[8]) {
	//ETSI EN 302 636-4-1 [6.3]
	out_addr[0] = 0x00 | ITSType << 2; //Initial GeoNetAddress ->M=0 and 5bit ITS-S type
	out_addr[1] = 0x00; //Reserved
	memcpy(out_addr + 2, addr, 6);
}

GeoNet::GeoNet()
{
	m_socket_tx = -1;
	m_station_id = ULONG_MAX;
	m_stationtype = LONG_MAX;
	m_seqNumber = 0;
    enableSecurity = false;
    isCertificate = false;
    m_messageType = 0;
	// m_GNAddress = GNAddress(); Should be already initialized to all zeros

	m_RSU_epv_set=false;

	std::ofstream file;
	file.open(m_GN_log_file_tx, std::ios::out);
	file << "timestamp_ms,gn_addr,tst,gn_lat,gn_lon,message_id,state\n";
	file.close();
	file.open(m_GN_log_file_rx, std::ios::out);
	file << "timestamp_ms,gn_addr,tst,gn_lat,gn_lon,message_id,state\n";
	file.close();
}

GeoNet::~GeoNet() {
    if (f_out != nullptr) {
        fclose(f_out);
    }
	closeUDPsocket();
}

void
GeoNet::setStationProperties(unsigned long fixed_stationid,long fixed_stationtype) {
	m_station_id=fixed_stationid;
	m_stationtype=fixed_stationtype;

	if(fixed_stationtype==StationType_roadSideUnit) {
		m_GnIsMobile=false;
	}

	// ETSI EN 302 636-4-1 [10.2.2] : the egoPV shall be updated with a minimum freq of th GN constant itsGNminUpdateFrequencyEPV
	MakeManagedconfiguredAddress (m_GnLocalGnAddr,m_stationtype,m_GNAddress); //! Initial address config on MANAGED(1) mode ETSI EN 302 636-4-1 [10.2.1.3]
}

void GeoNet::setLogFile2(const std::string &filename) {
    m_log_filename2 = filename;
    if (m_log_filename2 != "dis" && m_log_filename2 != "") {
        char file[filename.size() + 1];
        snprintf(file, sizeof(file), "%s", filename.c_str());

        f_out = fopen(file, "w");
        if (f_out == nullptr) {
            std::cerr << "Cannot open log file for writing." << std::endl;
        }
    }
}


void
GeoNet::setStationID(unsigned long fixed_stationid)
{
	m_station_id=fixed_stationid;
}

void
GeoNet::setStationType(long fixed_stationtype)
{
	m_stationtype=fixed_stationtype;
	if(fixed_stationtype==StationType_roadSideUnit) m_GnIsMobile=false;
}

void
GeoNet::setVDP (VDPGPSClient* vdp)
{
	m_vdp = vdp;
}

void
GeoNet::setVRUdp (VDPGPSClient* vrudp)
{
	m_vrudp = vrudp;
}

void
GeoNet::setSocketTx(int socket_tx_descr,int ifindex,uint8_t srcmac[6]) {
	m_socket_tx = socket_tx_descr;
	m_ifindex = ifindex;
	// ETSI EN 302 636-4-1 [10.2.1.3.2]

	memcpy(m_GnLocalGnAddr,srcmac,6);
	memcpy(m_mac_src,srcmac,6);
}

uint8_t
GeoNet::encodeLT (double seconds)
{
uint8_t base,multiplier,retval;
//Encoding of lifeTime field for Basic Header as specified in ETSI EN 302 636-4-1 [9.6.4]
	if (seconds >= 630.0) {
		base = 0x03;
		multiplier = seconds / 100.0;
	} else if (seconds >= 63.0) {
		base = 0x02;
		multiplier = seconds / 10.0;
	} else if (seconds >= 3.15) {
		base = 0x01;
		multiplier = seconds / 1.0;
	} else {
		base = 0x00;
		multiplier = seconds / 0.050;
	}

	retval = (multiplier << 2) | base;
	return retval;
}

bool GeoNet::decodeLT (uint8_t lifeTime, double*seconds)
{
        uint8_t base,multiplier;

        base = lifeTime & 0x03;
        multiplier = (lifeTime & 0xFC) >> 2;

        switch (base)
        {
            case 0:
                *seconds = multiplier * 0.050;
                break;
            case 1:
                *seconds = multiplier * 1; // Put 1 just for completion
                break;
            case 2:
                *seconds = multiplier * 10.0;
                break;
            case 3:
                *seconds = multiplier * 100.0;
                break;
            default:
                return false;
                break;
        };

        return true;
}

std::tuple<GNDataConfirm_t, MessageId_t>
GeoNet::sendGN (GNDataRequest_t dataRequest, int priority, MessageId_t message_id) {
	GNDataConfirm_t dataConfirm = ACCEPTED;
	basicHeader basicHeader;
	commonHeader commonHeader;
	GNlpv_t longPV;

	if(m_socket_tx==-1 || m_ifindex==-1 || 
		(
		m_mac_src[0]==0x00 &&
		m_mac_src[1]==0x00 &&
		m_mac_src[2]==0x00 &&
		m_mac_src[3]==0x00 &&
		m_mac_src[4]==0x00 &&
		m_mac_src[5]==0x00
		)
	) {
		std::cerr << "GeoNetworking error: either no socket, no interface index or no source MAC address are available. Initialize them first before calling sendGN()!" << std::endl;
		return std::tuple<GNDataConfirm_t, MessageId_t>(UNSPECIFIED_ERROR, message_id);
	}

	if(m_stationtype==StationType_roadSideUnit && m_RSU_epv_set==false)	{
		std::cerr << "GeoNetworking error: no position has been set for an RSU object. Please use setFixedPositionRSU() on the Facilities Layer object." << std::endl;
		return std::tuple<GNDataConfirm_t, MessageId_t>(UNSPECIFIED_ERROR, message_id);
	}

	if(dataRequest.lenght > m_GnMaxSduSize) {
		return std::tuple<GNDataConfirm_t, MessageId_t>(MAX_LENGHT_EXCEEDED, message_id);
	} else if(dataRequest.GNMaxLife > m_GNMaxPacketLifetime) {
		return std::tuple<GNDataConfirm_t, MessageId_t>(MAX_LIFE_EXCEEDED, message_id);
	} else if(dataRequest.GNRepInt != 0 && dataRequest.GNRepInt < m_GNMinPacketRepetitionInterval) {
		return std::tuple<GNDataConfirm_t, MessageId_t>(REP_INTERVAL_LOW, message_id);
	}

    //Basic Header field setting according to ETSI EN 302 636-4-1 [10.3.2]
    basicHeader.SetVersion (m_GnPtotocolVersion);

    if (enableSecurity && dataRequest.GNType == TSB) {
        basicHeader.SetNextHeader (2);
    } else basicHeader.SetNextHeader (1);

    if(dataRequest.GNMaxLife != 0) {
		basicHeader.SetLifeTime(encodeLT(dataRequest.GNMaxLife));
	} else {
		basicHeader.SetLifeTime(encodeLT(m_GnDefaultPacketLifetime));
	}
	
	if(dataRequest.GNMaxHL != 0) {
		basicHeader.SetRemainingHL (dataRequest.GNMaxHL);
	} else {
		basicHeader.SetRemainingHL (m_GnDefaultHopLimit);
	}

	// Common Header field setting according to ETSI EN 302 636-4-1 [10.3.4]
	commonHeader.SetNextHeader(dataRequest.upperProtocol); //0 if ANY(beacon) 1 if btp-a or 2 if btp-b
	commonHeader.SetHeaderType(dataRequest.GNType);

	// Initialize the header subtype to 0
	commonHeader.SetHeaderSubType(0);

	if((dataRequest.GNType == GBC) || (dataRequest.GNType == GAC) ) {
		commonHeader.SetHeaderSubType(dataRequest.GnAddress.shape);
	} else if(dataRequest.GNType == TSB) {
		if(dataRequest.GNMaxHL>1) {
			commonHeader.SetHeaderSubType(1); // Shouldn't happen for the time being
		}
	}

	commonHeader.SetTrafficClass (dataRequest.GNTraClass);
	commonHeader.SetFlag(m_GnIsMobile);

	if(dataRequest.GNMaxHL != 0) { // Equal to the remaining hop limit
		commonHeader.SetMaxHL (dataRequest.GNMaxHL);
	} else {
		commonHeader.SetMaxHL(m_GnDefaultHopLimit);
	}

	commonHeader.SetPayload (dataRequest.lenght);

	memcpy(longPV.GnAddress,m_GNAddress,8);

	longPV.TST = get_timestamp_ms_gn32();

	if(m_stationtype == StationType_pedestrian){
		VDPGPSClient::VRU_position_latlon_t POS_EPV = m_vrudp->getPedPosition();
		longPV.latitude = (int32_t) (POS_EPV.lat*DOT_ONE_MICRO);
		longPV.longitude = (int32_t) (POS_EPV.lon*DOT_ONE_MICRO);
		longPV.positionAccuracy = false;
		longPV.speed = (int16_t) m_vrudp->getPedSpeedValue()*CENTI; // [m/s] to [0.01 m/s]
		longPV.heading = (uint16_t) m_vrudp->getPedHeadingValue()*DECI;// [degrees] to [0.1 degrees]
	} else{
		std::pair<double,double> POS_EPV_pair = m_vdp->getCurrentPositionDbl();
        // TODO: manage the case in which unavailable values are returned by getCurrentPositionDbl() (e.g., when the serial stram is stopped and the serial parser is no more able to provide positioning data)
		longPV.latitude = (int32_t) (POS_EPV_pair.first*DOT_ONE_MICRO);
		longPV.longitude = (int32_t) (POS_EPV_pair.second*DOT_ONE_MICRO);
		longPV.positionAccuracy = false;
		longPV.speed = (int16_t) m_vdp->getSpeedValueDbl()*CENTI; // [m/s] to [0.01 m/s]
		longPV.heading = (uint16_t) m_vdp->getHeadingValueDbl()*DECI;// [degrees] to [0.1 degrees]
	}

	std::string use_dcc;
	if (m_dcc != nullptr) use_dcc = m_dcc->getModality();
	else use_dcc = "";
	struct timespec tv;
	clock_gettime (CLOCK_MONOTONIC, &tv);
	int64_t now = (tv.tv_sec * 1e9 + tv.tv_nsec)/1e6;
	bool gate_open = false;
	Packet pkt = {static_cast<double>(now), basicHeader, commonHeader, longPV, dataRequest, message_id};
	char GNAddr [8];
	memcpy(GNAddr, longPV.GnAddress, sizeof(GNAddr));
	auto now_unix = static_cast<double>(get_timestamp_us_realtime())/1000000.0;
	std::ofstream file;
    file.open(m_GN_log_file_tx, std::ios::app);
	file << std::fixed << now_unix << ",";
	for (unsigned char c : GNAddr) {
		file << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c);
	}
	file << std::dec;
	file << "," << longPV.TST << "," << longPV.latitude << "," << longPV.longitude << "," << message_id << "," << "0\n";
	file.close();
	if (use_dcc != "")
	{
		m_dcc->updatePktToSend();
		gate_open = m_dcc->checkGateOpen(now);
		if (gate_open == false)
		{
			// Gate is closed
			m_dcc->enqueue(priority, pkt);
			// std::cout << "[ENQUEUE]" << std::endl;
			return std::tuple<GNDataConfirm_t, MessageId_t>(BLOCKED_BY_GK, message_id);
		}
		else
		{
			// Gate is opened
			std::tuple<bool, Packet> value = m_dcc->dequeue(priority);
			int64_t aoi;
			if (std::get<0>(value) == true)
			{
				// Found a packet in queue with higher priority
				m_dcc->enqueue(priority, pkt);
				Packet pkt_to_send = std::get<1>(value);
				basicHeader = pkt_to_send.bh;
				commonHeader = pkt_to_send.ch;
				longPV = pkt_to_send.long_PV;
				dataRequest = pkt_to_send.dataRequest;
				message_id = pkt_to_send.message_id;
				aoi = static_cast<double>(now) - pkt_to_send.time;
				// std::cout << "[DEQUEUE]" << std::endl;
				memcpy(GNAddr, longPV.GnAddress, sizeof(GNAddr));
				now_unix = static_cast<double>(get_timestamp_us_realtime())/1000000.0;
				file.open(m_GN_log_file_tx, std::ios::app);
				file << std::fixed << now_unix << ",";
				for (unsigned char c : GNAddr) {
					file << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c);
				}
				file << std::dec;
				file << "," << longPV.TST << "," << longPV.latitude << "," << longPV.longitude << "," << message_id << "," << "1\n";
				file.close();
			}
			else
			{
				// std::cout << "[ORIGINAL]" << std::endl;
				memcpy(GNAddr, longPV.GnAddress, sizeof(GNAddr));
				now_unix = static_cast<double>(get_timestamp_us_realtime())/1000000.0;
				file.open(m_GN_log_file_tx, std::ios::app);
				file << std::fixed << now_unix << ",";
				for (unsigned char c : GNAddr) {
					file << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c);
				}
				file << std::dec;
				file << "," << longPV.TST << "," << longPV.latitude << "," << longPV.longitude << "," << message_id << "," << "1\n";
				file.close();
				aoi = 0;
			}
			m_dcc->updateAoI(priority, aoi);
			clock_gettime (CLOCK_MONOTONIC, &tv);
			now = (tv.tv_sec * 1e9 + tv.tv_nsec)/1e6;
			m_dcc->setLastTx(now);
		}
	}
	else
	{
		// std::cout << "[ORIGINAL NO DCC]" << std::endl;
		memcpy(GNAddr, longPV.GnAddress, sizeof(GNAddr));
		now_unix = static_cast<double>(get_timestamp_us_realtime())/1000000.0;
		file.open(m_GN_log_file_tx, std::ios::app);
		file << std::fixed << now_unix << ",";
		for (unsigned char c : GNAddr) {
			file << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c);
		}
		file << std::dec;
		file << "," << longPV.TST << "," << longPV.latitude << "," << longPV.longitude << "," << message_id << "," << "1\n";
		file.close();
	}

	if (use_dcc == "adaptive")
	{
		m_dcc->updateTgoAfterTransmission();
	}

	switch(dataRequest.GNType)
	{
		case GBC:
			dataConfirm = sendGBC (dataRequest,commonHeader,basicHeader,longPV);
			break;

		case TSB:
			if(commonHeader.GetHeaderSubType () == 0) dataConfirm = sendSHB (dataRequest,commonHeader,basicHeader,longPV);
			break;

		default:
			std::cerr << "Error: requested an unsupported GeoNetworking packet type. Only GBC and TSB are currently supported. No message will be sent." << std::endl;
			dataConfirm = UNSPECIFIED_ERROR;
	}

	return std::tuple<GNDataConfirm_t, MessageId_t>(dataConfirm, message_id);
}

void GeoNet::attachSendFromDCCQueue()
{
	if (m_dcc == nullptr) return;
    m_dcc->setSendCallback([this](const Packet& pkt) {
        // replicate the previous sending logic here; GeoNet has access to sendGBC/sendSHB
        basicHeader bh = pkt.bh;
        commonHeader ch = pkt.ch;
        GNlpv_t longPV = pkt.long_PV;
        GNDataRequest_t dataRequest = pkt.dataRequest;
        MessageId_t message_id = pkt.message_id;

		GNDataConfirm_t dataConfirm;

		struct timespec tv;
		clock_gettime (CLOCK_MONOTONIC, &tv);
		int64_t now = (tv.tv_sec * 1e9 + tv.tv_nsec)/1e6;
		m_dcc->setLastTx(now);

        // set last tx etc. is handled by DCC; here we just call the appropriate send
        switch(dataRequest.GNType)
        {
            case GBC:
                dataConfirm = this->sendGBC(dataRequest, ch, bh, longPV);
                break;
            case TSB:
                if(ch.GetHeaderSubType () == 0) dataConfirm = this->sendSHB(dataRequest, ch, bh, longPV);
                break;
            default:
                std::cerr << "GeoNet: unsupported GNType in DCC callback." << std::endl;
        }

		if (dataConfirm == ACCEPTED)
		{
			m_dcc->metricSupervisorSignalSentPacket(message_id);
		}
    });
}

gnError_e
GeoNet::decodeGN(unsigned char *packet, GNDataIndication_t* dataIndication)
{
		if (m_dcc != nullptr) m_dcc->updatePktReceived();
        basicHeader basicH;
        commonHeader commonH;

        dataIndication->data = packet;

        basicH.removeHeader(dataIndication->data);
        dataIndication->data += 4;
        dataIndication->GNRemainingLife = basicH.GetLifeTime ();
        dataIndication->GNRemainingHL = basicH.GetRemainingHL ();

        //Basic Header Procesing according to ETSI EN 302 636-4-1 [10.3.3]
        //1)Check version field
        if(basicH.GetVersion() != m_GnPtotocolVersion && basicH.GetVersion() != 0)
        {
            std::cerr<< "[ERROR] [Decoder] Incorrect version of GN protocol" << std::endl;
            return GN_VERSION_ERROR;

        } 
        // This warning can be useful, but, as in the 5G-CARMEN tests all the packets have a GeoNetworking version equal to "0",
        // it has been commented out not to make the logs grow too much
        // else if(basicH.GetVersion() == 0) {
            // std::cerr<< "[WARN] [Decoder] Unexpected GeoNetworking version \"0\"" << std::endl;
        // }
        // 2) Check NH field
        if(enableSecurity && basicH.GetNextHeader()==2)
        {

            long int start_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            if (f_out != nullptr) {
                fprintf(f_out, "[DECODE] Start time: %ld us, ", start_us);
            }

            if(m_security.extractSecurePacket (*dataIndication, isCertificate) == Security::SECURITY_VERIFICATION_FAILED) {
                std::cout << "[INFO] [Decoder] Security verification failed" << std::endl;
                if (f_out != nullptr) {
                    fprintf(f_out, "[INFO] Security verification failed\n");
                }
                return GN_SECURED_ERROR;
            } else {
                // std::cout << "[INFO] [Decoder] Security verification successful" << std::endl;
            }

            long int end_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            long int diff_us = end_us - start_us;

            if (f_out != nullptr) {
                fprintf(f_out, "End time: %ld us, Difference: %ld us. ", end_us, diff_us);
                if (isCertificate) {
                    fprintf(f_out, "[CERTIFICATE]\n");
                } else {
                    fprintf(f_out, "[DIGEST]\n");
                }
            }

        }

        if(!decodeLT(basicH.GetLifeTime(),&dataIndication->GNRemainingLife))
        {
            std::cerr << "[ERROR] [Decoder] Unable to decode lifetime field" << std::endl;
            return GN_LIFETIME_ERROR;
        }
        //Common Header Processing according to ETSI EN 302 636-4-1 [10.3.5]
        commonH.removeHeader(dataIndication->data);
        dataIndication->data += 8;
        dataIndication->upperProtocol = commonH.GetNextHeader (); //!Information needed for step 7
        dataIndication->GNTraClass = commonH.GetTrafficClass (); //!Information needed for step 7
        //1) Check MHL field
        if(commonH.GetMaxHopLimit() < basicH.GetRemainingHL())
        {
            std::cerr << "[ERROR] [Decoder] Max hop limit greater than remaining hop limit" << std::endl; //a) if MHL<RHL discard packet and omit execution of further steps
            return GN_HOP_LIMIT_ERROR;
        }
        //2) process the BC forwarding buffer, for now not implemented (SCF in traffic class disabled)
        //3) check HT field
        dataIndication->GNType = commonH.GetHeaderType();
        dataIndication->lenght = commonH.GetPayload ();

        switch(dataIndication->GNType)
        {
            case GBC:
                dataIndication = processGBC (dataIndication, commonH.GetHeaderSubType ());
                break;
            case TSB:
                if((commonH.GetHeaderSubType ()==0)) dataIndication = processSHB(dataIndication);
                else {
                    std::cerr << "[ERROR] [Decoder] GeoNet packet not supported" << std::endl;
                    return GN_TYPE_ERROR;
                  }
                break;
            default:
                std::cerr << "[ERROR] [Decoder] GeoNet packet not supported. GNType: " << static_cast<unsigned int>(dataIndication->GNType) << std::endl;
                return GN_TYPE_ERROR;
        }
        return GN_OK;
}

GNDataConfirm_t
GeoNet::sendSHB (GNDataRequest_t dataRequest,commonHeader commonHeader,basicHeader basicHeader,GNlpv_t longPV) {
	shbHeader header;

	//1) Create SHB GN-PDU with SHB header setting according to ETSI EN 302 636-4-1 [10.3.10.2]
	//a) and b) already done
	//c) SHB extended header
	header.SetLongPositionV (longPV);

	header.setDCC(m_dcc);

	/*
	uint32_t tx_power = getTxPower();
	header.SetOutputPower(static_cast<uint8_t>(tx_power));

	// m_cbr_reader.wrapper_start_reading_cbr();
    // double currentCbr = m_cbr_reader.get_current_cbr() * 100;
	// TODO understand how to read the CBR
	header.SetLocalCBR(static_cast<uint8_t>(0));

	// TODO check the meaning of this parameter
	header.SetMaxNeighbouringCBR((uint8_t) 0);
	*/
	
	// Serialize the headers
	packetBuffer shbHeaderSerialized;
	header.serializeInto(shbHeaderSerialized);

	packetBuffer commonHeaderSerialized;
	commonHeader.serializeInto(commonHeaderSerialized);

	packetBuffer basicHeaderSerialized;
	basicHeader.serializeInto(basicHeaderSerialized);

	if(m_log_filename!="dis" && m_log_filename!="") {
		basicHeader::printBasicHeader(basicHeaderSerialized,m_log_filename);
		commonHeader::printCommonHeader(commonHeaderSerialized,m_log_filename);
		shbHeader::printTSBPheader(shbHeaderSerialized,m_log_filename);
	}
	
	// Add them to the final packet (the innermost header goes first)
	dataRequest.data.addHeader(shbHeaderSerialized);

    dataRequest.data.addHeader(commonHeaderSerialized);
    if(enableSecurity){

        long int start_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        if (f_out != nullptr) {
            fprintf(f_out, "[ENCODE] Start time: %ld us, ", start_us);
        }
		m_security.setATmanager(m_atmanager);
        dataRequest = m_security.createSecurePacket (dataRequest, isCertificate);

        long int end_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        long int diff_us = end_us - start_us;

        if (f_out != nullptr) {
            fprintf(f_out, "End time: %ld us, Difference: %ld us. ", end_us, diff_us);
            if (isCertificate) {
                fprintf(f_out, "[CERTIFICATE]\n");
            } else {
                fprintf(f_out, "[DIGEST]\n");
            }
        }

    }

    dataRequest.data.addHeader(basicHeaderSerialized);

	//3)If not suitable neighbour exist in the LocT and the SCF for the traffic class is set:
	// This part is not yet implemented in OCABS
	// if((dataRequest.GNTraClass > 128) && (!hasNeighbour ())) {
	// 	//a)Buffer the SHB packet in the BC forwarding buffer and omit execution of further steps
	// 	return UNSUPPORTED_TRA_CLASS;// Not implemented yet
	// }
	//4)If the optional repetition interval paramter in the GN-dataRequest parameter is set
	if(dataRequest.GNRepInt != 0)
	{
		if(dataRequest.GNRepInt < m_GNMinPacketRepetitionInterval) return REP_INTERVAL_LOW;
		else return UNSUPPORTED_GN_REP;
	}

	//5)Media dependent procedures -Omitted-
	//6)Pass the GN-PDU to the LL protocol entity
	if(m_socket_tx==-1)
	{
		std::cerr << "GeoNet: SOCKET NOT FOUND " << std::endl;
		return UNSPECIFIED_ERROR;
	}

	struct sockaddr_ll addrll;
	// Prepare sockaddr_ll structure
	memset(&addrll,0,sizeof(addrll));
	addrll.sll_ifindex=m_ifindex;
	addrll.sll_family=AF_PACKET;
	addrll.sll_protocol=htons(ETH_P_ALL);

	ssize_t finalPktSize = dataRequest.data.getBufferSize() + sizeof(struct ether_header);
	uint8_t *finalPktBuffer = new uint8_t[finalPktSize];

	uint8_t *finalPktBufferUDP = nullptr;
	ssize_t finalPktSizeUDP = 0;

	if(m_udp_sockfd>0) {
		if(m_extra_position_udp) {
			extralatlon_t extra_position;
			extra_position.lat=htonl(longPV.latitude);
			extra_position.lon=htonl(longPV.longitude);

			finalPktSizeUDP=dataRequest.data.getBufferSize()+sizeof(extralatlon_t);
			finalPktBufferUDP = new uint8_t[finalPktSizeUDP];

			memcpy(finalPktBufferUDP,&extra_position,sizeof(extralatlon_t));
		} else {
			finalPktSizeUDP=dataRequest.data.getBufferSize();
			finalPktBufferUDP = new uint8_t[finalPktSizeUDP];
		}
	}

	struct ether_header ethHead;

	uint64_t broadcastMAC=0x0000FFFFFFFFFFFF; // A MAC address should be 48 bits long; however, we can declare it as uint64_t, then make memcpy() copy only 48 bits (i.e., ETHER_ADDR_LEN = 6 [bytes])
	memcpy(ethHead.ether_dhost,&broadcastMAC,ETHER_ADDR_LEN);
	memcpy(ethHead.ether_shost,m_mac_src,ETHER_ADDR_LEN);
	ethHead.ether_type=htons(GN_ETHERTYPE);

	memcpy(finalPktBuffer,&ethHead,sizeof(struct ether_header));

	memcpy(finalPktBuffer+sizeof(struct ether_header),dataRequest.data.getBufferPointer(),dataRequest.data.getBufferSize());

	if(m_udp_sockfd>0) {
		if(m_extra_position_udp) {
			memcpy(finalPktBufferUDP+sizeof(extralatlon_t),dataRequest.data.getBufferPointer(),dataRequest.data.getBufferSize());
		} else {
			memcpy(finalPktBufferUDP,dataRequest.data.getBufferPointer(),dataRequest.data.getBufferSize());
		}
	}

	errno=0;
	if(sendto(m_socket_tx,finalPktBuffer,finalPktSize,0,(struct sockaddr *)&addrll, sizeof(addrll))!=finalPktSize) {
		std::cerr << "Cannot send SHB GN packet. Error details: " << strerror(errno) << std::endl;
	}

	if(m_udp_sockfd>0) {
		if(send(m_udp_sockfd,finalPktBufferUDP,finalPktSizeUDP,0)!=finalPktSizeUDP) {
			std::cerr << "Cannot send SHB GN packet via UDP. Error details: " << strerror(errno) << std::endl;
		}
	}

	size_t pktSize = finalPktSize - sizeof(struct ether_header) + IEEE80211_DATA_PKT_HDR_LEN + IEEE80211_FCS_LEN + 8; // 8 = bytes layer LLC
	if (m_dcc != nullptr) m_dcc->updateTonpp(pktSize);

	delete []finalPktBuffer;

	if(m_extra_position_udp && finalPktBufferUDP!=nullptr) {
		delete []finalPktBufferUDP;
	}

	//7)reset beacon timer to prevent dissemination of unnecessary beacon packet [Not yet implemented in OCABS]
	return ACCEPTED;
}

GNDataConfirm_t
GeoNet::sendGBC (GNDataRequest_t dataRequest,commonHeader commonHeader, basicHeader basicHeader,GNlpv_t longPV) {
	gbcHeader header;

	//1) Create SHB GN-PDU with GBC header setting according to ETSI EN 302 636-4-1 [10.3.11.2]
	//a) and b) already done
	//GBC extended header
	header.SetSeqNumber (m_seqNumber);
	header.SetLongPositionV (longPV);
	header.SetGeoArea(dataRequest.GnAddress);

	packetBuffer finalPkt;

	header.serializeInto(dataRequest.data);

	// Serialize the headers
	packetBuffer gbcHeaderSerialized;
	header.serializeInto(gbcHeaderSerialized);

	packetBuffer commonHeaderSerialized;
	commonHeader.serializeInto(commonHeaderSerialized);

	packetBuffer basicHeaderSerialized;
	basicHeader.serializeInto(basicHeaderSerialized);
	
	if(m_log_filename!="dis" && m_log_filename!="") {
		basicHeader::printBasicHeader(basicHeaderSerialized,m_log_filename);
		commonHeader::printCommonHeader(commonHeaderSerialized,m_log_filename);
		// shbHeader::printTSBPheader(gbcHeaderSerialized,m_log_filename);
		// There is currently no print function for the GBC header - we are currently working on it!
	}

	// Add them to the final packet (the innermost header goes first)
	dataRequest.data.addHeader(gbcHeaderSerialized);
	dataRequest.data.addHeader(commonHeaderSerialized);
	dataRequest.data.addHeader(basicHeaderSerialized);

	/*
	 * 2)If not suitable neighbour exist in the LocT and the SCF for the traffic class is set:
	 * a)Buffer the SHB packet in the BC forwarding buffer and omit execution of further steps
	*/
	// This part is not yet implemented in OCABS
	// if((dataRequest.GNTraClass >= 128) && (!hasNeighbour ())) {
	//   return UNSUPPORTED_TRA_CLASS;//Not implemented yet
	// }

	//!3)Execute forwarding algorithm selection procedure, not implemented because RSU in our case will always be inside the target area
	//4) if packet is buffered in any of the forwarding packets, omit further steps
	if(m_GNAreaForwardingAlgorithm == 2) {
	  /* push packet into CBF buffer */
	  return UNSPECIFIED_ERROR;
	}

	//!5)Security profile settings not implemented
	//6)If the optional repetition interval paramter in the GN-dataRequest parameter is set
	// Not yet implemented in OCABS
	// if(dataRequest.GNRepInt != 0) {
	// 	if(dataRequest.GNRepInt < m_GNMinPacketRepetitionInterval) return REP_INTERVAL_LOW;
	// 	//a)save the SHB packet
	// 	saveRepPacket(dataRequest);
	// }
	//!7)Media dependent procedures -Omited-
	//8)Pass the GN-PDU to the LL protocol entity

	struct sockaddr_ll addrll;

	// Prepare sockaddr_ll structure
	memset(&addrll,0,sizeof(addrll));
	addrll.sll_ifindex=m_ifindex;
	addrll.sll_family=AF_PACKET;
	addrll.sll_protocol=htons(ETH_P_ALL);

	ssize_t finalPktSize = dataRequest.data.getBufferSize() + sizeof(struct ether_header);
	uint8_t *finalPktBuffer = new uint8_t[finalPktSize];

	struct ether_header ethHead;

	uint64_t broadcastMAC=0x0000FFFFFFFFFFFF; // A MAC address should be 48 bits long; however, we can declare it as uint64_t, then make memcpy() copy only 48 bits (i.e., ETHER_ADDR_LEN = 6 [bytes])
	memcpy(ethHead.ether_dhost,&broadcastMAC,ETHER_ADDR_LEN);
	memcpy(ethHead.ether_shost,m_mac_src,ETHER_ADDR_LEN);
	ethHead.ether_type=htons(GN_ETHERTYPE);

	memcpy(finalPktBuffer,&ethHead,sizeof(struct ether_header));
	memcpy(finalPktBuffer+sizeof(struct ether_header),dataRequest.data.getBufferPointer(),dataRequest.data.getBufferSize());

	errno=0;
	if(sendto(m_socket_tx,finalPktBuffer,finalPktSize,0,(struct sockaddr *)&addrll, sizeof(addrll))!=finalPktSize) {
		std::cerr << "Cannot send GBC GN packet. Error details: " << strerror(errno) << std::endl;
	}

	if(m_udp_sockfd>0) {
		if(send(m_udp_sockfd,finalPktBuffer,finalPktSize,0)!=finalPktSize) {
			std::cerr << "Cannot send GBC GN packet via UDP. Error details: " << strerror(errno) << std::endl;
		}
	}

	size_t pktSize = finalPktSize - sizeof(struct ether_header) + IEEE80211_DATA_PKT_HDR_LEN + IEEE80211_FCS_LEN + 8; // 8 = bytes layer LLC
	if (m_dcc != nullptr) m_dcc->updateTonpp(pktSize);

	delete []finalPktBuffer;

	//Update sequence number
	m_seqNumber = (m_seqNumber+1)% SN_MAX;
	return ACCEPTED;
}

GNDataIndication_t*
GeoNet::processSHB (GNDataIndication_t* dataIndication)
{
        shbHeader shbH;
        double cbrr0, cbrr1;
        if(dataIndication->GNType == BEACON)
        {
           // TODO
        }
        else
        {
            shbH.removeHeader(dataIndication->data);
            dataIndication->data += 28;
            dataIndication->SourcePV = shbH.GetLongPositionV ();
			char GNAddr [8];
			memcpy(GNAddr, dataIndication->SourcePV.GnAddress, sizeof(GNAddr));
			auto now_unix = static_cast<double>(get_timestamp_us_realtime())/1000000.0;
			std::ofstream file;
			file.open(m_GN_log_file_rx, std::ios::app);
			// m_mutex_log_rx.lock();
			file << std::fixed << now_unix << ",";
			for (unsigned char c : GNAddr) {
				file << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c);
			}
			file << std::dec;
			file << "," << dataIndication->SourcePV.TST << "," << dataIndication->SourcePV.latitude << "," << dataIndication->SourcePV.longitude << ",";
			file.close();
            cbrr0 = shbH.GetCBRR0Hop();
            cbrr1 = shbH.GetCBRR1Hop();
        }
        m_LocT_Mutex.lock ();
        if (dataIndication->GNType != BEACON)
        {
            struct timespec tv;
            clock_gettime (CLOCK_MONOTONIC, &tv);
            int64_t now = (tv.tv_sec * 1e9 + tv.tv_nsec)/1e6;
            uint64_t key;
            std::memcpy(&key, dataIndication->SourcePV.GnAddress, 8);
            m_GNLocT[key].cbr_extension.CBR_R0_Hop.push_back (std::make_tuple(now, cbrr0));
            m_GNLocT[key].cbr_extension.CBR_R1_Hop.push_back (std::make_tuple(now, cbrr1));
        }
        m_LocT_Mutex.unlock ();
        dataIndication->GNType = TSB;

        //7) Pass the payload to the upper protocol entity
        return dataIndication;
}

GNDataIndication_t*
GeoNet::processGBC (GNDataIndication_t* dataIndication, uint8_t shape)
{
        gbcHeader gbcH;

        gbcH.removeHeader(dataIndication->data);
        dataIndication->data += 44;
        dataIndication->SourcePV = gbcH.GetLongPositionV ();
        dataIndication->GnAddressDest = gbcH.GetGeoArea ();
        dataIndication->GnAddressDest.shape = shape;

        //3)Determine function F as specified in ETSI EN 302 931
        /*Not implemented for this decoder*/

        //Pass the payload to the upper protocol entity
        dataIndication->GNType = GBC;

        return dataIndication;
}

int 
GeoNet::openUDPsocket(std::string udp_sock_addr,std::string interface_ip,bool extra_position_udp) {
	size_t delimiter_pos=udp_sock_addr.find(":");
	std::string dest_ip=udp_sock_addr.substr(0, delimiter_pos);
	udp_sock_addr.erase(0,delimiter_pos+1);
	long dest_port=strtol(udp_sock_addr.c_str(),nullptr,0);

	printf("%s:%ld [bind: %s]\n",dest_ip.c_str(),dest_port,interface_ip.c_str());

	// Generic size of a struct sockaddr_in (used multiple times below)
	socklen_t addrlen = sizeof(struct sockaddr_in);

	// Create a UDP socket for packet transmission
	m_udp_sockfd=socket(AF_INET,SOCK_DGRAM,0);

	if(m_udp_sockfd<0) {
		return -1;
	}

	// Bind the socket to the interface with the IP address specified by "interface_ip"
	// (or do not bind to any specific address/interface if "interface_ip" is set to "0.0.0.0")
	struct sockaddr_in bind_address;
	memset(&bind_address,0,addrlen);
	bind_address.sin_family = AF_INET;

	if(interface_ip=="0.0.0.0") {
		bind_address.sin_addr.s_addr = INADDR_ANY;
	} else {
		if(inet_pton(AF_INET,interface_ip.c_str(),&bind_address.sin_addr)<1) {
			closeUDPsocket();
			return -2;
		}
	}

	bind_address.sin_port = htons(0);

	if(bind(m_udp_sockfd,(struct sockaddr*) &bind_address,addrlen)<0) {
		closeUDPsocket();
		return -3;
	}

	// "connect" the UDP socket (i.e., set the default destination address and port) 
	struct sockaddr_in dest_address;
	memset(&dest_address,0,addrlen);
	dest_address.sin_family = AF_INET;
	
	if(dest_ip=="0.0.0.0" || inet_pton(AF_INET,dest_ip.c_str(),&dest_address.sin_addr)<1) {
		std::cout << dest_ip << " does not appear to be a valid destination IP address for the UDP packet. Attempting address resolution..." << std::endl;
		// Attempt to resolve host name if inet_pton fails (maybe the user specified a name and not an IP address?)
		if(dest_ip!="0.0.0.0") {
			struct hostent *hostaddrs;
			struct in_addr **addr_list;
			
			hostaddrs=gethostbyname(dest_ip.c_str());

			if(hostaddrs==nullptr) {
				herror("Address resolution failed for UDP destination address. Details");
				closeUDPsocket();
				return -4;
			}

			addr_list=(struct in_addr **)hostaddrs->h_addr_list;

			// Gather the first address corresponding to the given name
			if(addr_list[0]==nullptr) {
				std::cerr << "Error: address resolution failed for " << dest_ip << ". No IP addresses found for given host name." << std::endl;
				closeUDPsocket();
				return -4;
			} else {
				dest_address.sin_addr=*addr_list[0];
			}
		} else {
			closeUDPsocket();
			return -4;
		}
	}

	dest_address.sin_port = htons(dest_port);

	if(connect(m_udp_sockfd,(struct sockaddr*) &dest_address,addrlen)<0) {
		closeUDPsocket();
		return -5;
	}

	m_extra_position_udp=extra_position_udp;

	return m_udp_sockfd;
}

void 
GeoNet::closeUDPsocket() {
	if(m_udp_sockfd>0) {
		close(m_udp_sockfd);
		m_udp_sockfd=-1;
	}
}

void GeoNet::attachGlobalCBRCheck ()
{
    if (m_dcc == nullptr) return;
    m_dcc->setCBRGCallback([this](){
        struct timespec tv;
        clock_gettime (CLOCK_MONOTONIC, &tv);
        int64_t now = (tv.tv_sec * 1e9 + tv.tv_nsec)/1e6;
        double mean_cbr_r0_hop = 0.0;
        long tot_r0 = 0;
        double max_cbr_r0 = 0.0, second_max_cbr_r0 = 0.0;
        double mean_cbr_r1_hop = 0.0;
        long tot_r1 = 0;
        double max_cbr_r1 = 0.0, second_max_cbr_r1 = 0.0;
        m_LocT_Mutex.lock ();
        for(auto it = m_GNLocT.begin(); it != m_GNLocT.end(); ++it)
        {
            auto& cbr_data = it->second.cbr_extension;
            // Clean old data
            size_t counter = 0;
            std::vector<size_t> to_delete;
            for (auto it2 = cbr_data.CBR_R0_Hop.begin(); it2 != cbr_data.CBR_R0_Hop.end(); ++it2)
            {
                if(now - m_GNLocTTimerCBR_ms > std::get<0>(*it2)) to_delete.push_back (counter);
                counter ++;
            }
            std::sort(to_delete.begin(), to_delete.end(), std::greater<size_t>());
            for (size_t idx : to_delete) {
                if (idx < cbr_data.CBR_R0_Hop.size()) {
                    cbr_data.CBR_R0_Hop.erase(cbr_data.CBR_R0_Hop.begin() + idx);
                }
            }

            counter = 0;
            to_delete.clear();
            for (auto it2 = cbr_data.CBR_R1_Hop.begin(); it2 != cbr_data.CBR_R1_Hop.end(); ++it2)
            {
                if(now - m_GNLocTTimerCBR_ms > std::get<0>(*it2)) to_delete.push_back (counter);
                counter ++;
            }
            std::sort(to_delete.begin(), to_delete.end(), std::greater<size_t>());
            for (size_t idx : to_delete) {
                if (idx < cbr_data.CBR_R1_Hop.size()) {
                    cbr_data.CBR_R1_Hop.erase(cbr_data.CBR_R1_Hop.begin() + idx);
                }
            }

            for (auto it2 = cbr_data.CBR_R0_Hop.begin(); it2 != cbr_data.CBR_R0_Hop.end(); ++it2)
            {
                double val = std::get<1>(*it2);
                mean_cbr_r0_hop += val;

                if (val > max_cbr_r0)
                {
                    second_max_cbr_r0 = max_cbr_r0;
                    max_cbr_r0 = val;
                }
                else if (val < max_cbr_r0 && val > second_max_cbr_r0)
                {
                    second_max_cbr_r0 = val;
                }
            }
            tot_r0 += cbr_data.CBR_R0_Hop.size();

            for (auto it2 = cbr_data.CBR_R1_Hop.begin(); it2 != cbr_data.CBR_R1_Hop.end(); ++it2)
            {
                double val = std::get<1>(*it2);
                mean_cbr_r1_hop += val;

                if (val > max_cbr_r1)
                {
                    second_max_cbr_r1 = max_cbr_r1;
                    max_cbr_r1 = val;
                }
                else if (val < max_cbr_r1 && val > second_max_cbr_r1)
                {
                    second_max_cbr_r1 = val;
                }
            }
            tot_r1 += cbr_data.CBR_R1_Hop.size();
        }
        m_LocT_Mutex.unlock ();
		if (tot_r0 == 0) mean_cbr_r0_hop = 0;
		else mean_cbr_r0_hop /= tot_r0;
		if (tot_r1 == 0) mean_cbr_r1_hop = 0;
		else mean_cbr_r1_hop /= tot_r1;
        double CBR_L1_Hop, CBR_L2_Hop;
        if (mean_cbr_r0_hop > m_dcc->getCBRTarget())
        {
            CBR_L1_Hop = max_cbr_r0;
        }
        else
        {
            CBR_L1_Hop = second_max_cbr_r0;
        }

        if (mean_cbr_r1_hop > m_dcc->getCBRTarget())
        {
            CBR_L2_Hop = max_cbr_r1;
        }
        else
        {
            CBR_L2_Hop = second_max_cbr_r1;
        }
        double CBR_G = std::max(m_dcc->getCBRL0Prev(), CBR_L1_Hop);
        CBR_G = std::max(CBR_G, CBR_L2_Hop);
        m_dcc->setCBRG(CBR_G);
        m_dcc->setCBRL1(CBR_L1_Hop);
    });
}

void GeoNet::writeEndRowLogRX (MessageId_t msg_id)
{
	std::ofstream file;
	file.open(m_GN_log_file_rx, std::ios::app);
	file << msg_id << "," << "2\n";
	file.close();
	// m_mutex_log_rx.unlock();
}
