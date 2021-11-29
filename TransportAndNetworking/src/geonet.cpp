//
// Created by Carlos Mateo Risma Carletti on 06/05/21.
// Updated by Francesco Raviglione in November 2021
//

#include <iostream>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <cstring>
#include "functional"
#include "geonet.h"

#define SN_MAX 65536

// This funciton fills the out_addr 8-byte array with a GN address, starting from the local MAC address specified as "addr", and from the current stationType
void 
GeoNet::MakeManagedconfiguredAddress (uint8_t addr[6], uint8_t ITSType, uint8_t out_addr[8]) {
	//ETSI EN 302 636-4-1 [6.3]
	out_addr[0] = 0x00 | ITSType << 2; //Initial GeoNetAddress ->M=0 and 5bit ITS-S type
	out_addr[1] = 0x00; //Reserved
	memcpy(out_addr + 2, addr, 6);
}

GeoNet::GeoNet() {
	m_socket_tx = -1;
	m_station_id = ULONG_MAX;
	m_stationtype = LONG_MAX;
	m_seqNumber = 0;
	// m_GNAddress = GNAddress(); Should be already initialized to all zeros

	m_RSU_epv_set=false;
}

GeoNet::~GeoNet() = default;

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

GNDataConfirm_t
GeoNet::sendGN (GNDataRequest_t dataRequest) {
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
		return UNSPECIFIED_ERROR;
	}

	if(m_stationtype==StationType_roadSideUnit && m_RSU_epv_set==false)	{
		std::cerr << "GeoNetworking error: no position has been set for an RSU object. Please use setFixedPositionRSU() on the Facilities Layer object." << std::endl;
		return UNSPECIFIED_ERROR;
	}

	if(dataRequest.lenght > m_GnMaxSduSize) {
		return MAX_LENGHT_EXCEEDED;
	} else if(dataRequest.GNMaxLife > m_GNMaxPacketLifetime) {
		return MAX_LIFE_EXCEEDED;
	} else if(dataRequest.GNRepInt != 0 && dataRequest.GNRepInt < m_GNMinPacketRepetitionInterval) {
		return REP_INTERVAL_LOW;
	}

	//Basic Header field setting according to ETSI EN 302 636-4-1 [10.3.2]
	basicHeader.SetVersion (m_GnPtotocolVersion);

	//Security option not implemented
	basicHeader.SetNextHeader (1);//! Next Header: Common Header (1)
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

	std::pair<double,double> POS_EPV_pair = m_vdp->getCurrentPositionDbl();
	longPV.latitude = (int32_t) (POS_EPV_pair.first*DOT_ONE_MICRO);
	longPV.longitude = (int32_t) (POS_EPV_pair.second*DOT_ONE_MICRO);
	longPV.positionAccuracy = false;
	longPV.speed = (int16_t) m_vdp->getSpeedValueDbl()*CENTI; // [m/s] to [0.01 m/s]
	longPV.heading = (uint16_t) m_vdp->getHeadingValueDbl()*DECI;// [degrees] to [0.1 degrees]

	switch(dataRequest.GNType) {
		case GBC:
			dataConfirm = sendGBC (dataRequest,commonHeader,basicHeader,longPV);
			break;

		case TSB:
			if(commonHeader.GetHeaderSubType ()==0) dataConfirm = sendSHB (dataRequest,commonHeader,basicHeader,longPV);
			break;

		default:
			std::cerr << "Error: requested an unsupported GeoNetworking packet type. Only GBC and TSB are currently supported. No message will be sent." << std::endl;
			dataConfirm = UNSPECIFIED_ERROR;
	}

	return dataConfirm;
}

GNDataConfirm_t
GeoNet::sendSHB (GNDataRequest_t dataRequest,commonHeader commonHeader,basicHeader basicHeader,GNlpv_t longPV) {
	shbHeader header;
	//1) Create SHB GN-PDU with SHB header setting according to ETSI EN 302 636-4-1 [10.3.10.2]
	//a) and b) already done
	//c) SHB extended header
	header.SetLongPositionV (longPV);

	// Serialize the headers
	packetBuffer shbHeaderSerialized;
	header.serializeInto(shbHeaderSerialized);

	packetBuffer commonHeaderSerialized;
	commonHeader.serializeInto(commonHeaderSerialized);

	packetBuffer basicHeaderSerialized;
	basicHeader.serializeInto(basicHeaderSerialized);

	// Add them to the final packet (the innermost header goes first)
	dataRequest.data.addHeader(shbHeaderSerialized);

	dataRequest.data.addHeader(commonHeaderSerialized);
	dataRequest.data.addHeader(basicHeaderSerialized);

	//2)Security setting -not implemeted yet-
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

	struct ether_header ethHead;

	uint64_t broadcastMAC=0x0000FFFFFFFFFFFF; // A MAC address should be 48 bits long; however, we can declare it as uint64_t, then make memcpy() copy only 48 bits (i.e., ETHER_ADDR_LEN = 6 [bytes])
	memcpy(ethHead.ether_dhost,&broadcastMAC,ETHER_ADDR_LEN);
	memcpy(ethHead.ether_shost,m_mac_src,ETHER_ADDR_LEN);
	ethHead.ether_type=htons(GN_ETHERTYPE);

	memcpy(finalPktBuffer,&ethHead,sizeof(struct ether_header));
	memcpy(finalPktBuffer+sizeof(struct ether_header),dataRequest.data.getBufferPointer(),dataRequest.data.getBufferSize());

	errno=0;
	if(sendto(m_socket_tx,finalPktBuffer,finalPktSize,0,(struct sockaddr *)&addrll, sizeof(addrll))!=finalPktSize) {
		std::cerr << "Cannot send SHB GN packet. Error details: " << strerror(errno) << std::endl;
	}

	delete []finalPktBuffer;

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
		std::cerr << "Cannot send SHB GN packet. Error details: " << strerror(errno) << std::endl;
	}

	delete []finalPktBuffer;

	//Update sequence number
	m_seqNumber = (m_seqNumber+1)% SN_MAX;
	return ACCEPTED;
}
