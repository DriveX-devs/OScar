//
// Created by Carlos Mateo Risma Carletti on 07/05/21.
// Updated by Francesco Raviglione in November 2021
//

#include <iostream>
#include "btp.h"

btp::btp() = default;

btp::~btp() = default;

void
btp::sendBTP(BTPDataRequest_t dataRequest) {
	GNDataConfirm_t dataConfirm;
	GNDataRequest_t GnDataRequest = {};
	btpHeader btpheader;
	btpheader.setDestinationPort(dataRequest.destPort);

	if(dataRequest.BTPType==BTP_A) {
		btpheader.setSourcePort (dataRequest.sourcePort);
		GnDataRequest.upperProtocol = BTP_A;
	} else {
		btpheader.setDestinationPortInfo (dataRequest.destPInfo);
		GnDataRequest.upperProtocol = BTP_B;
	}

	packetBuffer serializedBTPheader;
	btpheader.serializeInto(serializedBTPheader);
	
	if(m_log_filename!="dis" && m_log_filename!="") {
		btpHeader::printBTPheader(serializedBTPheader,m_log_filename);
	}

	dataRequest.data.addHeader(serializedBTPheader);

	// Filling the GN-dataRequest
	GnDataRequest.GNType = dataRequest.GNType;
	GnDataRequest.GnAddress = dataRequest.GnAddress;
	GnDataRequest.GNCommProfile = dataRequest.GNCommProfile;
	GnDataRequest.GNRepInt = dataRequest.GNRepInt;
	GnDataRequest.GNMaxRepTime = dataRequest.GNMaxRepInt;
	GnDataRequest.GNMaxLife = dataRequest.GNMaxLife;
	GnDataRequest.GNMaxHL = dataRequest.GNMaxHL;
	GnDataRequest.GNTraClass = dataRequest.GNTraClass;
	GnDataRequest.data = dataRequest.data;
	GnDataRequest.lenght = dataRequest.lenght + 4;

	dataConfirm = m_geonet->sendGN(GnDataRequest);
	if(dataConfirm != ACCEPTED) {
		std::cerr << "Error! GeoNetworking could not send any packet." << std::endl;
	}
}
