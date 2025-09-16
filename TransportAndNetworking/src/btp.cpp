//
// Created by Carlos Mateo Risma Carletti on 07/05/21.
// Updated by Francesco Raviglione in November 2021
//

#include <iostream>
#include "btp.h"

btp::btp() = default;

btp::~btp() = default;

GNDataConfirm_t
btp::sendBTP(BTPDataRequest_t dataRequest, int priority) {
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

	dataConfirm = m_geonet->sendGN(GnDataRequest, priority);
	if(dataConfirm != ACCEPTED && dataConfirm != BLOCKED_BY_GK) {
		std::cerr << "Error! GeoNetworking could not send any packet." << std::endl;
	}
	return dataConfirm;
}

btpError_e
btp::decodeBTP(GNDataIndication_t dataIndication, BTPDataIndication_t* btpDataIndication) {
	btpHeader header;


	btpDataIndication->data = dataIndication.data;

	header.removeHeader(btpDataIndication->data);
	btpDataIndication->data += 4;

	btpDataIndication->BTPType = dataIndication.upperProtocol;

	if((header.getDestPort ()!= CA_PORT) && (header.getDestPort ()!= DEN_PORT) && (header.getDestPort () != VA_PORT) && (header.getDestPort () != CP_PORT))
	{
		std::cerr << "[ERROR] [Decoder] BTP port not supported" << std::endl;
		return BTP_ERROR;
	}

    if(header.getDestPort () == CP_PORT)
    {
        std::cerr << "[ERROR] [Decoder] Reception of CPMs is not yet fully supported" << std::endl;
        return BTP_ERROR;
    }

	btpDataIndication->destPort = header.getDestPort ();

	if(btpDataIndication->BTPType == BTP_A)
	{
		btpDataIndication->sourcePort = header.getSourcePort ();
		btpDataIndication->destPInfo = 0;
	}
	else if(btpDataIndication->BTPType == BTP_B)  //BTP-B
	{
		btpDataIndication->destPInfo = header.getDestPortInfo ();
		btpDataIndication->sourcePort = 0;
	}
	else
	{
		std::cerr << "[ERROR] [Decoder] Incorrect transport protocol " << std::endl;
		return BTP_ERROR;
	}

	btpDataIndication->GnAddress = dataIndication.GnAddressDest;
	btpDataIndication->GNTraClass = dataIndication.GNTraClass;
	btpDataIndication->GNRemPLife = dataIndication.GNRemainingLife;
	btpDataIndication->GNPositionV = dataIndication.SourcePV;
	btpDataIndication->data = dataIndication.data + 4;
	btpDataIndication->lenght = dataIndication.lenght - 4;

	return BTP_OK;
}
