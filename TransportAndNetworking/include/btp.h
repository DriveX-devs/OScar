//
// Created by Carlos Mateo Risma Carletti on 07/05/21.
// Updated by Francesco Raviglione in November 2021
//

#ifndef OCABS_BTP_H
#define OCABS_BTP_H
#include "utils.h"
#include "btpHeader.h"
#include "geonet.h"
#include "gpsc.h"
#include <cstdint>
#include <cstring>

#define CA_PORT 2001
#define DEN_PORT 2002
#define CP_PORT 2009
#define VA_PORT 2018

class btp {
    public :
        btp();
        ~btp();

        void setGeoNet(GeoNet* geoNet){m_geonet = geoNet;}
        void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype) {m_geonet->setStationProperties(fixed_stationid,fixed_stationtype);}
        void setStationID(unsigned long fixed_stationid) {m_geonet->setStationID(fixed_stationid);}
        void setStationType(long fixed_stationtype) {m_geonet->setStationType(fixed_stationtype);}
        void setVDP(VDPGPSClient* vdp){m_geonet->setVDP(vdp);}
        void setVRUdp(VDPGPSClient* vrudp){m_geonet->setVRUdp(vrudp);}
        GNDataConfirm_t sendBTP(BTPDataRequest_t dataRequest, int priority);
        
        btpError_e decodeBTP(GNDataIndication_t dataIndication, BTPDataIndication_t* btpDataIndication);

	void setLogFile(std::string camfile) {m_log_filename=camfile;}
    private:
        GeoNet *m_geonet;
        
        std::string m_log_filename = "dis";

};
#endif // OCABS_BTP_H
