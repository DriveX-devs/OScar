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

class btp {
    public :
        btp();
        ~btp();

        void setGeoNet(GeoNet* geoNet){m_geonet = geoNet;}
        void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype) {m_geonet->setStationProperties(fixed_stationid,fixed_stationtype);}
        void setStationID(unsigned long fixed_stationid) {m_geonet->setStationID(fixed_stationid);}
        void setStationType(long fixed_stationtype) {m_geonet->setStationType(fixed_stationtype);}
        void setVDP(VDPGPSClient* vdp){m_geonet->setVDP(vdp);}
        void sendBTP(BTPDataRequest_t dataRequest);

    private:
        GeoNet *m_geonet;

};
#endif // OCABS_BTP_H
