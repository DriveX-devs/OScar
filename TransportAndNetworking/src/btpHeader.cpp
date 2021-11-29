//
// Created by Carlos Mateo Risma Carletti on 11/05/21.
// Updated by Francesco Raviglione in November 2021
//

#include "btpHeader.h"

btpHeader::btpHeader() {
    m_source_destInfo = 0;
    m_destinationPort = 0;
}

btpHeader::~btpHeader() = default;

void 
btpHeader::serializeInto(packetBuffer &packet) {
    //ETSI EN 302 636-5-1 [7.2.1-2]
    packet.addHtonU16(m_destinationPort);
    packet.addHtonU16(m_source_destInfo);
}