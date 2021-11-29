//
// Created by Carlos Mateo Risma Carletti on 07/05/21.
// Updated by Francesco Raviglione in November 2021
//

#include "basicHeader.h"

basicHeader::basicHeader() {
    m_version = 0;
    m_nextHeader = 0;
    m_reserved = 0;
    m_lifeTime = 0;
    m_remainingHopLimit = 0;
}

basicHeader::~basicHeader() = default;

void 
basicHeader::serializeInto(packetBuffer &packet) {
    //ETSI EN 302 636-4-1 [9.8.4]
    //Basic Header aux variables
    uint8_t version_NH = 0;
    version_NH = (m_version << 4) | (m_nextHeader);

    //Basic Header     ETSI EN 302 636-4-1 [9.6]
    packet.addU8(version_NH);
    packet.addU8(0x00); //! Reserved
    packet.addU8(m_lifeTime);
    packet.addU8(m_remainingHopLimit);
}
