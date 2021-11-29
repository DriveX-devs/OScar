//
// Created by Carlos Mateo Risma Carletti on 07/05/21.
// Updated by Francesco Raviglione in November 2021
//

#include "commonHeader.h"
#include <cstring>

commonHeader::commonHeader() {
    m_nextHeader = 0;
    m_headerType = 0;
    m_headerSubType = 0;
    m_trafficClass = 0;
    m_flag = false;
    m_payload = 0;
    m_maxHopLimit = 0;
    m_reserved = 0;
}

commonHeader::~commonHeader() = default;

void 
commonHeader::serializeInto(packetBuffer &packet) {
    //ETSI EN 302 636-4-1 [9.8.4]
    //Common Header aux variables
    uint8_t chNH = 0;
    chNH = m_nextHeader << 4;
    uint8_t headerType = 0;
    headerType = (m_headerType << 4) | (m_headerSubType);
    uint8_t chflag = 0;
    chflag = m_flag << 7;

    //Common Header - ETSI EN 302 636-4-1 [9.7]
    packet.addU8(chNH);
    packet.addU8(headerType);
    packet.addU8(m_trafficClass);
    packet.addU8(chflag);
    packet.addHtonU16(m_payload);
    packet.addU8(m_maxHopLimit);
    packet.addU8(0x00); //! Reserved
}