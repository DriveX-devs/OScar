//
// Created by Carlos Mateo Risma Carletti on 07/05/21.
// Updated by Francesco Raviglione in November 2021
//

#include "shbHeader.h"

#include <cstring>

shbHeader::shbHeader() {
	m_sourcePV = {};  //!Source long position vector
	m_reserved = 0; //! aux variable for reading reserved fields
	m_reserved32 = 0;
}

shbHeader::~shbHeader() = default;

void
shbHeader::serializeInto(packetBuffer &packet) {
	//ETSI EN 302 636-4-1 [9.8.4]
	//Source long position vector aux varaibles
	uint16_t pai_speed = 0;
	pai_speed = (m_sourcePV.positionAccuracy << 15) | (((m_sourcePV.speed>>1)*2)&0x7FFF); //Speed >> 1 so that sign is not lost, but multiplied by 2 to compensate bit shift

	//Source long position vector

	packet.addGNAddress((uint8_t *)&m_sourcePV.GnAddress[0]);
	packet.addHtonU32(m_sourcePV.TST);
	packet.addHtonU32(m_sourcePV.latitude);
	packet.addHtonU32(m_sourcePV.longitude);
	packet.addHtonU16(pai_speed);
	packet.addHtonU16(m_sourcePV.heading);
	//Reserved
	packet.addHtonU32(0x00000000);
}