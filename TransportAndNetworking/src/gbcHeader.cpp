//
// Created by Carlos Mateo Risma Carletti on 07/05/21.
// Updated by Francesco Raviglione in November 2021
//

#include "gbcHeader.h"

gbcHeader::gbcHeader()
{
  m_sourcePV = { };  //!Source long position vector
  m_seqNumber = 0;
  m_posLong = 0;
  m_posLat = 0;
  m_distA = 0;
  m_distB = 0;
  m_angle = 0;
  m_reserved = 0; //! aux variable for reading reserved fields
}

void 
gbcHeader::serializeInto(packetBuffer &packet) {
	//Source long position vector aux varaibles
	uint16_t pai_speed = 0;
	pai_speed = (m_sourcePV.positionAccuracy << 15) | ((m_sourcePV.speed)>>1); //Speed >> 1 so that sign isnt lost

	//ETSI EN 302 636-4-1 [9.8.5]
	//Sequence Number
	packet.addHtonU16(m_seqNumber);
	//Reserved
	packet.addHtonU16(0x0000);

	//Source long position vector
	packet.addGNAddress((uint8_t *)&m_sourcePV.GnAddress[0]);
	packet.addHtonU32(m_sourcePV.TST);
	packet.addHtonU32(m_sourcePV.latitude);
	packet.addHtonU32(m_sourcePV.longitude);
	packet.addHtonU16(pai_speed);
	packet.addHtonU16(m_sourcePV.heading);
	//GeoArea position latitude
	packet.addHtonU32(m_posLat);
	//GeoArea position longitude
	packet.addHtonU32(m_posLong);
	//Distance A
	packet.addHtonU16(m_distA);
	//Distance B
	packet.addHtonU16(m_distB);
	//Angle
	packet.addHtonU16(m_angle);
	//Reserved
	packet.addHtonU16(0x0000);
}

gbcHeader::~gbcHeader() = default;

GeoArea_t
gbcHeader::GetGeoArea() const {
	GeoArea_t retval;
	retval.posLat = m_posLat;
	retval.posLong = m_posLong;
	retval.distA = m_distA;
	retval.distB = m_distB;
	retval.angle = m_angle;
	return retval;
}

void
gbcHeader::SetGeoArea (GeoArea_t geoArea) {
	m_posLat = geoArea.posLat;
	m_posLong = geoArea.posLong;
	m_distA = geoArea.distA;
	m_distB = geoArea.distB;
	m_angle = geoArea.angle;
}