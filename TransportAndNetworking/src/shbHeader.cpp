//
// Created by Carlos Mateo Risma Carletti on 07/05/21.
// Updated by Francesco Raviglione in November 2021
//

#include "shbHeader.h"

#include <cstring>
#include <bitset>

shbHeader::shbHeader() {
	m_sourcePV = {};  //!Source long position vector
	m_reserved = 0; //! aux variable for reading reserved fields
	m_reserved32 = 0;
	m_local_CBR = 0;
	m_max_CBR_neighbouring = 0;
	m_tx_power_reserved = 0;
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

    if (m_dcc == nullptr)
    {
        //Reserved
        packet.addHtonU32(0x00000000);
    }
    else
    {
        double cbr0 = m_dcc->getCBRR0();
        if (cbr0 < 0.0f) cbr0 = 0.0f;
        if (cbr0 > 1.0f) cbr0 = 1.0f;
        uint8_t cbr0_enc = static_cast<uint8_t>(std::floor(cbr0 * 255.0f));
        double cbr1 = m_dcc->getCBRR1();
        uint8_t cbr1_enc = static_cast<uint8_t>(std::floor(cbr1 * 255.0f));
        int tp = 0;
        tp = getTxPower(); // 0..31
        if (tp < 0)  tp = 0;
        if (tp > 31) tp = 31;
        uint8_t txp_byte = static_cast<uint8_t>(tp & 0x1F); // bits 0–4
        uint8_t reserved = 0;
        uint32_t field =
                (uint32_t(cbr0_enc) << 24) |
                (uint32_t(cbr1_enc) << 16) |
                (uint32_t(txp_byte) <<  8) |
                uint32_t(reserved);
        packet.addHtonU32(field);
    }
}

void 
shbHeader::printTSBPheader(packetBuffer &packet,std::string filename) {
	FILE *f_out;
	const uint8_t *m_internal_buff=packet.getBufferPointer();

	char file[strlen(filename.c_str())+1];
	snprintf(file,sizeof(file),"%s",filename.c_str());
		
	f_out=fopen(file,"a");

	fprintf(f_out,"    <Topologically-Scoped Broadcast Packet>\n");
	fprintf(f_out,"        <Source Position>\n");
	fprintf(f_out,"            <GeoNetworking Address>\n");
	
	std::string gn_addr=std::bitset<8>(m_internal_buff[0]).to_string()+std::bitset<8>(m_internal_buff[1]).to_string();
	fprintf(f_out,"                <Manual>%c</Manual>\n",gn_addr[0]);
	
	std::string itssType;
	if(gn_addr.substr(1,5) == "00000")
		itssType="Unknown";
	if(gn_addr.substr(1,5) == "00001")
		itssType="Pedestrian";
	if(gn_addr.substr(1,5) == "00010")
		itssType="Cyclist";
	if(gn_addr.substr(1,5) == "00011")
		itssType="Moped";
	if(gn_addr.substr(1,5) == "00100")
		itssType="Motorcycle";
	if(gn_addr.substr(1,5) == "00101")
		itssType="Passenger Car";
	if(gn_addr.substr(1,5) == "00110")
		itssType="Bus";
	if(gn_addr.substr(1,5) == "00111")
		itssType="Light Truck";
	if(gn_addr.substr(1,5) == "01000")
		itssType="Heavy Truck";
	if(gn_addr.substr(1,5) == "01001")
		itssType="Trailer";
	if(gn_addr.substr(1,5) == "01010")
		itssType="Special Vehicle";
	if(gn_addr.substr(1,5) == "01011")
		itssType="Tram";
	if(gn_addr.substr(1,5) == "01100" or gn_addr.substr(1,5) == "01101" or gn_addr.substr(1,5) == "01110" or gn_addr.substr(1,2).compare("1") == 0)
		itssType="Unavailable";
	if(gn_addr.substr(1,5) == "01111")
		itssType="Road Side Unit";
	fprintf(f_out,"                <ITS-S type>%s</ITS-S type>\n",itssType.c_str());
	
	if(gn_addr.substr(6,15) == "0000000000")
		fprintf(f_out,"                <ITS-S Country Code>Reserved</ITS-S Country Code>\n");
	else
		fprintf(f_out,"                <ITS-S Country Code>TO BE DONE</ITS-S Country Code> #TBD\n");
	
	fprintf(f_out,"                <MID>%02X:%02X:%02X:%02X:%02X:%02X</MID>\n",m_internal_buff[2],m_internal_buff[3],m_internal_buff[4],m_internal_buff[5],m_internal_buff[6],m_internal_buff[7]);

	fprintf(f_out,"            </GeoNetworking Address>\n");
	
	uint32_t timestamp=(m_internal_buff[8]<<24) | (m_internal_buff[9]<<16) | (m_internal_buff[10]<<8) | m_internal_buff[11];
	fprintf(f_out,"            <Timestamp>%s</Timestamp>\n",std::to_string(timestamp).c_str());
	
	uint32_t latitude=(m_internal_buff[12]<<24) | (m_internal_buff[13]<<16) | (m_internal_buff[14]<<8) | m_internal_buff[15];
	fprintf(f_out,"            <Latitude>%s</Latitude>\n",std::to_string(latitude).c_str());
	
	uint32_t longitude=(m_internal_buff[16]<<24) | (m_internal_buff[17]<<16) | (m_internal_buff[18]<<8) | m_internal_buff[19];
	fprintf(f_out,"            <Longitude>%s</Longitude>\n",std::to_string(longitude).c_str());
	
	std::string pai_speed=std::bitset<8>(m_internal_buff[20]).to_string()+std::bitset<8>(m_internal_buff[21]).to_string();
	fprintf(f_out,"            <Position Accuracy Indicator>%c</Position Accuracy Indicator>\n",pai_speed[0]);
	
	fprintf(f_out,"            <Speed>%03d</Speed>\n",stoi(pai_speed.substr(1,7),nullptr,2));
	
	uint16_t heading=(m_internal_buff[22]<<8) | m_internal_buff[23];
	fprintf(f_out,"            <Heading>%04d</Heading>\n",heading);
	
	fprintf(f_out,"        </Source Position>\n");
	
	uint32_t reserved=(m_internal_buff[24]<<24) | (m_internal_buff[25]<<16) | (m_internal_buff[26]<<8) | m_internal_buff[27];;
	fprintf(f_out,"        <Reserved>%d</Reserved>\n",reserved);
	
	fprintf(f_out,"    </Topologically-Scoped Broadcast Packet>\n");
	
	fprintf(f_out,"</GeoNetworking>\n\n");
	
	fclose(f_out);
}

void shbHeader::removeHeader(unsigned char *buffer) {

        //Source long position vector
        memcpy(m_sourcePV.GnAddress, buffer, 8);
        buffer += 8;
        memcpy(&m_sourcePV.TST, buffer, sizeof(uint32_t));
        buffer += 4;
        m_sourcePV.TST = swap_32bit(m_sourcePV.TST);
        memcpy(&m_sourcePV.latitude, buffer, sizeof(uint32_t));
        buffer += 4;
        m_sourcePV.latitude = swap_32bit(m_sourcePV.latitude);
        memcpy(&m_sourcePV.longitude, buffer, sizeof(uint32_t));
        buffer += 4;
        m_sourcePV.longitude = swap_32bit(m_sourcePV.longitude);
        uint16_t pai_speed = 0;
        memcpy(&pai_speed, buffer, sizeof(uint16_t));
        buffer += 2;
        pai_speed = swap_16bit(pai_speed);
        m_sourcePV.positionAccuracy = pai_speed >> 15;
        m_sourcePV.speed = pai_speed & 0x7f;
        memcpy(&m_sourcePV.heading, buffer, sizeof(uint16_t));
        buffer += 2;
        m_sourcePV.heading = swap_16bit(m_sourcePV.heading);

		uint32_t field;
		memcpy(&field, buffer, sizeof(uint32_t));
		buffer += 4;
		field = swap_32bit(field);
        uint8_t cbr0_enc = (field >> 24) & 0xFF;
		uint8_t cbr1_enc = (field >> 16) & 0xFF;
		uint8_t txp_byte = (field >> 8)  & 0xFF;
		(void) txp_byte;
		// uint8_t reserved = field & 0xFF;
        m_CBR_R0_Hop = static_cast<double>(cbr0_enc) / 255.0f;
        m_CBR_R1_Hop = static_cast<double>(cbr1_enc) / 255.0f;
    }
