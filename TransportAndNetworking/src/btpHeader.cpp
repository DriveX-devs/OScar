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

void 
btpHeader::printBTPheader(packetBuffer &packet,std::string filename) {
    FILE* f_out;
    const uint8_t *m_internal_buff = packet.getBufferPointer();
    
    char file[strlen(filename.c_str())+1];
    snprintf(file,sizeof(file),"%s",filename.c_str());
    
    f_out=fopen(file,"a");
    
    fprintf(f_out,"<BTP-B Header>\n");
    
    uint16_t destPort=(m_internal_buff[0]<<8) | m_internal_buff[1];
    fprintf(f_out,"    <Destination Port>%d</Destination Port>\n",destPort);
    
    uint16_t destPortInfo=(m_internal_buff[2]<<8) | m_internal_buff[3];
    fprintf(f_out,"    <Destination Port Info>%d</Destination Port Info>\n",destPortInfo);
    
    fprintf(f_out,"</BTP-B Header>\n");
    
    fclose(f_out);
}
