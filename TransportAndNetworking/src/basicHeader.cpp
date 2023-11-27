//
// Created by Carlos Mateo Risma Carletti on 07/05/21.
// Updated by Francesco Raviglione in November 2021
//

#include "basicHeader.h"
#include <bitset>
#include <cstring>

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

void
basicHeader::printBasicHeader(packetBuffer &packet,std::string filename) {
    FILE *f_out;
    const uint8_t *m_internal_buff = packet.getBufferPointer();
    
    char file[strlen(filename.c_str())+1];
    snprintf(file,sizeof(file),"%s",filename.c_str());
    
    f_out=fopen(file,"a");

    fprintf(f_out,"<GeoNetworking>\n");
    fprintf(f_out,"    <Basic Header>\n");
    
    std::string version_next=std::bitset<8>(m_internal_buff[0]).to_string();
    std::string version=version_next.substr(0,4);

    fprintf(f_out,"        <Version>%d</Version>\n",stoi(version,nullptr,2));
    
    std::string next=version_next.substr(4,8);
    std::string nextHeader;

    if(stoi(next,nullptr,2)==0) {
        nextHeader="Unspecified";
    }

    if(stoi(next,nullptr,2)==1) {
        nextHeader="Common Header";
    }

    if(stoi(next,nullptr,2)==2) {
        nextHeader="Secured Packet";
    }

    if(stoi(next,nullptr,2)>2) {
        nextHeader="Unknown";
    }

    fprintf(f_out,"        <Next Header>%s</Next Header>\n",nextHeader.c_str());
    
    uint8_t reserved=m_internal_buff[1];
    fprintf(f_out,"        <Reserved>%d</Reserved>\n",reserved);
    
    fprintf(f_out,"        <Life Time>\n");
    
    uint8_t lifetime=m_internal_buff[2];
    fprintf(f_out,"            <Life Time Value>%d</Life Time Value>\n",lifetime);
    
    std::string lifetimeFlags=std::bitset<8>(m_internal_buff[2]).to_string();
    std::string multiplier=lifetimeFlags.substr(0,6);
    fprintf(f_out,"            <Life Time Multiplier>%d</Life Time Multiplier>\n",stoi(multiplier,nullptr,2));
    
    std::string base=lifetimeFlags.substr(6,8);
    std::string lifetimeBase;
    if(stoi(base,nullptr,2)==0) {
        lifetimeBase="50ms";
    }

    if(stoi(base,nullptr,2)==1) {
        lifetimeBase="1s";
    }

    if(stoi(base,nullptr,2)==2) {
        lifetimeBase="10s";
    }

    if(stoi(base,nullptr,2)==3) {
        lifetimeBase="100s";
    }
    
    fprintf(f_out,"            <Life Time Base>%s</Life Time Base>\n",lifetimeBase.c_str());
    
    fprintf(f_out,"        </Life Time>\n");
    
    uint8_t remainingHops=m_internal_buff[3];
    fprintf(f_out,"        <Remaining Hop Limit>%d</Remaining Hop Limit>\n",remainingHops);
    
    fprintf(f_out,"    </Basic Header>\n");

    fclose(f_out);
}

void basicHeader::removeHeader(unsigned char *buffer) {
    uint8_t version_NH = 0;
    version_NH = (uint8_t) *buffer;
    m_version = version_NH >> 4;
    m_nextHeader = version_NH & 0x0f;

    buffer++;
    m_reserved = (uint8_t) *buffer;
    buffer++;
    m_lifeTime = (uint8_t) *buffer;
    buffer++;
    m_remainingHopLimit = (uint8_t) *buffer;
}
