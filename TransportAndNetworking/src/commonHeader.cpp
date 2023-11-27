//
// Created by Carlos Mateo Risma Carletti on 07/05/21.
// Updated by Francesco Raviglione in November 2021
//

#include "commonHeader.h"
#include <cstring>
#include <bitset>

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

void 
commonHeader::printCommonHeader(packetBuffer &packet,std::string filename) {
    FILE *f_out;
    const uint8_t *m_internal_buff=packet.getBufferPointer();
    
    char file[strlen(filename.c_str())+1];
    snprintf(file,sizeof(file),"%s",filename.c_str());
    
    f_out=fopen(file,"a");
    
    fprintf(f_out,"    <Common Header>\n");
    
    std::string next=std::bitset<8>(m_internal_buff[0]).to_string().substr(0,4);
    std::string nextHeader;
    if(stoi(next,nullptr,2)==0)
        nextHeader="Header Type";
    if(stoi(next,nullptr,2)==1)
        nextHeader="BTP-A Transport Protocol";
    if(stoi(next,nullptr,2)==2)
        nextHeader="BTP-B Transport Protocol";
    if(stoi(next,nullptr,2)==3)
        nextHeader="Upper Protocol Entity";
    if(stoi(next,nullptr,2)>3)
        nextHeader="Unknown";
    fprintf(f_out,"        <Next Header>%s</Next Header>\n",nextHeader.c_str());
    
    std::string reserved=std::bitset<8>(m_internal_buff[0]).to_string().substr(4,8);
    fprintf(f_out,"        <Reserved>%d</Reserved>\n",stoi(reserved,nullptr,2));
    
    uint8_t type=m_internal_buff[1];
    std::string headerType;
    bool ht=false;
    if(type==0) {
        headerType="Any Unspecified";
        ht=true;
    }
    if(type==16) {
        headerType="Beacon Unspecified";
        ht=true;
    }
    if(type==32) {
        headerType="GeoUnicast Unspecified";
        ht=true;
    }
    if(type==48) {
        headerType="Geographically-Scoped Anycast - Circular Area";
        ht=true;
    }
    if(type==49) {
        headerType="Geographically-Scoped Anycast - Rectangular Area";
        ht=true;
    }
    if(type==50) {
        headerType="Geographically-Scoped Anycast - Ellipsoidal Area";
        ht=true;
    }
    if(type==64) {
        headerType="Geographically-Scoped Broadcast - Circular Area";
        ht=true;
    }
    if(type==65) {
        headerType="Geographically-Scoped Broadcast - Rectangular Area";
        ht=true;
    }
    if(type==66) {
        headerType="Geographically-Scoped Broadcast - Ellipsoidal Area";
        ht=true;
    }
    if(type==80) {
        headerType="Topologically-Scoped Broadcast - Single-Hop Broadcast";
        ht=true;
    }
    if(type==81) {
        headerType="Topologically-Scoped Broadcast - Multi-Hop TSB";
        ht=true;
    }
    if(type==96) {
        headerType="Location Service Request";
        ht=true;
    }
    if(type==97) {
        headerType="Location Service Reply";
        ht=true;
    }
    if(ht==false)
        headerType="Unknown";
    fprintf(f_out,"        <Header Type>%s</Header Type>\n",headerType.c_str());
    
    fprintf(f_out,"        <Traffic Class>\n");
    
    std::string trafficClass=std::bitset<8>(m_internal_buff[2]).to_string();
    std::string scf=trafficClass.substr(0,1);
    fprintf(f_out,"            <Store Carry Forward>%d</Store Carry Forward>\n",stoi(scf));
    
    std::string channelOffload=trafficClass.substr(1,2);
    fprintf(f_out,"            <Channel Offload>%d</Channel Offload>\n",stoi(channelOffload));
    
    std::string tcID=trafficClass.substr(2,8);
    std::string trafficClassID;
    if(stoi(tcID,nullptr,2)==0)
        trafficClassID="ITS-G5 Access Category Voice";
    if(stoi(tcID,nullptr,2)==1)
        trafficClassID="ITS-G5 Access Category Video";
    if(stoi(tcID,nullptr,2)==2)
        trafficClassID="ITS-G5 Access Category Best Effort";
    if(stoi(tcID,nullptr,2)==3)
        trafficClassID="ITS-G5 Access Category Background";
    if(stoi(tcID,nullptr,2)>3)
        trafficClassID="Unknown";
    fprintf(f_out,"            <Traffic Class ID>%s</Traffic Class ID>\n",trafficClassID.c_str());
    
    fprintf(f_out,"        </Traffic Class>\n");
    
    fprintf(f_out,"        <Flags>\n");
    
    std::string flags=std::bitset<8>(m_internal_buff[3]).to_string();
    std::string mobilityFlag=flags.substr(0,1);
    fprintf(f_out,"            <Mobility Flag>%d</Mobility Flag>\n",stoi(mobilityFlag));
    
    std::string reserved2=flags.substr(1,8);
    fprintf(f_out,"            <Reserved>%d</Reserved>\n",stoi(reserved2,nullptr,2));
    
    fprintf(f_out,"        </Flags>\n");
    
    uint16_t payloadLength=(m_internal_buff[4]<<8) | m_internal_buff[5];
    fprintf(f_out,"        <Payload Length>%d</Payload Length>\n",payloadLength);
    
    uint8_t maximumHopLimit=m_internal_buff[6];
    fprintf(f_out,"        <Maximum Hop Limit>%d</Maximum Hop Limit>\n",maximumHopLimit);
    
    uint8_t reserved3=m_internal_buff[7];
    fprintf(f_out,"        <Reserved>%d</Reserved>\n",reserved3);
    
    fprintf(f_out,"    </Common Header>\n");
    
    fclose(f_out);
}

void commonHeader::removeHeader(unsigned char *buffer) {

    uint8_t chNH = 0;
    chNH = (uint8_t) *buffer;
    buffer++;
    m_nextHeader = chNH >> 4;
    uint8_t headerType = 0;
    headerType = (uint8_t) *buffer;
    buffer++;
    m_headerType = headerType >> 4;
    m_headerSubType = headerType & 0x0f;
    m_trafficClass = (uint8_t) *buffer;
    buffer++;
    uint8_t chflag = 0;
    chflag = (uint8_t) *buffer;
    buffer++;
    m_flag = chflag >> 7;
    memcpy(&m_payload, buffer, sizeof(uint16_t));
    buffer += 2;
    m_payload = swap_16bit(m_payload);
    m_maxHopLimit = (uint8_t) *buffer;
    buffer++;
    m_reserved = (uint8_t) *buffer;
    buffer++;
}
