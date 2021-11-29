//
// Created by Carlos Mateo Risma Carletti on 11/05/21.
// Updated by Francesco Raviglione in November 2021
//

#ifndef OCABS_BTPHEADER_H
#define OCABS_BTPHEADER_H

#include <cstdint>
#include <cstring>
#include "utils.h"
#include "packetBuffer.h"

class btpHeader {
public:
    btpHeader();
    ~btpHeader();
    void serializeInto(packetBuffer &packet);

    void setDestinationPort(uint16_t port) {m_destinationPort = port;}
    void setDestinationPortInfo(uint16_t portInfo) {m_source_destInfo = portInfo;}
    void setSourcePort(uint16_t port) {m_source_destInfo = port;}

    //getters
    [[nodiscard]] uint16_t getDestPort() const {return m_destinationPort;}
    [[nodiscard]] uint16_t getSourcePort() const {return m_source_destInfo;}
    [[nodiscard]] uint16_t getDestPortInfo() const {return m_source_destInfo;}
private:
    uint16_t m_destinationPort; //!< Destination port
    uint16_t m_source_destInfo; //!< Source port/Destination port info
};

#endif // OCABS_BTPHEADER_H
