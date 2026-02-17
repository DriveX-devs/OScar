#ifndef ITSSORIGINATINGTABLEENTRY_H
#define ITSSORIGINATINGTABLEENTRY_H

#include "denData.h"
#include "packetBuffer.h"
#include "TransportAndNetworking/include/gn_utils.h"

class ITSSOriginatingTableEntry {
public:
    typedef enum {
        STATE_UNSET,
        STATE_ACTIVE,
        STATE_CANCELLED,
        STATE_NEGATED
    } denm_table_state_t;

    ITSSOriginatingTableEntry();
    ITSSOriginatingTableEntry(packetBuffer asnDenmPacket, denm_table_state_t status, denData::DEN_ActionID_t actionID);
    ITSSOriginatingTableEntry(packetBuffer asnDenmPacket, denm_table_state_t status, denData::DEN_ActionID_t actionID, long referenceTime);

    /* Setters */
    void setStatus(denm_table_state_t status) {
        m_status = status;
    }

    void setDENMPacket(packetBuffer DENMPacket) {
        m_denm_encoded = DENMPacket;
    }

    void setLocation (double lat, double lon) {
        m_location.first = lat;
        m_location.second = lon;
    }

    /* Getters */
    denm_table_state_t getStatus(void) {
        return m_status;
    }

    packetBuffer getDENMPacket(void) {
        return m_denm_encoded;
    }

    long getReferenceTime(void) {
        return m_referenceTime;
    }

    std::pair<double,double> getLocation(void) {
        return m_location;
    }

    GeoArea_t getGeoArea(void) {return m_geoArea;}
    void setGeoArea(GeoArea_t geoArea) {m_geoArea = geoArea;}
private:
    denData::DEN_ActionID_t m_actionid;
    packetBuffer m_denm_encoded;
    denm_table_state_t m_status;
    long m_referenceTime;
    GeoArea_t m_geoArea;
    std::pair<double,double> m_location; // lat + lon of the location of the event
};

#endif // ITSSORIGINATINGTABLEENTRY_H
