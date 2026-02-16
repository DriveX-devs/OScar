#ifndef ITSSRECEIVINGTABLEENTRY_H
#define ITSSRECEIVINGTABLEENTRY_H

#include "denData.h"
#include "packetBuffer.h"
#include "TransportAndNetworking/include/utils.h"

#define ITSS_RX_ENTRY_TERMINATION_UNSET -1


class ITSSReceivingTableEntry {
public:
    typedef enum {
        STATE_UNSET,
        STATE_ACTIVE,
        STATE_CANCELLED,
        STATE_NEGATED
    } denm_table_state_t;

    ITSSReceivingTableEntry();

    ITSSReceivingTableEntry(packetBuffer asnDenmPacket, denm_table_state_t status, denData::DEN_ActionID_t actionID, long referenceTime, long detectionTime);

    ITSSReceivingTableEntry(packetBuffer asnDenmPacket, denm_table_state_t status, denData::DEN_ActionID_t actionID, long referenceTime, long detectionTime, long *termination_ptr);

    /* Setters */
    void setStatus(denm_table_state_t status) {
        m_status = status;
    }

    void setDENMPacket(packetBuffer DENMPacket) {
        m_denm_encoded = DENMPacket;
    }

    void setTermination_if_available(Termination_t *termination_ptr) {
        m_termination = termination_ptr == NULL ? ITSS_RX_ENTRY_TERMINATION_UNSET : *(termination_ptr);
    }

    void setTermination(Termination_t termination) {
        m_termination = termination;
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

    long getDetectionTime(void) {
        return m_detectionTime;
    }

    long getTermination(void) {
        return m_termination;
    }

    bool isTerminationSet(void) {
        return m_termination != ITSS_RX_ENTRY_TERMINATION_UNSET;
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
    long m_detectionTime;
    long m_termination;
    GeoArea_t m_geoArea;
    std::pair<double,double> m_location; // lat + lon of the location of the event
};

#endif // ITSSRECEIVINGTABLEENTRY_H
