#ifndef MCDATA_H
#define MCDATA_H

#include "Getter.hpp"
#include <vector>

extern "C" {
    #include "MCM.h"
}

class mcData {
public:

    template<class T>
    class MCDataItem {
        private:
            bool m_available;
            T m_dataitem;

        public:
            MCDataItem(T data) : m_dataitem(data) { m_available = true; }
            MCDataItem(bool availability) { m_available = availability; }

            MCDataItem() {
                m_available = false;
                m_dataitem = {};
            }

            T getData() { return m_dataitem; }
            bool isAvailable() { return m_available; }

            void setData(T data) {
                m_dataitem = data;
                m_available = true;
            }

            T getDataRef() { return &m_dataitem; }
    };

    template<class V = int, class C = int>
    class MCValueConfidence {
        private:
            V m_value;
            C m_confidence;

        public:
            MCValueConfidence() {
            }

            MCValueConfidence(V value, C confidence) : m_value(value), m_confidence(confidence) {
            }

            V getValue() { return m_value; }
            C getConfidence() { return m_confidence; }
            void setValue(V value) { m_value = value; }
            void setConfidence(C confidence) { m_confidence = confidence; }
    };

    typedef struct MC_PosConfidenceEllipse {
        long semiMajorConfidence;
        long semiMinorConfidence;
        long semiMajorOrientation;
    } MC_PosConfidenceEllipse_t;

    typedef struct _header {
        long messageID;
        long protocolVersion;
        unsigned long stationID;
    } mcDataHeader;

    typedef struct _basicContainer {
        long stationID;
        long itsRole;
        long stationType;
        long mcmType;
        long maneuverID;
        long concept;
        MCValueConfidence<long, long> altitude;
        MC_PosConfidenceEllipse_t posConfidenceEllipse;
        MCDataItem<long> relevanceDistance;
        MCDataItem<long> relevanceTrafficDirection;
        MCDataItem<long> validityDuration;
        MCDataItem<long> transmissionInterval;
        MCDataItem<long> cost;
        MCDataItem<long> goal;
        MCDataItem<long> executionStatus;
    } mcBasicContainer;

    typedef struct MCSubmaneuvers {

    } MCSubmaneuvers;

    typedef struct MCManeuverAdvice {

    } MCManeuverAdvice;

    typedef struct _maneuverContainer {
        long vehicleType;
        MCDataItem<std::vector<MCSubmaneuvers>> submaneuvers;
        MCDataItem<std::vector<MCManeuverAdvice>> advices;
    } mcManeuverContainer;

    typedef struct _adviceContainer {
        MCDataItem<std::vector<MCManeuverAdvice>> advices;
    } mcAdviceContainer;

    typedef struct _responseContainer {
        long response;
        MCDataItem<long> declineReason;
        MCDataItem<std::vector<MCSubmaneuvers>> submaneuvers;
    } mcResponseContainer;

    typedef struct _acknowledgmentContainer {
        long type;
    } mcAcknowledgeContainer;

    typedef struct _terminationContainer {
    } mcTerminationContainer;

    mcData()=default;
    ~mcData()=default;

    MCDataItem<mcDataHeader> getHeader() {return m_header;};
    MCDataItem<mcBasicContainer> getBasicContainer() {return m_basic_container;};
    MCDataItem<mcManeuverContainer> getManeuverContainer() {return m_maneuver_container;};
    MCDataItem<mcAdviceContainer> getAdviceContainer() {return m_advice_container;};
    MCDataItem<mcResponseContainer> getResponseContainer() {return m_response_container;};
    MCDataItem<mcAcknowledgeContainer> getAcknowledgmentContainer() (return m_acknowledgment_container;);
    MCDataItem<mcTerminationContainer>

    private:
    MCDataItem<mcDataHeader> m_header;
    MCDataItem<mcBasicContainer> m_basic_container;
    MCDataItem<mcManeuverContainer> m_maneuver_container;
    MCDataItem<mcAdviceContainer> m_advice_container;
    MCDataItem<mcResponseContainer> m_response_container;
    MCDataItem<mcAcknowledgeContainer> m_acknowledgment_container;
    MCDataItem<mcTerminationContainer> m_termination_container;
};

#endif
