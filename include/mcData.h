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

            T getData() const { return m_dataitem; }
            bool isAvailable() const { return m_available; }

            void setData(T data) {
                m_dataitem = data;
                m_available = true;
            }

            T* getDataRef() { return &m_dataitem; }
    };

    template<class V = int, class C = int>
    class MCValueConfidence {
        private:
            V m_value;
            C m_confidence;

        public:
            MCValueConfidence() : m_value(0), m_confidence(0) {}
            MCValueConfidence(V value, C confidence) : m_value(value), m_confidence(confidence) {}

            V getValue() const { return m_value; }
            C getConfidence() const { return m_confidence; }
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
        long cost;
        long goal;
        MCDataItem<long> executionStatus;
    } mcBasicContainer;

    // --- Added Structures for Submaneuvers and Advice ---

    typedef struct _PathPoint {
        long deltaLatitude;
        long deltaLongitude;
        long deltaAltitude;
        MCDataItem<long> pathDeltaTime;
    } PathPoint;

    typedef struct _Trajectory {
        long wayPointType;
        std::vector<PathPoint> wayPoints;
        std::vector<MCValueConfidence<long, long>> speed;
        std::vector<MCValueConfidence<long, long>> headings;
        std::vector<long> longitudePositions;
        std::vector<long> latitudePositions;
        std::vector<MCValueConfidence<long, long>> altitudePositions;
    } Trajectory;

    typedef struct _TrrDescription {
        long trrType;
        long laneCount;
        long trrWidth;
        long trrLength;
        MCDataItem<long> startingLaneNumber;
        MCDataItem<long> endingLaneNumber;
        std::vector<PathPoint> waypoints;
        std::vector<MCValueConfidence<long, long>> heading;
    } TrrDescription;

    typedef struct _TemporalCharacteristics {
        long tRROccupancyStartTime;
        long tRROccupancyEndTime;
    } TemporalCharacteristics;

    typedef struct _AdvisedTargetRoadResource {
        TrrDescription trrDescription;
        TemporalCharacteristics temporalCharacteristics;
    } TargetRoadResource;

    typedef struct _SubmanoeuvreStrategy {
        int present;  // SubmanoeuvreStrategy_PR enum value
        long value;   // il valore del choice corrispondente
    } MCSubmanoeuvreStrategy;

    typedef struct MCSubmaneuvers {
        long submanoeuvreId;
        MCDataItem<Trajectory> referenceTrajectory;
        MCDataItem<MCSubmanoeuvreStrategy> submanoeuvreStrategy;
        MCDataItem<TargetRoadResource> targetRoadResource;
    } MCSubmaneuvers;

    typedef struct MCManeuverAdvice {
        long executantID;
        MCDataItem<long> currentStateAdvisedChange; 
        std::vector<MCSubmaneuvers> submaneuvres;
    } MCManeuverAdvice;

    // ----------------------------------------------------

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
        // Add fields if necessary based on ASN.1
    } mcTerminationContainer;

    mcData()=default;
    ~mcData()=default;

    MCDataItem<mcDataHeader> getHeader() const { return m_header; }
    MCDataItem<mcBasicContainer> getBasicContainer() const { return m_basic_container; }
    MCDataItem<mcManeuverContainer> getManeuverContainer() const { return m_maneuver_container; }
    MCDataItem<mcAdviceContainer> getAdviceContainer() const { return m_advice_container; }
    MCDataItem<mcResponseContainer> getResponseContainer() const { return m_response_container; }
    MCDataItem<mcAcknowledgeContainer> getAcknowledgmentContainer() const { return m_acknowledgment_container; }
    MCDataItem<mcTerminationContainer> getTerminationContainer() const { return m_termination_container; }
    long getStationID() const { return m_basic_container.getData().stationID; }
    long getITSRole() const { return m_basic_container.getData().itsRole; }
    long getStationType() const { return m_basic_container.getData().stationType; }
    long getMCMType() const { return m_basic_container.getData().mcmType; }
    long getManeuverID() const { return m_basic_container.getData().maneuverID; }
    long getConcept() const { return m_basic_container.getData().concept; }
    long getCost() const { return m_basic_container.getData().cost; }
    long getGoal() const { return m_basic_container.getData().goal; }
    const MCDataItem<long>& getExecutionStatus() const { return m_basic_container.getData().executionStatus; }


    // Setters
    void setHeader(const mcDataHeader& v) { m_header.setData(v); }
    void setBasicContainer(const mcBasicContainer& v) { m_basic_container.setData(v); }
    void setManeuverContainer(const mcManeuverContainer& v) { m_maneuver_container.setData(v); }
    void setAdviceContainer(const mcAdviceContainer& v) { m_advice_container.setData(v); }
    void setResponseContainer(const mcResponseContainer& v) { m_response_container.setData(v); }
    void setAcknowledgmentContainer(const mcAcknowledgeContainer& v) { m_acknowledgment_container.setData(v); }
    void setTerminationContainer(const mcTerminationContainer& v) { m_termination_container.setData(v); }

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