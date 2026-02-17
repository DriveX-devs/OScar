#ifndef DENDATA_H
#define DENDATA_H

#include "Getter.hpp"
#include <vector>

extern "C" {
    #include "DENM.h"
}

typedef struct CauseCode sCauseCode_t;
typedef struct EventHistory sEventHistory_t;

typedef struct Speed sSpeed_t;
typedef struct Heading sHeading_t;

typedef struct ImpactReductionContainer sImpactReductionContainer_t;
typedef struct RoadWorksContainerExtended sRoadWorksContainerExtended_t;
typedef struct StationaryVehicleContainer sStationaryVehicleContainer_t;

#define DEN_DEFAULT_VALIDITY_S 600

class denData {
public:
    typedef struct _internals {
        uint32_t repetitionDuration;
        uint32_t repetitionInterval;
        bool isMandatorySet;
    } denDataInternals;

    typedef struct _header {
        long messageID;
        long protocolVersion;
        unsigned long stationID;
    } denDataHeader;

    template<class T>
    class DENDataItem {
        private:
            bool m_available;
            T m_dataitem;

        public:
            DENDataItem(T data) : m_dataitem(data) { m_available = true; }
            DENDataItem(bool availability) { m_available = availability; }

            DENDataItem() {
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
    class DENValueConfidence {
        private:
            V m_value;
            C m_confidence;

        public:
            DENValueConfidence() {
            }

            DENValueConfidence(V value, C confidence) : m_value(value), m_confidence(confidence) {
            }

            V getValue() { return m_value; }
            C getConfidence() { return m_confidence; }
            void setValue(V value) { m_value = value; }
            void setConfidence(C confidence) { m_confidence = confidence; }
    };

    typedef struct DEN_PosConfidenceEllipse {
        long semiMajorConfidence;
        long semiMinorConfidence;
        long semiMajorOrientation;
    } DEN_PosConfidenceEllipse_t;

    typedef struct DEN_ActionID{
        unsigned long originatingStationID;
        long sequenceNumber;
    } DEN_ActionID_t;

    typedef struct DEN_ReferencePosition{
        long latitude;
        long longitude;
        DEN_PosConfidenceEllipse_t positionConfidenceEllipse;
        DENValueConfidence<long,long> altitude;
    } DEN_ReferencePosition_t;

    typedef struct DEN_DeltaReferencePosition {
        long deltaLatitude;
        long deltaLongitude;
        long deltaAltitude;
    } DEN_DeltaReferencePosition_t;

    typedef struct DEN_EventPoint{
        DEN_DeltaReferencePosition_t	 eventPosition;
        DENDataItem<long>	eventDeltaTime;
        long	 informationQuality;
    } DEN_EventPoint_t;

    typedef struct DEN_PathPoint{
        DEN_DeltaReferencePosition_t pathPosition;
        DENDataItem<long> pathDeltaTime;
    }DEN_PathPoint_t;

    typedef struct _management {
        //ActionID_t actionID; // Defined when calling appDENM_trigger
        unsigned long stationID;
        long sequenceNumber;
        long detectionTime;
        long referenceTime;
        DENDataItem<long> termination;
        //ReferencePosition_t eventPosition;
        long longitude;
        long latitude;
        DENValueConfidence<long, long> altitude;
        DEN_PosConfidenceEllipse_t posConfidenceEllipse;
        DENDataItem<long> relevanceDistance;
        DENDataItem<long> relevanceTrafficDirection;
        DENDataItem<long> validityDuration;
        DENDataItem<long> transmissionInterval;
        // StationType_t stationType; // Defined during the creation of the DEN Basic service object
    } denDataManagement;

    typedef struct _situation {
        long informationQuality;
        long causeCode;
        long subCauseCode;
        DENDataItem<long> linkedCauseCode;
        DENDataItem<long> linkedSubCauseCode;
        DENDataItem<std::vector<DEN_EventPoint_t> > eventHistory;
    } denDataSituation;

    typedef struct _location {
        DENDataItem<DENValueConfidence<long, long> > eventSpeed;
        DENDataItem<DENValueConfidence<long, long> > eventPositionHeading;
        std::vector<std::vector<DEN_PathPoint_t> > traces;
        DENDataItem<long> roadType;
    } denDataLocation;

    typedef struct DEN_VehicleIdentification{
        DENDataItem<std::string> wMInumber; // OCTET_STRING !
        DENDataItem<std::string> vDS; // OCTET_STRING !
    } DEN_VehicleIdentification_t;

    typedef struct _DEN_DangerousGoodsExtended {
        long	 dangerousGoodsType;
        long	 unNumber;
        bool	 elevatedTemperature;
        bool	 tunnelsRestricted;
        bool	 limitedQuantity;
        DENDataItem<std::string>	emergencyActionCode; // OCTET_STRING !
        DENDataItem<std::string>	phoneNumber; // OCTET_STRING !
        DENDataItem<std::string>	companyName; // OCTET_STRING !
    } DEN_DangerousGoodsExtended_t;

    typedef struct _DEN_ImpactReductionCont {
        long heightLonCarrLeft;
        long heightLonCarrRight;
        long posLonCarrLeft;
        long posLonCarrRight;
        std::vector<long> positionOfPillars;
        long posCentMass;
        long wheelBaseVehicle;
        long turningRadius;
        long posFrontAx;
        long vehicleMass;
        long requestResponseIndication;
        uint32_t positionOfOccupants; // BIT_STRING 20 bits
    } DEN_ImpactReductionContainer_t;

    typedef struct DEN_RoadWorksContainerExtended{
        DENDataItem<uint8_t> lightBarSirenInUse;
        DENDataItem<uint8_t> innerhardShoulderStatus;
        DENDataItem<uint8_t> outerhardShoulderStatus;
        DENDataItem<uint16_t> drivingLaneStatus;
        DENDataItem<std::vector<long>> restriction;
        DENDataItem<long> speedLimit;
        DENDataItem<long> causeCode;
        DENDataItem<long> subCauseCode;
        DENDataItem<std::vector<DEN_ReferencePosition_t>> recommendedPath;
        DENDataItem<DEN_DeltaReferencePosition_t> startingPointSpeedLimit;
        DENDataItem<long> trafficFlowRule;
        DENDataItem<std::vector<DEN_ActionID_t>> referenceDenms;
    }DEN_RoadWorksContainerExtended_t;

    typedef struct DEN_StationaryVehicleContainer{
        DENDataItem<long> stationarySince;
        DENDataItem<long> causeCode;
        DENDataItem<long> subCauseCode;
        DENDataItem<DEN_DangerousGoodsExtended_t> carryingDangerousGoods;
        DENDataItem<long> numberOfOccupants;
        DENDataItem<DEN_VehicleIdentification_t> vehicleIdentification;
        DENDataItem<uint8_t> energyStorageType; // BIT_STRING 7 bits
    } DEN_StationaryVehicleContainer_t;

    typedef struct _alacarte {
        DENDataItem<long> lanePosition;
        DENDataItem<DEN_ImpactReductionContainer_t> impactReduction;
        DENDataItem<long> externalTemperature;
        DENDataItem<DEN_RoadWorksContainerExtended_t> roadWorks;
        DENDataItem<long> positioningSolution;
        DENDataItem<DEN_StationaryVehicleContainer_t> stationaryVehicle;
    } denDataAlacarte;

public:
    denData();

    /* AppDENM_trigger mandatory setters */
    // TODO: rename the arguments: these are 0.1 microdegrees, not deg
    void setDenmMandatoryFields(long detectionTime_ms, double latReference_deg, double longReference_deg);

    // TODO: rename the arguments: these are 0.1 microdegrees, not deg
    void setDenmMandatoryFields(long detectionTime_ms, double latReference_deg, double longReference_deg,
                                double altitude_m);

    void setDenmMandatoryFields_asn_types(TimestampIts_t detectionTime, ReferencePosition_t eventPosition);

    /*
     * Header setters (they can be used for experimentation purposes, but they shall not be called normally, as the header is typically
     * set inside the DEN Basic Service, without the need of a manual user intervention
    */
    void setDenmHeader(long messageID, long protocolVersion, unsigned long stationID);

    void setDenmMessageID(long messageID) { m_header.messageID = messageID; }
    void setDenmProtocolVersion(long protocolVersion) { m_header.protocolVersion = protocolVersion; }
    void setDenmStationID(unsigned long stationID) { m_header.stationID = stationID; }

    /* AppDENM_update mandatory setters */
    /* AppDENM_terminate mandatory setters */
    void setDenmMandatoryFields(unsigned long originatingStationID, long sequenceNumber, long detectionTime_ms,
                                double latReference_deg, double longReference_deg);

    void setDenmMandatoryFields(unsigned long originatingStationID, long sequenceNumber, long detectionTime_ms,
                                double latReference_deg, double longReference_deg, double altitude_m);

    void setDenmMandatoryFields_asn_types(ActionId_t actionID, TimestampIts_t detectionTime,
                                          ReferencePosition_t eventPosition);

    /* receiveDENM setters */
    void setDenmActionID(DEN_ActionID_t actionID);

    /* Optional information setters */

    /* Header getters */
    long getDenmHeaderMessageID() { return m_header.messageID; }
    long getDenmHeaderProtocolVersion() { return m_header.protocolVersion; }
    long getDenmHeaderStationID() { return m_header.stationID; }

    /* Container getters */
    denDataHeader getDenmHeader_asn_types() { return m_header; }
    denDataManagement getDenmMgmtData_asn_types() { return m_management; }
    DENDataItem<denDataSituation> getDenmSituationData_asn_types() { return m_situation; }
    DENDataItem<denDataLocation> getDenmLocationData_asn_types() { return m_location; }
    DENDataItem<denDataAlacarte> getDenmAlacarteData_asn_types() { return m_alacarte; }

    bool isDenmSituationDataSet() { return m_situation.isAvailable(); }
    bool isDenmLocationDataSet() { return m_location.isAvailable(); }
    bool isDenmAlacarteDataSet() { return m_alacarte.isAvailable(); }

    long getDenmMgmtDetectionTime() { return m_management.detectionTime; }

    long getDenmMgmtValidityDuration() {
        if (m_management.validityDuration.isAvailable()) {
            return m_management.validityDuration.getData();
        } else {
            return DEN_DEFAULT_VALIDITY_S;
        }
    }

    long getDenmMgmtReferenceTime() { return m_management.validityDuration.getData(); }

    long getDenmMgmtLatitude() { return m_management.latitude; }
    long getDenmMgmtLongitude() { return m_management.longitude; }
    long getDenmMgmtAltitude() { return m_management.altitude.getValue(); }

    ActionId_t getDenmActionID();

    // TODO: rename the arguments: these are 0.1 microdegrees, not deg
    void changeDenmLocationFields(double latReference_deg, double longReference_deg);

    /* Internals setters */
    // Units are milliseconds
    void setDenmRepetition(uint32_t repetitionDuration, uint32_t repetitionInterval) {
        m_internals.repetitionInterval = repetitionInterval;
        m_internals.repetitionDuration = repetitionDuration;
    }

    void setDenmRepetitionInterval(uint32_t repetitionInterval) { m_internals.repetitionInterval = repetitionInterval; }
    void setDenmRepetitionDuration(uint32_t repetitionDuration) { m_internals.repetitionDuration = repetitionDuration; }

    int setValidityDuration(long validityDuration_s);

    /* Internal getters */
    uint32_t getDenmRepetitionDuration() { return m_internals.repetitionDuration; }
    uint32_t getDenmRepetitionInterval() { return m_internals.repetitionInterval; }

    /* Container setters */
    void setDenmMgmtData_asn_types(denDataManagement management) { m_management = management; }
    void setDenmSituationData_asn_types(denDataSituation situation) { m_situation.setData(situation); }
    void setDenmLocationData_asn_types(denDataLocation location) { m_location.setData(location); }
    void setDenmAlacarteData_asn_types(denDataAlacarte alacarte) { m_alacarte.setData(alacarte); }

    /* Object integrity check */
    bool isDenDataRight();

    void denDataFree();
private:
    INTEGER_t asnTimeConvert(long time);

    denDataInternals m_internals;
    denDataHeader m_header;
    denDataManagement m_management;

    DENDataItem<denDataSituation> m_situation;

    DENDataItem<denDataLocation> m_location;

    DENDataItem<denDataAlacarte> m_alacarte;
};
#endif // DENDATA_H
