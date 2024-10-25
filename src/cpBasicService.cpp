#include "cpBasicService.h"
#include "gpsc.h"
#include "asn_utils.h"
#include "utils.h"
#include "PHpoints.h"
#include "LDMmap.h"
#include <future>
#include <chrono>
#include <ctime>
#include <iostream>
#include <cfloat>
#include <cmath>
#include <sys/sysinfo.h>
#include <netinet/ether.h>
#include <algorithm>
#include <arpa/inet.h>


using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

#define SCHEDULE(msecs,fcn) \
  (void) std::async(std::launch::async, [&] { \
    std::this_thread::sleep_for(std::chrono::milliseconds(msecs)); \
    fcn(); \
  });

CPBasicService::CPBasicService()
{
    m_station_id = ULONG_MAX;
    m_stationtype = LONG_MAX;
    m_btp = NULL;
    m_LDM = NULL;
    m_real_time=false;

    // Setting a default value of m_T_CheckCpmGen_ms equal to 100 ms (i.e. T_GenCpmMin_ms)
    m_T_CheckCpmGen_ms=T_GenCpmMin_ms;

    m_prev_heading=-1;
    m_prev_speed=-1;
    m_prev_distance=-1;

    m_T_GenCpm_ms=T_GenCpmMax_ms;

    lastCpmGen=-1;
    lastCpmGenLowFrequency=-1;
    lastCpmGenSpecialVehicle=-1;

    m_T_LastSensorInfoContainer = -1;

    m_N_GenCpmMax=1000;
    m_N_GenCpm=100;

    m_vehicle=true;
    m_redundancy_mitigation = true;

    m_cpm_sent=0;

    // The log file is disabled by default
    m_log_filename="cps_dis";

    // CAM print log file - this is currently unused and kept here just for future reference
    //m_log_cam_sent="dis";

    m_own_private_IP="0.0.0.0";
    m_own_public_IP="0.0.0.0";

    m_check_faulty_acceleration = false;
    m_speed_triggering = true;

    m_acceleration_threshold = 17.0;
    m_verbose = false;
}

void
CPBasicService::setStationID(unsigned long fixed_stationid)
{
    m_station_id=fixed_stationid;
    m_btp->setStationID(fixed_stationid);
}

void
CPBasicService::setStationType(long fixed_stationtype)
{
    m_stationtype=fixed_stationtype;
    m_btp->setStationType(fixed_stationtype);
}

void
CPBasicService::setStationProperties(unsigned long fixed_stationid,long fixed_stationtype)
{
    m_station_id=fixed_stationid;
    m_stationtype=fixed_stationtype;
    m_btp->setStationProperties(fixed_stationid,fixed_stationtype);
}


void
CPBasicService::initDissemination() {
    // Set the termination condition to false
    m_terminateFlag=false;
    FILE* f_out=nullptr;

    // Error check
    if (m_btp == nullptr) {
        fprintf(stderr, "Error: no BTP object has been set. The CAM dissemination will not start.\n");
        return;
    }
    //generateAndEncodeCPM();

    // Create a new timer to periodically check the CAM conditions, according to the standard
    struct pollfd pollfddata;
    int clockFd;
    // The last argument of timer_fd_create should be in microseconds
    if(timer_fd_create(pollfddata, clockFd, m_T_CheckCpmGen_ms*1e3)<0) {
        std::cerr << "[ERROR] Fatal error! Cannot create timer for the CAM dissemination" << std::endl;
        terminateDissemination();
        return;
    }
    POLL_DEFINE_JUNK_VARIABLE();

    // If the print on log file is enabled, create the log file
    if(m_log_filename!="cps_dis" && m_log_filename!="") {
        char filename[strlen(m_log_filename.c_str())+1];
        snprintf(filename,sizeof(filename),"%s",m_log_filename.c_str());

        f_out=fopen(filename,"w");
    }

    while (m_terminateFlag == false)
    {
        if(poll(&pollfddata,1,0)>0) {
            POLL_CLEAR_EVENT(clockFd);
            std::string log_data;
            log_data = generateAndEncodeCPM();
            if(f_out!=nullptr) {
                fprintf(f_out,"%s",log_data.c_str());
            }
        }
    }
    if(m_log_filename!="dis" && m_log_filename!="") {
        char filename[strlen(m_log_filename.c_str())+1];
        snprintf(filename,sizeof(filename),"%s",m_log_filename.c_str());

        f_out=fopen(filename,"a");
        fclose(f_out);
    }

    close(clockFd);
}

std::string
CPBasicService::generateAndEncodeCPM()
{
    VDPGPSClient::CPM_mandatory_data_t cpm_mandatory_data;

    BTPDataRequest_t dataRequest = {};

    int64_t now = computeTimestampUInt64 () / NANO_TO_MILLI;

    std::string encode_result;

    long numberOfPOs = 0;

    m_T_GenCpm_ms=now-lastCpmGen;

    /* Collect data for mandatory containers */
    auto cpm = asn1cpp::makeSeq (CollectivePerceptionMessage);

    if (bool (cpm) == false)
    {
        std::cerr <<"Warning: unable to encode CPM." << std::endl;
        std::string retdata = "\n[LOG] CPM generation event -- Sent = FALSE\n";
        retdata += "[ERROR] Unable to encode CPM\n";
        return  retdata;
    }

    //Schedule new CPM
    std::string data="";
    std::string sent="FALSE";

    cpm_mandatory_data = m_vdp->getCPMMandatoryData ();

    auto CPMcontainers = asn1cpp::makeSeq (WrappedCpmContainers);
    auto POsContainer = asn1cpp::makeSeq (PerceivedObjectContainer);
    auto CPM_POs = asn1cpp::makeSeq (PerceivedObjects);

    if (m_LDM != nullptr)
    {
        std::vector<ldmmap::LDMMap::returnedVehicleData_t> LDM_POs;
        if (m_LDM->getAllPOs (LDM_POs) == ldmmap::LDMMap::LDMMAP_OK) // If there are any POs in the LDM
        {
            /* Fill Perceived Object Container as detailed in ETSI TS 103 324, Section 7.1.8 */
            std::vector<ldmmap::LDMMap::returnedVehicleData_t>::iterator it;
            for (it = LDM_POs.begin (); it != LDM_POs.end (); it++)
            {
                if (it->vehData.perceivedBy != (uint64_t) m_station_id)
                {
                    continue;
                }
                if ((computeTimestampUInt64()/NANO_TO_MICRO - it->vehData.timestamp_us)/1000 > 2000)
                {
                    //std::cout << "Vehicle " << it->vehData.stationID << " is too old to be in the LDM -- > " << (computeTimestampUInt64()/NANO_TO_MICRO - it->vehData.timestamp_us)/1000
                    //<< " ms old" << std::endl;
                    // remove it from m_lastCPM
                    m_lastCPM.erase(it->vehData.stationID);
                    continue;
                }

                data += "[OBJECT] ID="+std::to_string(it->vehData.stationID)+"\n";
                //auto check = checkCPMconditions (it);
                auto check = checkCPMconditions(it, cpm_mandatory_data);
                data += check.second;
                if (!check.first && m_redundancy_mitigation){
//                    m_lastCPM[it->vehData.stationID].latest_lat = it->vehData.lat;
//                    m_lastCPM[it->vehData.stationID].latest_lon = it->vehData.lon;
//                    m_lastCPM[it->vehData.stationID].latest_speed_ms = it->vehData.speed_ms;
//                    m_lastCPM[it->vehData.stationID].latest_timestamp_us = computeTimestampUInt64()/NANO_TO_MICRO;
                    continue;
                }
                else
                {
                    auto PO = asn1cpp::makeSeq (PerceivedObject);
                    asn1cpp::setField (PO->objectId, it->vehData.stationID);
                    long timeOfMeasurement =
                            (computeTimestampUInt64()/NANO_TO_MICRO - it->vehData.timestamp_us) /
                            1000; // time of measuremente in ms
                    if (timeOfMeasurement > 1500)
                        timeOfMeasurement = 1500;
                    asn1cpp::setField (PO->measurementDeltaTime, timeOfMeasurement);
                    asn1cpp::setField (PO->position.xCoordinate.value,
                                       it->vehData.xDistance);
                    asn1cpp::setField (PO->position.xCoordinate.confidence,
                                       CoordinateConfidence_unavailable);
                    asn1cpp::setField (PO->position.yCoordinate.value,
                                       it->vehData.yDistance);
                    asn1cpp::setField (PO->position.yCoordinate.confidence,
                                       CoordinateConfidence_unavailable);

                    auto velocity = asn1cpp::makeSeq (Velocity3dWithConfidence);
                    asn1cpp::setField (velocity->present,
                                       Velocity3dWithConfidence_PR_cartesianVelocity);
                    auto cartesianVelocity = asn1cpp::makeSeq (VelocityCartesian);
                    asn1cpp::setField (cartesianVelocity->xVelocity.value,
                                       it->vehData.xSpeed);
                    asn1cpp::setField (cartesianVelocity->xVelocity.confidence,
                                       SpeedConfidence_unavailable);
                    asn1cpp::setField (cartesianVelocity->yVelocity.value,
                                       it->vehData.ySpeed);
                    asn1cpp::setField (cartesianVelocity->yVelocity.confidence,
                                       SpeedConfidence_unavailable);
                    asn1cpp::setField (velocity->choice.cartesianVelocity, cartesianVelocity);
                    asn1cpp::setField (PO->velocity, velocity);

                    auto acceleration = asn1cpp::makeSeq (Acceleration3dWithConfidence);
                    asn1cpp::setField (acceleration->present,
                                       Acceleration3dWithConfidence_PR_cartesianAcceleration);
                    auto cartesianAcceleration = asn1cpp::makeSeq (AccelerationCartesian);
                    // We skip acceleration for now (we don't have it from the GPS client)
                    /*
                    asn1cpp::setField (cartesianAcceleration->xAcceleration.value,
                                       it->vehData.xAcc.getData ());
                    asn1cpp::setField (cartesianAcceleration->xAcceleration.confidence,
                                       AccelerationConfidence_unavailable);
                    asn1cpp::setField (cartesianAcceleration->yAcceleration.value,
                                       it->vehData.yAcc.getData ());
                    asn1cpp::setField (cartesianAcceleration->yAcceleration.confidence,
                                       AccelerationConfidence_unavailable);
                                       */
                    asn1cpp::setField (acceleration->choice.cartesianAcceleration,
                                       cartesianAcceleration);
                    asn1cpp::setField (PO->acceleration, acceleration);

                    //Only z angle
                    auto angle = asn1cpp::makeSeq (EulerAnglesWithConfidence);
                    if ((it->vehData.heading*DECI) < CartesianAngleValue_unavailable &&
                        (it->vehData.heading*DECI) > 0)
                        asn1cpp::setField (angle->zAngle.value, (it->vehData.heading*DECI));
                    else
                        asn1cpp::setField (angle->zAngle.value, CartesianAngleValue_unavailable);
                    asn1cpp::setField (angle->zAngle.confidence, AngleConfidence_unavailable);
                    asn1cpp::setField (PO->angles, angle);
                    auto OD1 = asn1cpp::makeSeq (ObjectDimension);
                    if (it->vehData.vehicleLength.getData () < 255 &&
                        it->vehData.vehicleLength.getData () > 0)
                        asn1cpp::setField (OD1->value, it->vehData.vehicleLength.getData ());
                    else
                        asn1cpp::setField (OD1->value, ObjectDimensionValue_unavailable); //usual value for SUMO vehicles
                    asn1cpp::setField (OD1->confidence, ObjectDimensionConfidence_unavailable);
                    asn1cpp::setField (PO->objectDimensionX, OD1);
                    auto OD2 = asn1cpp::makeSeq (ObjectDimension);
                    if (it->vehData.vehicleWidth.getData () < 255 &&
                        it->vehData.vehicleWidth.getData () > 0)
                        asn1cpp::setField (OD2->value, it->vehData.vehicleWidth.getData ());
                    else
                        asn1cpp::setField (OD2->value, ObjectDimensionValue_unavailable); //usual value for SUMO vehicles
                    asn1cpp::setField (OD2->confidence, ObjectDimensionConfidence_unavailable);
                    asn1cpp::setField (PO->objectDimensionY, OD2);

                    /*Rest of optional fields handling left as future work*/

                    //Push Perceived Object to the container
                    asn1cpp::sequenceof::pushList (*CPM_POs, PO);
                    //Update the timestamp of the last time this PO was included in a CPM
                    //m_LDM->updateCPMincluded (it->vehData);
                    lastCPM_t lastCPM;
                    lastCPM.heading = it->vehData.heading;
                    lastCPM.lat = it->vehData.lat;
                    lastCPM.lon = it->vehData.lon;
                    lastCPM.speed_ms = it->vehData.speed_ms;
                    lastCPM.timestamp_us = computeTimestampUInt64()/NANO_TO_MICRO;
                    lastCPM.latest_lat = it->vehData.lat;
                    lastCPM.latest_lon = it->vehData.lon;
                    lastCPM.latest_speed_ms = it->vehData.speed_ms;
                    lastCPM.latest_timestamp_us = computeTimestampUInt64()/NANO_TO_MICRO;
                    m_lastCPM[it->vehData.stationID] = lastCPM;

                    //Increase number of POs for the numberOfPerceivedObjects field in cpmParameters container
                    numberOfPOs++;
                }
            }
            if (numberOfPOs != 0)
            {
                asn1cpp::setField (POsContainer->perceivedObjects, CPM_POs);
                asn1cpp::setField (POsContainer->numberOfPerceivedObjects, numberOfPOs);
            }
        }

        if (LDM_POs.empty())
        {
            // Clean the lastCPM map
            m_lastCPM.clear();
        }
    }

    /* Fill the header */
    asn1cpp::setField (cpm->header.messageId, MessageId_cpm);
    asn1cpp::setField (cpm->header.protocolVersion, 2);
    asn1cpp::setField (cpm->header.stationId, m_station_id);

    /*
     * Compute the generationDeltaTime, "computed as the time corresponding to the
     * time of the reference position in the CPM, considered as time of the CPM generation.
     * The value of the generationDeltaTime shall be wrapped to 65 536. This value shall be set as the
     * remainder of the corresponding value of TimestampIts divided by 65 536 as below:
     * generationDeltaTime = TimestampIts mod 65 536"
    */
    asn1cpp::setField (cpm->payload.managementContainer.referenceTime,
                       compute_timestampIts () % 65536);

    /* Fill the managementContainer */
    asn1cpp::setField (cpm->payload.managementContainer.referencePosition.altitude.altitudeValue,
                       cpm_mandatory_data.altitude.getValue ());
    asn1cpp::setField (
            cpm->payload.managementContainer.referencePosition.altitude.altitudeConfidence,
            cpm_mandatory_data.altitude.getConfidence ());
    asn1cpp::setField (cpm->payload.managementContainer.referencePosition.latitude,
                       cpm_mandatory_data.latitude);
    asn1cpp::setField (cpm->payload.managementContainer.referencePosition.longitude,
                       cpm_mandatory_data.longitude);
    asn1cpp::setField (cpm->payload.managementContainer.referencePosition.positionConfidenceEllipse
                               .semiMajorConfidence,
                       cpm_mandatory_data.posConfidenceEllipse.semiMajorConfidence);
    asn1cpp::setField (cpm->payload.managementContainer.referencePosition.positionConfidenceEllipse
                               .semiMinorConfidence,
                       cpm_mandatory_data.posConfidenceEllipse.semiMinorConfidence);
    asn1cpp::setField (cpm->payload.managementContainer.referencePosition.positionConfidenceEllipse
                               .semiMajorOrientation,
                       cpm_mandatory_data.posConfidenceEllipse.semiMajorOrientation);
    //TODO:  compute segmentInfo, get MTU and deal with needed segmentation

    /* Fill the originatingVehicleContainer */
    auto wrappedCpmContainer = asn1cpp::makeSeq (WrappedCpmContainer);
    asn1cpp::setField (wrappedCpmContainer->containerId, 1);
    auto originatingVehicleContainer = asn1cpp::makeSeq (OriginatingVehicleContainer);
    asn1cpp::setField (originatingVehicleContainer->orientationAngle.value,
                       cpm_mandatory_data.heading.getValue ());
    asn1cpp::setField (originatingVehicleContainer->orientationAngle.confidence,
                       cpm_mandatory_data.heading.getConfidence ());
    asn1cpp::setField (wrappedCpmContainer->containerData.present,
                       WrappedCpmContainer__containerData_PR_OriginatingVehicleContainer);
    asn1cpp::setField (wrappedCpmContainer->containerData.choice.OriginatingVehicleContainer,
                       originatingVehicleContainer);
    asn1cpp::sequenceof::pushList (cpm->payload.cpmContainers, wrappedCpmContainer);

    /* Generate Sensor Information Container as detailed in ETSI TS 103 324, Section 6.1.2.2 */
    if (now - m_T_LastSensorInfoContainer >= m_T_AddSensorInformation)
    {
        auto CPMcontainer = asn1cpp::makeSeq (WrappedCpmContainer);
        asn1cpp::setField (CPMcontainer->containerId, 3);
        auto sensorInfoContainer = asn1cpp::makeSeq (SensorInformationContainer);

        //For now we only consider one sensor
        //We assume sensor fusion or aggregation of 50m sensing range from the vehicle front bumper
        auto sensorInfo = asn1cpp::makeSeq (SensorInformation);
        asn1cpp::setField (sensorInfo->sensorId, 2);
        asn1cpp::setField (sensorInfo->sensorType, SensorType_localAggregation);
        asn1cpp::setField (sensorInfo->shadowingApplies, true);
        auto detectionArea = asn1cpp::makeSeq (Shape);
        asn1cpp::setField (detectionArea->present, Shape_PR_circular);
        auto circularArea = asn1cpp::makeSeq (CircularShape);
        auto refPos = asn1cpp::makeSeq (CartesianPosition3d);
        asn1cpp::setField (refPos->xCoordinate, 0);
        asn1cpp::setField (refPos->yCoordinate, 0);
        asn1cpp::setField (circularArea->shapeReferencePoint, refPos);
        asn1cpp::setField (circularArea->radius, 50);
        asn1cpp::setField (detectionArea->choice.circular, circularArea);
        asn1cpp::setField (sensorInfo->perceptionRegionShape, detectionArea);

        asn1cpp::sequenceof::pushList (*sensorInfoContainer, sensorInfo);

        asn1cpp::setField (CPMcontainer->containerData.present,
                           WrappedCpmContainer__containerData_PR_SensorInformationContainer);
        asn1cpp::setField (CPMcontainer->containerData.choice.SensorInformationContainer,
                           sensorInfoContainer);
        asn1cpp::sequenceof::pushList (cpm->payload.cpmContainers, CPMcontainer);
        m_T_LastSensorInfoContainer = now;
        data += "  CPM sent due to sensor information container\n";
    }
    else
    {
        //If no sensorInformationContainer and no perceivedObjectsContainer
        if (numberOfPOs == 0){
            //No CPM is generated in the current cycle
            long int time=duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
            std::string retdata = "\n[LOG] CPM generation event -- Sent = FALSE -- Timestamp = " + std::to_string(time)
                    + " -- ETSI_timestamp=" + std::to_string(compute_timestampIts () % 65536)
                    + " -- Time since last CPM = " + std::to_string(m_T_GenCpm_ms) + " ms\n";
            // retdata += data + "No perceived objects and no sensor information container\n";
            return retdata;
        }
    }

    if (numberOfPOs != 0)
    {
        auto CPMcontainer = asn1cpp::makeSeq (WrappedCpmContainer);
        asn1cpp::setField (CPMcontainer->containerId, 5);
        asn1cpp::setField (CPMcontainer->containerData.present,
                           WrappedCpmContainer__containerData_PR_PerceivedObjectContainer);
        asn1cpp::setField (CPMcontainer->containerData.choice.PerceivedObjectContainer,
                           POsContainer);
        asn1cpp::sequenceof::pushList(cpm->payload.cpmContainers,CPMcontainer);
    }

    // TODO: Support for Perception Region information from LDM (to be implemented in both SUMOensor and CARLAsensor)

    encode_result = asn1cpp::uper::encode(cpm);
    if(encode_result.size()<1)
    {
        std::cerr << "Warning: unable to encode CPM." << std::endl;
        std::string retdata = "\n[LOG] CPM generation event -- Sent = FALSE\n";
        retdata += data + "[ERROR] Unable to encode CPM\n";
        return retdata;
    }

    dataRequest.BTPType = BTP_B; //!< BTP-B
    dataRequest.destPort = CP_PORT;
    dataRequest.destPInfo = 0;
    dataRequest.GNType = TSB;
    dataRequest.GNCommProfile = UNSPECIFIED;
    dataRequest.GNRepInt =0;
    dataRequest.GNMaxRepInt=0;
    dataRequest.GNMaxLife = 1;
    dataRequest.GNMaxHL = 1;
    dataRequest.GNTraClass = 0x02; // Store carry foward: no - Channel offload: no - Traffic Class ID: 2
    dataRequest.lenght = encode_result.size();
    /* Create the packet and the BTP header */
    packetBuffer pktbuf(encode_result.c_str(),static_cast<unsigned int>(encode_result.size()));
    dataRequest.data = pktbuf;
    m_btp->sendBTP(dataRequest);

    m_cpm_sent++;

    // Store the time in which the last CPM (i.e. this one) has been generated and successfully sent
    m_T_GenCpm_ms=now-lastCpmGen;
    lastCpmGen = now;
    long int time=duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    std::string retdata = "\n[LOG] CPM generation event -- Sent = TRUE -- Timestamp=" + std::to_string(time)
            + " -- ETSI_timestamp=" + std::to_string(compute_timestampIts () % 65536)
            + " -- Time since last CPM=" + std::to_string(m_T_GenCpm_ms) + " ms\n";
    retdata += data;
    return retdata;
}

uint64_t
CPBasicService::terminateDissemination()
{
    if(m_terminateFlag==false) {
        m_terminateFlag=true;
    }
    return m_cpm_sent;
}
int64_t
CPBasicService::computeTimestampUInt64()
{
    int64_t int_tstamp=0;

    struct timespec tv;

    clock_gettime (CLOCK_MONOTONIC, &tv);

    int_tstamp=tv.tv_sec*1e9+tv.tv_nsec;

    return int_tstamp;
}

std::pair<bool, std::string>
CPBasicService::checkCPMconditions(std::vector<ldmmap::LDMMap::returnedVehicleData_t>::iterator PO_data, VDPGPSClient::CPM_mandatory_data_t ego_data) {

    bool condition_verified=false;
    std::string motivation="None";
    std::string data="";
    std::string sent="FALSE";
    std::string data_head="";
    std::string data_pos="";
    std::string data_speed="";
    std::string data_time="";

    /*Perceived Object Container Inclusion Management as mandated by TS 103 324 Section 6.1.2.3*/
    /* 1.a The object has first been detected by the perception system after the last CPM generation event.*/
    if (m_lastCPM.find(PO_data->vehData.stationID) == m_lastCPM.end())
    {
        motivation = "First detection";
        data = "  [FIRST DETECTION]\n";
        return std::make_pair(true, data);
    }

    lastCPM_t previousCPM = m_lastCPM[PO_data->vehData.stationID];
    PO_data->vehData.lon = PO_data->kf->get_state().lon;
    PO_data->vehData.lat = PO_data->kf->get_state().lat;
    /* 1.b The Euclidian absolute distance between the current estimated position of the reference point of the
     * object and the estimated position of the reference point of this object lastly included in a CPM exceeds
     * 4 m. */
    double dist = haversineDist(previousCPM.lat,previousCPM.lon,PO_data->vehData.lat,PO_data->vehData.lon);
    data_pos="  [DISTANCE] PrevLat="+std::to_string(previousCPM.lat)+" PrevLon="+std::to_string(previousCPM.lon)+" CurrLat="+std::to_string(PO_data->vehData.lat)+" CurrLon="+std::to_string(PO_data->vehData.lon)+" PosDiff="+std::to_string(dist)+"\n";
    if(!condition_verified && dist > 4.0)
    {
        //std::cout << "[CP service] Distance check passed: " << dist << " m since last CPM"<< std::endl;
        condition_verified = true;
        if (motivation == "None"){
            motivation = "Distance";
        }
    }

    // -----------------------------------------------------TESTING KALMAN FILTER-----------------------------------------------------//
    double dist_latest, kf_xSpeed, kf_ySpeed, ego_xSpeed, ego_ySpeed, ego_heading;
    dist_latest = haversineDist(previousCPM.latest_lat,previousCPM.latest_lon,PO_data->vehData.lat,PO_data->vehData.lon);

    kf_xSpeed = PO_data->kf->get_state().vx;
    kf_ySpeed = PO_data->kf->get_state().vy;
    //Compute absolute speed x and y from ego_data speed and heading and PO_data x and y components
    ego_heading = 90 - (double) (ego_data.heading.getValue()) / DECI;
    if (ego_heading < 0){
        ego_heading += 360;
    }
    ego_xSpeed = ego_data.speed.getValue() * cos(ego_heading * M_PI / 180);
    ego_ySpeed = ego_data.speed.getValue() * sin(ego_heading * M_PI / 180);
    //Compute absolute speed
    PO_data->vehData.xSpeed =  (long) (kf_xSpeed * CENTI + ego_xSpeed);
    PO_data->vehData.ySpeed =  (long) (kf_ySpeed * CENTI + ego_ySpeed);
    PO_data->vehData.speed_ms = sqrt(pow((double) PO_data->vehData.xSpeed, 2) + pow((double) PO_data->vehData.ySpeed, 2)) / 100;

    //print all the data
    //std::cout << "[OBJECT "<< PO_data->vehData.stationID << "] KF xSpeed: " << kf_xSpeed << " KF ySpeed: " << kf_ySpeed << " Ego xSpeed: " << ego_xSpeed << " Ego ySpeed: " << ego_ySpeed << " Computed xSpeed: " << PO_data->vehData.xSpeed << " Computed ySpeed: " << PO_data->vehData.ySpeed << std::endl;

    // -----------------------------------------------------TESTING KALMAN FILTER-----------------------------------------------------//

    /* 1.c The difference between the current estimated absolute speed of the reference point of the object and the
     * estimated absolute speed of the reference point of this object lastly included in a CPM exceeds 0,5 m/s. */
    double speed_diff = abs (PO_data->vehData.speed_ms - previousCPM.speed_ms);

    if (m_check_faulty_acceleration)
    {
        // Check what is the acceleration from the speed diff
        double time_diff_ms = computeTimestampUInt64()/(MICRO) - previousCPM.timestamp_us/MILLI;
        double acceleration = speed_diff / time_diff_ms;
        acceleration = acceleration * 1000; // Convert to m/s^2
        if (acceleration > m_acceleration_threshold)
        {
            speed_diff = 0;
            if(m_verbose)
                std::cout << "[CP service] Object " << PO_data->vehData.stationID
                          << " Faulty acceleration detected: " << acceleration << " m/s^2 since last CPM - Threshold " << m_acceleration_threshold << " m/s^2 "<<  std::endl;
        }

    }

    data_speed="  [SPEED] SpeedUnavailable="+std::to_string((float)SpeedValue_unavailable)
            +" PrevSpeed="+std::to_string(previousCPM.speed_ms)
            +" CurrSpeed="+std::to_string(PO_data->vehData.speed_ms) + " (LatestLat=" + std::to_string(previousCPM.latest_lat) + " LatestLon=" + std::to_string(previousCPM.latest_lon) + " DistFromLatest=" + std::to_string(dist_latest) + " LatestTimestamp=" + std::to_string(previousCPM.latest_timestamp_us/1000) + ")"
            +" SpeedDiff="+std::to_string(speed_diff)+"\n";
    if(!condition_verified && speed_diff > 0.5 && m_speed_triggering)
    {
        //std::cout << "[CP service] Speed check passed: " << speed_diff << " m/s since last CPM"<< std::endl;
        condition_verified = true;
        if (motivation == "None"){
            motivation = "Speed";
        }
    }

    /* 1.d The difference between the orientation of the vector of the current estimated absolute velocity of the
     * reference point of the object and the estimated orientation of the vector of the absolute velocity of the
     * reference point of this object lastly included in a CPM exceeds 4 degrees. */
    double heading_diff = abs (PO_data->vehData.heading - previousCPM.heading);
    heading_diff += (heading_diff>180.0) ? -360.0 : (heading_diff<-180.0) ? 360.0 : 0.0;
    data_head="  [HEADING] HeadingUnavailable="+std::to_string((float)HeadingValue_unavailable/10)+" PrevHead="+std::to_string(previousCPM.heading)+" CurrHead="+std::to_string(PO_data->vehData.heading)+" HeadDiff="+std::to_string(heading_diff)+"\n";

    if(!condition_verified && abs(previousCPM.heading - PO_data->vehData.heading) > 4)
    {
        if(m_verbose)
            std::cout << "[CP service] Object " << PO_data->vehData.stationID << " Heading check passed: " << abs(previousCPM.heading - PO_data->vehData.heading) << " degrees since last CPM"<< std::endl;
        condition_verified = true;
        if (motivation == "None"){
            motivation = "Heading";
        }
    }

    /* 1.e The time elapsed since the last time the object was included in a CPM exceeds T_GenCpmMax. */
    uint64_t now = computeTimestampUInt64 () / NANO_TO_MILLI;
    uint64_t time_diff = now - previousCPM.timestamp_us/1000;
    data_time="  [TIME] Timestamp="+std::to_string(now)+" LastCPMSend="+std::to_string(previousCPM.timestamp_us/1000)+" TimeThreshold="+std::to_string(m_N_GenCpmMax)+" TimeDiff="+std::to_string(time_diff)+"\n";
    if(!condition_verified && time_diff > m_N_GenCpmMax)
    {
        //std::cout << "[CP service] Time check passed: " << time_diff << " ms since last CPM"<< std::endl;
        condition_verified = true;
        if (motivation == "None"){
            motivation = "Time";
        }
    }

    if (condition_verified) {
        sent = "TRUE";
    }
    data += "  Condition verified:" + sent + " \n  [MOTIVATION] " + motivation + "\n" + data_pos + data_speed + data_head + data_time;

    return std::make_pair(condition_verified, data);

}


