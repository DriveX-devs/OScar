#include "caBasicService.h"
#include "gpsc.h"
#include "asn_utils.h"
#include "utils.h"
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

// This macro allows the user to schedule the asynchronous execution of a function (fcn) after "msec" milliseconds
#define SCHEDULE(msecs,fcn) \
  (void) std::async(std::launch::async, [&] { \
    std::this_thread::sleep_for(std::chrono::milliseconds(msecs)); \
    fcn(); \
  });

// CABasicService constructor
CABasicService::CABasicService()
{
  m_station_id = ULONG_MAX;
  m_stationtype = LONG_MAX;
  // m_socket_tx_fd=-1;
  m_btp = nullptr;
  m_LDM = nullptr;

  // Setting a default value of m_T_CheckCamGen_ms equal to 100 ms (i.e. T_GenCamMin_ms)
  m_T_CheckCamGen_ms=T_GenCamMin_ms;
  
  // Setting a default value of m_T_GenCam_ms equal to 1000 ms (i.e. T_GenCamMax_ms)
  m_T_GenCam_ms=T_GenCamMax_ms;
  
  // Set to 3 as described by the ETSI EN 302 637-2 V1.3.1 standard
  m_N_GenCamMax=3;
  m_N_GenCam=0;
  
  // CAM generation interval for RSU ITS-Ss (default: 1 s)
  m_RSU_GenCam_ms=1000;

  // Initializing parameters
  m_prev_heading=-1;
  m_prev_speed=-1;
  m_prev_pos=std::pair<double,double>(-DBL_MAX,-DBL_MAX);

  lastCamGen=-1;
  lastCamGenLowFrequency=-1;
  lastCamGenSpecialVehicle=-1;

  m_vehicle=true;

  m_cam_sent=0;

  // All the optional containers are disabled by default
  // This is not yet supported in OScar
  // m_lowFreqContainerEnabled = false;
  // m_specialVehContainerEnabled = false;

  m_terminateFlag=false;
  
  // The log file is disabled by default
  m_log_filename="dis";
  
  // CAM print log file - this is currently unused and kept here just for future reference
  //m_log_cam_sent="dis";

  m_own_private_IP="0.0.0.0";
  m_own_public_IP="0.0.0.0";

  m_force_20Hz_freq=false;

  m_met_sup_ptr = nullptr;
}

// This function contains the code for generating a new CAM with all the standard containers
CABasicService_error_t
CABasicService::fillInCam(asn1cpp::Seq<CAM> &msgstruct, VDPGPSClient::CAM_mandatory_data_t &cam_mandatory_data) {
    CABasicService_error_t errval=CAM_NO_ERROR;

    if(bool(msgstruct) == false) {
        return CAM_ALLOC_ERROR;
    }

    /* Collect data for mandatory containers */

    /* Fill the header */
    asn1cpp::setField(msgstruct->header.messageId, FIX_CAMID);
    asn1cpp::setField(msgstruct->header.protocolVersion , 2);
    asn1cpp::setField(msgstruct->header.stationId, m_station_id);

    asn1cpp::setField(msgstruct->cam.generationDeltaTime, compute_timestampIts () % 65536);

    /* Fill the basicContainer's station type */
    asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.stationType, m_stationtype);
    if(m_vehicle == true) {
        cam_mandatory_data=m_vdp->getCAMMandatoryData();
        /* Debug print: leave commented when releasing for testing or using for a use case */
        /*int64_t after=get_timestamp_us(); */
        /*fprintf(stdout,"Proc_time: %.3lf\n",(after-before)/1000.0); */

        /* Fill the basicContainer */
        asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue, cam_mandatory_data.altitude.getValue ());
        asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence, cam_mandatory_data.altitude.getConfidence ());
        asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.latitude,cam_mandatory_data.latitude);
        asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.longitude,cam_mandatory_data.longitude);
        asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorAxisLength, cam_mandatory_data.posConfidenceEllipse.semiMajorConfidence);
        asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorAxisLength, cam_mandatory_data.posConfidenceEllipse.semiMinorConfidence);
        asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorAxisOrientation, cam_mandatory_data.posConfidenceEllipse.semiMajorOrientation);

        /* Fill the highFrequencyContainer */
        asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.present, HighFrequencyContainer_PR_basicVehicleContainerHighFrequency);
        asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue, cam_mandatory_data.heading.getValue ());
        asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingConfidence, cam_mandatory_data.heading.getConfidence ());
        asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue, cam_mandatory_data.speed.getValue ());
        asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedConfidence, cam_mandatory_data.speed.getConfidence ());
        asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection, cam_mandatory_data.driveDirection);
        asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue, cam_mandatory_data.VehicleLength.getValue());
        asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication, cam_mandatory_data.VehicleLength.getConfidence());
        asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth, cam_mandatory_data.VehicleWidth);
        asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.value, cam_mandatory_data.longAcceleration.getValue ());
        asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.confidence, cam_mandatory_data.longAcceleration.getConfidence ());
        asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureValue, cam_mandatory_data.curvature.getValue ());
        asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureConfidence, cam_mandatory_data.curvature.getConfidence ());
        asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvatureCalculationMode, cam_mandatory_data.curvature_calculation_mode);
        asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateValue, cam_mandatory_data.yawRate.getValue ());
        asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateConfidence, cam_mandatory_data.yawRate.getConfidence ());

        /* Store all the "previous" values used in checkCamConditions() */
        m_prev_pos=m_vdp->getCurrentPositionDbl();
        m_prev_speed=m_vdp->getSpeedValueDbl();
        m_prev_heading=m_vdp->getHeadingValue().getValue();

    } else {
        /* Fill the basicContainer */
        /* There is still no full RSU support in this release */
        asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence,AltitudeConfidence_unavailable);
        asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue,AltitudeValue_unavailable);
        asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.latitude,Latitude_unavailable);
        asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.longitude,Longitude_unavailable);
        asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorAxisLength,SemiAxisLength_unavailable);
        asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorAxisLength,SemiAxisLength_unavailable);
        asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorAxisOrientation,HeadingValue_unavailable);
        /* Fill the highFrequencyContainer */
        /* auto RSUContainerHighFreq = asn1cpp::makeSeq(RSUContainerHighFrequency); */

        /* High frequency RSU container */
        asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.present,HighFrequencyContainer_PR_rsuContainerHighFrequency);

        auto protectedComm = asn1cpp::makeSeq(ProtectedCommunicationZone);
        asn1cpp::setField(protectedComm->protectedZoneType,ProtectedZoneType_permanentCenDsrcTolling);
        asn1cpp::setField(protectedComm->protectedZoneLatitude,Latitude_unavailable);
        asn1cpp::setField(protectedComm->protectedZoneLongitude,Longitude_unavailable);

        asn1cpp::sequenceof::pushList(msgstruct->cam.camParameters.highFrequencyContainer.choice.rsuContainerHighFrequency.protectedCommunicationZonesRSU,protectedComm);
    }

    return errval;
}

// Function to set the properties of a RSU station
void
CABasicService::setStationProperties(unsigned long fixed_stationid,long fixed_stationtype)
{
  m_station_id=fixed_stationid;
  m_stationtype=fixed_stationtype;
}

// Function to set a fixed position of a RSU station
void
CABasicService::setFixedPositionRSU(double latitude_deg, double longitude_deg)
{
  m_vehicle = false;
  m_RSUlon = longitude_deg;
  m_RSUlat = latitude_deg;

  // High frequency RSU container
  m_protectedCommunicationsZonesRSU = asn1cpp::makeSeq(RSUContainerHighFrequency);
  auto protectedComm = asn1cpp::makeSeq(ProtectedCommunicationZone);
  asn1cpp::setField(protectedComm->protectedZoneType,ProtectedZoneType_permanentCenDsrcTolling);
  asn1cpp::setField(protectedComm->protectedZoneLatitude,Latitude_unavailable);
  asn1cpp::setField(protectedComm->protectedZoneLongitude,Longitude_unavailable);
  asn1cpp::sequenceof::pushList(m_protectedCommunicationsZonesRSU->protectedCommunicationZonesRSU,protectedComm);
}

void
CABasicService::setStationID(unsigned long fixed_stationid)
{
  m_station_id=fixed_stationid;
}

void
CABasicService::setStationType(long fixed_stationtype)
{
  m_stationtype=fixed_stationtype;
}

// Function to start the CABasicService
void
CABasicService::startCamDissemination()
{
  // Set the termination condition to false
  m_terminateFlag=false;

  // Error check
  if(m_btp==nullptr) {
    fprintf(stderr,"Error: no BTP object has been set. The CAM dissemination will not start.\n");
    return;
  }

  // If the station is a vehicle start the vehicular CAMs dissemination
  if(m_vehicle) {
    SCHEDULE(0,initDissemination);
  }
  // If the station is an RSU start the fixed station CAMs dissemination - this is currently not fully supported (work in ongoing to fully support RSU CAMs)
  else {
    SCHEDULE(0,RSUDissemination);
  }

  while(m_terminateFlag==false); // Disseminate CAMs
}

// Function to start the CABasicService
void
CABasicService::startCamDissemination(int desync_ms)
{
  // Set the termination condition to false
  m_terminateFlag=false;

  // Error check
  if(m_btp==nullptr) {
    fprintf(stderr,"Error: no BTP object has been set. The CAM dissemination will not start.\n");
    return;
  }
  // If the station is a vehicle start the vehicular CAMs dissemination
  if(m_vehicle) {
    SCHEDULE(desync_ms,initDissemination);
  }
  // If the station is an RSU start the fixed station CAMs dissemination - this is currently not fully supported (work in ongoing to fully support RSU CAMs)
  else {
    SCHEDULE(desync_ms,RSUDissemination);
  }

  while(m_terminateFlag==false); // Disseminate CAMs
}

// Function to send the first vehicular CAM and start the following dissemination
void
CABasicService::initDissemination()
{
  generateAndEncodeCam();
  SCHEDULE(m_T_CheckCamGen_ms,CABasicService::checkCamConditions);
}

// Function to send the first fixed station CAM and start the following dissemination
void
CABasicService::RSUDissemination()
{
  generateAndEncodeCam();
  SCHEDULE(m_RSU_GenCam_ms,CABasicService::RSUDissemination);
}

// Function to check whether send a CAM or not and with which frequency
void
CABasicService::checkCamConditions()
{
  // Initializing
  int64_t now=computeTimestampUInt64()/NANO_TO_MILLI;
  CABasicService_error_t cam_error;
  bool condition_verified;
  static bool dyn_cond_verified = false;
  FILE* f_out = nullptr;
  bool first = true;
  double currHead = (double)m_vdp->getHeadingValue().getValue()/10;
  std::pair<double,double> currPos = m_vdp->getCurrentPositionDbl();
  double currSpeed = m_vdp->getSpeedValue().getValue();
  long int time_difference=0;
  double head_diff=-1;
  double pos_diff=-1;
  double speed_diff=-1;

  // Create a new timer to periodically check the CAM conditions, according to the standard
  struct pollfd pollfddata;
  int clockFd;

  if(!m_force_20Hz_freq) {
      // The last argument of timer_fd_create should be in microseconds
      if (timer_fd_create(pollfddata, clockFd, m_T_CheckCamGen_ms * 1e3) < 0) {
          std::cerr << "[ERROR] Fatal error! Cannot create timer for the CAM dissemination" << std::endl;
          terminateDissemination();
          return;
      }
  } else {
      // Force 20 Hz CAM transmission
      if (timer_fd_create(pollfddata, clockFd, 50 * 1e3) < 0) {
          std::cerr << "[ERROR] Fatal error! Cannot create timer for the CAM dissemination" << std::endl;
          terminateDissemination();
          return;
      }
  }

  POLL_DEFINE_JUNK_VARIABLE();

  // If the print on log file is enabled, create the log file
  if(m_log_filename!="dis" && m_log_filename!="") {
    char filename[strlen(m_log_filename.c_str())+1];
    snprintf(filename,sizeof(filename),"%s",m_log_filename.c_str());

    f_out=fopen(filename,"w");
  }

  // The dissemination goes on until it is interrupted
  while(m_terminateFlag == false) {
    if(poll(&pollfddata,1,-1)>0) {
      POLL_CLEAR_EVENT(clockFd);

      // Initializing
      condition_verified = false;
      std::string data_head = "";
      std::string data_pos = "";
      std::string data_speed = "";
      std::string data_time = "";
      std::string data_gen_time = "";

      // Parser logging
      std::string parser_log_data = "[PARSER]";
      std::string data_fix = "";
      std::string data_accs = "";
      std::string data_att = "";
      std::string data_longit_acc = "";
      std::string data_yaw_rate = "";
      std::string data_cog_ubx = "";
      std::string data_cog_nmea = "";
      std::string data_lat_ubx = "";
      std::string data_lat_nmea = "";
      std::string data_lon_ubx = "";
      std::string data_lon_nmea = "";
      std::string data_sog_ubx = "";
      std::string data_sog_nmea = "";

      // If no initial CAM has been triggered before checkCamConditions() has been called, throw an error
      if(m_prev_heading == -1 || m_prev_speed == -1 || m_prev_pos.first == -DBL_MAX || m_prev_pos.second == -DBL_MAX)
      {
        std::cerr << "Error. checkCamConditions() was called before sending any CAM and this is not allowed." << std::endl;
        terminateDissemination();
        close(clockFd);
        break;
        //throw std::runtime_error("Error. checkCamConditions() was called before sending any CAM and this is not allowed.");
      }

      // Retrieve the motion parameters from GNSS
/*      long int headCheck=m_vdp->getHeadingValue().getValue();
      double headCheckDbl=(float)m_vdp->getHeadingValueDbl();
      currPos = m_vdp->getCurrentPositionDbl();
      long int speedCheck=m_vdp->getSpeedValue().getValue();*/
        auto cam_conditions = m_vdp->getCAMConditionsData();
        long int headCheck = cam_conditions.headCheck;
        double headCheckDbl = cam_conditions.headCheckDbl;
        currPos = cam_conditions.currPos;
        long int speedCheck = cam_conditions.speedCheck;

      // Retrieve parser logging data
        if (m_vdp->getSerialParser() == true) {
            std::string fix_ubx = m_vdp->getParserFixModeUbx();
            std::string fix_nmea = m_vdp->getParserFixModeNmea();
            data_fix = " Fix[UBX]=" + fix_ubx + " " + "Fix[NMEA]=" + fix_nmea;

            std::tuple<double,double,double> accs = m_vdp->getParserAccelerations();
            data_accs =
                    " Acc_x=" + std::to_string(std::get<0>(accs)) +
                    " Acc_y=" + std::to_string(std::get<1>(accs)) +
                    " Acc_z=" + std::to_string(std::get<2>(accs));

            std::tuple<double,double,double> att = m_vdp->getParserAttitude();
            data_att =
                    " Roll="  + std::to_string(std::get<0>(att)) +
                    " Pitch=" + std::to_string(std::get<1>(att)) +
                    " Yaw="   + std::to_string(std::get<2>(att));

            double longit_acc = m_vdp->getParserLongitudinalAcceleration();
            data_longit_acc = " LongitudinalAcceleration=" + std::to_string(longit_acc);

            double yaw_rate = m_vdp->getParserYawRate();
            data_yaw_rate = " YawRate=" + std::to_string(yaw_rate);
        }

        /*
         * ETSI EN 302 637-2 V1.3.1 chap. 6.1.3 condition 1) (no DCC)
         * One of the following ITS-S dynamics related conditions is given:
        */

      if(m_force_20Hz_freq) {
          auto cam_result = generateAndEncodeCam();
          cam_error = cam_result.error;

          if(cam_error==CAM_NO_ERROR)
          {
              m_N_GenCam=0;
              condition_verified=true;
              dyn_cond_verified=true;
              data_gen_time = "\n[GENERATION_TIME] CAMTimestamp=" + std::to_string(cam_result.generationDeltaTime);
              data_gen_time += "\n[GENERATION_POSITION] CurrLat=" + (currPos.first == -DBL_MAX ? "Unavailable" : doubleToString(currPos.first))+" CurrLon="+(currPos.second == -DBL_MAX ? "Unavailable" : doubleToString(currPos.second));
          } else {
              std::cerr << "Cannot generate CAM. Error code: " << std::to_string(cam_error) << std::endl;
          }

          goto goto_print;
      }
      // std::cout << "Next CAM in: " << m_T_next_dcc << std::endl;
      /* 1a)
       * The absolute difference between the current heading of the originating
       * ITS-S and the heading included in the CAM previously transmitted by the
       * originating ITS-S exceeds 4°;
      */
      if(headCheck != HeadingValue_unavailable && abs(headCheckDbl) < abs(-DBL_MAX)) { // Check if the heading has an out of range value
        // Compute the heading difference with the previous CAM sent
        currHead=round(headCheckDbl*10)/10;
        head_diff = currHead - m_prev_heading/10;
        head_diff += (head_diff>180.0) ? -360.0 : (head_diff<-180.0) ? 360.0 : 0.0;

        // Create the data for the log print
        data_head="[HEADING] HeadingUnavailable="+std::to_string((float)HeadingValue_unavailable/10)+" PrevHead="+std::to_string(m_prev_heading/10)+" CurrHead="+std::to_string(currHead)+" HeadDiff="+std::to_string(head_diff)+"\n";
        if (m_vdp->getSerialParser() == true) {
            double cog_ubx = m_vdp->getParserCourseOverGroundUbx();
            data_cog_ubx = " Cog(UBX)=" + std::to_string(cog_ubx);

            double cog_nmea = m_vdp->getParserCourseOverGroundNmea();
            data_cog_nmea = " Cog(NMEA)=" + std::to_string(cog_nmea);
        }

          // If the heading difference with the previous CAM sent is more than 4°, then generate the CAM
        if (head_diff > 4.0 || head_diff < -4.0)
        {
          if (m_T_next_dcc == -1 || now - lastCamGen >= m_T_next_dcc)
          {
            auto cam_result = generateAndEncodeCam();
            cam_error = cam_result.error;

            if(cam_error == CAM_NO_ERROR)
            {
              m_N_GenCam=0;
              condition_verified=true;
              dyn_cond_verified=true;
              data_gen_time = "\n[GENERATION_TIME] CAMTimestamp=" + std::to_string(cam_result.generationDeltaTime);
            } else {
              std::cerr << "Cannot generate CAM. Error code: " << std::to_string(cam_error) << std::endl;
            }
          }
        }
      }
      // If the heading has an out of range value, set it to unavailable
      else {
        m_prev_heading=(float)HeadingValue_unavailable;
        currHead=(float)HeadingValue_unavailable;
        head_diff = currHead - m_prev_heading;

        // Create the data for the log print
        data_head="[HEADING] HeadingUnavailable="+std::to_string((float)HeadingValue_unavailable/10)+" PrevHead="+std::to_string(m_prev_heading/10)+" CurrHead="+std::to_string(currHead)+" HeadDiff="+std::to_string(head_diff)+"\n";
        if (m_vdp->getSerialParser() == true) {
            data_cog_ubx = " Cog(UBX)=Unavailable";
            data_cog_nmea = " Cog(NMEA)=Unavailable";
        }
      }


      /* 1b)
       * the distance between the current position of the originating ITS-S and
       * the position included in the CAM previously transmitted by the originating
       * ITS-S exceeds 4 m;
      */
      if(currPos.first != -DBL_MAX && currPos.second != -DBL_MAX) {
          pos_diff = haversineDist(currPos.first, currPos.second, m_prev_pos.first,m_prev_pos.second);
      } else {
          pos_diff = 0;
      }

      // Create the data for the log print
      data_pos="[DISTANCE] PrevLat="+doubleToString(m_prev_pos.first)+" PrevLon="+doubleToString(m_prev_pos.second)+" CurrLat="+(currPos.first == -DBL_MAX ? "Unavailable" : doubleToString(currPos.first))+" CurrLon="+(currPos.second == -DBL_MAX ? "Unavailable" : doubleToString(currPos.second))+" PosDiff="+doubleToString(pos_diff)+"\n";
      if (m_vdp->getSerialParser() == true) {
          std::pair<double,double> pos_ubx = m_vdp->getParserPositionUbx();
          std::pair<double,double> pos_nmea = m_vdp->getParserPositionNmea();
          data_lat_ubx = " Lat(UBX)=" + std::to_string(pos_ubx.first);
          data_lon_ubx = " Lon(UBX)=" + std::to_string(pos_ubx.second);
          data_lat_nmea = " Lat(NMEA)=" + std::to_string(pos_nmea.first);
          data_lon_nmea = " Lon(NMEA)=" + std::to_string(pos_nmea.second);
      }

      // If the position difference with the previous CAM sent is more than 4m, then generate the CAM
      if (!condition_verified && (pos_diff > 4.0 || pos_diff < -4.0))
      {
        if (m_T_next_dcc == -1 || now - lastCamGen >= m_T_next_dcc)
        {
          auto cam_result = generateAndEncodeCam();
          cam_error = cam_result.error;

          if(cam_error==CAM_NO_ERROR)
          {
            m_N_GenCam=0;
            condition_verified=true;
            dyn_cond_verified=true;
            data_gen_time = "\n[GENERATION_TIME] CAMTimestamp=" + std::to_string(cam_result.generationDeltaTime);
          } else {
            std::cerr << "Cannot generate CAM. Error code: " << std::to_string(cam_error) << std::endl;
          }
        }
      }


      /* 1c)
       * the absolute difference between the current speed of the originating ITS-S
       * and the speed included in the CAM previously transmitted by the originating
       * ITS-S exceeds 0,5 m/s.
      */
      if(speedCheck != SpeedValue_unavailable && abs(speedCheck) < 1000000) { // Check if the speed has an out of range value
        // Compute the speed difference with the previous CAM sent
        currSpeed=m_vdp->getSpeedValueDbl();
        speed_diff = currSpeed - m_prev_speed;

        // Create the data for the log print
        data_speed="[SPEED] SpeedUnavailable="+std::to_string((float)SpeedValue_unavailable)+" PrevSpeed="+std::to_string(m_prev_speed)+" CurrSpeed="+std::to_string(currSpeed)+" SpeedDiff="+std::to_string(speed_diff)+"\n";
        if (m_vdp->getSerialParser() == true) {
            double speed_ubx = m_vdp->getParserSpeedUbx();
            double speed_nmea = m_vdp->getParserSpeedNmea();
            data_sog_ubx = " Speed(UBX)=" + std::to_string(speed_ubx);
            data_sog_nmea = " Speed(NMEA)=" + std::to_string(speed_nmea);
        }


        // If the speed difference with the previous CAM sent is more than 0.5 m/s, then generate the CAM
        if (!condition_verified && (speed_diff > 0.5 || speed_diff < -0.5))
        {
          if (m_T_next_dcc == -1 || (now - lastCamGen >= m_T_next_dcc))
          {
            auto cam_result = generateAndEncodeCam();
            cam_error = cam_result.error;
            if(cam_error==CAM_NO_ERROR)
            {
              m_N_GenCam=0;
              condition_verified=true;
              dyn_cond_verified=true;
              data_gen_time = "\n[GENERATION_TIME] CAMTimestamp=" + std::to_string(cam_result.generationDeltaTime);
            } else {
              std::cerr << "Cannot generate CAM. Error code: " << std::to_string(cam_error) << std::endl;
            }
          }
        }
      }
      // If the speed has an out of range value, set it to unavailable
      else {
        m_prev_speed=SpeedValue_unavailable;
        currSpeed=SpeedValue_unavailable;
        speed_diff = currSpeed - m_prev_speed;

        // Create the data for the log print
        data_speed="[SPEED] SpeedUnavailable="+std::to_string((float)SpeedValue_unavailable)+" PrevSpeed="+std::to_string(m_prev_speed)+" CurrSpeed="+std::to_string(currSpeed)+" SpeedDiff="+std::to_string(speed_diff)+"\n";
        if (m_vdp->getSerialParser() == true) {
            data_sog_ubx = " Speed(UBX)=Unavailable";
            data_sog_nmea = " Speed(NMEA)=Unavailable";
        }
      }


      /* 2)
       * The time elapsed since the last CAM generation is equal to or greater than T_GenCam
      */
      now=computeTimestampUInt64()/NANO_TO_MILLI; // Compute the time difference with the previous CAM sent
      time_difference=now-lastCamGen;

      // Create the data for the log print
      data_time="[TIME] Timestamp="+std::to_string(now)+" LastCAMSend="+std::to_string(lastCamGen)+" NumThreshold="+std::to_string(m_N_GenCamMax)+" NumCAM="+std::to_string(m_N_GenCam)+" TimeThreshold="+std::to_string(m_T_GenCam_ms)+" TimeDiff="+std::to_string(time_difference)+" TimeNextCAM="+std::to_string(m_T_GenCam_ms - time_difference)+"\n";
      // If the time difference with the previous CAM sent is more than T_GenCam or the timeout expired for more than m_N_GenCamMax times, then generate the CAM
      if(!condition_verified && (abs(time_difference - m_T_GenCam_ms) <= 10 || (m_T_GenCam_ms - time_difference) <= 0))
      {
        if (m_T_next_dcc == -1 || now - lastCamGen >= m_T_next_dcc)
        {
          auto cam_result = generateAndEncodeCam();
          cam_error = cam_result.error;
          if(cam_error==CAM_NO_ERROR)
          {

            condition_verified=true;

            if(dyn_cond_verified==true)
            {
              m_N_GenCam++;
              if(m_N_GenCam>=m_N_GenCamMax)
              {
                m_N_GenCam=0;
                m_T_GenCam_ms=T_GenCamMax_ms;
                dyn_cond_verified=false;
              }
            }

            data_gen_time = "\n[GENERATION_TIME] CAMTimestamp=" + std::to_string(cam_result.generationDeltaTime);
          } else {
            std::cerr << "Cannot generate CAM. Error code: " << std::to_string(cam_error) << std::endl;
          }
        }
      }

      goto_print:

      // If the print on log file is enabled
      if(m_log_filename!="dis" && m_log_filename!="") {
        // If the log file has not been set to append mode, then set it to append mode
        if(first==true) {
          char filename[strlen(m_log_filename.c_str())+1];
          snprintf(filename,sizeof(filename),"%s",m_log_filename.c_str());

          f_out=fopen(filename,"a");
          first=false;
        }

        // Initializing
        long int time=duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

        std::string data="";
        std::string sent="false";
        std::string motivation;
        std::string joint="";
        int numConditions=0;
        motivation="";

        // Check the motivation of the CAM sent
        if(!m_force_20Hz_freq) {
            if (!condition_verified) {
                motivation = "none";
            } else {
                data = "[CAM] CAM sent\n";
                sent = "true";

                if (head_diff > 4.0 || head_diff < -4.0) {
                    motivation = "heading";
                    joint = joint + "H";
                    numConditions++;
                }

                if ((pos_diff > 4.0 || pos_diff < -4.0)) {
                    motivation = "position";
                    joint = joint + "P";
                    numConditions++;
                }

                if (speed_diff > 0.5 || speed_diff < -0.5) {
                    motivation = "speed";
                    joint = joint + "S";
                    numConditions++;
                }

                if (abs(time_difference - m_T_GenCam_ms) <= 10 || (m_T_GenCam_ms - time_difference) <= 0) {
                    motivation = "time";
                    joint = joint + "T";
                    numConditions++;
                }

                // When joint with a single other motivation, the joint motivation should not be considered
                if (numConditions > 1) {
                    motivation = "joint(" + joint + ")";
                    if (joint == "HT") {
                        motivation = "heading";
                    }
                    if (joint == "PT") {
                        motivation = "position";
                    }
                    if (joint == "ST") {
                        motivation = "speed";
                    }
                }

                if (condition_verified && motivation.empty()) {
                    motivation = "numPkt";
                }
            }
        } else {
            data = "[CAM] CAM sent\n";
            sent = "true";

            motivation = "20Hz_forced";
            numConditions++;
        }

        // Create the data for the log print
        data+="[LOG] Timestamp="+std::to_string(time)+" CAMSend="+sent+" Motivation="+motivation+" HeadDiff="+std::to_string(head_diff)+" PosDiff="+std::to_string(pos_diff)+" SpeedDiff="+std::to_string(speed_diff)+" TimeDiff="+std::to_string(time_difference)+"\n";
        data=data+data_head+data_pos+data_speed+data_time;
        if (m_vdp->getSerialParser() == false) data = data + data_gen_time + "\n";
        else {
            parser_log_data = parser_log_data + data_fix + data_cog_ubx + data_cog_nmea
                    + data_lat_ubx + data_lon_ubx + data_lat_nmea + data_lon_nmea
                    + data_sog_ubx + data_sog_nmea + data_accs + data_att + data_longit_acc
                    + data_yaw_rate;
            data = data + parser_log_data + data_gen_time + "\n\n";
        }

        // Print the data for the log
        fprintf(f_out,"%s", data.c_str());
      }

      // Debug print: leave commented when releasing for testing or using for a use case
      // std::cout << "Check for m_T_GenCam_ms:" << m_T_GenCam_ms << std::endl;

      // try {
      //   SCHEDULE(m_T_CheckCamGen_ms,CABasicService::checkCamConditions);
      //   //SCHEDULE(10,CABasicService::checkCamConditions);
      // } catch(const std::exception &excp) {
      //   std::cerr << "Error: cannot schedule any new CAM transmission. Reason: " << excp.what() << std::endl;
      //   terminateDissemination();
      // }

    }
  } // End of the dissemination cycle

  // If the print on log file is enabled, close the file before exit
  // This needs to be double-checked
  if(m_log_filename!="dis" && m_log_filename!="") {
    char filename[strlen(m_log_filename.c_str())+1];
    snprintf(filename,sizeof(filename),"%s",m_log_filename.c_str());

    f_out=fopen(filename,"a");
    fclose(f_out);
  }

  close(clockFd);
} // End of the checkCamConditions function

// Function to generate, encode and send the CAM
CAMGeneration_return_t
CABasicService::generateAndEncodeCam()
{
  // Debug print: leave commented when releasing for testing or using for a use case
  //int64_t before=get_timestamp_us();

  CAMGeneration_return_t retval;
  retval.error = CAM_NO_ERROR;

  VDPGPSClient::CAM_mandatory_data_t cam_mandatory_data;

  int64_t now;

  auto cam = asn1cpp::makeSeq(CAM);

  // Function for filling the CAM fields
    retval.error=fillInCam(cam,cam_mandatory_data);

    /* CAM print
     * Unused - kept here just for future reference (this comment will be removed in the final deployed version)
     *
    if(m_log_cam_sent!="dis" && m_log_cam_sent!="") {
      char filename2[strlen(m_log_cam_sent.c_str())+1];
      snprintf(filename2,sizeof(filename2),"%s",m_log_cam_sent.c_str());

      std::string test=asn1cpp::xer::encode(cam);
      f_out2=fopen(filename2,"a");
      fprintf(f_out2,"%s",test.c_str());
      fclose(f_out2);
    }
    */

    retval.generationDeltaTime=asn1cpp::getField(cam->cam.generationDeltaTime,int);

    // Encode and send the CAM
    std::string encode_result = asn1cpp::uper::encode(cam);
    /* In case of an encoding error, print some basic data which we just tried to encode into a CAM. This may help debugging the encoding issue */ \
    if(encode_result.size()<1)
    {
        std::cerr << "CAM encoding error." << std::endl;
        std::cerr << "Info: Lat: " << cam->cam.camParameters.basicContainer.referencePosition.latitude
          << " Lon: " << cam->cam.camParameters.basicContainer.referencePosition.longitude
          << " Heading: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue
          << " Speed: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue
          << " Altitude: " << cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue
          << std::endl;
        retval.error = CAM_ASN1_UPER_ENC_ERROR;
        return retval;
    }
    
    /* Initialize parameters */
    BTPDataRequest_t dataRequest = {};
    dataRequest.BTPType = BTP_B;
    dataRequest.destPort = CA_PORT;
    dataRequest.destPInfo = 0;
    dataRequest.GNType = TSB;
    dataRequest.GNCommProfile = UNSPECIFIED;
    dataRequest.GNRepInt =0;
    dataRequest.GNMaxRepInt=0;
    dataRequest.GNMaxLife = 1;
    dataRequest.GNMaxHL = 1;
    dataRequest.GNTraClass = 0x02;
    dataRequest.lenght = encode_result.size();
    /* Create the packet and the BTP header */
    packetBuffer pktbuf(encode_result.c_str(),static_cast<unsigned int>(encode_result.size()));
    dataRequest.data = pktbuf;
    std::tuple<GNDataConfirm_t, MessageId_t> status = m_btp->sendBTP(dataRequest, m_priority, MessageId_cam);
    GNDataConfirm_t dataConfirm = std::get<0>(status);
    MessageId_t message_id = std::get<1>(status);
    /* Update the CAM statistics */
    if(m_met_sup_ptr!=nullptr && dataConfirm == ACCEPTED) {
      if (message_id == MessageId_cam) m_cam_sent++;
      m_met_sup_ptr->signalSentPacket(message_id);
    }

    /* Store the time in which the last CAM (i.e. this one) has been generated and successfully sent */
    now=computeTimestampUInt64()/NANO_TO_MILLI;

    m_T_GenCam_ms=now-lastCamGen;

    /* Always avoid sending CAMs less often than every second (this may happen in case of issues with the GNSS device) */
    if(m_T_GenCam_ms>T_GenCamMax_ms) {
        m_T_GenCam_ms=T_GenCamMax_ms;
    }
    /* Save the time of the CAM sent */
    lastCamGen = now;

    return retval;
}

uint64_t
CABasicService::terminateDissemination()
{
  if(m_terminateFlag==false) {
    m_terminateFlag=true;
  }

  return m_cam_sent;
}

int64_t
CABasicService::computeTimestampUInt64()
{
  int64_t int_tstamp=0;

  struct timespec tv;

  clock_gettime (CLOCK_MONOTONIC, &tv);

  int_tstamp=tv.tv_sec*1e9+tv.tv_nsec;

  return int_tstamp;
}