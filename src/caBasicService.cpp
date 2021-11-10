#include "caBasicService.h"
#include "gpsc.h"
#include "asn_utils.h"
#include <future>
#include <chrono>
#include <iostream>
#include <cfloat>
#include <cmath>

#define SCHEDULE(msecs,fcn) \
  (void) std::async(std::launch::async, [&] { \
    std::this_thread::sleep_for(std::chrono::milliseconds(msecs)); \
    fcn(); \
  });

CABasicService::CABasicService()
{
  m_station_id = ULONG_MAX;
  m_stationtype = LONG_MAX;
  m_socket_tx_fd=-1;
//  m_btp = NULL;
  m_real_time=false;

  // Setting a default value of m_T_CheckCamGen_ms equal to 100 ms (i.e. T_GenCamMin_ms)
  m_T_CheckCamGen_ms=T_GenCamMin_ms;

  m_prev_heading=-1;
  m_prev_speed=-1;
  m_prev_pos=std::pair<double,double>(-DBL_MAX,-DBL_MAX);

  m_T_GenCam_ms=T_GenCamMax_ms;

  lastCamGen=-1;
  lastCamGenLowFrequency=-1;
  lastCamGenSpecialVehicle=-1;

  // Set to 3 as described by the ETSI EN 302 637-2 V1.3.1 standard
  m_N_GenCamMax=3;
  m_N_GenCam=0;

  m_vehicle=true;

  // CAM generation interval for RSU ITS-Ss (default: 1 s)
  m_RSU_GenCam_ms=1000;

  m_cam_sent=0;

  // All the optional containers are disabled by default
  m_lowFreqContainerEnabled = false;
  m_specialVehContainerEnabled = false;

  // m_refPositions.clear ();
}

void
CABasicService::setStationProperties(unsigned long fixed_stationid,long fixed_stationtype)
{
  m_station_id=fixed_stationid;
  m_stationtype=fixed_stationtype;
//  m_btp->setStationProperties(fixed_stationid,fixed_stationtype);
}

void
CABasicService::setFixedPositionRSU(double latitude_deg, double longitude_deg)
{
  m_vehicle = false;
  m_RSUlon = longitude_deg;
  m_RSUlat = latitude_deg;
  //High frequency RSU container
  m_protectedCommunicationsZonesRSU = asn1cpp::makeSeq(RSUContainerHighFrequency);
  auto protectedComm = asn1cpp::makeSeq(ProtectedCommunicationZone);
  asn1cpp::setField(protectedComm->protectedZoneType,ProtectedZoneType_permanentCenDsrcTolling);
  asn1cpp::setField(protectedComm->protectedZoneLatitude,Latitude_unavailable);
  asn1cpp::setField(protectedComm->protectedZoneLongitude,Longitude_unavailable);
  asn1cpp::sequenceof::pushList(m_protectedCommunicationsZonesRSU->protectedCommunicationZonesRSU,protectedComm);
//  m_btp->setFixedPositionRSU(latitude_deg,longitude_deg);
}

void
CABasicService::setStationID(unsigned long fixed_stationid)
{
  m_station_id=fixed_stationid;
//  m_btp->setStationID(fixed_stationid);
}

void
CABasicService::setStationType(long fixed_stationtype)
{
  m_stationtype=fixed_stationtype;
//  m_btp->setStationType(fixed_stationtype);
}

// void
// CABasicService::setSocketRx (Ptr<Socket> socket_rx)
// {
//   m_btp->setSocketRx(socket_rx);
//   m_btp->addCAMRxCallback (std::bind(&CABasicService::receiveCam,this,std::placeholders::_1,std::placeholders::_2));
// }

void
CABasicService::startCamDissemination()
{
  if(m_vehicle)
    {
      SCHEDULE(0,initDissemination);
    }
  else
    {
      SCHEDULE(0,RSUDissemination);
    }
}

void
CABasicService::startCamDissemination(int desync_ms)
{
  if(m_vehicle)
    {
      SCHEDULE(desync_ms,initDissemination);
    }
  else
    {
      SCHEDULE(desync_ms,RSUDissemination);
    }
}

// void
// CABasicService::receiveCam (BTPDataIndication_t dataIndication, Address from)
// {
//   Ptr<Packet> packet;
//   asn1cpp::Seq<CAM> decoded_cam;

//   uint8_t *buffer; //= new uint8_t[packet->GetSize ()];
//   buffer=(uint8_t *)malloc((dataIndication.data->GetSize ())*sizeof(uint8_t));
//   dataIndication.data->CopyData (buffer, dataIndication.data->GetSize ());
//   std::string packetContent((char *)buffer,(int) dataIndication.data->GetSize ());

//    Try to check if the received packet is really a CAM 
//   if (buffer[1]!=FIX_CAMID)
//     {
//       NS_LOG_ERROR("Warning: received a message which has messageID '"<<buffer[1]<<"' but '2' was expected.");
//       free(buffer);
//       return;
//     }

//   free(buffer);

//   /** Decoding **/
//   decoded_cam = asn1cpp::uper::decode(packetContent, CAM);

//   if(bool(decoded_cam)==false) {
//       NS_LOG_ERROR("Warning: unable to decode a received CAM.");
//       return;
//     }

//   m_CAReceiveCallback(decoded_cam,from);
// }

void
CABasicService::initDissemination()
{
  generateAndEncodeCam();
  SCHEDULE(m_T_CheckCamGen_ms,CABasicService::checkCamConditions);
//  m_event_camCheckConditions = Simulator::Schedule (MilliSeconds(m_T_CheckCamGen_ms), &CABasicService::checkCamConditions, this);
}

void
CABasicService::RSUDissemination()
{
  generateAndEncodeCam();
  SCHEDULE(m_RSU_GenCam_ms,CABasicService::RSUDissemination);
//  m_event_camRsuDissemination = Simulator::Schedule (MilliSeconds(m_RSU_GenCam_ms), &CABasicService::RSUDissemination, this);
}

void
CABasicService::checkCamConditions()
{
  int64_t now=computeTimestampUInt64 ()/NANO_TO_MILLI;
  CABasicService_error_t cam_error;
  bool condition_verified=false;
  static bool dyn_cond_verified=false;

  // If no initial CAM has been triggered before checkCamConditions() has been called, throw an error
  if(m_prev_heading==-1 || m_prev_speed==-1 || m_prev_pos.first==-DBL_MAX || m_prev_pos.second==-DBL_MAX)
    {
      throw std::runtime_error("Error. checkCamConditions() was called before sending any CAM and this is not allowed.");
    }
  /*
   * ETSI EN 302 637-2 V1.3.1 chap. 6.1.3 condition 1) (no DCC)
   * One of the following ITS-S dynamics related conditions is given:
  */

  /* 1a)
   * The absolute difference between the current heading of the originating
   * ITS-S and the heading included in the CAM previously transmitted by the
   * originating ITS-S exceeds 4°;
  */
  if(m_vdp->getHeadingValue ().getValue() !=HeadingValue_unavailable) {
    double head_diff = m_vdp->getHeadingValue ().getValue() - m_prev_heading;
    head_diff += (head_diff>180.0) ? -360.0 : (head_diff<-180.0) ? 360.0 : 0.0;
    if (head_diff > 4.0 || head_diff < -4.0)
      {
        cam_error=generateAndEncodeCam ();
        if(cam_error==CAM_NO_ERROR)
          {
            m_N_GenCam=0;
            condition_verified=true;
            dyn_cond_verified=true;
          } else {
            std::cerr << "Cannot generate CAM. Error code: " << std::to_string(cam_error) << std::endl;
          }
      }
    }

  /* 1b)
   * the distance between the current position of the originating ITS-S and
   * the position included in the CAM previously transmitted by the originating
   * ITS-S exceeds 4 m;
  */
  std::pair<long,long> currPos = m_vdp->getCurrentPosition();
  double pos_diff = haversineDist(currPos.first, currPos.second, m_prev_pos.first, m_prev_pos.second);
  if (!condition_verified && (pos_diff > 4.0 || pos_diff < -4.0))
    {
      cam_error=generateAndEncodeCam ();
      if(cam_error==CAM_NO_ERROR)
        {
          m_N_GenCam=0;
          condition_verified=true;
          dyn_cond_verified=true;
        } else {
          std::cerr << "Cannot generate CAM. Error code: " << std::to_string(cam_error) << std::endl;
        }
    }

  /* 1c)
   * he absolute difference between the current speed of the originating ITS-S
   * and the speed included in the CAM previously transmitted by the originating
   * ITS-S exceeds 0,5 m/s.
  */
  if(m_vdp->getSpeedValue ().getValue() !=SpeedValue_unavailable) {
    double speed_diff = m_vdp->getSpeedValue ().getValue() - m_prev_speed;
    if (!condition_verified && (speed_diff > 0.5 || speed_diff < -0.5))
      {
        cam_error=generateAndEncodeCam ();
        if(cam_error==CAM_NO_ERROR)
          {
            m_N_GenCam=0;
            condition_verified=true;
            dyn_cond_verified=true;
          } else {
            std::cerr << "Cannot generate CAM. Error code: " << std::to_string(cam_error) << std::endl;
          }
      }
  }

//  printf("Condition verified? %d\n",condition_verified);

  /* 2)
   * The time elapsed since the last CAM generation is equal to or greater than T_GenCam
  */
  if(!condition_verified && (now-lastCamGen>=m_T_GenCam_ms))
    {
       cam_error=generateAndEncodeCam ();
       if(cam_error==CAM_NO_ERROR)
         {

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
         } else {
           std::cerr << "Cannot generate CAM. Error code: " << std::to_string(cam_error) << std::endl;
         }
    }

//  printf("[INFO] m_T_GenCam_ms: %ld\n",m_T_GenCam_ms);
  SCHEDULE(m_T_CheckCamGen_ms,CABasicService::checkCamConditions);
//  m_event_camCheckConditions = Simulator::Schedule (MilliSeconds(m_T_CheckCamGen_ms), &CABasicService::checkCamConditions, this);
}

CABasicService_error_t
CABasicService::generateAndEncodeCam()
{
  VDPGPSClient::CAM_mandatory_data_t cam_mandatory_data;
  CABasicService_error_t errval=CAM_NO_ERROR;

  // Ptr<Packet> packet;

  // BTPDataRequest_t dataRequest = {};

  int64_t now;
  // int64_t now_centi;

  /* Collect data for mandatory containers */
  auto cam = asn1cpp::makeSeq(CAM);

  if(bool(cam)==false)
    {
      return CAM_ALLOC_ERROR;
    }

  /* Fill the header */
  asn1cpp::setField(cam->header.messageID, FIX_CAMID);
  asn1cpp::setField(cam->header.protocolVersion , protocolVersion_currentVersion);
  asn1cpp::setField(cam->header.stationID, m_station_id);

  /*
   * Compute the generationDeltaTime, "computed as the time corresponding to the
   * time of the reference position in the CAM, considered as time of the CAM generation.
   * The value of the generationDeltaTime shall be wrapped to 65 536. This value shall be set as the
   * remainder of the corresponding value of TimestampIts divided by 65 536 as below:
   * generationDeltaTime = TimestampIts mod 65 536"
  */
  asn1cpp::setField(cam->cam.generationDeltaTime, compute_timestampIts (m_real_time) % 65536);

  /* Fill the basicContainer's station type */
  asn1cpp::setField(cam->cam.camParameters.basicContainer.stationType, m_stationtype);
  if(m_vehicle==true)
    {

      cam_mandatory_data=m_vdp->getCAMMandatoryData();

      // printf("[INFO] %.7lf-%.7lf-%ld\n",cam_mandatory_data.latitude/1e7,cam_mandatory_data.longitude/1e7,m_station_id);

      /* Fill the basicContainer */
      asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue, cam_mandatory_data.altitude.getValue ());
      asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence, cam_mandatory_data.altitude.getConfidence ());
      asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.latitude,cam_mandatory_data.latitude);
      asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.longitude,cam_mandatory_data.longitude);
      asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence, cam_mandatory_data.posConfidenceEllipse.semiMajorConfidence);
      asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence, cam_mandatory_data.posConfidenceEllipse.semiMinorConfidence);
      asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation, cam_mandatory_data.posConfidenceEllipse.semiMajorOrientation);

      /* Fill the highFrequencyContainer */

      asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.present, HighFrequencyContainer_PR_basicVehicleContainerHighFrequency);
      asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue, cam_mandatory_data.heading.getValue ());
      asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingConfidence, cam_mandatory_data.heading.getConfidence ());
      asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue, cam_mandatory_data.speed.getValue ());
      asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedConfidence, cam_mandatory_data.speed.getConfidence ());
      asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection, cam_mandatory_data.driveDirection);
      asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue, cam_mandatory_data.VehicleLength.getValue());
      asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication, cam_mandatory_data.VehicleLength.getConfidence());
      asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth, cam_mandatory_data.VehicleWidth);
      asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue, cam_mandatory_data.longAcceleration.getValue ());
      asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationConfidence, cam_mandatory_data.longAcceleration.getConfidence ());
      asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureValue, cam_mandatory_data.curvature.getValue ());
      asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureConfidence, cam_mandatory_data.curvature.getConfidence ());
      asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvatureCalculationMode, cam_mandatory_data.curvature_calculation_mode);
      asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateValue, cam_mandatory_data.yawRate.getValue ());
      asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateConfidence, cam_mandatory_data.yawRate.getConfidence ());

      // Store all the "previous" values used in checkCamConditions()
      m_prev_pos=m_vdp->getCurrentPosition();
      m_prev_speed=m_vdp->getSpeedValue ().getValue();
      m_prev_heading=m_vdp->getHeadingValue ().getValue();
    }
 else
    {
      /* Fill the basicContainer */
      /* There is still no full RSU support in this release */
      asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence,AltitudeConfidence_unavailable);
      asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue,AltitudeValue_unavailable);
      asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.latitude,Latitude_unavailable);
      asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.longitude,Longitude_unavailable);
      asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence,SemiAxisLength_unavailable);
      asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence,SemiAxisLength_unavailable);
      asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation,HeadingValue_unavailable);
      /* Fill the highFrequencyContainer */
      //auto RSUContainerHighFreq = asn1cpp::makeSeq(RSUContainerHighFrequency);

      //High frequency RSU container

      asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.present,HighFrequencyContainer_PR_rsuContainerHighFrequency);

      //auto zones = asn1cpp::makeSeq(ProtectedCommunicationZonesRSU);
      //auto highfreq = asn1cpp::makeSeq(RSUContainerHighFrequency);

      auto protectedComm = asn1cpp::makeSeq(ProtectedCommunicationZone);
      asn1cpp::setField(protectedComm->protectedZoneType,ProtectedZoneType_permanentCenDsrcTolling);
      asn1cpp::setField(protectedComm->protectedZoneLatitude,Latitude_unavailable);
      asn1cpp::setField(protectedComm->protectedZoneLongitude,Longitude_unavailable);

      asn1cpp::sequenceof::pushList(cam->cam.camParameters.highFrequencyContainer.choice.rsuContainerHighFrequency.protectedCommunicationZonesRSU,protectedComm);

    }

  std::string encode_result = asn1cpp::uper::encode(cam);

  if(encode_result.size()<1)
  {
    std::cerr << "CAM encoding error." << m_stationtype << std::endl;
    return CAM_ASN1_UPER_ENC_ERROR;
  }

  // std::cout << "[INFO] Generating a new CAM:" << std::endl;
  // uint8_t *buf=(uint8_t *)encode_result.c_str();
  // for(int i=0;i<encode_result.size();i++) {
  //   printf("%02X",buf[i]);
  // }
  // std::cout << std::endl;

  // Send the CAM as a broadcast UDP packet
  struct sockaddr_in sendSockAddr;

  memset(&sendSockAddr,0,sizeof(sendSockAddr));
  sendSockAddr.sin_family=AF_INET;
  sendSockAddr.sin_port=htons(UDP_PACKET_DEST_PORT);
  sendSockAddr.sin_addr.s_addr=INADDR_BROADCAST;

  errno=0;
  if(sendto(m_socket_tx_fd,(uint8_t*) encode_result.c_str(),encode_result.size(),0,(struct sockaddr *)&sendSockAddr, sizeof(sendSockAddr))!=encode_result.size()) {
    std::cerr << "Cannot send CAM. Error details: " << strerror(errno) << std::endl;
  }

  m_cam_sent++;

  // Store the time in which the last CAM (i.e. this one) has been generated and successfully sent
  now=computeTimestampUInt64 ()/NANO_TO_MILLI;
  // now_centi = computeTimestampUInt64 ()/NANO_TO_CENTI; //Time in centiseconds(now[ms]/10->centiseconds) for Reference Position
  m_T_GenCam_ms=now-lastCamGen;

  printf("[TBR] m_T_GenCam_ms: %ld\n",m_T_GenCam_ms);

  lastCamGen = now;

  return errval;
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
