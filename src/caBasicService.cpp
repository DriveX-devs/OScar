#include "caBasicService.h"
#include "gpsc.h"
#include "asn_utils.h"
#include <future>
#include <chrono>
#include <iostream>
#include <cfloat>
#include <cmath>

// This macro allows the user to schedule the asynchronous execution of a function (fcn) after "msec" milliseconds
#define SCHEDULE(msecs,fcn) \
  (void) std::async(std::launch::async, [&] { \
	std::this_thread::sleep_for(std::chrono::milliseconds(msecs)); \
	fcn(); \
  });

CABasicService::CABasicService()
{
  m_station_id = ULONG_MAX;
  m_stationtype = LONG_MAX;
  // m_socket_tx_fd=-1;
	m_btp = nullptr;

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
  // This is not yet supported in OCABS
  // m_lowFreqContainerEnabled = false;
  // m_specialVehContainerEnabled = false;

  m_terminateFlag=false;
}

void
CABasicService::setStationProperties(unsigned long fixed_stationid,long fixed_stationtype)
{
  m_station_id=fixed_stationid;
  m_stationtype=fixed_stationtype;
}

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

void
CABasicService::startCamDissemination()
{
  m_terminateFlag=false;

  if(m_btp==nullptr) {
  	fprintf(stderr,"Error: no BTP object has been set. The CAM dissemination will not start.\n");
  	return;
  }

  if(m_vehicle)
	{
	  SCHEDULE(0,initDissemination);
	}
  else
	{
	  SCHEDULE(0,RSUDissemination);
	}

  while(m_terminateFlag==false); // Disseminate CAMs
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

void
CABasicService::initDissemination()
{
  generateAndEncodeCam();
  SCHEDULE(m_T_CheckCamGen_ms,CABasicService::checkCamConditions);
}

void
CABasicService::RSUDissemination()
{
  generateAndEncodeCam();
  SCHEDULE(m_RSU_GenCam_ms,CABasicService::RSUDissemination);
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

  SCHEDULE(m_T_CheckCamGen_ms,CABasicService::checkCamConditions);
}

CABasicService_error_t
CABasicService::generateAndEncodeCam()
{
  VDPGPSClient::CAM_mandatory_data_t cam_mandatory_data;
  CABasicService_error_t errval=CAM_NO_ERROR;

  int64_t now;

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
  asn1cpp::setField(cam->cam.generationDeltaTime, compute_timestampIts () % 65536);

  /* Fill the basicContainer's station type */
  asn1cpp::setField(cam->cam.camParameters.basicContainer.stationType, m_stationtype);
  if(m_vehicle==true)
	{

	  cam_mandatory_data=m_vdp->getCAMMandatoryData();

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

	  // High frequency RSU container
	  asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.present,HighFrequencyContainer_PR_rsuContainerHighFrequency);

	  auto protectedComm = asn1cpp::makeSeq(ProtectedCommunicationZone);
	  asn1cpp::setField(protectedComm->protectedZoneType,ProtectedZoneType_permanentCenDsrcTolling);
	  asn1cpp::setField(protectedComm->protectedZoneLatitude,Latitude_unavailable);
	  asn1cpp::setField(protectedComm->protectedZoneLongitude,Longitude_unavailable);

	  asn1cpp::sequenceof::pushList(cam->cam.camParameters.highFrequencyContainer.choice.rsuContainerHighFrequency.protectedCommunicationZonesRSU,protectedComm);

	}

  std::string encode_result = asn1cpp::uper::encode(cam);

  // In case of an encoding error, print some basic data which we just tried to encode into a CAM
  // This may help debugging the encoding issue
  if(encode_result.size()<1)
  {
	std::cerr << "CAM encoding error." << std::endl;
	std::cerr << "Info: Lat: " << cam->cam.camParameters.basicContainer.referencePosition.latitude
		<< " Lon: " << cam->cam.camParameters.basicContainer.referencePosition.longitude
		<< " Heading: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue
		<< " Speed: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue
		<< " Altitude: " << cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue
		<< std::endl;
	return CAM_ASN1_UPER_ENC_ERROR;
  }

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
	dataRequest.GNTraClass = 0x02; // Store carry foward: no - Channel offload: no - Traffic Class ID: 2
	dataRequest.lenght = encode_result.size();

	packetBuffer pktbuf(encode_result.c_str(),static_cast<unsigned int>(encode_result.size()));
	dataRequest.data = pktbuf;
	m_btp->sendBTP(dataRequest);

  m_cam_sent++;

  // Store the time in which the last CAM (i.e. this one) has been generated and successfully sent
  now=computeTimestampUInt64 ()/NANO_TO_MILLI;

  m_T_GenCam_ms=now-lastCamGen;

  // Always avoid sending CAMs less often than every second (this may happen in case of issues with the GNSS device)
  if(m_T_GenCam_ms>T_GenCamMax_ms) {
  	m_T_GenCam_ms=T_GenCamMax_ms;
  }

  // Debug "printf" - keep commented unless strictly needed
  // printf("[TBR] m_T_GenCam_ms: %ld\n",m_T_GenCam_ms);

  lastCamGen = now;

  return errval;
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
