#include "caBasicService.h"
#include "gpsc.h"
#include "asn_utils.h"
#include "utils.h"
#include "extraDeviceCommunicationProtocol.h"
#include <future>
#include <chrono>
#include <iostream>
#include <cfloat>
#include <cmath>
#include <sys/sysinfo.h>
#include <netinet/ether.h>
#include <algorithm>
#include <arpa/inet.h>

// This macro allows the user to schedule the asynchronous execution of a function (fcn) after "msec" milliseconds
#define SCHEDULE(msecs,fcn) \
  (void) std::async(std::launch::async, [&] { \
	std::this_thread::sleep_for(std::chrono::milliseconds(msecs)); \
	fcn(); \
  });

// This macro contains the code for generating a new CAM with all the standard containers
// This code is written as a macro as it is the same for both standard and enhanced CAMs
#define FILL_IN_CAM(msgstruct,cam_mandatory_data,errval) \
 if(bool(cam)==false) \
	{ \
	  return CAM_ALLOC_ERROR; \
	} \
	/* Collect data for mandatory containers */ \
\
  /* Fill the header */ \
  asn1cpp::setField(msgstruct->header.messageID, FIX_CAMID); \
  asn1cpp::setField(msgstruct->header.protocolVersion , protocolVersion_currentVersion); \
  asn1cpp::setField(msgstruct->header.stationID, m_station_id); \
\
  asn1cpp::setField(msgstruct->cam.generationDeltaTime, compute_timestampIts () % 65536); \
\
  /* Fill the basicContainer's station type */ \
  asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.stationType, m_stationtype); \
  if(m_vehicle==true) \
	{ \
\
	  cam_mandatory_data=m_vdp->getCAMMandatoryData(); \
\
	  /* Fill the basicContainer */ \
	  asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue, cam_mandatory_data.altitude.getValue ()); \
	  asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence, cam_mandatory_data.altitude.getConfidence ()); \
	  asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.latitude,cam_mandatory_data.latitude); \
	  asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.longitude,cam_mandatory_data.longitude); \
	  asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence, cam_mandatory_data.posConfidenceEllipse.semiMajorConfidence); \
	  asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence, cam_mandatory_data.posConfidenceEllipse.semiMinorConfidence); \
	  asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation, cam_mandatory_data.posConfidenceEllipse.semiMajorOrientation); \
\
	  /* Fill the highFrequencyContainer */ \
\
	  asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.present, HighFrequencyContainer_PR_basicVehicleContainerHighFrequency); \
	  asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue, cam_mandatory_data.heading.getValue ()); \
	  asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingConfidence, cam_mandatory_data.heading.getConfidence ()); \
	  asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue, cam_mandatory_data.speed.getValue ()); \
	  asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedConfidence, cam_mandatory_data.speed.getConfidence ()); \
	  asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection, cam_mandatory_data.driveDirection); \
	  asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue, cam_mandatory_data.VehicleLength.getValue()); \
	  asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication, cam_mandatory_data.VehicleLength.getConfidence()); \
	  asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth, cam_mandatory_data.VehicleWidth); \
	  asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue, cam_mandatory_data.longAcceleration.getValue ()); \
	  asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationConfidence, cam_mandatory_data.longAcceleration.getConfidence ()); \
	  asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureValue, cam_mandatory_data.curvature.getValue ()); \
	  asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureConfidence, cam_mandatory_data.curvature.getConfidence ()); \
	  asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvatureCalculationMode, cam_mandatory_data.curvature_calculation_mode); \
	  asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateValue, cam_mandatory_data.yawRate.getValue ()); \
	  asn1cpp::setField(msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateConfidence, cam_mandatory_data.yawRate.getConfidence ()); \
\
	  /* Store all the "previous" values used in checkCamConditions() */ \
	  m_prev_pos=m_vdp->getCurrentPosition(); \
	  m_prev_speed=m_vdp->getSpeedValue ().getValue(); \
	  m_prev_heading=m_vdp->getHeadingValue ().getValue(); \
	} \
 else \
	{ \
	  /* Fill the basicContainer */ \
	  /* There is still no full RSU support in this release */ \
	  asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence,AltitudeConfidence_unavailable); \
	  asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue,AltitudeValue_unavailable); \
	  asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.latitude,Latitude_unavailable); \
	  asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.longitude,Longitude_unavailable); \
	  asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence,SemiAxisLength_unavailable); \
	  asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence,SemiAxisLength_unavailable); \
	  asn1cpp::setField(msgstruct->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation,HeadingValue_unavailable); \
	  /* Fill the highFrequencyContainer */ \
	  /* auto RSUContainerHighFreq = asn1cpp::makeSeq(RSUContainerHighFrequency); */ \
\
	  /* High frequency RSU container */ \
	  asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.present,HighFrequencyContainer_PR_rsuContainerHighFrequency); \
\
	  auto protectedComm = asn1cpp::makeSeq(ProtectedCommunicationZone); \
	  asn1cpp::setField(protectedComm->protectedZoneType,ProtectedZoneType_permanentCenDsrcTolling); \
	  asn1cpp::setField(protectedComm->protectedZoneLatitude,Latitude_unavailable); \
	  asn1cpp::setField(protectedComm->protectedZoneLongitude,Longitude_unavailable); \
\
	  asn1cpp::sequenceof::pushList(msgstruct->cam.camParameters.highFrequencyContainer.choice.rsuContainerHighFrequency.protectedCommunicationZonesRSU,protectedComm); \
\
	}

#define ENCODE_AND_SEND_CAM(msgstruct,now) \
	std::string encode_result = asn1cpp::uper::encode(msgstruct); \
\
  /* In case of an encoding error, print some basic data which we just tried to encode into a CAM. This may help debugging the encoding issue */ \
  if(encode_result.size()<1) \
  { \
	std::cerr << "CAM encoding error." << std::endl; \
	std::cerr << "Info: Lat: " << msgstruct->cam.camParameters.basicContainer.referencePosition.latitude \
		<< " Lon: " << msgstruct->cam.camParameters.basicContainer.referencePosition.longitude \
		<< " Heading: " << msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue \
		<< " Speed: " << msgstruct->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue \
		<< " Altitude: " << msgstruct->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue \
		<< std::endl; \
	return CAM_ASN1_UPER_ENC_ERROR; \
  } \
\
 	BTPDataRequest_t dataRequest = {}; \
	dataRequest.BTPType = BTP_B; \
	dataRequest.destPort = CA_PORT; \
	dataRequest.destPInfo = 0; \
	dataRequest.GNType = TSB; \
	dataRequest.GNCommProfile = UNSPECIFIED; \
	dataRequest.GNRepInt =0; \
	dataRequest.GNMaxRepInt=0; \
	dataRequest.GNMaxLife = 1; \
	dataRequest.GNMaxHL = 1; \
	dataRequest.GNTraClass = 0x02; \
	dataRequest.lenght = encode_result.size(); \
\
	packetBuffer pktbuf(encode_result.c_str(),static_cast<unsigned int>(encode_result.size())); \
	dataRequest.data = pktbuf; \
	m_btp->sendBTP(dataRequest); \
\
  m_cam_sent++; \
\
  /* Store the time in which the last CAM (i.e. this one) has been generated and successfully sent */ \
  now=computeTimestampUInt64 ()/NANO_TO_MILLI; \
\
  m_T_GenCam_ms=now-lastCamGen; \
\
  /* Always avoid sending CAMs less often than every second (this may happen in case of issues with the GNSS device) */ \
  if(m_T_GenCam_ms>T_GenCamMax_ms) { \
  	m_T_GenCam_ms=T_GenCamMax_ms; \
  } \
\
  lastCamGen = now;

void
CABasicService::routerOS_RSSI_retriever(void) {
	// Create a new timer
	struct pollfd pollfddata;
	int clockFd;

	// Popen buffer
	char popen_buff[2000];

	if(timer_fd_create(pollfddata, clockFd, m_rssi_aux_update_interval_msec*1e3)<0) {
		std::cerr << "[ERROR] Fatal error! Cannot create timer for the routerOS RSSI retriever thread. No RSSI data will be available." << std::endl;
		m_routeros_rssi={};
		return;
	}

	POLL_DEFINE_JUNK_VARIABLE();

	while(m_terminate_routeros_rssi_flag==false) {
		if(poll(&pollfddata,1,0)>0) {
			POLL_CLEAR_EVENT(clockFd);

			// Original command via ssh: ssh admin@192.168.88.2 interface w60g monitor wlan60-1 once | grep -E "rssi|remote-address"
			std::string ssh_command = "stdbuf -o L ssh admin@" + m_auxiliary_device_ip_addr + " interface w60g monitor wlan60-1 once | stdbuf -o L grep -E \"rssi|remote-address\"";
			FILE *ssh = popen(ssh_command.c_str(),"r");

			if(ssh==NULL) {
				// Sleep at least 1 second, and then try again after a timer expiration
				sleep(1);
				continue;
			}

			std::vector<std::string> m_remotes;

			while(fgets(popen_buff,2000,ssh)!=NULL) {
				char* pch;
				if(strstr(popen_buff,"remote-address")) {
					m_remotes.clear();

					pch=strtok(popen_buff,":");
					pch=strtok(NULL," ,");

					while(pch!=nullptr) {
						std::string pchstr=std::string(pch);
						pchstr.erase(std::remove_if(pchstr.begin(), pchstr.end(), isspace), pchstr.end());
						m_remotes.push_back(pchstr);

						pch=strtok(NULL, " ,");
				  	}
				} else if(strstr(popen_buff,"rssi")) {
					pch = strtok (popen_buff,":");
					pch = strtok (NULL," ,");

					m_routeros_rssi_mutex.lock();
					for(size_t i=0;i<m_remotes.size() && pch!=NULL;i++) {
						m_routeros_rssi[m_remotes[i]]=strtod(pch,nullptr);
						// fprintf(stdout,"[DEBUG] ----- %lf [%s] -----\n",m_routeros_rssi[m_remotes[i]],m_remotes[i].c_str());
						pch=strtok(NULL, " ,");
					}
					m_routeros_rssi_mutex.unlock();
				}
			}
		}
	}

	close(clockFd);
}

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

  m_enhanced_CAMs=false;
  // The value of m_encam_auxiliary_MAC is used only when generating enhanced CAMs
  m_encam_auxiliary_MAC="unavailable";

  m_terminate_routeros_rssi_flag=false;

  m_rssi_aux_update_interval_msec=-1; // RSSI retrieval disabled by default
  m_auxiliary_device_ip_addr="0.0.0.0";
  m_aux_rssi_thr_ptr=nullptr;

  m_extra_computation_device_ip_addr="0.0.0.0"; // 0.0.0.0 means "invalid" or "unavailable"
  m_edcp_sock=-1;

  m_own_private_IP="0.0.0.0";
  m_own_public_IP="0.0.0.0";
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

  if(m_extra_computation_device_ip_addr!="0.0.0.0") {
  	m_edcp_sock=socket(AF_INET,SOCK_DGRAM,0);

  	if(m_edcp_sock<0) {
        perror("Cannot create socket for extra computation device communication. Details");
        fprintf(stderr,"The CAM dissemination will not start. If this error persists, try to disable the connection to extra computation devices.\n");
  			return;
    }

    struct sockaddr_in bind_addr;
    memset(&bind_addr,0,sizeof(bind_addr));
    bind_addr.sin_family=AF_INET;
    bind_addr.sin_addr.s_addr=INADDR_ANY;
    bind_addr.sin_port=htons(EDCP_PORT);

    if(bind(m_edcp_sock,(struct sockaddr *)&bind_addr,sizeof(struct sockaddr_in))<0) {
    	close(m_edcp_sock);
  		perror("Cannot bind socket for extra computation device communication. Details");
			fprintf(stderr,"The CAM dissemination will not start. If this error persists, try to disable the connection to extra computation devices.\n");
			return;
    }

    struct sockaddr_in extra_dev_addr;
    struct in_addr sa_addr;

    extra_dev_addr.sin_family=AF_INET;
    extra_dev_addr.sin_port=htons(EDCP_PORT);
    inet_pton(AF_INET,m_extra_computation_device_ip_addr.c_str(),&sa_addr);
    extra_dev_addr.sin_addr=sa_addr;

    if(connect(m_edcp_sock,(struct sockaddr *)(&extra_dev_addr),sizeof(extra_dev_addr))<0) {
  		close(m_edcp_sock);
  		perror("Cannot connect socket for extra computation device communication. Details");
			fprintf(stderr,"The CAM dissemination will not start. If this error persists, try to disable the connection to extra computation devices.\n");
			return;
  	}

  	// Set a timeout on the socket for the reception of the reply
  	struct timeval timeout_val;
  	timeout_val.tv_sec=0;
  	timeout_val.tv_usec=40000; // 40 ms maxium timeout - future work: better tweak this value!
  	if(setsockopt(m_edcp_sock,SOL_SOCKET,SO_RCVTIMEO,&timeout_val,sizeof(timeout_val))<0) {
  		close(m_edcp_sock);
  		perror("Cannot set timeout for socket for extra computation device communication. Details:");
			fprintf(stderr,"The CAM dissemination will not start. If this error persists, try to disable the connection to extra computation devices.\n");
			return;
  	}
  }

  // If requested and enhanced CAMs are used, create a thread to manage the Aux RSSI retrieval from a connected RouterOS device
	if(m_enhanced_CAMs==true && m_rssi_aux_update_interval_msec>0) {
		m_terminate_routeros_rssi_flag=false;

		m_aux_rssi_thr_ptr = std::unique_ptr<std::thread>(new std::thread(&CABasicService::routerOS_RSSI_retriever,this));
	}

  if(m_vehicle)
	{
	  SCHEDULE(0,initDissemination);
	}
  else
	{
	  SCHEDULE(0,RSUDissemination);
	}

	if(m_aux_rssi_thr_ptr!=nullptr) {
		m_aux_rssi_thr_ptr->join();
	}
  while(m_terminateFlag==false); // Disseminate CAMs
}

void
CABasicService::startCamDissemination(int desync_ms)
{
	m_terminateFlag=false;

  if(m_btp==nullptr) {
  	fprintf(stderr,"Error: no BTP object has been set. The CAM dissemination will not start.\n");
  	return;
  }

  if(m_extra_computation_device_ip_addr!="0.0.0.0") {
  	m_edcp_sock=socket(AF_INET,SOCK_DGRAM,0);

  	if(m_edcp_sock<0) {
        perror("Cannot create socket for extra computation device communication. Details:");
        fprintf(stderr,"The CAM dissemination will not start. If this error persists, try to disable the connection to extra computation devices.\n");
  			return;
    }

    struct sockaddr_in bind_addr;
    memset(&bind_addr,0,sizeof(bind_addr));
    bind_addr.sin_family=AF_INET;
    bind_addr.sin_addr.s_addr=INADDR_ANY;
    bind_addr.sin_port=htons(EDCP_PORT);

    if(bind(m_edcp_sock,(struct sockaddr *)&bind_addr,sizeof(struct sockaddr_in))<0) {
    	close(m_edcp_sock);
  		perror("Cannot bind socket for extra computation device communication. Details");
			fprintf(stderr,"The CAM dissemination will not start. If this error persists, try to disable the connection to extra computation devices.\n");
			return;
    }

    struct sockaddr_in extra_dev_addr;
    struct in_addr sa_addr;

    extra_dev_addr.sin_family=AF_INET;
    extra_dev_addr.sin_port=htons(EDCP_PORT);
    inet_pton(AF_INET,m_extra_computation_device_ip_addr.c_str(),&sa_addr);
    extra_dev_addr.sin_addr=sa_addr;

    if(connect(m_edcp_sock,(struct sockaddr *)(&extra_dev_addr),sizeof(extra_dev_addr))<0) {
  		close(m_edcp_sock);
  		perror("Cannot connect socket for extra computation device communication. Details");
			fprintf(stderr,"The CAM dissemination will not start. If this error persists, try to disable the connection to extra computation devices.\n");
			return;
  	}

  	// Set a timeout on the socket for the reception of the reply
  	struct timeval timeout_val;
  	timeout_val.tv_sec=0;
  	timeout_val.tv_usec=40000; // 40 ms maxium timeout - future work: better tweak this value!
  	if(setsockopt(m_edcp_sock,SOL_SOCKET,SO_RCVTIMEO,&timeout_val,sizeof(timeout_val))<0) {
  		close(m_edcp_sock);
  		perror("Cannot set timeout for socket for extra computation device communication. Details:");
			fprintf(stderr,"The CAM dissemination will not start. If this error persists, try to disable the connection to extra computation devices.\n");
			return;
  	}
  }

  // If requested and enhanced CAMs are used, create a thread to manage the Aux RSSI retrieval from a connected RouterOS device
	if(m_enhanced_CAMs==true && m_rssi_aux_update_interval_msec>0) {
		m_terminate_routeros_rssi_flag=false;

		m_aux_rssi_thr_ptr = std::unique_ptr<std::thread>(new std::thread(&CABasicService::routerOS_RSSI_retriever,this));
	}

  if(m_vehicle)
	{
	  SCHEDULE(desync_ms,initDissemination);
	}
  else
	{
	  SCHEDULE(desync_ms,RSUDissemination);
	}

	if(m_aux_rssi_thr_ptr!=nullptr) {
		m_aux_rssi_thr_ptr->join();
	}
	while(m_terminateFlag==false); // Disseminate CAMs
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
  int64_t now;
  CABasicService_error_t cam_error;
  bool condition_verified;
  static bool dyn_cond_verified=false;

  // Create a new timer to periodically check the CAM conditions, according to the standard
	struct pollfd pollfddata;
	int clockFd;

	// The last argument of timer_fd_create should be in microseconds
	if(timer_fd_create(pollfddata, clockFd, m_T_CheckCamGen_ms*1e3)<0) {
		std::cerr << "[ERROR] Fatal error! Cannot create timer for the CAM dissemination" << std::endl;
		terminateDissemination();
		return;
	}

	POLL_DEFINE_JUNK_VARIABLE();

  while(m_terminateFlag==false) {
  	if(poll(&pollfddata,1,0)>0) {
			POLL_CLEAR_EVENT(clockFd);

			condition_verified=false;
			now=computeTimestampUInt64()/NANO_TO_MILLI;

		  // If no initial CAM has been triggered before checkCamConditions() has been called, throw an error
		  if(m_prev_heading==-1 || m_prev_speed==-1 || m_prev_pos.first==-DBL_MAX || m_prev_pos.second==-DBL_MAX)
			{
				std::cerr << "Error. checkCamConditions() was called before sending any CAM and this is not allowed." << std::endl;
				terminateDissemination();
				close(clockFd);
				break;
			  //throw std::runtime_error("Error. checkCamConditions() was called before sending any CAM and this is not allowed.");
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

			// Debug print: leave commented when releasing for testing or using for a use case
			// std::cout << "Check for m_T_GenCam_ms:" << m_T_GenCam_ms << std::endl;

			// try {
			//   SCHEDULE(m_T_CheckCamGen_ms,CABasicService::checkCamConditions);
			//   //SCHEDULE(10,CABasicService::checkCamConditions);
			// } catch(const std::exception &excp) {
			// 	std::cerr << "Error: cannot schedule any new CAM transmission. Reason: " << excp.what() << std::endl;
			// 	terminateDissemination();
			// }
		}
	}

	close(clockFd);
}

CABasicService_error_t
CABasicService::generateAndEncodeCam()
{
  VDPGPSClient::CAM_mandatory_data_t cam_mandatory_data;
  CABasicService_error_t errval=CAM_NO_ERROR;

  int64_t now;

  auto cam = asn1cpp::makeSeq(CAM);
  auto encam = asn1cpp::makeSeq(CAMEnhanced);

  if(m_enhanced_CAMs==false) {
 		FILL_IN_CAM(cam,cam_mandatory_data,errval);
 	} else {
 		FILL_IN_CAM(encam,cam_mandatory_data,errval);
 	}

	// Extra part: fill in the proposed channelNodeStatusContainer (only for the enhanced CAMs)
	if(m_enhanced_CAMs==true) {
		// Gather the current node load
		struct sysinfo sysdata;
		memset(&sysdata,0,sizeof(sysdata));

		auto channelNodeStatusSeq = asn1cpp::makeSeq(ChannelNodeStatusContainer);

		if(!sysinfo(&sysdata)) {
			float floatload = 1.f / (1 << SI_LOAD_SHIFT);
			asn1cpp::setField(channelNodeStatusSeq->cpuLoad,sysdata.loads[0]*floatload*100/get_nprocs()*100);
			asn1cpp::setField(channelNodeStatusSeq->ramLoad,sysdata.freeram/1048576);
			
			if(m_encam_auxiliary_MAC!="unavailable" && m_encam_auxiliary_MAC!="") {
				struct ether_addr *MAC_addr_bytes_ptr;
				MAC_addr_bytes_ptr=ether_aton(m_encam_auxiliary_MAC.c_str());

				std::string encoded_mac;
				encoded_mac.push_back(MAC_addr_bytes_ptr->ether_addr_octet[0]);
				encoded_mac.push_back(MAC_addr_bytes_ptr->ether_addr_octet[1]);
				encoded_mac.push_back(MAC_addr_bytes_ptr->ether_addr_octet[2]);
				encoded_mac.push_back(MAC_addr_bytes_ptr->ether_addr_octet[3]);
				encoded_mac.push_back(MAC_addr_bytes_ptr->ether_addr_octet[4]);
				encoded_mac.push_back(MAC_addr_bytes_ptr->ether_addr_octet[5]);

				asn1cpp::setField(channelNodeStatusSeq->auxilliaryLinkMac,encoded_mac);
			}
		} else {
			asn1cpp::setField(channelNodeStatusSeq->cpuLoad,CPULoad_unavailable);
			asn1cpp::setField(channelNodeStatusSeq->ramLoad,RAMLoad_unavailable);
		}

		asn1cpp::setField(channelNodeStatusSeq->gpuLoad,GPULoad_unavailable);

		// Insert the IP information, if available
		if(m_own_private_IP!="0.0.0.0") {
			std::string encoded_own_IP;

			// Convert the IP address from std::string to IP address
			struct in_addr sain;

			inet_pton(AF_INET,m_own_private_IP.c_str(),&sain);

			encoded_own_IP.push_back(sain.s_addr & 0xFF);
			encoded_own_IP.push_back((sain.s_addr>>8) & 0xFF);
			encoded_own_IP.push_back((sain.s_addr>>16) & 0xFF);
			encoded_own_IP.push_back((sain.s_addr>>24) & 0xFF);

			asn1cpp::setField(channelNodeStatusSeq->ipAddress,encoded_own_IP);
		}

		// Insert the public IP information, if available
		if(m_own_public_IP!="0.0.0.0") {
			std::string encoded_public_own_IP;

			// Convert the IP address from std::string to IP address
			struct in_addr sain;

			inet_pton(AF_INET,m_own_public_IP.c_str(),&sain);

			encoded_public_own_IP.push_back(sain.s_addr & 0xFF);
			encoded_public_own_IP.push_back((sain.s_addr>>8) & 0xFF);
			encoded_public_own_IP.push_back((sain.s_addr>>16) & 0xFF);
			encoded_public_own_IP.push_back((sain.s_addr>>24) & 0xFF);

			asn1cpp::setField(channelNodeStatusSeq->publicIpAddress,encoded_public_own_IP);
		}

		if(m_rssi_aux_update_interval_msec>0) {
			double aux_RSSI=-DBL_MAX;
			m_routeros_rssi_mutex.lock();
			
			// The map should theoretically contain only one MAC address in the current setup with an auxilliary AP
			// Future work should consider the possibility of having multiple MACs and RSSI values store inside
			// aux_RSSI=m_routeros_rssi[m_encam_auxiliary_MAC];
			aux_RSSI=m_routeros_rssi.begin()->second;

			m_routeros_rssi_mutex.unlock();

			asn1cpp::setField(channelNodeStatusSeq->auxilliaryLinkRSSI,aux_RSSI==-DBL_MAX ? AuxilliaryLinkRSSI_unavailable : static_cast<long>(aux_RSSI*100));
		} else {
			asn1cpp::setField(channelNodeStatusSeq->auxilliaryLinkRSSI,AuxilliaryLinkRSSI_unavailable);
		}

		if(m_extra_computation_device_ip_addr!="0.0.0.0") {
			extraDeviceCommProtHeader_t edcp_head={0};
			extraDeviceCommProtHeader_t edcp_head_info_from_client={0};
			edcp_head.type=REQUEST_TYPE;
			
			if(send(m_edcp_sock,&edcp_head,sizeof(edcp_head),0)>0) {
				// Wait for a reply; if it is valid, use the received information to inform the receiver about the load status of the connected extra computation device
				size_t recvbytes=recv(m_edcp_sock,&edcp_head_info_from_client,sizeof(edcp_head_info_from_client),0);
				if(recvbytes==sizeof(edcp_head_info_from_client)) {
					long cpuUsage=ntohs(edcp_head_info_from_client.cpuUsage);
					long gpuUsage=ntohs(edcp_head_info_from_client.gpuUsage);
					long freeRam=ntohl(edcp_head_info_from_client.ramUsage);

					asn1cpp::setField(channelNodeStatusSeq->extraComputationDeviceCpuLoad,cpuUsage>CPULoad_unavailable ? CPULoad_unavailable : cpuUsage);
					asn1cpp::setField(channelNodeStatusSeq->extraComputationDeciceGpuLoad,gpuUsage>CPULoad_unavailable ? GPULoad_unavailable : gpuUsage);
					asn1cpp::setField(channelNodeStatusSeq->extraComputationDeviceRamLoad,freeRam>RAMLoad_unavailable ? RAMLoad_unavailable : freeRam);
				}
			} else {
				fprintf(stderr,"[ERROR] Cannot send EDCP request.\n");
			}
		}

		asn1cpp::setField(encam->cam.camParameters.channelNodeStatusContainer,channelNodeStatusSeq);
	}

 	if(m_enhanced_CAMs==false) {
 		ENCODE_AND_SEND_CAM(cam,now);
 	} else {
 		ENCODE_AND_SEND_CAM(encam,now);
 	}

  return errval;
}

uint64_t 
CABasicService::terminateDissemination()
{
	m_terminate_routeros_rssi_flag=true;

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
