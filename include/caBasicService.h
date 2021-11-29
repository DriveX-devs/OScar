#ifndef CABASICSERVICE_H
#define CABASICSERVICE_H

#include "gpsc.h"
#include "asn1cpp/Seq.hpp"
#include "asn1cpp/Setter.hpp"
#include "asn1cpp/SequenceOf.hpp"
#include "btp.h"
#include <functional>
#include <atomic>
extern "C" {
  #include "CAM.h"
}

typedef enum {
  CAM_NO_ERROR=0,
  CAM_WRONG_INTERVAL=1,
  CAM_ALLOC_ERROR=2,
  CAM_NO_RSU_CONTAINER=3,
  CAM_ASN1_UPER_ENC_ERROR=4,
  CAM_CANNOT_SEND=5
} CABasicService_error_t;

class CABasicService
{
public:
  CABasicService();

  // This function sets the station properties for the CA Basic Service only
  // The GeoNetworking station properties shall be set separately on the GeoNetworking object
  void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype);
  void setFixedPositionRSU(double latitude_deg, double longitude_deg);
  void setStationID(unsigned long fixed_stationid);
  void setStationType(long fixed_stationtype);
  void setRSU() {m_vehicle=false;}
  void setVDP(VDPGPSClient* vdp) {m_vdp=vdp;}

  // A BTP object must always be associated with the CA Basic service
  void setBTP(btp *btp){m_btp = btp;}

  void changeNGenCamMax(int16_t N_GenCamMax) {m_N_GenCamMax=N_GenCamMax;}
  void changeRSUGenInterval(long RSU_GenCam_ms) {m_RSU_GenCam_ms=RSU_GenCam_ms;}

  // Not yet suppported in OCABS
  // void setLowFrequencyContainer(bool enable) {m_lowFreqContainerEnabled = enable;}
  // void setSpecialVehicleContainer(bool enabled) {m_specialVehContainerEnabled = enabled;}

  void startCamDissemination();
  void startCamDissemination(int desync_ms);

  //High frequency RSU container setters
  void setProtectedCommunicationsZonesRSU(asn1cpp::Seq<RSUContainerHighFrequency> sequence) {m_protectedCommunicationsZonesRSU = sequence;}

  uint64_t terminateDissemination();

  const long T_GenCamMin_ms = 100;
  const long T_GenCamMax_ms = 1000;

private:
  const size_t m_MaxPHLength = 23;

  void initDissemination();
  void RSUDissemination();
  void checkCamConditions();
  CABasicService_error_t generateAndEncodeCam();
  int64_t computeTimestampUInt64();

  btp *m_btp; // The BTP object has a reference to a GeoNetworking object, which in turn has the right socket descriptor to enable the dissemination of CAMs

  long m_T_CheckCamGen_ms;
  long m_T_GenCam_ms;
  int16_t m_N_GenCam;
  int16_t m_N_GenCamMax;

  long m_RSU_GenCam_ms; // CAM generation interval for RSU ITS-Ss

  int64_t lastCamGen;
  int64_t lastCamGenLowFrequency;
  int64_t lastCamGenSpecialVehicle;

  bool m_vehicle;
  VDPGPSClient* m_vdp;

  StationID_t m_station_id;
  StationType_t m_stationtype;

  // Previous CAM relevant values
  double m_prev_heading;
  // Pair with <lat,lon>
  std::pair<double,double> m_prev_pos;
  double m_prev_speed;

  // Statistic: number of CAMs successfully sent since the CA Basic Service has been started
  // The CA Basic Service can count up to 18446744073709551615 (UINT64_MAX) CAMs
  uint64_t m_cam_sent;

  //High frequency RSU container
  asn1cpp::Seq<RSUContainerHighFrequency> m_protectedCommunicationsZonesRSU;
  double m_RSUlon;
  double m_RSUlat;

  // Boolean/Enum variables to enable/disable the presence of certain optional containers in the CAM messages
  // Not yet supported in OCABS
  // bool m_lowFreqContainerEnabled;
  // bool m_specialVehContainerEnabled;

  std::atomic<bool> m_terminateFlag;
};

#endif // CABASICSERVICE_H
