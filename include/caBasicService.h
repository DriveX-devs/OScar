#ifndef CABASICSERVICE_H
#define CABASICSERVICE_H

#include "gpsc.h"
#include "asn1cpp/Seq.hpp"
#include "asn1cpp/Setter.hpp"
#include "asn1cpp/SequenceOf.hpp"
#include <functional>
extern "C" {
  #include "CAM.h"
}

// Destination port for the CAMs encapsulated inside UDP
// According to ETSI TS 103 301 v1.1.1 (page 36) this value should be set to 47101 when using UDP/IP 
// (even if this would be intended for infrastructure services - in the future, this code will not use UDP/IPv4, but it will disseminate by
// directly leveraging BTP and GeoNetworking)
#define UDP_PACKET_DEST_PORT 47101

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

  void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype);
  void setFixedPositionRSU(double latitude_deg, double longitude_deg);
  void setStationID(unsigned long fixed_stationid);
  void setStationType(long fixed_stationtype);
  void setSocketTx(int socket_tx_fd) {m_socket_tx_fd = socket_tx_fd;}
  // void setSocketRx(int socket_rx_fd);
  void setRSU() {m_vehicle=false;}
  void setVDP(VDPGPSClient* vdp) {m_vdp=vdp;}

  // void receiveCam(BTPDataIndication_t dataIndication, Address from);
  void changeNGenCamMax(int16_t N_GenCamMax) {m_N_GenCamMax=N_GenCamMax;}
  void changeRSUGenInterval(long RSU_GenCam_ms) {m_RSU_GenCam_ms=RSU_GenCam_ms;}
//  void addCARxCallback(std::function<void(asn1cpp::Seq<CAM>, Address)> rx_callback) {m_CAReceiveCallback=rx_callback;}
  void setRealTime(bool real_time){m_real_time=real_time;}

  void setLowFrequencyContainer(bool enable) {m_lowFreqContainerEnabled = enable;}
  void setSpecialVehicleContainer(bool enabled) {m_specialVehContainerEnabled = enabled;}

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

  // std::function<void(CAM_t *, Address)> m_CAReceiveCallback;
  // std::function<void(asn1cpp::Seq<CAM>, Address)> m_CAReceiveCallback;

  // Ptr<btp> m_btp;

  int m_socket_tx_fd = -1;
  int m_socket_rx_fd = -1;

  long m_T_CheckCamGen_ms;
  long m_T_GenCam_ms;
  int16_t m_N_GenCam;
  int16_t m_N_GenCamMax;

  long m_RSU_GenCam_ms; // CAM generation interval for RSU ITS-Ss

  int64_t lastCamGen;
  int64_t lastCamGenLowFrequency;
  int64_t lastCamGenSpecialVehicle;

  bool m_real_time;
  bool m_vehicle;
  VDPGPSClient* m_vdp;

  // Ptr<Socket> m_socket_tx; // Socket TX

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

  // std::vector<std::pair<ReferencePosition_t,PathHistoryDeltas_t>> m_refPositions;

  //High frequency RSU container
  asn1cpp::Seq<RSUContainerHighFrequency> m_protectedCommunicationsZonesRSU;
  double m_RSUlon;
  double m_RSUlat;

  // Boolean/Enum variables to enable/disable the presence of certain optional containers in the CAM messages
  bool m_lowFreqContainerEnabled;
  bool m_specialVehContainerEnabled;
};

#endif // CABASICSERVICE_H
