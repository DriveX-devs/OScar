#ifndef CABASICSERVICE_H
#define CABASICSERVICE_H

#include "gpsc.h"
#include "asn1cpp/Seq.hpp"
#include "asn1cpp/Setter.hpp"
#include "asn1cpp/SequenceOf.hpp"
#include "btp.h"
#include <functional>
#include <atomic>
#include <thread>
extern "C" {
  #include "CAM.h"
  #include "CAMEnhanced.h"
}

typedef enum {
  CAM_NO_ERROR=0,
  CAM_WRONG_INTERVAL=1,
  CAM_ALLOC_ERROR=2,
  CAM_NO_RSU_CONTAINER=3,
  CAM_ASN1_UPER_ENC_ERROR=4,
  CAM_CANNOT_SEND=5
} CABasicService_error_t;


// Important future work: implement terminateDissemination(), which should also close m_edcp_sock if needed
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

  void enableEnhancedCAMs() {m_enhanced_CAMs=true;}
  void disableEnhancedCAMs() {m_enhanced_CAMs=false;}

  void setEnhancedCAMAuxiliaryMAC(std::string encam_auxiliary_MAC) {m_encam_auxiliary_MAC=encam_auxiliary_MAC;}
  void setExtraComputationDeviceIP(std::string extra_dev_ip) {m_extra_computation_device_ip_addr=extra_dev_ip;}

  void setOwnPrivateIP(std::string own_private_IP) {m_own_private_IP=own_private_IP;}
  void setOwnPublicIP(std::string own_public_IP) {m_own_public_IP=own_public_IP;}
  void disableOwnPrivateIP() {m_own_private_IP="0.0.0.0";}
  void disableOwnPublicIP() {m_own_private_IP="0.0.0.0";}

  // This function has an effect only if called before startCamDissemination()
  void enableAuxRSSIRetrieval(double rssi_aux_update_interval_msec, std::string auxiliary_device_ip_addr) {m_rssi_aux_update_interval_msec=rssi_aux_update_interval_msec; m_auxiliary_device_ip_addr=auxiliary_device_ip_addr;}

  const long T_GenCamMin_ms = 100;
  const long T_GenCamMax_ms = 1000;

private:
  const size_t m_MaxPHLength = 23;

  void initDissemination();
  void RSUDissemination();
  void checkCamConditions();
  // Main function to generate and send a new CAM
  CABasicService_error_t generateAndEncodeCam();
  int64_t computeTimestampUInt64();

  // Special thread function to retrieve RSSI values from a connected RouterOS-based device
  void routerOS_RSSI_retriever();

  std::map<std::string,double> m_routeros_rssi; // Auxiliary RouterOS-based device RSSI map (<MAC address>,<RSSI value>)
  std::mutex m_routeros_rssi_mutex;

  std::atomic<bool> m_terminate_routeros_rssi_flag;

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

  bool m_enhanced_CAMs;

  std::string m_encam_auxiliary_MAC;

  // Any value <= 0 disables the Aux RSSI retrieval for enhanced CAMs (default value set in the constructor = -1)
  double m_rssi_aux_update_interval_msec;
  // The value of this option is used only if m_rssi_aux_update_interval_msec>0
  std::string m_auxiliary_device_ip_addr;
  // Pointer to the thread object for the Aux RSSI retrieval
  std::unique_ptr<std::thread> m_aux_rssi_thr_ptr;

  std::string m_extra_computation_device_ip_addr;
  int m_edcp_sock;

  // Extra information which can be optionally disseminated through Enhanced CAMs
  std::string m_own_private_IP;
  std::string m_own_public_IP;
};

#endif // CABASICSERVICE_H
