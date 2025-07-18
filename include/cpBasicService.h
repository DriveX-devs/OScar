#ifndef CPBASICSERVICE_H
#define CPBASICSERVICE_H

#include "gpsc.h"
#include "asn1cpp/Seq.hpp"
#include "asn1cpp/Setter.hpp"
#include "asn1cpp/SequenceOf.hpp"
#include "btp.h"
#include "LDMmap.h"
#include <functional>
#include <atomic>
#include <thread>

extern "C" {
#include "CollectivePerceptionMessage.h"
}

typedef struct{
    double heading;
    double lat;
    double lon;
    double speed_ms;
    uint64_t timestamp_us;

    double latest_lat;
    double latest_lon;
    double latest_speed_ms;
    uint64_t latest_timestamp_us;

}lastCPM_t;

class CPBasicService
{
public:
    CPBasicService();
    void setStationID(unsigned long fixed_stationid);
    void setStationType(long fixed_stationtype);
    void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype);
    void setLogfile(std::string filename) {m_log_filename=filename;}
    void setVDP(VDPGPSClient* vdp) {m_vdp=vdp;}
    void setLDM(ldmmap::LDMMap* LDM) {m_LDM=LDM;}
    void setBTP(btp *btp){m_btp = btp;}

    void changeNGenCpmMax(int16_t N_GenCpmMax) {m_N_GenCpmMax=N_GenCpmMax;}
    void setRealTime(bool real_time){m_real_time=real_time;}
    uint64_t terminateDissemination();
    void setRedundancyMitigation(bool choice){m_redundancy_mitigation = choice;}
    void disableRedundancyMitigation(){m_redundancy_mitigation = false;}

    void setOwnPrivateIP(std::string own_private_IP) {m_own_private_IP=own_private_IP;}
    void setOwnPublicIP(std::string own_public_IP) {m_own_public_IP=own_public_IP;}
    void disableOwnPrivateIP() {m_own_private_IP="0.0.0.0";}
    void disableOwnPublicIP() {m_own_private_IP="0.0.0.0";}
    void setFaultyAccelerationCheck(double value) {m_acceleration_threshold = value; m_check_faulty_acceleration = true;}
    void setSpeedTriggering(bool value) {m_speed_triggering = value;}
    void setVerbose(bool value) {m_verbose = value;}

    void initDissemination();

    void setCheckCpmGenMs(long nextCPM) {m_cpm_gen_mutex.lock(); m_N_GenCpm=nextCPM; m_cpm_gen_mutex.unlock();};

    void toffUpdateAfterDeltaUpdate(double delta);

    void toffUpdateAfterTransmission();

    const long T_GenCpmMin_ms = 100;
    const long T_GenCpm_ms = 100;
    const long T_GenCpmMax_ms = 1000;
    const long m_T_AddSensorInformation = 1000;

    uint64_t get_CPM_sent() {m_sent_mutex.lock(); uint64_t c = m_cpm_sent; m_sent_mutex.unlock(); return c;};

    private:

    std::string generateAndEncodeCPM();
    int64_t computeTimestampUInt64();
    std::pair<bool, std::string> checkCPMconditions(std::vector<ldmmap::LDMMap::returnedVehicleData_t>::iterator PO_data, VDPGPSClient::CPM_mandatory_data_t ego_data);

    btp *m_btp;

    long m_T_CheckCpmGen_ms;
    long m_T_LastSensorInfoContainer;

    long m_T_GenCpm_ms;
    int16_t m_N_GenCpm;
    int16_t m_N_GenCpmMax;

    int64_t lastCpmGen;
    int64_t lastCpmGenLowFrequency;
    int64_t lastCpmGenSpecialVehicle;

    bool m_real_time;
    bool m_vehicle;
    bool m_redundancy_mitigation;
    bool m_check_faulty_acceleration;
    bool m_speed_triggering;
    VDPGPSClient* m_vdp;

    ldmmap::LDMMap* m_LDM;

    std::map<uint64_t, lastCPM_t> m_lastCPM;

    std::atomic<bool> m_terminateFlag;

    StationId_t m_station_id;
    StationType_t m_stationtype;

    std::string m_log_filename;

    // Previous Cpm relevant values
    double m_prev_heading;
    double m_prev_distance;
    double m_prev_speed;
    std::vector<long> m_lastCPM_POs;

    // Statistic: number of Cpms successfully sent since the CA Basic Service has been started
    // The CA Basic Service can count up to 18446744073709551615 (UINT64_MAX) Cpms
    uint64_t m_cpm_sent;
    std::mutex m_sent_mutex;

    // Extra information which can be optionally disseminated through Enhanced CAMs
    std::string m_own_private_IP;
    std::string m_own_public_IP;

    // OScar custom variables
    double m_acceleration_threshold;
    bool m_verbose;

    double m_last_transmission = 0;
    double m_Ton_pp = 0;
    double m_last_delta = 0;

    std::mutex m_cpm_gen_mutex;
};


#endif //CPBASICSERVICE_H

