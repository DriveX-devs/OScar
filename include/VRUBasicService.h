#ifndef VRUBasicService_h
#define VRUBasicService_h

#include "gpsc.h"
#include "LDMmap.h"
#include "asn1cpp/Seq.hpp"
#include "asn1cpp/Setter.hpp"
#include "asn1cpp/SequenceOf.hpp"
#include "btp.h"
#include <functional>
#include <atomic>
#include <thread>

extern "C" {
  #include "VAM.h"
}

typedef enum{
    VAM_NO_ERROR = 0,
    VAM_WRONG_INTERVAL = 1,
    VAM_ALLOC_ERROR = 2,
    VAM_ASN1_UPER_ENC_ERROR = 3,
    VAM_CANNOT_SEND = 4,
} VRUBasicService_error_t;

typedef enum{
    VRU_ROLE_OFF = 0,
    VRU_ROLE_ON = 1,
} VRURole_t;

typedef enum{
    VRU_IDLE = 0,
    VRU_ACTIVE_STANDALONE = 1,
    VRU_ACTIVE_CLUSTER_LEADER = 2,
    VRU_PASSIVE = 3,
} VRUClusteringstate_t;

typedef enum{
    NOT_VALID = -1,
    DISSEMINATION_START = 0,
    MAX_TIME_ELAPSED = 1,
    HEADING_CHANGE = 2,
    POSITION_CHANGE = 3,
    SPEED_CHANGE = 4,
    SAFE_DISTANCES = 5,
} triggcond_t;

typedef struct distance {
    double longitudinal,lateral,vertical;
    StationId_t ID;
    StationType_t station_type;
    bool safe_dist;
} distance_t;

class VRUBasicService
{
public:
    VRUBasicService();
    
    void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype);
    void setLogfile(std::string filename) {m_log_filename=filename;}

    std::vector<distance_t> get_min_distance(ldmmap::LDMMap* LDM);
    void setStationID(unsigned long fixed_stationid);
    void setStationType(long fixed_stationtype);
    void setLDM(ldmmap::LDMMap* LDM){m_LDM = LDM;}
    void setBTP(btp* btp){m_btp = btp;}
    void setVRUdp(VDPGPSClient* VRUdp) {m_VRUdp=VRUdp;}
    void setPositionThreshold(double pos_th) {m_pos_th=pos_th;}
    void setSpeedThreshold(double speed_th) {m_speed_th=speed_th;}
    void setHeadingThreshold(double head_th) {m_head_th=head_th;}
    void setSafeLateralDistance(double safe_lat_d) {m_lat_safe_d = safe_lat_d;}
    void setSafeVerticalDistance(double safe_vert_d) {m_vert_safe_d = safe_vert_d;}

    void startVamDissemination();
    void startVamDissemination(int desync_ms);

    uint64_t terminateDissemination();

    void setCheckVamGenMs(long nextVAM) {m_vam_gen_mutex.lock(); m_T_CheckVamGen_ms = nextVAM; m_vam_gen_mutex.unlock();};

    void toffUpdateAfterDeltaUpdate(double delta);

    void toffUpdateAfterTransmission();

    
    const long T_GenVamMin_ms = 100;
    const long T_GenVamMax_ms = 5000;
private:
    const size_t m_MaxPHLength = 23;
    
    std::string printMinDist(double minDist);
    void initDissemination();
    void checkVamConditions();
    bool checkVamRedundancyMitigation();

    
    VRUBasicService_error_t generateAndEncodeVam();
    
    int64_t computeTimestampUInt64();

    btp* m_btp;

    ldmmap::LDMMap* m_LDM;
    
    long m_T_GenVam_ms;
    long m_T_CheckVamGen_ms;
    int64_t lastVamGen;

    int16_t m_N_GenVam_red;
    int16_t m_N_GenVam_max_red;
    
    StationId_t m_station_id;
    StationType_t m_stationtype;

    VDPGPSClient* m_VRUdp;

    // Previous VAM relevant values
    double m_prev_heading;
    VDPGPSClient::VRU_position_latlon_t m_prev_pos;
    double m_prev_speed;

    // Triggering conditions thresholds
    double m_pos_th;
    double m_speed_th;
    double m_head_th;

    // Safe distances
    double m_long_safe_d;
    double m_lat_safe_d;
    double m_vert_safe_d;

    // Statistic: number of VAMs successfully sent since the VRU Basic Service has been started
    // The VRU Basic Service can count up to 18446744073709551615 (UINT64_MAX) VAMs
    uint64_t m_vam_sent;

    // Statistics: number of VAMs sent per triggering conditions
    uint64_t m_pos_sent;
    uint64_t m_speed_sent;
    uint64_t m_head_sent;
    uint64_t m_safedist_sent;
    uint64_t m_time_sent;
    
    // Logging file
    std::string m_log_filename;

    // Triggering condition
    triggcond_t m_trigg_cond;

    // Variable containing the distance of the nearest vehicle and pedestrian from the current pedestrian
    std::vector<distance_t> m_min_dist;

    // VRU state variables
    int m_VRU_role;
    int m_VRU_clust_state;
    
    std::atomic<bool> m_terminateFlag;

    int64_t m_last_transmission = 0;
    double m_Ton_pp = 0;
    double m_last_delta = 0;

    std::mutex m_vam_gen_mutex;
};

#endif /* VRUBasicService_h */

