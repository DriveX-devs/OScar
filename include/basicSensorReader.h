#ifndef BASICSENSORREADER_H
#define BASICSENSORREADER_H

#include <atomic>
#include <thread>
#include "LDMmap.h"
#include "gpsc.h"

class BasicSensorReader
{
public:
    typedef enum classification{
        UNKNOWN = 0,
        CAR = 1,
        SEMI = 2,
        CYCLE = 3,
        NOT_DEFINED = 4,
    }classification_t;

    BasicSensorReader(std::string interface, ldmmap::LDMMap* ldm, VDPGPSClient* gpsc, int vehicleID){
        m_interface = interface;
        m_LDM = ldm;
        m_gpsc_ptr = gpsc;
        m_stationID = vehicleID;
        m_enable_classification = false;
        m_verbose = false;}
    bool startReader();
    bool stopReader();
    void setLDMmap(ldmmap::LDMMap* ldm){m_LDM = ldm;}
    std::string getInterface(){return m_interface;}
    ldmmap::LDMMap* getLDMmap(){return m_LDM;}
    VDPGPSClient* getGPSClient(){return m_gpsc_ptr;}
    void readerLoop();
    bool getThreadRunning(){return m_thread_running;}
    int getCanSocket(){return m_can_socket;}
    int getStationID(){return m_stationID;}
    void setEnableClassification(bool enable){m_enable_classification = enable;}
    void setVerbose(bool verbose){m_verbose = verbose;}

private:

    uint64_t computeTimestamp();

    std::string m_interface;
    ldmmap::LDMMap* m_LDM;
    VDPGPSClient *m_gpsc_ptr;
    int m_stationID;

    int m_can_socket = -1;
    pthread_t m_tid = -1;
    std::atomic<bool> m_thread_running = false;
    std::thread m_thread;

    bool m_enable_classification = false;
    bool m_verbose = false;

    void reader_thread();
};
#endif //BASICSENSORREADER_H
