#ifndef BASICSENSORREADER_H
#define BASICSENSORREADER_H

#include <atomic>
#include <thread>
#include "LDMmap.h"
#include "gpsc.h"

class BasicSensorReader
{
public:
    BasicSensorReader(std::string interface, ldmmap::LDMMap* ldm, VDPGPSClient* gpsc, int vehicleID){
        m_interface = interface; m_LDM = ldm; m_gpsc_ptr = gpsc; m_stationID = vehicleID;}
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

    void reader_thread();
};
#endif //BASICSENSORREADER_H
