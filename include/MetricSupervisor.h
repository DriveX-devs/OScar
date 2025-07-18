#include <string>
#include <vector>
#include <fstream>
#include <mutex>
#include "caBasicService.h"
#include "cpBasicService.h"
#include "VRUBasicService.h"
#include "SocketClient.h"
#include "utils.h"

#ifndef OSCAR_METRICSUPERVISOR_H
#define OSCAR_METRICSUPERVISOR_H

#define CAPACITY_LIMIT 500;

class MetricSupervisor {

private:
    std::thread m_thread;
    uint64_t m_time_window;
    bool m_enable_CAM_dissemination;
    CABasicService *m_cabs;
    bool m_enable_CPM_dissemination;
    CPBasicService *m_cpbs;
    bool m_enable_VAM_dissemination;
    VRUBasicService *m_vrubs;
    SocketClient *m_sock_client;
    std::string m_log_filename;
public:

MetricSupervisor(std::string log_filename, uint64_t time_window_ms, bool enable_CAM_dissemination, bool enable_CPM_dissemination, bool enable_VAM_dissemination, CABasicService *cabs, CPBasicService *cpbs, VRUBasicService *vrub, SocketClient *sockClient);
~MetricSupervisor();
void writeLogFile();
void start();

};

#endif