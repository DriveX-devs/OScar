#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <functional>
#include <condition_variable>
#include "CBRReader.h"
#include "utils.h"
#include "basicHeader.h"
#include "commonHeader.h"
#include "MessageId.h"
#include "MetricSupervisor.h"

#ifndef OSCAR_DCC_H
#define OSCAR_DCC_H

#define MAXIMUM_TIME_WINDOW_DCC 2000

typedef struct Packet {
    int64_t time;
    basicHeader bh;
    commonHeader ch;
    GNlpv_t long_PV;
    GNDataRequest_t dataRequest;
    MessageId_t message_id;
} Packet;

class DCC {
public:

DCC();
~DCC();

void setupDCC(unsigned long long dcc_interval, std::string modality, std::string dissemination_interface, float cbr_target, float tolerance=0.01, bool verbose=false, int queue_length=0, int max_lifetime=100, std::string log_file="", std::string profile_DCC="etsi");
void startDCC();
void reactiveDCC();
void adaptiveDCC();
float getTonpp();
void updateTgoAfterStateCheck(uint32_t Toff);
void updateTonpp(ssize_t pktSize);
bool checkGateOpen(int64_t now);
void updateTgoAfterDeltaUpdate();
void updateTgoAfterTransmission();
std::string getModality() {return m_modality;}
void setBitRate(long bitrate) {m_bitrate_bps = bitrate;}
void updateDelta(float delta) {m_gate_mutex.lock(); m_delta = delta; m_gate_mutex.unlock();}
float getDelta() {float delta; m_gate_mutex.lock(); delta = m_delta; m_gate_mutex.unlock(); return delta;}
void cleanQueues(int now);
void enqueue(int priority, Packet p);
std::tuple<bool, Packet> dequeue(int priority);
void setLastTx(int64_t t) {m_gate_mutex.lock(); m_last_tx = static_cast<double>(t); m_gate_mutex.unlock();}
void setSendCallback(std::function<void(const Packet&)> cb);
void setCBRGCallback(std::function<void()> cb);
void setMetricSupervisor(MetricSupervisor *met_sup_ptr) {m_met_sup_ptr = met_sup_ptr;}
void metricSupervisorSignalSentPacket(MessageId_t message_id) {m_met_sup_ptr->signalSentPacket(message_id);}
double getCBRTarget() const {return m_CBR_target;};
void setCBRG(double cbr_g);
void setNewCBRL0Hop (double cbr);
double getCBRR0 () {double cbr; m_cbr_g_mutex.lock(); cbr = m_CBR_L0_Hop[0]; m_cbr_g_mutex.unlock(); return cbr;}
double getCBRL0Prev () {double cbr; m_cbr_g_mutex.lock(); cbr = m_CBR_L0_Hop[1]; m_cbr_g_mutex.unlock(); return cbr;}
void setCBRL1 (double cbr_r1) {m_cbr_g_mutex.lock(); m_CBR_L1_Hop = cbr_r1; m_cbr_g_mutex.unlock();};
void setCBRL2 (double cbr_r2) {m_cbr_g_mutex.lock(); m_CBR_L2_Hop = cbr_r2; m_cbr_g_mutex.unlock();};
double getCBRR1 () {double cbr; m_cbr_g_mutex.lock(); cbr = m_CBR_L1_Hop; m_cbr_g_mutex.unlock(); return cbr;};
double getCBRR2 () {double cbr; m_cbr_g_mutex.lock(); cbr = m_CBR_L2_Hop; m_cbr_g_mutex.unlock(); return cbr;};
void updateAoI (int priority, int64_t time);
void updatePktToSend() {m_gate_mutex.lock(); m_pkt_to_send ++; m_gate_mutex.unlock();};

private:

enum ReactiveState {
    Relaxed,
    Active1,
    Active2,
    Active3,
    Restrictive
};

typedef struct ReactiveParameters {
    double cbr_threshold;
    double tx_power;
    double data_rate;
    long tx_inter_packet_time;
    double sensitivity;
} ReactiveParameters;

const std::unordered_map<ReactiveState, ReactiveParameters> m_reactive_parameters_Ton_1ms = 
{
{Relaxed,     {0.3, 24.0, -1, 100, -95.0}},
{Active1,     {0.4, 24.0, -1, 200,  -95.0}},
{Active2,     {0.5, 24.0, -1, 400, -95.0}},
{Active3,     {0.6, 24.0, -1, 500, -95.0}},
{Restrictive, {1.0, 10.0, -1, 1000, -65.0}}
};

const std::unordered_map<ReactiveState, ReactiveParameters> m_reactive_parameters_Ton_500_us = 
{
{Relaxed,     {0.3, 24.0, -1, 50, -95.0}},
{Active1,     {0.4, 24.0, -1, 100,  -95.0}},
{Active2,     {0.5, 24.0, -1, 200, -95.0}},
{Active3,     {0.65, 10.0, -1, 250, -95.0}},
{Restrictive, {1.0, 5.0, -1, 1000, -65.0}}
};

void functionReactive();
void functionAdaptive();
std::unordered_map<DCC::ReactiveState, DCC::ReactiveParameters> getConfiguration(double Ton, double currentCBR);
void adaptiveDCCCheckCBR();
void DCCcheckCBRG();
void checkQueue();

unsigned long long m_dcc_interval = 0;
std::string m_dissemination_interface;
bool m_verbose;
float m_tolerance;
std::string m_modality = "";
std::string m_profile;

double m_alpha = 0.016;
double m_beta = 0.0012;
//double m_CBR_target = 0.68;
// double m_CBR_target = 0.2;
double m_CBR_target;
double m_delta_max = 0.03;
double m_delta_min = 0.0006;
double m_Gmax = 0.0005;
double m_Gmin = -0.00025;
uint32_t m_T_CBR = 100; // Check the CBR value each 100 ms for Adaptive DCC from standard suggestion
std::mutex m_read_cbr_mutex;

ReactiveState m_current_state = ReactiveState::Relaxed;

double m_CBR_its = -1.0f;

std::thread m_reactive_thread;
std::thread m_adaptive_thread;
std::thread m_check_cbr_thread;

bool m_cam_enabled;
bool m_cpm_enabled;
bool m_vam_enabled;

std::string m_log_file;
CBRReader m_main_cbr_reader;
CBRReader m_second_cbr_reader;

std::mutex m_gate_mutex;
double m_Tpg_ms = 0.0;
double m_Tgo_ms = 0.0;
float m_Ton_pp = 0.5;
double m_Toff_ms = 0.0;
double m_last_tx = 0.0;
float m_delta = 0;
long m_bitrate_bps;
std::string m_dcc = "";
float m_cbr = 0.0;
uint8_t m_queue_length;
long m_lifetime; // ms
struct GNDataIndication_t; // forward declaration to avoid circular import with geonet.h

std::vector<Packet> m_dcc_queue_dp0;
uint64_t m_packet_sent_dp0 = 0;
uint64_t m_cumulative_time_dp0 = 0;
std::vector<Packet> m_dcc_queue_dp1;
uint64_t m_packet_sent_dp1 = 0;
uint64_t m_cumulative_time_dp1 = 0;
std::vector<Packet> m_dcc_queue_dp2;
uint64_t m_packet_sent_dp2 = 0;
uint64_t m_cumulative_time_dp2 = 0;
std::vector<Packet> m_dcc_queue_dp3;
uint64_t m_packet_sent_dp3 = 0;
uint64_t m_cumulative_time_dp3 = 0;

bool m_first_send = true;
bool m_stop_thread = false;
std::thread m_check_queue_thread;
std::mutex m_check_queue_mutex;
std::condition_variable m_check_queue_cv;
std::function<void(const Packet&)> m_send_callback;
std::function<void()> m_cbr_g_callback;
MetricSupervisor *m_met_sup_ptr = nullptr;
uint32_t m_dropped_full_queue = 0;
uint32_t m_dropped_lifetime = 0;
uint32_t m_pkt_to_send = 0;

std::thread m_cbr_g_thread;
std::mutex m_cbr_g_mutex;
uint8_t m_T_DCC_NET_Trig = 100;
std::vector<double> m_CBR_G = {-1, -1};
std::vector<double> m_CBR_L0_Hop = {0, 0};
double m_CBR_L1_Hop = 0.0;
double m_CBR_L2_Hop = 0.0;

double m_current_cbr;

};

#endif // OSCAR_DCC_H
