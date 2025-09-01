#include <string>
#include "utils.h"
#include "caBasicService.h"
#include "cpBasicService.h"
#include "VRUBasicService.h"
#include "CBRReader.h"

#ifndef OSCAR_DCC_H
#define OSCAR_DCC_H

#define MAXIMUM_TIME_WINDOW_DCC 2000

class DCC {
public:

DCC();
~DCC();

void setupDCC(unsigned long long dcc_interval, std::string modality, std::string dissemination_interface, float tolerance=0.01, bool verbose=false, std::string log_file="");
void startDCC();
void addCaBasicService(CABasicService* caBasicService);
void addCpBasicService(CPBasicService* cpBasicService);
void addVruBasicService(VRUBasicService* vruBasicService);
void reactiveDCC();
void adaptiveDCC();

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
{Active1,     {0.4, 18.0, -1, 200,  -95.0}},
{Active2,     {0.5, 12.0, -1, 400, -95.0}},
{Active3,     {0.6, 6.0, -1, 500, -95.0}},
{Restrictive, {1.0, 2.0, -1, 1000, -65.0}}
};

const std::unordered_map<ReactiveState, ReactiveParameters> m_reactive_parameters_Ton_500_us = 
{
{Relaxed,     {0.3, 24.0, -1, 50, -95.0}},
{Active1,     {0.4, 18.0, -1, 100,  -95.0}},
{Active2,     {0.5, 12.0, -1, 200, -95.0}},
{Active3,     {0.65, 6.0, -1, 250, -95.0}},
{Restrictive, {1.0, 2.0, -1, 1000, -65.0}}
};

void functionReactive();
void functionAdaptive();
std::unordered_map<DCC::ReactiveState, DCC::ReactiveParameters> getConfiguration(double Ton, double currentCBR);
void adaptiveDCCCheckCBR();

unsigned long long m_dcc_interval = 0;
std::string m_dissemination_interface;
bool m_verbose;
float m_tolerance;
std::string m_modality;
CABasicService* m_caBasicService = nullptr;
CPBasicService* m_cpBasicService = nullptr;
VRUBasicService* m_vruBasicService = nullptr;

double m_alpha = 0.016;
double m_beta = 0.0012;
double m_CBR_target = 0.68;
double m_delta_max = 0.03;
double m_delta_min = 0.0006;
double m_Gmax = 0.0005;
double m_Gmin = -0.00025;
double m_delta = 0;
uint32_t m_T_CBR = 100; // Check the CBR value each 100 ms for Adaptive DCC from standard suggestion
std::mutex m_previous_cbr_mutex;
double m_previous_cbr = -1.0f;

ReactiveState m_current_state = ReactiveState::Relaxed;

float m_CBR_its = -1.0f;

std::thread m_reactive_thread;
std::thread m_adaptive_thread;
std::thread m_check_cbr_thread;

bool m_cam_enabled;
bool m_cpm_enabled;
bool m_vam_enabled;

std::string m_log_file;
CBRReader m_cbr_reader;

};

#endif // OSCAR_DCC_H
