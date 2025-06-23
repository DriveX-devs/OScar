#include <string>
#include "utils.h"
#include "caBasicService.h"
#include "cpBasicService.h"
#include "VRUBasicService.h"

#ifndef OSCAR_DCC_H
#define OSCAR_DCC_H

class DCC {
public:
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
    long tx_inter_packet_time;
    double sensitivity;
} ReactiveParameters;

DCC();
~DCC();

void setupDCC(unsigned long long dcc_interval, std::string dissemination_interface, CABasicService* cabs, CPBasicService* cpbs, VRUBasicService* vrubs, bool enable_CAM_dissemination, bool enable_CPM_dissemination, bool enable_VAM_dissemination, float tolerance=0.01, bool verbose=false);
void reactiveDCC();
void functionReactive();
void adaptiveDCC();
void functionAdaptive();

private:

unsigned long long m_dcc_interval = 0;
std::string m_dissemination_interface;
bool m_verbose;
float m_tolerance;
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

std::unordered_map <ReactiveState, ReactiveParameters> m_parameters_map;

ReactiveState m_current_state = ReactiveState::Relaxed;

float m_previous_cbr = 0.0f;
float m_CBR_its = -1.0f;

std::thread m_reactive_thread;
std::thread m_adaptive_thread;
std::thread m_cbr_thread;

bool m_cam_enabled;
bool m_cpm_enabled;
bool m_vam_enabled;

};

#endif // OSCAR_DCC_H
