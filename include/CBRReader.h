#include <string>
#include <netlink/netlink.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/ctrl.h>
#include <linux/nl80211.h>
#include <iostream>
#include <net/if.h>
#include <errno.h>
#include <netlink/genl/family.h>
#include <netlink/msg.h>
#include <netlink/attr.h>
#include <thread>
#include "utils.h"

#ifndef OSCAR_CBRREADER_H
#define OSCAR_CBRREADER_H

int cbr_handler(struct nl_msg *msg, void *arg);

class CBRReader {
private:

typedef struct CBRData {
    bool firstTime = true;
    bool verbose = false;
    unsigned long long startActiveTime;
    unsigned long long startBusyTime;
    unsigned long long startReceiveTime;
    unsigned long long startTransmitTime;
    float currentCBR = -1.0f;
    float currentTxTime = -1.0f;
    float currentRxTime = -1.0f;
    float currentBusyTime = -1.0f;
} CBRData;

void start_reading_cbr(nl_sock_info_t m_nl_sock_info);
void read_cbr_from_netlink(nl_sock_info_t nl_sock_info);

std::string m_dissemination_interface;
std::thread m_cbr_thread;

public:

CBRReader();
~CBRReader();

void setupCBRReader(bool verbose, std::string dissemination_interface);
float get_current_cbr();
float get_current_busy_time();
float get_current_tx_time();
float get_current_rx_time();
void wrapper_start_reading_cbr();

CBRData m_cbr_data;

};

#endif // OSCAR_CBRREADER_H
