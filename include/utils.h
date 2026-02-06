#ifndef OSCAR_UTILS_H
#define OSCAR_UTILS_H

#include <cinttypes>
#include <ctime>
#include <cmath>
#include <cstdio>
#include <poll.h>
#include <sys/timerfd.h>
#include <string>
#include <unistd.h>
#include <shared_mutex>
#include <chrono>
#include <thread>
#include <unordered_map>

#define POLL_DEFINE_JUNK_VARIABLE() long int junk
#define POLL_CLEAR_EVENT(clockFd) junk=read(clockFd,&junk,sizeof(junk))

typedef struct {
    struct nl_sock *nl_sock;
    unsigned int ifindex;
    int nl80211_id;
    bool sock_valid;
} nl_sock_info_t;

uint64_t get_timestamp_us(void);
uint64_t get_timestamp_us_realtime(void);
uint64_t get_timestamp_ns(void);
uint64_t get_timestamp_ms_gn(void);
uint64_t get_timestamp_ms_cam(void);
int timer_fd_create(struct pollfd &pollfd,int &clockFd,uint64_t time_us);
std::string exteriorLights_bit_to_string(uint8_t extLights);
bool doublecomp(double d1, double d2, double eps = 0.0001);
// logfprintf is an alternative to fprintf for logging purposes
// It works just like fprintf, by printing:
// "[LOG - <modulename>] (<current date and time>) <fprintf content>"
// As it is retrieving the time for each call to logfprintf() it is expected 
// be slightly slower than fprintf and should thus be used only when really needed
int logfprintf(FILE *stream,std::string modulename,const char *format,...);
double get_rssi_from_iw(uint8_t macaddr[6],std::string interface_name);
// This function returns a TimestampIts (as defined in ETSI-ITS-CDD.asn) for the referenceTime field in CPMs v2
uint64_t get_timestamp_ms_cpm(void);

nl_sock_info_t open_nl_socket(std::string interface_name);
void free_nl_socket(nl_sock_info_t nl_sock_info);
double get_rssi_from_netlink(uint8_t macaddr[6],nl_sock_info_t nl_sock_info);
std::unordered_map<std::string,double> get_all_rssi_from_netlink(nl_sock_info_t nl_sock_info);
void setNewTxPower(double txPower, std::string dissemination_interface);
uint32_t getTxPower();
// Function to convert a double to a string, with a given number of digits after the comma ("precision", by default 7)
std::string doubleToString(double value, int precision = 7);
#endif // OSCAR_UTILS_H