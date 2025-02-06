#include "utils.h"
#include <unistd.h>
#include <cmath>
#include <cstdarg>
#include <cfloat>
#include <vehicleDataDef.h>
#include <cstring>
#include <netlink/netlink.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/ctrl.h>
#include <linux/nl80211.h>
#include <iostream>
#include <net/if.h>

// Epoch time at 2004-01-01 (in ms)
#define TIME_SHIFT_MILLI 1072915200000

// Size of the popen iw buffer, to read the RSSI given a MAC address
// This is currently a bit oversized -> need to define a "more tight" size in the near future
#define POPEN_IW_BUFF_SIZE 2000

uint64_t get_timestamp_us(void) {
	time_t seconds;
	uint64_t microseconds;
	struct timespec now;

	if(clock_gettime(CLOCK_MONOTONIC, &now) == -1) {
		perror("Cannot get the current microseconds UTC timestamp");
		return -1;
	}

	seconds=now.tv_sec;
	microseconds=round(now.tv_nsec/1e3);

	// milliseconds, due to the rounding operation, shall not exceed 999999
	if(microseconds > 999999) {
		seconds++;
		microseconds=0;
	}

	return seconds*1000000+microseconds;
}

uint64_t get_timestamp_ns(void) {
	struct timespec now;

	if(clock_gettime(CLOCK_MONOTONIC, &now) == -1) {
		perror("Cannot get the current nanosecond UTC timestamp");
		return -1;
	}

	return now.tv_sec*1000000000+now.tv_nsec;
}

uint64_t get_timestamp_ms_gn(void) {
	time_t seconds;
	uint64_t microseconds;
	struct timespec now;

	if(clock_gettime(CLOCK_TAI, &now) == -1) {
		perror("Cannot get the current microseconds TAI timestamp");
		return -1;
	}

	seconds=now.tv_sec;
	microseconds=round(now.tv_nsec/1e3);

	// milliseconds, due to the rounding operation, shall not exceed 999999
	if(microseconds > 999999) {
		seconds++;
		microseconds=0;
	}

	return (static_cast<uint64_t>(floor((seconds*1000000+microseconds)/1000.0))-TIME_SHIFT_MILLI)%4294967296;
}

uint64_t get_timestamp_ms_cam(void) {
	return (static_cast<uint64_t>(floor(get_timestamp_us()/1000.0))-TIME_SHIFT_MILLI)%65536;
}

int timer_fd_create(struct pollfd &pollfd,int &clockFd,uint64_t time_us) {
	struct itimerspec new_value;
	time_t sec;
	long nanosec;

	// Create monotonic (increasing) timer
	clockFd=timerfd_create(CLOCK_MONOTONIC,0);
	if(clockFd==-1) {
		return -1;
	}

	// Convert time, in us, to seconds and nanoseconds
	sec=(time_t) ((time_us)/1000000);
	nanosec=1000*time_us-sec*1000000000;
	new_value.it_value.tv_nsec=nanosec;
	new_value.it_value.tv_sec=sec;
	new_value.it_interval.tv_nsec=nanosec;
	new_value.it_interval.tv_sec=sec;

	// Fill pollfd structure
	pollfd.fd=clockFd;
	pollfd.revents=0;
	pollfd.events=POLLIN;

	// Start timer
	if(timerfd_settime(clockFd,0,&new_value,NULL)==-1) {
		close(clockFd);
		return -2;
	}

	return 0;
}

std::string exteriorLights_bit_to_string(uint8_t extLights) {
	std::string extLightsStr="";
	const char *bitnames[]={"lowBeamHeadlightsOn", "highBeamHeadlightsOn", "leftTurnSignalOn", 
							"rightTurnSignalOn", "daytimeRunningLightsOn", "reverseLightOn", 
							"fogLightOn", "parkingLightsOn"};

	for(int i=0;i<8;i++) {
		if(extLights & (1 << (7 - i))) {
			if(extLightsStr.length()!=0) {
				extLightsStr += ",";
			}
			extLightsStr += bitnames[i];
		}
	}

	if(extLightsStr=="") {
		extLightsStr="off";
	}

	return extLightsStr;
}


bool doublecomp(double d1, double d2, double eps) {
	return std::abs(d1-d2) < eps;
}

int logfprintf(FILE *stream,std::string modulename,const char *format,...) {
	va_list arg;
	int retval;

	std::time_t now = std::time(nullptr);

	va_start(arg,format);
	fprintf(stream,"[LOG - %s] (%.24s) ",modulename.c_str(),std::ctime(&now));
	retval=vfprintf(stream,format,arg);
	va_end(arg);

	return retval;
}

nl_sock_info_t open_nl_socket(std::string interface_name) {
    nl_sock_info_t nl_sock_info;
    nl_sock_info.nl_sock = nl_socket_alloc();

    if (!nl_sock_info.nl_sock) {
        std::cerr << "[WARN] Failed to allocate netlink socket for RSSI retrieval!" << std::endl;
        nl_sock_info.sock_valid=false;
        return nl_sock_info;
    }

    if (genl_connect(nl_sock_info.nl_sock)) {
        std::cerr << "[WARN] Failed to connect to generic netlink for RSSI retrieval!" << std::endl;
        nl_socket_free(nl_sock_info.nl_sock);
        nl_sock_info.sock_valid=false;
        return nl_sock_info;
    }

    nl_sock_info.nl80211_id = genl_ctrl_resolve(nl_sock_info.nl_sock,"nl80211");
    if (nl_sock_info.nl80211_id < 0) {
        std::cerr << "[WARN] No nl80211 interface found!" << std::endl;
        nl_socket_free(nl_sock_info.nl_sock);
        nl_sock_info.sock_valid=false;
        return nl_sock_info;
    }

    nl_sock_info.ifindex=if_nametoindex(interface_name.c_str());
    if (nl_sock_info.ifindex == 0) {
        std::cerr << "[WARN] Cannot get ifindex for interface " << interface_name << ". No RSSI retrieval will occur." << std::endl;
        nl_socket_free(nl_sock_info.nl_sock);
        nl_sock_info.sock_valid=false;

        return nl_sock_info;
    }

    nl_sock_info.sock_valid=true;

    return nl_sock_info;
}

void free_nl_socket(nl_sock_info_t nl_sock_info) {
    if(nl_sock_info.sock_valid) {
        nl_socket_free(nl_sock_info.nl_sock);
    }
}

double get_rssi_from_netlink(uint8_t macaddr[6],nl_sock_info_t nl_sock_info) {
    struct nl_cb_args {
        int signal_lv;
        uint8_t macaddr[6];
    } nl_cb_args;

    nl_cb_args.signal_lv=RSSI_UNAVAILABLE;
    std::copy(macaddr,macaddr+6,nl_cb_args.macaddr);

    if(!nl_sock_info.sock_valid || nl_sock_info.ifindex==0) {
        return nl_cb_args.signal_lv;
    }

    struct nl_msg *nl_msg=nlmsg_alloc();
    if(nl_msg) {
        genlmsg_put(nl_msg,0,0,nl_sock_info.nl80211_id,0,NLM_F_DUMP,NL80211_CMD_GET_STATION,0);
        nla_put_u32(nl_msg,NL80211_ATTR_IFINDEX,nl_sock_info.ifindex);

        nl_socket_modify_cb(nl_sock_info.nl_sock,NL_CB_VALID,NL_CB_CUSTOM,
            [] (struct nl_msg *msg, void *arg) -> int {
                struct nl_cb_args *nl_cb_args_ptr = static_cast<struct nl_cb_args *>(arg);
                struct nlattr *attrs[NL80211_ATTR_MAX + 1];
                struct genlmsghdr *gnlh = (struct genlmsghdr *) nlmsg_data(nlmsg_hdr(msg));

                nla_parse(attrs,NL80211_ATTR_MAX,genlmsg_attrdata(gnlh, 0), genlmsg_attrlen(gnlh, 0), nullptr);

                if(attrs[NL80211_ATTR_MAC]) {
                    // Skip the RSSI retrieval if the RSSI is not the one of interest, passed as "macaddr"
                    uint8_t *rssi_mac = static_cast<uint8_t *>(nla_data(attrs[NL80211_ATTR_MAC]));

                    char mac_str[18];
                    char rssi_mac_str[18];
                    std::snprintf(mac_str,18,"%02X:%02X:%02X:%02X:%02X:%02X",
                                  nl_cb_args_ptr->macaddr[0],
                                  nl_cb_args_ptr->macaddr[1],
                                  nl_cb_args_ptr->macaddr[2],
                                  nl_cb_args_ptr->macaddr[3],
                                  nl_cb_args_ptr->macaddr[4],
                                  nl_cb_args_ptr->macaddr[5]);

                    std::snprintf(rssi_mac_str,18,"%02X:%02X:%02X:%02X:%02X:%02X",
                                    rssi_mac[0],
                                    rssi_mac[1],
                                    rssi_mac[2],
                                    rssi_mac[3],
                                    rssi_mac[4],
                                    rssi_mac[5]);

                    if(std::string(rssi_mac_str)!=std::string(mac_str)) {
                        return NL_SKIP;
                    }
                } else {
                    // Skip the RSSI retrieval if the MAC address of the remote device cannot be retrieved with nl80211
                    return NL_SKIP;
                }

                // Retrieve the signal level (RSSI)
                if(attrs[NL80211_ATTR_STA_INFO]) {
                    struct nlattr *sinfo[NL80211_STA_INFO_MAX + 1];

                    nla_parse(sinfo,NL80211_STA_INFO_MAX,(struct nlattr *) nla_data(attrs[NL80211_ATTR_STA_INFO]),nla_len(attrs[NL80211_ATTR_STA_INFO]),nullptr);

                    if(sinfo[NL80211_STA_INFO_SIGNAL]) {
                        nl_cb_args_ptr->signal_lv = static_cast<int>(static_cast<int8_t>(nla_get_u8(sinfo[NL80211_STA_INFO_SIGNAL])));
                    }
                }
                return NL_SKIP;
            },
            static_cast<void *>(&nl_cb_args));

        nl_send_auto_complete(nl_sock_info.nl_sock, nl_msg);
        nl_recvmsgs_default(nl_sock_info.nl_sock);
    }

    return static_cast<double>(nl_cb_args.signal_lv);
}

double get_rssi_from_iw(uint8_t macaddr[6],std::string interface_name) {
	// ioctl() does not seem to work for some drivers, which are instead using nl80211
	// The following code is a "quick and dirty" way of getting the RSSI, leveraging popen and calling the iw tool
	// If you want to use nl80211, use instead the get_rssi_from_netlink() function
	// struct iw_statistics wifistats;
	// struct iwreq wifireq;
	// strncpy(wifireq.ifr_name,options_string_pop(m_opts_ptr->udp_interface),IFNAMSIZ);
	// wifireq.u.data.pointer=&wifistats;
	// wifireq.u.data.length = sizeof(wifistats);

	// if(ioctl(m_raw_rx_sock,SIOCGIWSTATS,&wifireq) == -1 || !(wifistats.qual.updated & IW_QUAL_DBM)) {
	// 	vehdata.rssi_dBm=RSSI_UNAVAILABLE;
	// } else {
	// 	vehdata.rssi_dBm=wifistats.qual.level;
	// }
	double signal_lv;
	char popen_iw_buff[POPEN_IW_BUFF_SIZE];

	std::string ssh_command = "stdbuf -o L iw dev " + interface_name + " station dump | stdbuf -o L grep -E \"signal:|Station\" | tr -d '\t' | tr -s ' '";
		FILE *ssh = popen(ssh_command.c_str(),"r");

		signal_lv=RSSI_UNAVAILABLE;
		
		while(ssh!=nullptr && fgets(popen_iw_buff,POPEN_IW_BUFF_SIZE,ssh)!=NULL) {
			char* pch=nullptr;
			pch=strtok(popen_iw_buff," ,");
			if(strstr(pch,"Station")) {
				char mac[18];
				std::snprintf(mac,18,"%02X:%02X:%02X:%02X:%02X:%02X",
					macaddr[0],
					macaddr[1],
					macaddr[2],
					macaddr[3],
					macaddr[4],
					macaddr[5]);

				pch=strtok(NULL," ,");
				
				if(std::string(pch)!=std::string(mac)) {
					// Skip the RSSI/"signal:" line if the MAC is not the one of interest
					pch=strtok(NULL," ,");
				}
			} else if(strstr(pch,"signal:")) {
				pch=strtok(NULL," ,");

				// This is the desired RSSI/"signal" value
				signal_lv=strtod(pch,nullptr);
				
				break;
			}
		}

		return signal_lv;
}