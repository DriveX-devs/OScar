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
#include <errno.h>
#include <netlink/genl/family.h>
#include <netlink/msg.h>
#include <netlink/attr.h>

#include <linux/nl80211.h>

// Epoch time at 2004-01-01 (in ms)
#define TIME_SHIFT_MILLI 1072915200000

// UTC epoch time at 2004-01-01 00:00:00 (in ms)
#define ITS_EPOCH_TIME_MILLI TIME_SHIFT_MILLI // Equal to the TIME_SHIFT_MILLI; can be updated to a different value for future CPM versions, if needed

// Size of the popen iw buffer, to read the RSSI given a MAC address
// This is currently a bit oversized -> need to define a "more tight" size in the near future
#define POPEN_IW_BUFF_SIZE 2000

CBRUpdates cbrData;

std::mutex cbrMutex;

float currentRssiUtils = -1.0f;

std::mutex rssiMutex;

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

uint64_t get_timestamp_ms_cpm(void) {
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

    return (static_cast<uint64_t>(floor((seconds*1000000+microseconds)/1000.0))-ITS_EPOCH_TIME_MILLI)%4398046511103;
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

    // Allocate a new netlink socket (that should be freed up later)
    nl_sock_info.nl_sock = nl_socket_alloc();

    // If the socket allocation failed, do not perform any further step,
    // and return a structure with a flag indicating the socket is invalid and shall not be used
    if (!nl_sock_info.nl_sock) {
        std::cerr << "[WARN] Failed to allocate netlink socket for RSSI retrieval!" << std::endl;
        nl_sock_info.sock_valid=false;
        return nl_sock_info;
    }

    // Connect to the generic netlink socket
    // Optionally, the socket buffer size can be set with:
    // nl_socket_set_buffer_size(nl_sock_info.nl_sock,<rx buf size>,<tx buf size>);
    // Here, however, it is not necessary
    if (genl_connect(nl_sock_info.nl_sock)) {
        std::cerr << "[WARN] Failed to connect to generic netlink for RSSI retrieval!" << std::endl;
        nl_socket_free(nl_sock_info.nl_sock);
        nl_sock_info.sock_valid=false;
        return nl_sock_info;
    }

    // Resolve the family name "nl80211" to the corresponding kernel ID
    nl_sock_info.nl80211_id = genl_ctrl_resolve(nl_sock_info.nl_sock,"nl80211");
    if (nl_sock_info.nl80211_id < 0) {
        std::cerr << "[WARN] No nl80211 interface found!" << std::endl;
        nl_socket_free(nl_sock_info.nl_sock);
        nl_sock_info.sock_valid=false;
        return nl_sock_info;
    }

    // Retrieve the interface index for the given network interface name
    nl_sock_info.ifindex=if_nametoindex(interface_name.c_str());
    if (nl_sock_info.ifindex == 0) {
        std::cerr << "[WARN] Cannot get ifindex for interface " << interface_name << ". No RSSI retrieval will occur." << std::endl;
        nl_socket_free(nl_sock_info.nl_sock);
        nl_sock_info.sock_valid=false;

        return nl_sock_info;
    }

    // If we reach this point, everything went well with the creation of the Netlink socket
    nl_sock_info.sock_valid=true;

    return nl_sock_info;
}

void free_nl_socket(nl_sock_info_t nl_sock_info) {
    // Free the Netlink socket, if a nl_sock_info_t structure pointing to a valid socket is passed
    if(nl_sock_info.sock_valid) {
        nl_socket_free(nl_sock_info.nl_sock);
        // Now the socket is no more valid and shall no more be used
        nl_sock_info.sock_valid=false;
    }
}

double get_rssi_from_netlink(uint8_t macaddr[6],nl_sock_info_t nl_sock_info) {
    // Structure to store the Netlink callback arguments
    struct nl_cb_args {
        int signal_lv; // Retrieved RSSI
        uint8_t macaddr[6]; // Target MAC address (only the RSSI of the device with this MAC will be retrieved)
    } nl_cb_args;

    // Set the default RSSI value to "RSSI_UNAVAILABLE" and the macaddr field to the one of interest, passed as first argument
    nl_cb_args.signal_lv=RSSI_UNAVAILABLE;
    std::copy(macaddr,macaddr+6,nl_cb_args.macaddr);

    // Avoid doing any operation and leave the RSSI as "RSSI_UNAVAILABLE" if the Netlink socket passed as second argument
    // is not valid or could not be opened
    if(!nl_sock_info.sock_valid || nl_sock_info.ifindex==0) {
        return nl_cb_args.signal_lv;
    }

    // Allocate a new netlink message
    struct nl_msg *nl_msg=nlmsg_alloc();

    // If allocation is not successful, leave the RSSI as "RSSI_UNAVAILABLE" and do not perform any further operation
    // If it is successful, prepare a netlink message to request station information with genlmsg_put() and nla_put_u32()
    if(nl_msg) {
        // We set here:
        // 1. Pointer to the Netlink message buffer
        // 2.3. A port ID and sequence number that are managed automatically
        // 4. The Netlink family ID (nl80211), as we want to retrieve information about a wireless 802.11 device
        // 5. Length of user headers (there are no additional user headers here)
        // 6. Additional Netlink message flags (NLM_F_DUMP is used to request a list of all devices)
        // 7. The Netlink command: NL80211_CMD_GET_STATION gets station attributes for stations identified by
        //    NL80211_ATTR_MAC on the interface identified by NL80211_ATTR_IFINDEX; as we use here NLM_F_DUMP,
        //    in theory we do not need to specify a MAC and add the attribute NL80211_ATTR_MAC
        // 8. The interface version, set here to the default value of 0
        genlmsg_put(nl_msg,NL_AUTO_PORT,NL_AUTO_SEQ,nl_sock_info.nl80211_id,0,NLM_F_DUMP,NL80211_CMD_GET_STATION,0);
        // Add a 32-bit integer attribute to netlink message, i.e., the ifindex of the target interface, as required by NL80211_CMD_GET_STATION
        nla_put_u32(nl_msg,NL80211_ATTR_IFINDEX,nl_sock_info.ifindex);

        // Set a custom callback handler (NL_CB_CUSTOM) to process the response from the kernel, when a valid message is received (NL_CB_VALID)
        // Instead of defining a callback and passing the pointer to the callback as fourth argument, we define it as a lambda without any capture
        // Pass as arguments a pointer to the nl_cb_args structure (casted to void * as required by nl_socket_modify_cb())
        nl_socket_modify_cb(nl_sock_info.nl_sock,NL_CB_VALID,NL_CB_CUSTOM,
            [] (struct nl_msg *msg, void *arg) -> int {
                // Get the arguments casting again to struct nl_cb_args * from void *
                struct nl_cb_args *nl_cb_args_ptr = static_cast<struct nl_cb_args *>(arg);

                // Extract the header and attributes from the netlink message
                struct nlattr *attrs[NL80211_ATTR_MAX + 1];
                // nlmsg_hdr() returns the actual netlink message casted to the type of the netlink message header
                // nlmsg_data() returns a pointer to the payload of the netlink message given the message header
                struct genlmsghdr *gnlh = (struct genlmsghdr *) nlmsg_data(nlmsg_hdr(msg));
                // NL80211_ATTR_MAX: highest attribute number currently defined
                // This function Iterates over the stream of attributes received via Netlink and stores a pointer to each
                // attribute in the index array specified as first argument (attrs) using the attribute type as index to the array
                // genlmsg_attrdata() returns a pointer to the message attributes, while genlmsg_attrlen() returns the length of message attributes
                // The second argument of both functions is the length of user headers, that are not present here
                // With genlmsg_attrdata(gnlh, 0) we get therefore the head of the head of the attribute stream, and with
                // genlmsg_attrlen(gnlh, 0) the length of the attribute stream
                // The last argument is a possible policy for validating the attributes, that is not set here
                // Tip about nla_parse: attributes with a type greater than the maximum type specified will be silently
                // ignored in order to maintain backwards compatibility
                nla_parse(attrs,NL80211_ATTR_MAX,genlmsg_attrdata(gnlh, 0), genlmsg_attrlen(gnlh, 0), nullptr);

                // Check if the MAC attribute is available; if not, no RSSI will be retrieved
                if(attrs[NL80211_ATTR_MAC]) {
                    // Skip the RSSI retrieval if the RSSI is not the one of interest, passed as "macaddr"
                    uint8_t *rssi_mac = static_cast<uint8_t *>(nla_data(attrs[NL80211_ATTR_MAC]));

                    char mac_str[18];
                    char rssi_mac_str[18];
                    // Convert the target MAC and the received MAC to strings, both with uppercase hex digits
                    // (using uppercase hex in both MAC address is needed for the comparison)
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

                // Retrieve the signal level (RSSI), checking if the station information attribute is present
                if(attrs[NL80211_ATTR_STA_INFO]) {
                    // Get the nested attributes using again nla_parse()
                    // NL80211_STA_INFO_MAX: highest possible station information attribute
                    struct nlattr *sinfo[NL80211_STA_INFO_MAX + 1];

                    // Here we use nla_data to get a pointer to the actual data of the (nested) attribute stream
                    // Since the data inside struct nlattr is stored after the attribute header, we use nla_data() to get a pointer to the actual data
                    nla_parse(sinfo,NL80211_STA_INFO_MAX,(struct nlattr *) nla_data(attrs[NL80211_ATTR_STA_INFO]),nla_len(attrs[NL80211_ATTR_STA_INFO]),nullptr);

                    if(sinfo[NL80211_STA_INFO_SIGNAL]) {
                        // We get here some bytes that represent an 8-bit signed integer: we therefore need to convert the result of nla_get_u8() to int8_t
                        nl_cb_args_ptr->signal_lv = static_cast<int>(static_cast<int8_t>(nla_get_u8(sinfo[NL80211_STA_INFO_SIGNAL])));
                    }
                }
                return NL_SKIP;
            },
            static_cast<void *>(&nl_cb_args));

        // Finalize and send the netlink message to the kernel
        nl_send_auto_complete(nl_sock_info.nl_sock, nl_msg);

        // Receive the response from the ntlink socket using the callback handler in nl_sock, as previously set
        // As per official documentation, nl_recvmsgs_default() will call nl_recvmsgs()
        // nl_recvmsgs() repeatedly calls nl_recv() and parses the received data as netlink messages;
        // it stops reading if one of the callbacks returns NL_STOP or nl_recv() returns either 0 or a negative error code.
        nl_recvmsgs_default(nl_sock_info.nl_sock);
    }

    // Return the RSSI value
    rssiMutex.lock();
    currentRssiUtils = static_cast<float>(nl_cb_args.signal_lv);
    rssiMutex.unlock();
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

    // This code requires stdbuf to be installed
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

int cbr_handler(struct nl_msg *msg, void *arg)
{
    struct nlattr *tb[NL80211_ATTR_MAX + 1];
    struct genlmsghdr *gnlh = (struct genlmsghdr *) nlmsg_data(nlmsg_hdr(msg));
    struct nlattr *sinfo[NL80211_SURVEY_INFO_MAX + 1];
    char dev[20];

    static struct nla_policy survey_policy[NL80211_SURVEY_INFO_MAX + 1] = {};
    survey_policy[NL80211_SURVEY_INFO_FREQUENCY].type = NLA_U32;
    survey_policy[NL80211_SURVEY_INFO_NOISE].type = NLA_U8;

    nla_parse(tb, NL80211_ATTR_MAX, genlmsg_attrdata(gnlh, 0), genlmsg_attrlen(gnlh, 0), NULL);

    if (!tb[NL80211_ATTR_SURVEY_INFO]) {
        fprintf(stderr, "Survey data missing!\n");
        return NL_SKIP;
    }

    if (nla_parse_nested(sinfo, NL80211_SURVEY_INFO_MAX, tb[NL80211_ATTR_SURVEY_INFO], survey_policy)) {
        fprintf(stderr, "Failed to parse nested survey attributes!\n");
        return NL_SKIP;
    }

    unsigned long long activeTime = 0, busyTime = 0, rxTime = 0, txTime = 0;

    if (!sinfo[NL80211_SURVEY_INFO_IN_USE])
    {
        return NL_SKIP;  // Not the active channel
    }

    cbrMutex.lock();
    if (sinfo[NL80211_SURVEY_INFO_FREQUENCY] && cbrData.verbose && cbrData.firstTime)
    {
        printf("Frequency:\t\t\t%u MHz%s\n", nla_get_u32(sinfo[NL80211_SURVEY_INFO_FREQUENCY]),
               sinfo[NL80211_SURVEY_INFO_IN_USE] ? " [in use]" : "");
    }
    cbrMutex.unlock();

    if (sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME])
        activeTime = nla_get_u64(sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME]);

    if (sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_BUSY])
        busyTime = nla_get_u64(sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_BUSY]);

    if (sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_RX])
        rxTime = nla_get_u64(sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_RX]);

    if (sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_TX])
        txTime = nla_get_u64(sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_TX]);

    cbrMutex.lock();
    bool firstTime = cbrData.firstTime;
    cbrMutex.unlock();

    if (firstTime) {
        cbrMutex.lock();
        if (cbrData.verbose)
        {
            std::cout << "First Time for CBR Computing" << std::endl;
        }
        cbrData.firstTime = false;
        cbrData.startActiveTime = activeTime;
        cbrData.startBusyTime = busyTime;
        cbrData.startReceiveTime = rxTime;
        cbrData.startTransmitTime = txTime;
        cbrData.currentCBR = -1.0f;
        cbrMutex.unlock();
        return NL_SKIP;
    }

    cbrMutex.lock();
    // Check for counter reset or wraparound
    if (activeTime < cbrData.startActiveTime || busyTime < cbrData.startBusyTime ||
        rxTime < cbrData.startReceiveTime || txTime < cbrData.startTransmitTime) {
        if (cbrData.verbose)
        {
            std::cerr << "Warning: time counters reset or wrapped. Skipping...\n";
        }
        cbrData.startActiveTime = activeTime;
        cbrData.startBusyTime = busyTime;
        cbrData.startReceiveTime = rxTime;
        cbrData.startTransmitTime = txTime;
        cbrMutex.unlock();
        return NL_SKIP;
    }

    unsigned long long deltaActiveTime = activeTime - cbrData.startActiveTime;
    unsigned long long deltaBusyTime = busyTime - cbrData.startBusyTime;
    unsigned long long deltaReceiveTime = rxTime - cbrData.startReceiveTime;
    unsigned long long deltaTransmitTime = txTime - cbrData.startTransmitTime;

    if (cbrData.verbose)
    {
        std::cout << "Active: " << deltaActiveTime
                  << " Busy: " << deltaBusyTime
                  << " RX: " << deltaReceiveTime
                  << " TX: " << deltaTransmitTime << std::endl;
    }

    float cbr = -1.0f;

    if (deltaActiveTime > 0) {
        if (deltaBusyTime < 0) deltaBusyTime = 0;
        cbr = static_cast<float>(deltaBusyTime) / deltaActiveTime;
        if (cbr < 0.0f) cbr = 0.0f;
        if (cbr > 1.0f) cbr = 1.0f;
    }

    if (cbrData.verbose && cbr >= 0.0f)
        std::cout << "Current Channel Busy Ratio: " << cbr << std::endl;

    // Save new base values for next delta
    cbrData.currentCBR = cbr;
    cbrData.startActiveTime = activeTime;
    cbrData.startBusyTime = busyTime;
    cbrData.startReceiveTime = rxTime;
    cbrData.startTransmitTime = txTime;
    cbrData.currentBusyTime = deltaBusyTime;
    cbrData.currentRxTime = deltaReceiveTime;
    cbrData.currentTxTime = deltaTransmitTime;
    cbrMutex.unlock();

    return NL_SKIP;
}


void read_cbr_from_netlink(nl_sock_info_t nl_sock_info)
{
    struct nl_msg *nl_msg = nlmsg_alloc();

	genlmsg_put(nl_msg,NL_AUTO_PORT,NL_AUTO_SEQ,nl_sock_info.nl80211_id,0,NLM_F_DUMP | NLM_F_REQUEST,NL80211_CMD_GET_SURVEY,0);
	nla_put_u32(nl_msg,NL80211_ATTR_IFINDEX,nl_sock_info.ifindex);

	nl_socket_modify_cb(nl_sock_info.nl_sock,NL_CB_VALID,NL_CB_CUSTOM,cbr_handler,nullptr);

    nl_send_auto_complete(nl_sock_info.nl_sock, nl_msg);

    int err = nl_recvmsgs_default(nl_sock_info.nl_sock);

    free_nl_socket(nl_sock_info);
}

void setup_cbr_structure(bool verbose=false)
{
    cbrMutex.lock();
    cbrData.verbose = verbose;
    cbrMutex.unlock();
}

void start_reading_cbr(nl_sock_info_t m_nl_sock_info)
{
    if (m_nl_sock_info.sock_valid==true)
    {
        read_cbr_from_netlink(m_nl_sock_info);
    }
    else
    {
        throw std::runtime_error("Socket is invalid. Cannot read from netlink.");
    }
}

float get_current_cbr()
{
    try
    {
        float cbr;
        cbrMutex.lock();
        cbr = cbrData.currentCBR;
        cbrMutex.unlock();
        return cbr;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

float get_current_busy_time()
{
    try
    {
        float cbt;
        cbrMutex.lock();
        cbt = cbrData.currentBusyTime;
        cbrMutex.unlock();
        return cbt;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

float get_current_tx_time()
{
    try
    {
        float ctt;
        cbrMutex.lock();
        ctt = cbrData.currentBusyTime;
        cbrMutex.unlock();
        return ctt;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

float get_current_rx_time()
{
    try
    {
        float crt;
        cbrMutex.lock();
        crt = cbrData.currentBusyTime;
        cbrMutex.unlock();
        return crt;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

float get_current_rssi()
{
    try
    {
        float rssi;
        rssiMutex.lock();
        rssi = currentRssiUtils;
        rssiMutex.unlock();
        return rssi;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

void setNewTxPower(double txPower, std::string dissemination_interface)
{
    int mbm = static_cast<int> (txPower * 100);

    nl_sock_info_t nl_sock_info = open_nl_socket(dissemination_interface);

    struct nl_msg *nl_msg = nlmsg_alloc();

    // genlmsg_put(nl_msg,NL_AUTO_PORT,NL_AUTO_SEQ,nl_sock_info.nl80211_id,0,NLM_F_DUMP | NLM_F_REQUEST,NL80211_ATTR_WIPHY_TX_POWER_SETTING,0);
    genlmsg_put(nl_msg, NL_AUTO_PORT, NL_AUTO_SEQ, nl_sock_info.nl80211_id, 0, NLM_F_REQUEST, NL80211_CMD_SET_WIPHY, 0);

    enum nl80211_tx_power_setting type = NL80211_TX_POWER_FIXED;

	nla_put_u32(nl_msg, NL80211_ATTR_WIPHY_TX_POWER_SETTING, type);
    nla_put_u32(nl_msg, NL80211_ATTR_WIPHY_TX_POWER_LEVEL, mbm);
    nla_put_u32(nl_msg, NL80211_ATTR_IFINDEX, nl_sock_info.ifindex);

    // std::cout << txPower << " " << dissemination_interface << std::endl;

    // nla_put_u32(nl_msg, NL80211_ATTR_WIPHY_TX_POWER_LEVEL, mbm);

    int err = nl_send_auto_complete(nl_sock_info.nl_sock, nl_msg);

    if (err < 0)
    {
        std::cerr << "Failed to send message" << std::endl;
    }

    err = nl_recvmsgs_default(nl_sock_info.nl_sock);

    if (err < 0)
    {
        std::cerr << "Failed to receive response" << std::endl;
    }

    free_nl_socket(nl_sock_info);
}

