#include "utils.h"
#include <unistd.h>
#include <cmath>
#include <cstdarg>
#include <cfloat>
#include <vehicleDataDef.h>
#include <cstring>

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

double get_rssi_from_iw(uint8_t macaddr[6],std::string interface_name) {
	// ioctl() does not seem to work for some drivers, which are instead using nl80211
	// The following code is a "quick and dirty" way of getting the RSSI, leveraging popen and calling the iw tool
	// This part should hopefully be converted to the usage of nl80211 in the near future
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