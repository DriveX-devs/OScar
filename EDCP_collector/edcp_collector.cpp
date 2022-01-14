#include <iostream>
#include <inttypes.h>
#include <sys/sysinfo.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>

#define RXBUFSIZ 1024
#define EDCP_PORT 48888
#define LOAD_UNAVAILABLE 10001
#define RAM_UNAVAILABLE 512001

#define REQUEST_TYPE 0x01
#define REPLY_TYPE 0x02

typedef struct {
	int type:2; // Either 0x00 (reserved), 0x01 (request), 0x02 (reply), 0x03 (reserved)
	int reserved:30; // Should always be set to all zeros
	uint32_t ramUsage; // In MiB
	uint16_t cpuUsage; // In 0.01%
	uint16_t gpuUsage; // In 0.01%
} extraDeviceCommProtHeader_t;

int main(void) {
	struct sockaddr_in snd_addr, rcv_addr;
	socklen_t snd_addr_len=-1;
	extraDeviceCommProtHeader_t *rxMsgHdrPtr;
	extraDeviceCommProtHeader_t txMsgHdr={0}; // Must be initialized to all zeros

	int sockd=-1;
	uint8_t rxbuff[RXBUFSIZ];
	
	sockd=socket(AF_INET, SOCK_DGRAM, 0);

	if(sockd<0) {
		perror("Socket creation error. Details");
		exit(EXIT_FAILURE);
	}

	memset(&rcv_addr,0,sizeof(rcv_addr));
	rcv_addr.sin_family=AF_INET;
	rcv_addr.sin_port=htons(EDCP_PORT);
	rcv_addr.sin_addr.s_addr=INADDR_ANY;

	if(bind(sockd,(struct sockaddr *)&rcv_addr,sizeof(rcv_addr))<0) {
		perror("Socket binding failed. Details");
		close(sockd);
		exit(EXIT_FAILURE);
	}

	memset(&snd_addr,0,sizeof(snd_addr));
	snd_addr_len=sizeof(snd_addr);

	while(1) {
		size_t recv_bytes=0;

		fprintf(stdout,"[INFO] EDCP Collector: waiiting for new messages...\n");

		recv_bytes=recvfrom(sockd,rxbuff,RXBUFSIZ,0,(struct sockaddr *) &snd_addr,&snd_addr_len);

		fprintf(stdout,"[INFO] EDCP Collector: received a new message.\n");

		if(recv_bytes>0) {
			rxMsgHdrPtr=(extraDeviceCommProtHeader_t *)rxbuff;

			if(rxMsgHdrPtr->type==REQUEST_TYPE) {
				// Gather the current node load
				double cpuLoad=LOAD_UNAVAILABLE;
				double gpuLoad=LOAD_UNAVAILABLE;
				double freeRAM=RAM_UNAVAILABLE;

				struct sysinfo sysdata;
				memset(&sysdata,0,sizeof(sysdata));

				if(!sysinfo(&sysdata)) {
					float floatload = 1.f / (1 << SI_LOAD_SHIFT);
					int sysfd=-1;
					ssize_t rdbytes;
					char sysbuf[5]={0}; // Maximum 999 + '\0' + 1 just to be safe (not sure if 1000 can actually be reached)

					cpuLoad=sysdata.loads[0]*floatload*100/get_nprocs();
					freeRAM=sysdata.freeram/1048576;

					sysfd=open("/sys/devices/gpu.0/load",O_RDONLY);

					if(sysfd>=0) {
						rdbytes=read(sysfd,sysbuf,sizeof(sysbuf)-1);

						if(rdbytes>0) {
							gpuLoad=strtol(sysbuf,nullptr,10)/10.0;
						}
					} else {
						perror("Error: cannot open sysfs file for GPU usage. Details: ");
					}
				}

				// Send reply to client
				txMsgHdr.type=REPLY_TYPE;
				txMsgHdr.cpuUsage=htons((uint16_t)(cpuLoad*100));
				txMsgHdr.gpuUsage=htons((uint16_t)(gpuLoad*100));
				txMsgHdr.ramUsage=htonl((uint32_t)freeRAM);

				fprintf(stdout,"[INFO] CPU Usage: %.2lf - GPU Usage: %.2lf - Free RAM (MB): %.2lf.\n",cpuLoad,gpuLoad,freeRAM);
				fprintf(stdout,"[INFO] Replying to %s\n",inet_ntoa(snd_addr.sin_addr));

				if(sendto(sockd,&txMsgHdr,sizeof(txMsgHdr),0,(struct sockaddr *) &snd_addr,snd_addr_len)<0) {
					perror("Error: received a request but cannot send reply. Details");
				}
			} else {
				fprintf(stderr,"Error: received a new message with a wrong type. Expected type: REQUEST (0x01). Received type: 0x%02X\n",rxMsgHdrPtr->type);
			}
		} else {
			perror("Error on recvfrom(). Details: ");
		}
	}

	return 0;
}