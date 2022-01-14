#ifndef EXTRADEVICECOMMUNICATIONPROTOCOL_H
#define EXTRADEVICECOMMUNICATIONPROTOCOL_H

// EDCP (Extra Device Communication Protocol) main header

#include <inttypes.h>

#define REQUEST_TYPE 0x01
#define REPLY_TYPE 0x02
#define EDCP_PORT 48888 // EDCP is thought to be encapsulated over UDP and to use port 48888

typedef struct {
	int type:2; // Either 0x00 (reserved), 0x01 (request), 0x02 (reply), 0x03 (reserved)
	int reserved:30; // Should always be set to all zeros
	uint32_t ramUsage; // In MiB
	uint16_t cpuUsage; // In 0.01%
	uint16_t gpuUsage; // In 0.01%
} extraDeviceCommProtHeader_t;

#endif // EXTRADEVICECOMMUNICATIONPROTOCOL_H