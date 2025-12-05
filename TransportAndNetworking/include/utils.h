//
// Created by Carlos Mateo Risma Carletti on 10/05/21.
// Updated by Francesco Raviglione in November 2021
//

#ifndef OCABS_UTILS_H
#define OCABS_UTILS_H


#include <cmath>
#include <cstdint>
#include "packetBuffer.h"

// Epoch time at 2004-01-01 (in ms)
#define TIME_SHIFT_MILLI 1072915200000
#define TS_MAX1 4294967296 // (2^32)

typedef enum {
	GN_OK = 0,
	GN_VERSION_ERROR = 1,
	GN_SECURED_ERROR = 2,
	GN_LIFETIME_ERROR = 3,
	GN_HOP_LIMIT_ERROR = 4,
	GN_TYPE_ERROR = 5,
} gnError_e;

typedef enum {
	BTP_OK = 0,
	BTP_ERROR = 1
} btpError_e;

typedef struct longPositionVector{
    char GnAddress[8]; //! Address
    uint32_t TST; //! TimeSTamp at which lat and long were acquired by GeoAdhoc router
    int32_t latitude;
    int32_t longitude;
    bool positionAccuracy : 1;
    int16_t speed :15;
    uint16_t heading;
} GNlpv_t;

typedef enum {
    ANY_TT=0,
    BEACON=1,
    GUC=2,
    GAC=3,
    GBC=4,
    TSB=5,
    LS=6,
} TransportType_t;

typedef enum {
    UNSPECIFIED=0,
    ITS_G5=1
} CommProfile_t;

typedef enum {
    ANY_UP=0,
    BTP_A=1,
    BTP_B=2,
    GN6ASL=3
} BTPType_t;

typedef struct _geoarea {
    int32_t posLong;
    int32_t posLat;
    uint16_t distA;
    uint16_t distB;
    uint16_t angle;
    uint8_t shape;
} GeoArea_t;


typedef enum {
    ACCEPTED=0,
    MAX_LENGHT_EXCEEDED = 1,
    MAX_LIFE_EXCEEDED = 2,
    REP_INTERVAL_LOW = 3,
    UNSUPPORTED_TRA_CLASS = 4,
    MAX_GEOAREA_EXCEEDED = 5,
    UNSPECIFIED_ERROR =6,
    UNSUPPORTED_GN_REP=7,
	BLOCKED_BY_GK = 8,
} GNDataConfirm_t;

typedef struct dataIndication {
    uint8_t BTPType;
    uint16_t destPort;
    uint16_t sourcePort;
    uint16_t destPInfo;
    uint8_t GNType; // GN Packet transport type -- GeoUnicast, SHB, TSB, GeoBroadcast or GeoAnycast
    GeoArea_t GnAddress; // GN destination adress -- destination adress(GeoUnicast) or geo. area (GeoBroadcast or GeoAnycast)
    GNlpv_t GNPositionV; // GN Posistion vector

    uint16_t GNSecurityR; // GN Security Report /OPCIONAL/
    uint16_t GNCertID; //GN Certificate ID /OPCIONAL/
    uint16_t GNPermissions; // GN Permissions /OPCIONAL/
    uint16_t GNMaxRepInt; // GN maximum repetition Interval /OPCIONAL/
    uint8_t GNTraClass; // GN Traffic Class
    double GNRemPLife; // GN Reamianing Packet Lifetime /OPCIONAL/
    uint32_t lenght; // Payload size
    unsigned char* data;
} BTPDataIndication_t;

typedef struct gndataIndication {
    uint8_t upperProtocol;
    GNlpv_t SourcePV;
    GeoArea_t GnAddressDest; // GN destination adress -- destination adress(GeoUnicast) or geo. area (GeoBroadcast or GeoAnycast)
    uint8_t GNTraClass; // GN Traffic Class
    double GNRemainingLife; //GN Remaining Packet Lifetime in [s] /OPCIONAL/
    int16_t GNRemainingHL; // GN Remaining Hop Limit /OPCIONAL/
    uint8_t GNType; // GN Packet transport type -- GeoUnicast, SHB, TSB, GeoBroadcast or GeoAnycast
    uint32_t lenght; // Payload size
    unsigned char* data; // Payload
} GNDataIndication_t;

typedef struct dataRequest {
	BTPType_t BTPType;
	int16_t destPort;
	int16_t sourcePort;
	int16_t destPInfo;

	TransportType_t GNType; // GN Packet transport type -- GeoUnicast, SHB, TSB, GeoBroadcast or GeoAnycast
	GeoArea_t GnAddress; // GN destination adress -- destination adress(GeoUnicast) or geo. area (GeoBroadcast or GeoAnycast)
	CommProfile_t GNCommProfile; // GN Communication Profile -- determines de LL protocol entity

	int16_t GNSecurityP; // GN Security Profile /OPCIONAL/
	double GNMaxLife; //GN Maximum Packet Lifetime in [s] /OPCIONAL/
	int16_t GNRepInt; // GN Repetition Interval /OPCIONAL/
	int16_t GNMaxRepInt; // GN maximum repetition Interval /OPCIONAL/
	int16_t GNMaxHL; // GN Max Hop Limit /OPCIONAL/
	uint8_t GNTraClass; // GN Traffic Class

	uint32_t lenght; // Payload size
	packetBuffer data; // Payload
} BTPDataRequest_t;

typedef struct _gndataRequest {
	BTPType_t upperProtocol;
	TransportType_t GNType; // GN Packet transport type -- GeoUnicast, SHB, TSB, GeoBroadcast or GeoAnycast
	GeoArea_t GnAddress; // GN destination adress -- destination address(GeoUnicast) or geo. area (GeoBroadcast or GeoAnycast)
	CommProfile_t GNCommProfile; // GN Communication Profile -- determines de LL protocol entity

	int16_t GNSecurityP; // GN Security Profile /OPCIONAL/
	int32_t GNITS_AIDL;//Length of the value provided in the ITS-AID parameter /OPCIONAL/
	int32_t GNITS_AID;// ITS-AID for the payload to be sent/OPCIONAL/
	int32_t GNSecurityPermL;// Length of the security permissions parameter/OPCIONAL/
	int32_t GNSecurityPerm;// SSP associated with the ITS-AID/OPCIONAL/
	int32_t GNSecurityContInfo;// Information to be used to selecting properties of the security protocol/OPCIONAL/
	int32_t GNSecurityTargetIDListL; //Length for the value in the security target id list parameter /OPCIONAL/
	int32_t GNSecurityTargetIDList;// Unordered collection of target IDs used by the security entity for specifying multiple recipients /OPCIONAL/
	double GNMaxLife; //GN Maximum Packet Lifetime in [s] /OPCIONAL/
	int16_t GNRepInt; // GN Repetition Interval /OPCIONAL/
	int16_t GNMaxRepTime; // GN maximum repetition time /OPCIONAL/
	int16_t GNMaxHL; // GN Max Hop Limit /OPCIONAL/
	uint8_t GNTraClass; // GN Traffic Class

	uint32_t lenght; // Payload size
	packetBuffer data; // Payload
} GNDataRequest_t;

inline uint16_t swap_8bit(uint8_t us)
{
    return (uint8_t)(((us & 0xFF) >> 4) |
                      ((us & 0xFF) << 4));
}

inline uint16_t swap_16bit(uint16_t us)
{
    return (uint16_t)(((us & 0xFF00) >> 8) |
	                    ((us & 0x00FF) << 8));
}

inline uint32_t swap_32bit(uint32_t ul)
{
    return (uint32_t)(((ul & 0xFF000000) >> 24) |
	                   ((ul & 0x00FF0000) >>  8) |
	                   ((ul & 0x0000FF00) <<  8) |
	                   ((ul & 0x000000FF) << 24));
}

inline uint32_t get_timestamp_ms_gn32(void) {
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

	// TS_MAX1 is equal to 4294967296 (2^32), as required by the ETSI standards
	return (static_cast<uint32_t>(floor((seconds*1000000+microseconds)/1000.0))-TIME_SHIFT_MILLI)%TS_MAX1;
}

#endif // OCABS_UTILS_H
