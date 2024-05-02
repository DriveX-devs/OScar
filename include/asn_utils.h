#ifndef ASN_UTILS_H
#define ASN_UTILS_H

#include <stdint.h>
#include <string>

#define FIX_DENMID          0x01
#define FIX_CAMID           0x02
#define FIX_VAMID           0x10
#define DECI                10
#define CENTI               100
#define MILLI               1000
#define MICRO               1000000
#define DOT_ONE_MICRO       10000000
#define NANO_TO_MILLI       1000000
#define NANO_TO_CENTI       10000000
#define NANO_TO_MICRO       1000

#define DEG_2_RAD_ASN_UTILS(val) ((val)*M_PI/180.0)
#define RAD_2_DEG_ASN_UTILS(val) ((val)*180.0/M_PI)

//Epoch time at 2004-01-01
#define TIME_SHIFT 1072915200000

/* Maximum length of an asn1c error message (when decoding fails with respect to certain constraints) */
#define ERRORBUFF_LEN       128

long compute_timestampIts ();
double haversineDist(double lat_a, double lon_a, double lat_b, double lon_b);

uint8_t setByteMask(uint8_t mask);
uint8_t setByteMask(uint16_t mask, unsigned int i);
uint8_t setByteMask(uint32_t mask, unsigned int i);
uint8_t getFromMask(uint8_t mask);
inline uint16_t getFromMasks(uint8_t mask0,uint8_t mask1){return getFromMask(mask0) | (getFromMask(mask1)<<8);}
inline uint32_t getFromMasks(uint8_t mask0,uint8_t mask1,uint8_t mask2){return getFromMask(mask0) | (getFromMask(mask1)<<8) | (getFromMask(mask2)<<16);}

#endif // ASN_UTILS_H

