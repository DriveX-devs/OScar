#ifndef LONGPOSITIONVECTOR_H
#define LONGPOSITIONVECTOR_H
#include <stdint.h>
#include "gn-address.h"

namespace ns3 {

  typedef struct _longPositionVector{
    uint8_t GnAddress[8];
    uint32_t TST; //! Time at which lat and long were acquired by GeoAdhoc router
    int32_t latitude;
    int32_t longitude;
    bool positionAccuracy : 1;
    int16_t speed :15;
    uint16_t heading;
  }GNlpv_t;

}

#endif // LONGPOSITIONVECTOR_H
