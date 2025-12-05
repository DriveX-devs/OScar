#ifndef S_LDM_ETSIDECODERFRONTEND_H
#define S_LDM_ETSIDECODERFRONTEND_H

#include <cinttypes>
#include <cstddef>
#include "named_enums.h"
#include "Seq.hpp"
#include "geonet.h"

extern "C" {
    #include "CAM.h"
    #include "DENM.h"
    #include "VAM.h"
    #include "CollectivePerceptionMessage.h"
}

#define ETSI_DECODER_OK 	0
#define ETSI_DECODER_ERROR 	1

// Defined as "named enum" (see named_enums.h)
// Only the supported message types should be listed here
// The format to add a new supported type for decoding is:
// MSGTYPE(<message name>,<messageID>)
#define MSGTYPES(MSGTYPE) \
	MSGTYPE(DENM,=1) \
	MSGTYPE(CAM,=2)  \
    MSGTYPE(CPM,=14) \
	MSGTYPE(VAM,=16)

NAMED_ENUM_DECLARE(etsi_message_t,MSGTYPES);

namespace etsiDecoder {
	typedef enum {
		ETSI_DECODED_ERROR,
		ETSI_DECODED_CAM,
		ETSI_DECODED_DENM,
		ETSI_DECODED_IVIM,
		ETSI_DECODED_CPM,
		ETSI_DECODED_VAM,
		ETSI_DECODED_CAM_NOGN,
		ETSI_DECODED_DENM_NOGN,
		ETSI_DECODED_IVIM_NOGN,
		ETSI_DECODED_CPM_NOGN,
		ETSI_DECODED_VAM_NOGN
	} etsiDecodedType_e;

	typedef struct etsiDecodedData {
		void *decoded_msg;
		etsiDecodedType_e type;

        asn1cpp::Seq<CollectivePerceptionMessage> decoded_cpm;

		uint32_t gnTimestamp;
		uint8_t GNaddress[8];
		//For DENMs GeoArea
		int32_t posLong;
		int32_t posLat;
		uint16_t distA;
		uint16_t distB;
		uint16_t angle;
	} etsiDecodedData_t;

	class decoderFrontend {
		public:
			typedef enum {
				// The message to be decoded is a full ITS message (Facilities + BTP + GeoNetworking)
				MSGTYPE_ITS = 0,
				// The message to be decoded is a "pure" Facilities layer message (just a CAM or DENM without BTP and GeoNetworking)
				MSGTYPE_FACILITYONLY = 1,
				// MSGTYPE_AUTO is using a quite greedy algorithm to try to detect whether we receive either a full ITS message 
				// (Facilities + BTP + GeoNetworking), or a Facilities layer message with no BTP and GeoNetworking. The detection
				// may not always be 100% accurate under certain specific circumstances. 
				// Thus, if possible, the two other options are always preferred.
				MSGTYPE_AUTO = 2
			} msgType_e;

			decoderFrontend(bool enable_security = false, std::string logfile_security = "dis");
			int decodeEtsi(uint8_t *buffer,size_t buflen,etsiDecodedData_t &decoded_data, msgType_e msgtype = MSGTYPE_ITS);
			void setPrintPacket(bool print_pkt) {m_print_pkt=print_pkt;}
            void setEnableSecurity(bool enable) {m_enable_security = enable;};

		private:
			bool m_print_pkt;
            GeoNet m_gn;
            bool m_enable_security;  // Store security option
            std::string m_logfile_security;  // Store security logfile name
	};
}

#endif // ETSIDECODERFRONTEND_H
