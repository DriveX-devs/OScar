#include <iostream>
#include "etsiDecoderFrontend.h"
#include "geonet.h"
#include "btp.h"
#include "basicHeader.h"
#include "commonHeader.h"
#include "shbHeader.h"

extern "C" {
	#include "CAM.h"
	#include "DENM.h"
	#include "VAM.h"
}

NAMED_ENUM_DEFINE_FCNS(etsi_message_t,MSGTYPES);

namespace etsiDecoder {
	decoderFrontend::decoderFrontend() {
		m_print_pkt = false;
	}

	int decoderFrontend::decodeEtsi(uint8_t *buffer,size_t buflen,etsiDecodedData_t &decoded_data,msgType_e msgtype) {
		bool isGeoNet = true;

		if(buflen<=0) {
			return ETSI_DECODER_ERROR;
		}

		if(m_print_pkt==true) {
			std::cout << "[INFO] [Decoder] Full packet content :" << std::endl;
			for(uint32_t i=0;i<buflen;i++) {
				std::printf("%02X ",buffer[i]);
			}
			std::cout << std::endl;
		}

		// If msgtype is set to MSGTYPE_AUTO, try to automatically detect if the message is a full ITS message, or a simple CAM/VAM/DENM, without the BTP and GeoNetworking layers
		if(msgtype == MSGTYPE_AUTO) {
			// We are considering here that GeoNetworking should contain at least 40 bytes (for TSB - for GBC this value should be even higher), plus 4 bytes due to BTP
			// We check then if the second byte in the buffer corresponds to a valid ITS message; if yes, we detect the absence of BTP + GN, otherwise we consider this message
			// as a full ITS one (with BTP and GN)
			if(buflen<44 || (buflen>=44 && is_enum_valid_etsi_message_t(static_cast<etsi_message_t>(*((uint8_t *) buffer+1))))) {
				isGeoNet=false;
			} else {
				isGeoNet=true;
			}

			std::cout << "[INFO] [Decoder] Automatic detection of message type enabled. Message type: " << (isGeoNet == true ? "Full ITS message" : "Pure Facilities layer message") << std::endl;
		} else if(msgtype == MSGTYPE_FACILITYONLY) {
			isGeoNet=false;
		}
		// There is no need to check for else if(msgtype == MSGTYPE_ITS), as it is always the default option, which would just set "isGeoNet" to true, which is already true thanks to its initialization

		void *decoded_=nullptr;
		asn_dec_rval_t decode_result;

		if(isGeoNet == true) {
			GeoNet geonet;
			btp BTP;
			GNDataIndication_t gndataIndication;
			BTPDataIndication_t btpDataIndication;

			if(geonet.decodeGN(buffer,&gndataIndication)!= GN_OK)
			  {
			    std::cerr << "[WARN] [Decoder] Warning: GeoNet unable to decode a received packet." << std::endl;
			    return ETSI_DECODED_ERROR;
			  }

			if(BTP.decodeBTP(gndataIndication,&btpDataIndication)!= BTP_OK)
			  {
			    std::cerr << "[WARN] [Decoder] Warning: BTP unable to decode a received packet." << std::endl;
			    return ETSI_DECODED_ERROR;
			  }

			if(m_print_pkt==true) {
				std::cout << "[INFO] [Decoder] ETSI packet content :" << std::endl;
				for(uint32_t i=0;i<btpDataIndication.lenght;i++) {
					std::printf("%02X ",btpDataIndication.data[i]);
				}
				std::cout << std::endl;
			}

			decoded_data.gnTimestamp = gndataIndication.SourcePV.TST;
			// Save the GN Address
			memcpy(decoded_data.GNaddress,gndataIndication.SourcePV.GnAddress,8);

			if(btpDataIndication.destPort == CA_PORT) {
				decoded_data.type = ETSI_DECODED_CAM;

				decode_result = asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_CAM, &decoded_, btpDataIndication.data, btpDataIndication.lenght);

				if(decode_result.code!=RC_OK || decoded_==nullptr) {
					std::cerr << "[WARN] [Decoder] Warning: unable to decode a received CAM." << std::endl;
					if(decoded_) free(decoded_);
					return ETSI_DECODER_ERROR;
				}
			} else if(btpDataIndication.destPort == DEN_PORT) {

				decoded_data.posLat = gndataIndication.GnAddressDest.posLat;
				decoded_data.posLong = gndataIndication.GnAddressDest.posLong;
				decoded_data.distA = gndataIndication.GnAddressDest.distA;
				decoded_data.distB = gndataIndication.GnAddressDest.distB;
				decoded_data.angle = gndataIndication.GnAddressDest.angle;

				decoded_data.type = ETSI_DECODED_DENM;

				decode_result = asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_DENM, &decoded_, btpDataIndication.data, btpDataIndication.lenght);

				if(decode_result.code!=RC_OK || decoded_==nullptr) {
					std::cerr << "[WARN] [Decoder] Warning: unable to decode a received DENM." << std::endl;
					if(decoded_) free(decoded_);
					return ETSI_DECODER_ERROR;
				}
			} else if(btpDataIndication.destPort == VA_PORT){
		
				decoded_data.type = ETSI_DECODED_VAM;

				decode_result = asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_VAM, &decoded_, btpDataIndication.data, btpDataIndication.lenght);

				if(decode_result.code!=RC_OK || decoded_==nullptr) {
					std::cerr << "[WARN] [Decoder] Warning: unable to decode a received VAM." << std::endl;
					if(decoded_) free(decoded_);
					return ETSI_DECODER_ERROR;
				}
			} else {
					decoded_data.type = ETSI_DECODED_ERROR;
					return ETSI_DECODER_ERROR;
				}
		} else {
			// Check if the messageID is among the supported ones
			// The supported IDs are defined as #define MSGTYPES(MSGTYPE) in etsiDecoderFrontend.h
			etsi_message_t messageID = static_cast<etsi_message_t>(*((uint8_t *) buffer+1));

			if(is_enum_valid_etsi_message_t(messageID)) {
				if(messageID==CAM) {
					decoded_data.type = ETSI_DECODED_CAM_NOGN;

					decode_result = asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_CAM, &decoded_, buffer, buflen);

					if(decode_result.code!=RC_OK || decoded_==nullptr) {
						std::cerr << "[WARN] [Decoder] Warning: unable to decode a received CAM (no BTP/GN)." << std::endl;
						if(decoded_) free(decoded_);
						return ETSI_DECODER_ERROR;
					}
				} else if(messageID==DENM) {
					decoded_data.type = ETSI_DECODED_DENM_NOGN;

					decode_result = asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_DENM, &decoded_, buffer, buflen);

					if(decode_result.code!=RC_OK || decoded_==nullptr) {
						std::cerr << "[WARN] [Decoder] Warning: unable to decode a received DENM (no BTP/GN)." << std::endl;
						if(decoded_) free(decoded_);
						return ETSI_DECODER_ERROR;
					}
				} else if(messageID==VAM) {
					decoded_data.type = ETSI_DECODED_VAM_NOGN;

					decode_result = asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_VAM, &decoded_, buffer, buflen);

					if(decode_result.code!=RC_OK || decoded_==nullptr) {
						std::cerr << "[WARN] [Decoder] Warning: unable to decode a received VAM (no BTP/GN)." << std::endl;
						if(decoded_) free(decoded_);
						return ETSI_DECODER_ERROR;
					}
				} else {
					std::cerr << "[WARN] [Decoder] Unable to decode a reveived message with unknown/unsupported messageID: " << messageID << std::endl;
					std::cerr << "[ERROR] [Decoder] Error: this point in the code should never be reached. Please report this bug to the developers. Thank you!" << std::endl;
					decoded_data.type = ETSI_DECODED_ERROR;
					return ETSI_DECODER_ERROR;
				}
			} else {
				std::cerr << "[WARN] [Decoder] Unable to decode a reveived message with unknown/unsupported messageID: " << messageID << std::endl;
				decoded_data.type = ETSI_DECODED_ERROR;
				return ETSI_DECODER_ERROR;
			}
		}

		decoded_data.decoded_msg = decoded_;

		return ETSI_DECODER_OK;
	}
}
