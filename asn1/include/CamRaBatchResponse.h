/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "Ieee1609Dot2Dot1CamRaInterface"
 * 	found in "Ieee1609Dot2Dot1CamRaInterface.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_pki/`
 */

#ifndef	_CamRaBatchResponse_H_
#define	_CamRaBatchResponse_H_


#include "asn_application.h"

/* Including external dependencies */
#include "Uint8.h"
#include "HashedId8.h"
#include "asn_SEQUENCE_OF.h"
#include "constr_SEQUENCE_OF.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct BlindedKey;

/* CamRaBatchResponse */
typedef struct CamRaBatchResponse {
	Uint8_t	 version;
	HashedId8_t	 requestHash;
	struct CamRaBatchResponse__batch {
		A_SEQUENCE_OF(struct BlindedKey) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} batch;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CamRaBatchResponse_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CamRaBatchResponse;
extern asn_SEQUENCE_specifics_t asn_SPC_CamRaBatchResponse_specs_1;
extern asn_TYPE_member_t asn_MBR_CamRaBatchResponse_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "BlindedKey.h"

#endif	/* _CamRaBatchResponse_H_ */
#include "asn_internal.h"
