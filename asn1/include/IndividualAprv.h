/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "Ieee1609Dot2Dot1Acpc"
 * 	found in "Ieee1609Dot2Dot1Acpc.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_pki/`
 */

#ifndef	_IndividualAprv_H_
#define	_IndividualAprv_H_


#include "asn_application.h"

/* Including external dependencies */
#include "Uint8.h"
#include "Time32.h"
#include "IValue.h"
#include "AcpcTreeId.h"
#include "BIT_STRING.h"
#include "AcpcNodeValue.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* IndividualAprv */
typedef struct IndividualAprv {
	Uint8_t	 version;
	Time32_t	 generationTime;
	IValue_t	 currentI;
	AcpcTreeId_t	 acpcTreeId;
	BIT_STRING_t	 nodeId;
	AcpcNodeValue_t	 nodeValue;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} IndividualAprv_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_IndividualAprv;
extern asn_SEQUENCE_specifics_t asn_SPC_IndividualAprv_specs_1;
extern asn_TYPE_member_t asn_MBR_IndividualAprv_1[6];

#ifdef __cplusplus
}
#endif

#endif	/* _IndividualAprv_H_ */
#include "asn_internal.h"
