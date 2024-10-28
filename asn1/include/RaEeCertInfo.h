/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "Ieee1609Dot2Dot1EeRaInterface"
 * 	found in "Ieee1609Dot2Dot1EeRaInterface.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_pki/`
 */

#ifndef	_RaEeCertInfo_H_
#define	_RaEeCertInfo_H_


#include "asn_application.h"

/* Including external dependencies */
#include "Uint8.h"
#include "Time32.h"
#include "IValue.h"
#include "HashedId8.h"
#include "AcpcTreeId.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* RaEeCertInfo */
typedef struct RaEeCertInfo {
	Uint8_t	 version;
	Time32_t	 generationTime;
	IValue_t	 currentI;
	HashedId8_t	 requestHash;
	Time32_t	 nextDlTime;
	AcpcTreeId_t	*acpcTreeId;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RaEeCertInfo_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RaEeCertInfo;
extern asn_SEQUENCE_specifics_t asn_SPC_RaEeCertInfo_specs_1;
extern asn_TYPE_member_t asn_MBR_RaEeCertInfo_1[6];

#ifdef __cplusplus
}
#endif

#endif	/* _RaEeCertInfo_H_ */
#include "asn_internal.h"
