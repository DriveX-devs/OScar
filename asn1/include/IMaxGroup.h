/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "Ieee1609Dot2CrlBaseTypes"
 * 	found in "Ieee1609Dot2CrlBaseTypes.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_pki/`
 */

#ifndef	_IMaxGroup_H_
#define	_IMaxGroup_H_


#include "asn_application.h"

/* Including external dependencies */
#include "Uint16.h"
#include "SequenceOfIndividualRevocation.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* IMaxGroup */
typedef struct IMaxGroup {
	Uint16_t	 iMax;
	SequenceOfIndividualRevocation_t	 contents;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} IMaxGroup_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_IMaxGroup;
extern asn_SEQUENCE_specifics_t asn_SPC_IMaxGroup_specs_1;
extern asn_TYPE_member_t asn_MBR_IMaxGroup_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _IMaxGroup_H_ */
#include "asn_internal.h"
