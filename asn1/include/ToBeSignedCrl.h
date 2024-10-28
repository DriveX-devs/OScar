/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EtsiTs102941TrustLists"
 * 	found in "EtsiTs102941TrustLists.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_pki/`
 */

#ifndef	_ToBeSignedCrl_H_
#define	_ToBeSignedCrl_H_


#include "asn_application.h"

/* Including external dependencies */
#include "Version.h"
#include "Time32.h"
#include "CrlEntry.h"
#include "asn_SEQUENCE_OF.h"
#include "constr_SEQUENCE_OF.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ToBeSignedCrl */
typedef struct ToBeSignedCrl {
	Version_t	 version;
	Time32_t	 thisUpdate;
	Time32_t	 nextUpdate;
	struct ToBeSignedCrl__entries {
		A_SEQUENCE_OF(CrlEntry_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} entries;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ToBeSignedCrl_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ToBeSignedCrl;
extern asn_SEQUENCE_specifics_t asn_SPC_ToBeSignedCrl_specs_1;
extern asn_TYPE_member_t asn_MBR_ToBeSignedCrl_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _ToBeSignedCrl_H_ */
#include "asn_internal.h"
