/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EtsiTs102941TypesLinkCertificate"
 * 	found in "EtsiTs102941TypesLinkCertificate.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_pki/`
 */

#ifndef	_ToBeSignedLinkCertificate_H_
#define	_ToBeSignedLinkCertificate_H_


#include "asn_application.h"

/* Including external dependencies */
#include "Time32.h"
#include "HashedData.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ToBeSignedLinkCertificate */
typedef struct ToBeSignedLinkCertificate {
	Time32_t	 expiryTime;
	HashedData_t	 certificateHash;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ToBeSignedLinkCertificate_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ToBeSignedLinkCertificate;
extern asn_SEQUENCE_specifics_t asn_SPC_ToBeSignedLinkCertificate_specs_1;
extern asn_TYPE_member_t asn_MBR_ToBeSignedLinkCertificate_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _ToBeSignedLinkCertificate_H_ */
#include "asn_internal.h"
