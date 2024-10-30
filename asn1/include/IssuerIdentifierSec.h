/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "Ieee1609Dot2"
 * 	found in "Ieee1609Dot2.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_v2_1_1/`
 */

#ifndef	_IssuerIdentifierSec_H_
#define	_IssuerIdentifierSec_H_


#include "asn_application.h"

/* Including external dependencies */
#include "HashedId8.h"
#include "HashAlgorithm.h"
#include "constr_CHOICE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum IssuerIdentifierSec_PR {
	IssuerIdentifierSec_PR_NOTHING,	/* No components present */
	IssuerIdentifierSec_PR_sha256AndDigest,
	IssuerIdentifierSec_PR_self,
	/* Extensions may appear below */
	IssuerIdentifierSec_PR_sha384AndDigest
} IssuerIdentifierSec_PR;

/* IssuerIdentifierSec */
typedef struct IssuerIdentifierSec {
	IssuerIdentifierSec_PR present;
	union IssuerIdentifierSec_u {
		HashedId8_t	 sha256AndDigest;
		HashAlgorithm_t	 self;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
		HashedId8_t	 sha384AndDigest;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} IssuerIdentifierSec_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_IssuerIdentifierSec;
extern asn_CHOICE_specifics_t asn_SPC_IssuerIdentifierSec_specs_1;
extern asn_TYPE_member_t asn_MBR_IssuerIdentifierSec_1[3];
extern asn_per_constraints_t asn_PER_type_IssuerIdentifierSec_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _IssuerIdentifierSec_H_ */
#include "asn_internal.h"
