/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EfcDsrcApplication"
 * 	found in asn1/ISO14906-0-6.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_SessionClass_H_
#define	_SessionClass_H_


#include "asn_application.h"

/* Including external dependencies */
#include "Int1.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SessionClass */
typedef struct SessionClass {
	Int1_t	 sessionTariffClass;
	Int1_t	 sessionClaimedClass;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SessionClass_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SessionClass;

#ifdef __cplusplus
}
#endif

#endif	/* _SessionClass_H_ */
#include "asn_internal.h"