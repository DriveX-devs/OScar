/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EtsiTs102941TypesEnrolment"
 * 	found in "EtsiTs102941TypesEnrolment.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_pki/`
 */

#ifndef	_InnerEcRequest_H_
#define	_InnerEcRequest_H_


#include "asn_application.h"

/* Including external dependencies */
#include "OCTET_STRING.h"
#include "CertificateFormat.h"
#include "PublicKeys.h"
#include "CertificateSubjectAttributes.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* InnerEcRequest */
typedef struct InnerEcRequest {
	OCTET_STRING_t	 itsId;
	CertificateFormat_t	 certificateFormat;
	PublicKeys_t	 publicKeys;
	CertificateSubjectAttributes_t	 requestedSubjectAttributes;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} InnerEcRequest_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_InnerEcRequest;

#ifdef __cplusplus
}
#endif

#endif	/* _InnerEcRequest_H_ */
#include "asn_internal.h"
