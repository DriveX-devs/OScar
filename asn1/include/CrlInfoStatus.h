/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "Ieee1609Dot2Dot1CertManagement"
 * 	found in "Ieee1609Dot2Dot1CertManagement.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_pki/`
 */

#ifndef	_CrlInfoStatus_H_
#define	_CrlInfoStatus_H_


#include "asn_application.h"

/* Including external dependencies */
#include "HashedId8.h"
#include "CrlSeries.h"
#include "Time32.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CrlInfoStatus */
typedef struct CrlInfoStatus {
	HashedId8_t	 cracaId;
	CrlSeries_t	 series;
	Time32_t	 issueDate;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CrlInfoStatus_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CrlInfoStatus;
extern asn_SEQUENCE_specifics_t asn_SPC_CrlInfoStatus_specs_1;
extern asn_TYPE_member_t asn_MBR_CrlInfoStatus_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _CrlInfoStatus_H_ */
#include "asn_internal.h"
