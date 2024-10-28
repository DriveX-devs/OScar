/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "Ieee1609Dot2Dot1CertManagement"
 * 	found in "Ieee1609Dot2Dot1CertManagement.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_pki/`
 */

#ifndef	_FullIeeeTbsCtl_H_
#define	_FullIeeeTbsCtl_H_


#include "asn_application.h"

/* Including external dependencies */
#include "Ieee1609dot2dot1MsctlType.h"
#include "ElectorGroupId.h"
#include "CtlSequenceNumber.h"
#include "Time32.h"
#include "NativeInteger.h"
#include "CtlElectorEntry.h"
#include "asn_SEQUENCE_OF.h"
#include "constr_SEQUENCE_OF.h"
#include "CtlRootCaEntry.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* FullIeeeTbsCtl */
typedef struct FullIeeeTbsCtl {
	Ieee1609dot2dot1MsctlType_t	 type;
	ElectorGroupId_t	 electorGroupId;
	CtlSequenceNumber_t	 sequenceNumber;
	Time32_t	 effectiveDate;
	struct FullIeeeTbsCtl__electorApprove {
		A_SEQUENCE_OF(CtlElectorEntry_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} electorApprove;
	struct FullIeeeTbsCtl__electorRemove {
		A_SEQUENCE_OF(CtlElectorEntry_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} electorRemove;
	struct FullIeeeTbsCtl__rootCaApprove {
		A_SEQUENCE_OF(CtlRootCaEntry_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} rootCaApprove;
	struct FullIeeeTbsCtl__rootCaRemove {
		A_SEQUENCE_OF(CtlRootCaEntry_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} rootCaRemove;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	long	*quorum;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} FullIeeeTbsCtl_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_FullIeeeTbsCtl;
extern asn_SEQUENCE_specifics_t asn_SPC_FullIeeeTbsCtl_specs_1;
extern asn_TYPE_member_t asn_MBR_FullIeeeTbsCtl_1[9];

#ifdef __cplusplus
}
#endif

#endif	/* _FullIeeeTbsCtl_H_ */
#include "asn_internal.h"
