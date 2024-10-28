/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EtsiTs102941TrustLists"
 * 	found in "EtsiTs102941TrustLists.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_pki/`
 */

#ifndef	_CtlEntry_H_
#define	_CtlEntry_H_


#include "asn_application.h"

/* Including external dependencies */
#include "RootCaEntry.h"
#include "EaEntry.h"
#include "AaEntry.h"
#include "DcEntry.h"
#include "TlmEntry.h"
#include "constr_CHOICE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CtlEntry_PR {
	CtlEntry_PR_NOTHING,	/* No components present */
	CtlEntry_PR_rca,
	CtlEntry_PR_ea,
	CtlEntry_PR_aa,
	CtlEntry_PR_dc,
	CtlEntry_PR_tlm
	/* Extensions may appear below */
	
} CtlEntry_PR;

/* CtlEntry */
typedef struct CtlEntry {
	CtlEntry_PR present;
	union CtlEntry_u {
		RootCaEntry_t	 rca;
		EaEntry_t	 ea;
		AaEntry_t	 aa;
		DcEntry_t	 dc;
		TlmEntry_t	 tlm;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CtlEntry_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CtlEntry;
extern asn_CHOICE_specifics_t asn_SPC_CtlEntry_specs_1;
extern asn_TYPE_member_t asn_MBR_CtlEntry_1[5];
extern asn_per_constraints_t asn_PER_type_CtlEntry_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _CtlEntry_H_ */
#include "asn_internal.h"
