/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "PLU-PDU-Descriptions"
 * 	found in "PLU.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_PlatoonPerceivedObjectsContainer_H_
#define	_PlatoonPerceivedObjectsContainer_H_


#include "asn_application.h"

/* Including external dependencies */
#include "asn_SEQUENCE_OF.h"
#include "constr_SEQUENCE_OF.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct PlatoonPerceivedObject;

/* PlatoonPerceivedObjectsContainer */
typedef struct PlatoonPerceivedObjectsContainer {
	A_SEQUENCE_OF(struct PlatoonPerceivedObject) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PlatoonPerceivedObjectsContainer_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_PlatoonPerceivedObjectsContainer;
extern asn_SET_OF_specifics_t asn_SPC_PlatoonPerceivedObjectsContainer_specs_1;
extern asn_TYPE_member_t asn_MBR_PlatoonPerceivedObjectsContainer_1[1];
extern asn_per_constraints_t asn_PER_type_PlatoonPerceivedObjectsContainer_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "PlatoonPerceivedObject.h"

#endif	/* _PlatoonPerceivedObjectsContainer_H_ */
#include "asn_internal.h"
