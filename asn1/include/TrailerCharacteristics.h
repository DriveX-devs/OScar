/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "/home/carlosrisma/IVIM ASN1 files/asn1_IS_ISO_TS_19321_IVI.asn"
 * 	`asn1c -fincludes-quoted`
 */

#ifndef	_TrailerCharacteristics_H_
#define	_TrailerCharacteristics_H_


#include "asn_application.h"

/* Including external dependencies */
#include "asn_SEQUENCE_OF.h"
#include "constr_SEQUENCE_OF.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct VehicleCharacteristicsFixValues;
struct VehicleCharacteristicsRanges;

/* TrailerCharacteristics */
typedef struct TrailerCharacteristics {
	struct TrailerCharacteristics__equalTo {
		A_SEQUENCE_OF(struct VehicleCharacteristicsFixValues) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *equalTo;
	struct TrailerCharacteristics__notEqualTo {
		A_SEQUENCE_OF(struct VehicleCharacteristicsFixValues) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *notEqualTo;
	struct TrailerCharacteristics__ranges {
		A_SEQUENCE_OF(struct VehicleCharacteristicsRanges) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ranges;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} TrailerCharacteristics_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_TrailerCharacteristics;
extern asn_SEQUENCE_specifics_t asn_SPC_TrailerCharacteristics_specs_1;
extern asn_TYPE_member_t asn_MBR_TrailerCharacteristics_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "VehicleCharacteristicsFixValues.h"
#include "VehicleCharacteristicsRanges.h"

#endif	/* _TrailerCharacteristics_H_ */
#include "asn_internal.h"
