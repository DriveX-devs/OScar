/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in asn1/ISO19321.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_RscPart_H_
#define	_RscPart_H_


#include "asn_application.h"

/* Including external dependencies */
#include "ZoneIds.h"
#include "Direction.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ZoneIds;
struct RoadSurfaceStaticCharacteristics;
struct RoadSurfaceDynamicCharacteristics;

/* RscPart */
typedef struct RscPart {
	struct ZoneIds	*detectionZoneIds;	/* OPTIONAL */
	ZoneIds_t	 relevanceZoneIds;
	Direction_t	*direction;	/* OPTIONAL */
	struct RoadSurfaceStaticCharacteristics	*roadSurfaceStaticCharacteristics;	/* OPTIONAL */
	struct RoadSurfaceDynamicCharacteristics	*roadSurfaceDynamicCharacteristics;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RscPart_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RscPart;
extern asn_SEQUENCE_specifics_t asn_SPC_RscPart_specs_1;
extern asn_TYPE_member_t asn_MBR_RscPart_1[5];
extern asn_per_constraints_t asn_PER_type_RscPart_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "ZoneIds.h"
#include "RoadSurfaceStaticCharacteristics.h"
#include "RoadSurfaceDynamicCharacteristics.h"

#endif	/* _RscPart_H_ */
#include "asn_internal.h"