/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "PMU-PDU-Descriptions"
 * 	found in "PMU.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_AvailableCPU_H_
#define	_AvailableCPU_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum AvailableCPU {
	AvailableCPU_unavailable	= 0,
	AvailableCPU_onevCPU	= 1000,
	AvailableCPU_twovCPU	= 2000,
	AvailableCPU_threevCPU	= 3000
} e_AvailableCPU;

/* AvailableCPU */
typedef long	 AvailableCPU_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_AvailableCPU_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_AvailableCPU;
asn_struct_free_f AvailableCPU_free;
asn_struct_print_f AvailableCPU_print;
asn_constr_check_f AvailableCPU_constraint;
ber_type_decoder_f AvailableCPU_decode_ber;
der_type_encoder_f AvailableCPU_encode_der;
xer_type_decoder_f AvailableCPU_decode_xer;
xer_type_encoder_f AvailableCPU_encode_xer;
oer_type_decoder_f AvailableCPU_decode_oer;
oer_type_encoder_f AvailableCPU_encode_oer;
per_type_decoder_f AvailableCPU_decode_uper;
per_type_encoder_f AvailableCPU_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _AvailableCPU_H_ */
#include "asn_internal.h"
