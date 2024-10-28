/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "Ieee1609Dot2Dot1AcaRaInterface"
 * 	found in "Ieee1609Dot2Dot1AcaRaInterface.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_pki/`
 */

#ifndef	_RaAcaCertRequestFlags_H_
#define	_RaAcaCertRequestFlags_H_


#include "asn_application.h"

/* Including external dependencies */
#include "BIT_STRING.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum RaAcaCertRequestFlags {
	RaAcaCertRequestFlags_butterflyExplicit	= 0,
	RaAcaCertRequestFlags_cubk	= 1
} e_RaAcaCertRequestFlags;

/* RaAcaCertRequestFlags */
typedef BIT_STRING_t	 RaAcaCertRequestFlags_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_RaAcaCertRequestFlags_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_RaAcaCertRequestFlags;
asn_struct_free_f RaAcaCertRequestFlags_free;
asn_struct_print_f RaAcaCertRequestFlags_print;
asn_constr_check_f RaAcaCertRequestFlags_constraint;
ber_type_decoder_f RaAcaCertRequestFlags_decode_ber;
der_type_encoder_f RaAcaCertRequestFlags_encode_der;
xer_type_decoder_f RaAcaCertRequestFlags_decode_xer;
xer_type_encoder_f RaAcaCertRequestFlags_encode_xer;
jer_type_encoder_f RaAcaCertRequestFlags_encode_jer;
oer_type_decoder_f RaAcaCertRequestFlags_decode_oer;
oer_type_encoder_f RaAcaCertRequestFlags_encode_oer;
per_type_decoder_f RaAcaCertRequestFlags_decode_uper;
per_type_encoder_f RaAcaCertRequestFlags_encode_uper;
per_type_decoder_f RaAcaCertRequestFlags_decode_aper;
per_type_encoder_f RaAcaCertRequestFlags_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _RaAcaCertRequestFlags_H_ */
#include "asn_internal.h"
