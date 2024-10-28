/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "Ieee1609Dot2Dot1Protocol"
 * 	found in "Ieee1609Dot2Dot1Protocol.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_pki/`
 */

#ifndef	_X509Certificate_H_
#define	_X509Certificate_H_


#include "asn_application.h"

/* Including external dependencies */
#include "OCTET_STRING.h"

#ifdef __cplusplus
extern "C" {
#endif

/* X509Certificate */
typedef OCTET_STRING_t	 X509Certificate_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_X509Certificate;
asn_struct_free_f X509Certificate_free;
asn_struct_print_f X509Certificate_print;
asn_constr_check_f X509Certificate_constraint;
ber_type_decoder_f X509Certificate_decode_ber;
der_type_encoder_f X509Certificate_encode_der;
xer_type_decoder_f X509Certificate_decode_xer;
xer_type_encoder_f X509Certificate_encode_xer;
jer_type_encoder_f X509Certificate_encode_jer;
oer_type_decoder_f X509Certificate_decode_oer;
oer_type_encoder_f X509Certificate_encode_oer;
per_type_decoder_f X509Certificate_decode_uper;
per_type_encoder_f X509Certificate_encode_uper;
per_type_decoder_f X509Certificate_decode_aper;
per_type_encoder_f X509Certificate_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _X509Certificate_H_ */
#include "asn_internal.h"
