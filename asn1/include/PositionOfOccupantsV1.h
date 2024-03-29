/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "ITS-Container.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_PositionOfOccupantsV1_H_
#define	_PositionOfOccupantsV1_H_


#include "asn_application.h"

/* Including external dependencies */
#include "BIT_STRING.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PositionOfOccupantsV1 {
	PositionOfOccupantsV1_row1LeftOccupied	= 0,
	PositionOfOccupantsV1_row1RightOccupied	= 1,
	PositionOfOccupantsV1_row1MidOccupied	= 2,
	PositionOfOccupantsV1_row1NotDetectable	= 3,
	PositionOfOccupantsV1_row1NotPresent	= 4,
	PositionOfOccupantsV1_row2LeftOccupied	= 5,
	PositionOfOccupantsV1_row2RightOccupied	= 6,
	PositionOfOccupantsV1_row2MidOccupied	= 7,
	PositionOfOccupantsV1_row2NotDetectable	= 8,
	PositionOfOccupantsV1_row2NotPresent	= 9,
	PositionOfOccupantsV1_row3LeftOccupied	= 10,
	PositionOfOccupantsV1_row3RightOccupied	= 11,
	PositionOfOccupantsV1_row3MidOccupied	= 12,
	PositionOfOccupantsV1_row3NotDetectable	= 13,
	PositionOfOccupantsV1_row3NotPresent	= 14,
	PositionOfOccupantsV1_row4LeftOccupied	= 15,
	PositionOfOccupantsV1_row4RightOccupied	= 16,
	PositionOfOccupantsV1_row4MidOccupied	= 17,
	PositionOfOccupantsV1_row4NotDetectable	= 18,
	PositionOfOccupantsV1_row4NotPresent	= 19
} e_PositionOfOccupantsV1;

/* PositionOfOccupantsV1 */
typedef BIT_STRING_t	 PositionOfOccupantsV1_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_PositionOfOccupantsV1_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_PositionOfOccupantsV1;
asn_struct_free_f PositionOfOccupantsV1_free;
asn_struct_print_f PositionOfOccupantsV1_print;
asn_constr_check_f PositionOfOccupantsV1_constraint;
ber_type_decoder_f PositionOfOccupantsV1_decode_ber;
der_type_encoder_f PositionOfOccupantsV1_encode_der;
xer_type_decoder_f PositionOfOccupantsV1_decode_xer;
xer_type_encoder_f PositionOfOccupantsV1_encode_xer;
oer_type_decoder_f PositionOfOccupantsV1_decode_oer;
oer_type_encoder_f PositionOfOccupantsV1_encode_oer;
per_type_decoder_f PositionOfOccupantsV1_decode_uper;
per_type_encoder_f PositionOfOccupantsV1_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _PositionOfOccupantsV1_H_ */
#include "asn_internal.h"
