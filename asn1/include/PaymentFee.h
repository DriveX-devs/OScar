/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EfcDsrcApplication"
 * 	found in asn1/ISO14906-0-6.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_PaymentFee_H_
#define	_PaymentFee_H_


#include "asn_application.h"

/* Including external dependencies */
#include "Int2.h"
#include "PayUnit.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PaymentFee */
typedef struct PaymentFee {
	Int2_t	 paymentFeeAmount;
	PayUnit_t	 paymentFeeUnit;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PaymentFee_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_PaymentFee;
extern asn_SEQUENCE_specifics_t asn_SPC_PaymentFee_specs_1;
extern asn_TYPE_member_t asn_MBR_PaymentFee_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _PaymentFee_H_ */
#include "asn_internal.h"