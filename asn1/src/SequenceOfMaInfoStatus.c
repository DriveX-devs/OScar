/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "Ieee1609Dot2Dot1CertManagement"
 * 	found in "Ieee1609Dot2Dot1CertManagement.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_pki/`
 */

#include "SequenceOfMaInfoStatus.h"

asn_TYPE_member_t asn_MBR_SequenceOfMaInfoStatus_1[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_MaInfoStatus,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_SequenceOfMaInfoStatus_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
asn_SET_OF_specifics_t asn_SPC_SequenceOfMaInfoStatus_specs_1 = {
	sizeof(struct SequenceOfMaInfoStatus),
	offsetof(struct SequenceOfMaInfoStatus, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
asn_TYPE_descriptor_t asn_DEF_SequenceOfMaInfoStatus = {
	"SequenceOfMaInfoStatus",
	"SequenceOfMaInfoStatus",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_SequenceOfMaInfoStatus_tags_1,
	sizeof(asn_DEF_SequenceOfMaInfoStatus_tags_1)
		/sizeof(asn_DEF_SequenceOfMaInfoStatus_tags_1[0]), /* 1 */
	asn_DEF_SequenceOfMaInfoStatus_tags_1,	/* Same as above */
	sizeof(asn_DEF_SequenceOfMaInfoStatus_tags_1)
		/sizeof(asn_DEF_SequenceOfMaInfoStatus_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		SEQUENCE_OF_constraint
	},
	asn_MBR_SequenceOfMaInfoStatus_1,
	1,	/* Single element */
	&asn_SPC_SequenceOfMaInfoStatus_specs_1	/* Additional specs */
};

