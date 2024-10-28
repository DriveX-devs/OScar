/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "Ieee1609Dot2CrlBaseTypes"
 * 	found in "Ieee1609Dot2CrlBaseTypes.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_pki/`
 */

#include "LAGroup.h"

asn_TYPE_member_t asn_MBR_LAGroup_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct LAGroup, la1Id),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_LaId,
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
		"la1Id"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct LAGroup, la2Id),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_LaId,
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
		"la2Id"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct LAGroup, contents),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SequenceOfIMaxGroup,
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
		"contents"
		},
};
static const ber_tlv_tag_t asn_DEF_LAGroup_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_LAGroup_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* la1Id */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* la2Id */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* contents */
};
asn_SEQUENCE_specifics_t asn_SPC_LAGroup_specs_1 = {
	sizeof(struct LAGroup),
	offsetof(struct LAGroup, _asn_ctx),
	asn_MAP_LAGroup_tag2el_1,
	3,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	3,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_LAGroup = {
	"LAGroup",
	"LAGroup",
	&asn_OP_SEQUENCE,
	asn_DEF_LAGroup_tags_1,
	sizeof(asn_DEF_LAGroup_tags_1)
		/sizeof(asn_DEF_LAGroup_tags_1[0]), /* 1 */
	asn_DEF_LAGroup_tags_1,	/* Same as above */
	sizeof(asn_DEF_LAGroup_tags_1)
		/sizeof(asn_DEF_LAGroup_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_LAGroup_1,
	3,	/* Elements count */
	&asn_SPC_LAGroup_specs_1	/* Additional specs */
};

