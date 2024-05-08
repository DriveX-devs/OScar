/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-PDU-Descriptions"
 * 	found in "asn1/TR103562v211-CPM.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#include "AreaEllipse.h"

asn_TYPE_member_t asn_MBR_AreaEllipse_1[] = {
	{ ATF_POINTER, 1, offsetof(struct AreaEllipse, nodeCenterPoint),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OffsetPoint,
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
		"nodeCenterPoint"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct AreaEllipse, semiMinorRangeLength),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SemiRangeLength,
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
		"semiMinorRangeLength"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct AreaEllipse, semiMajorRangeLength),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SemiRangeLength,
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
		"semiMajorRangeLength"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct AreaEllipse, semiMajorRangeOrientation),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_Wgs84AngleValue,
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
		"semiMajorRangeOrientation"
		},
	{ ATF_POINTER, 1, offsetof(struct AreaEllipse, semiHeight),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SemiRangeLength,
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
		"semiHeight"
		},
};
static const int asn_MAP_AreaEllipse_oms_1[] = { 0, 4 };
static const ber_tlv_tag_t asn_DEF_AreaEllipse_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_AreaEllipse_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* nodeCenterPoint */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* semiMinorRangeLength */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* semiMajorRangeLength */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* semiMajorRangeOrientation */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 } /* semiHeight */
};
asn_SEQUENCE_specifics_t asn_SPC_AreaEllipse_specs_1 = {
	sizeof(struct AreaEllipse),
	offsetof(struct AreaEllipse, _asn_ctx),
	asn_MAP_AreaEllipse_tag2el_1,
	5,	/* Count of tags in the map */
	asn_MAP_AreaEllipse_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_AreaEllipse = {
	"AreaEllipse",
	"AreaEllipse",
	&asn_OP_SEQUENCE,
	asn_DEF_AreaEllipse_tags_1,
	sizeof(asn_DEF_AreaEllipse_tags_1)
		/sizeof(asn_DEF_AreaEllipse_tags_1[0]), /* 1 */
	asn_DEF_AreaEllipse_tags_1,	/* Same as above */
	sizeof(asn_DEF_AreaEllipse_tags_1)
		/sizeof(asn_DEF_AreaEllipse_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_AreaEllipse_1,
	5,	/* Elements count */
	&asn_SPC_AreaEllipse_specs_1	/* Additional specs */
};

