/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "Ieee1609Dot2Dot1LaRaInterface"
 * 	found in "Ieee1609Dot2Dot1LaRaInterface.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_pki/`
 */

#include "LaRaInterfacePdu.h"

/*
 * This type is implemented using NULL,
 * so here we adjust the DEF accordingly.
 */
static const ber_tlv_tag_t asn_DEF_LaRaInterfacePdu_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (5 << 2))
};
asn_TYPE_descriptor_t asn_DEF_LaRaInterfacePdu = {
	"LaRaInterfacePdu",
	"LaRaInterfacePdu",
	&asn_OP_NULL,
	asn_DEF_LaRaInterfacePdu_tags_1,
	sizeof(asn_DEF_LaRaInterfacePdu_tags_1)
		/sizeof(asn_DEF_LaRaInterfacePdu_tags_1[0]), /* 1 */
	asn_DEF_LaRaInterfacePdu_tags_1,	/* Same as above */
	sizeof(asn_DEF_LaRaInterfacePdu_tags_1)
		/sizeof(asn_DEF_LaRaInterfacePdu_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		NULL_constraint
	},
	0, 0,	/* No members */
	0	/* No specifics */
};

