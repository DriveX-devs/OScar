/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EtsiTs103097ExtensionModule"
 * 	found in "EtsiTs103097ExtensionModule.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_v2_1_1/`
 */

#include "EtsiTs102941DeltaCtlRequest.h"

/*
 * This type is implemented using EtsiTs102941CtlRequest,
 * so here we adjust the DEF accordingly.
 */
static const ber_tlv_tag_t asn_DEF_EtsiTs102941DeltaCtlRequest_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
asn_TYPE_descriptor_t asn_DEF_EtsiTs102941DeltaCtlRequest = {
	"EtsiTs102941DeltaCtlRequest",
	"EtsiTs102941DeltaCtlRequest",
	&asn_OP_SEQUENCE,
	asn_DEF_EtsiTs102941DeltaCtlRequest_tags_1,
	sizeof(asn_DEF_EtsiTs102941DeltaCtlRequest_tags_1)
		/sizeof(asn_DEF_EtsiTs102941DeltaCtlRequest_tags_1[0]), /* 1 */
	asn_DEF_EtsiTs102941DeltaCtlRequest_tags_1,	/* Same as above */
	sizeof(asn_DEF_EtsiTs102941DeltaCtlRequest_tags_1)
		/sizeof(asn_DEF_EtsiTs102941DeltaCtlRequest_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_EtsiTs102941CtlRequest_1,
	2,	/* Elements count */
	&asn_SPC_EtsiTs102941CtlRequest_specs_1	/* Additional specs */
};

