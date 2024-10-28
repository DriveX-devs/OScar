/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EtsiTs102941TypesAuthorization"
 * 	found in "EtsiTs102941TypesAuthorization.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_pki/`
 */

#include "AuthorizationResponseCode.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_OER_SUPPORT)
static asn_oer_constraints_t asn_OER_type_AuthorizationResponseCode_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
asn_per_constraints_t asn_PER_type_AuthorizationResponseCode_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  5,  5,  0,  26 }	/* (0..26,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_AuthorizationResponseCode_value2enum_1[] = {
	{ 0,	2,	"ok" },
	{ 1,	16,	"its-aa-cantparse" },
	{ 2,	21,	"its-aa-badcontenttype" },
	{ 3,	24,	"its-aa-imnottherecipient" },
	{ 4,	33,	"its-aa-unknownencryptionalgorithm" },
	{ 5,	23,	"its-aa-decryptionfailed" },
	{ 6,	20,	"its-aa-keysdontmatch" },
	{ 7,	24,	"its-aa-incompleterequest" },
	{ 8,	27,	"its-aa-invalidencryptionkey" },
	{ 9,	23,	"its-aa-outofsyncrequest" },
	{ 10,	16,	"its-aa-unknownea" },
	{ 11,	16,	"its-aa-invalidea" },
	{ 12,	24,	"its-aa-deniedpermissions" },
	{ 13,	17,	"aa-ea-cantreachea" },
	{ 14,	15,	"ea-aa-cantparse" },
	{ 15,	20,	"ea-aa-badcontenttype" },
	{ 16,	23,	"ea-aa-imnottherecipient" },
	{ 17,	32,	"ea-aa-unknownencryptionalgorithm" },
	{ 18,	22,	"ea-aa-decryptionfailed" },
	{ 19,	9,	"invalidaa" },
	{ 20,	18,	"invalidaasignature" },
	{ 21,	7,	"wrongea" },
	{ 22,	10,	"unknownits" },
	{ 23,	16,	"invalidsignature" },
	{ 24,	20,	"invalidencryptionkey" },
	{ 25,	17,	"deniedpermissions" },
	{ 26,	18,	"deniedtoomanycerts" }
	/* This list is extensible */
};
static const unsigned int asn_MAP_AuthorizationResponseCode_enum2value_1[] = {
	13,	/* aa-ea-cantreachea(13) */
	25,	/* deniedpermissions(25) */
	26,	/* deniedtoomanycerts(26) */
	15,	/* ea-aa-badcontenttype(15) */
	14,	/* ea-aa-cantparse(14) */
	18,	/* ea-aa-decryptionfailed(18) */
	16,	/* ea-aa-imnottherecipient(16) */
	17,	/* ea-aa-unknownencryptionalgorithm(17) */
	19,	/* invalidaa(19) */
	20,	/* invalidaasignature(20) */
	24,	/* invalidencryptionkey(24) */
	23,	/* invalidsignature(23) */
	2,	/* its-aa-badcontenttype(2) */
	1,	/* its-aa-cantparse(1) */
	5,	/* its-aa-decryptionfailed(5) */
	12,	/* its-aa-deniedpermissions(12) */
	3,	/* its-aa-imnottherecipient(3) */
	7,	/* its-aa-incompleterequest(7) */
	11,	/* its-aa-invalidea(11) */
	8,	/* its-aa-invalidencryptionkey(8) */
	6,	/* its-aa-keysdontmatch(6) */
	9,	/* its-aa-outofsyncrequest(9) */
	10,	/* its-aa-unknownea(10) */
	4,	/* its-aa-unknownencryptionalgorithm(4) */
	0,	/* ok(0) */
	22,	/* unknownits(22) */
	21	/* wrongea(21) */
	/* This list is extensible */
};
const asn_INTEGER_specifics_t asn_SPC_AuthorizationResponseCode_specs_1 = {
	asn_MAP_AuthorizationResponseCode_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_AuthorizationResponseCode_enum2value_1,	/* N => "tag"; sorted by N */
	27,	/* Number of elements in the maps */
	28,	/* Extensions before this member */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_AuthorizationResponseCode_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_AuthorizationResponseCode = {
	"AuthorizationResponseCode",
	"AuthorizationResponseCode",
	&asn_OP_NativeEnumerated,
	asn_DEF_AuthorizationResponseCode_tags_1,
	sizeof(asn_DEF_AuthorizationResponseCode_tags_1)
		/sizeof(asn_DEF_AuthorizationResponseCode_tags_1[0]), /* 1 */
	asn_DEF_AuthorizationResponseCode_tags_1,	/* Same as above */
	sizeof(asn_DEF_AuthorizationResponseCode_tags_1)
		/sizeof(asn_DEF_AuthorizationResponseCode_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		&asn_OER_type_AuthorizationResponseCode_constr_1,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_AuthorizationResponseCode_constr_1,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_AuthorizationResponseCode_specs_1	/* Additional specs */
};

