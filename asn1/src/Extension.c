/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EtsiTs103097ExtensionModule"
 * 	found in "EtsiTs103097ExtensionModule.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -D output_v2_1_1/`
 */

#include "Extension.h"

static const long asn_VAL_1_etsiTs102941CrlRequestId = 1;
static const long asn_VAL_2_etsiTs102941DeltaCtlRequestId = 2;
static const asn_ioc_cell_t asn_IOS_EtsiTs103097HeaderInfoExtensions_1_rows[] = {
	{ "&extId", aioc__value, &asn_DEF_ExtId, &asn_VAL_1_etsiTs102941CrlRequestId },
	{ "&ExtContent", aioc__type, &asn_DEF_EtsiTs102941CrlRequest },
	{ "&extId", aioc__value, &asn_DEF_ExtId, &asn_VAL_2_etsiTs102941DeltaCtlRequestId },
	{ "&ExtContent", aioc__type, &asn_DEF_EtsiTs102941DeltaCtlRequest }
};
static const asn_ioc_set_t asn_IOS_EtsiTs103097HeaderInfoExtensions_1[] = {
	{ 2, 2, asn_IOS_EtsiTs103097HeaderInfoExtensions_1_rows }
};
static int
memb_id_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0L && value <= 255L)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_type_selector_result_t
select_EtsiTs103097HeaderInfoExtensions_content_type(const asn_TYPE_descriptor_t *parent_type, const void *parent_sptr) {
	asn_type_selector_result_t result = {0, 0};
	const asn_ioc_set_t *itable = asn_IOS_EtsiTs103097HeaderInfoExtensions_1;
	size_t constraining_column = 0; /* &extId */
	size_t for_column = 1; /* &ExtContent */
	size_t row, presence_index = 0;
	const long *constraining_value = (const long *)((const char *)parent_sptr + offsetof(struct EtsiTs103097HeaderInfoExtensions, id));
	
	for(row=0; row < itable->rows_count; row++) {
	    const asn_ioc_cell_t *constraining_cell = &itable->rows[row * itable->columns_count + constraining_column];
	    const asn_ioc_cell_t *type_cell = &itable->rows[row * itable->columns_count + for_column];
	
	    if(type_cell->cell_kind == aioc__undefined)
	        continue;
	
	    presence_index++;
	    if(constraining_cell->type_descriptor->op->compare_struct(constraining_cell->type_descriptor, constraining_value, constraining_cell->value_sptr) == 0) {
	        result.type_descriptor = type_cell->type_descriptor;
	        result.presence_index = presence_index;
	        break;
	    }
	}
	
	return result;
}

static int
memb_content_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	
	if(1 /* No applicable constraints whatsoever */) {
		/* Nothing is here. See below */
	}
	
	return td->encoding_constraints.general_constraints(td, sptr, ctfailcb, app_key);
}

#if !defined(ASN_DISABLE_OER_SUPPORT)
static asn_oer_constraints_t asn_OER_memb_id_constr_2 CC_NOTUSED = {
	{ 1, 1 }	/* (0..255) */,
	-1};
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_id_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 8,  8,  0,  255 }	/* (0..255) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_OER_SUPPORT)
static asn_oer_constraints_t asn_OER_memb_content_constr_3 CC_NOTUSED = {
	{ 0, 0 },
	-1};
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_content_constr_3 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static asn_TYPE_member_t asn_MBR_content_3[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct EtsiTs103097HeaderInfoExtensions__content, choice.EtsiTs102941CrlRequest),
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_EtsiTs102941CrlRequest,
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
		"EtsiTs102941CrlRequest"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct EtsiTs103097HeaderInfoExtensions__content, choice.EtsiTs102941DeltaCtlRequest),
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_EtsiTs102941DeltaCtlRequest,
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
		"EtsiTs102941DeltaCtlRequest"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_content_tag2el_3[] = {
    { (ASN_TAG_CLASS_UNIVERSAL | (16 << 2)), 0, 0, 1 }, /* EtsiTs102941CrlRequest */
    { (ASN_TAG_CLASS_UNIVERSAL | (16 << 2)), 1, -1, 0 } /* EtsiTs102941DeltaCtlRequest */
};
static asn_CHOICE_specifics_t asn_SPC_content_specs_3 = {
	sizeof(struct EtsiTs103097HeaderInfoExtensions__content),
	offsetof(struct EtsiTs103097HeaderInfoExtensions__content, _asn_ctx),
	offsetof(struct EtsiTs103097HeaderInfoExtensions__content, present),
	sizeof(((struct EtsiTs103097HeaderInfoExtensions__content *)0)->present),
	asn_MAP_content_tag2el_3,
	2,	/* Count of tags in the map */
	0, 0,
	-1	/* Extensions start */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_content_3 = {
	"content",
	"content",
	&asn_OP_OPEN_TYPE,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		OPEN_TYPE_constraint
	},
	asn_MBR_content_3,
	2,	/* Elements count */
	&asn_SPC_content_specs_3	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_EtsiTs103097HeaderInfoExtensions_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct EtsiTs103097HeaderInfoExtensions, id),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ExtId,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			&asn_OER_memb_id_constr_2,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_id_constr_2,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
			memb_id_constraint_1
		},
		0, 0, /* No default value */
		"id"
		},
	{ ATF_OPEN_TYPE | ATF_NOFLAGS, 0, offsetof(struct EtsiTs103097HeaderInfoExtensions, content),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_content_3,
		select_EtsiTs103097HeaderInfoExtensions_content_type,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			&asn_OER_memb_content_constr_3,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_content_constr_3,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
			memb_content_constraint_1
		},
		0, 0, /* No default value */
		"content"
		},
};
static const ber_tlv_tag_t asn_DEF_EtsiTs103097HeaderInfoExtensions_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_EtsiTs103097HeaderInfoExtensions_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* id */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* content */
};
asn_SEQUENCE_specifics_t asn_SPC_EtsiTs103097HeaderInfoExtensions_specs_1 = {
	sizeof(struct EtsiTs103097HeaderInfoExtensions),
	offsetof(struct EtsiTs103097HeaderInfoExtensions, _asn_ctx),
	asn_MAP_EtsiTs103097HeaderInfoExtensions_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_EtsiTs103097HeaderInfoExtensions = {
	"EtsiTs103097HeaderInfoExtensions",
	"EtsiTs103097HeaderInfoExtensions",
	&asn_OP_SEQUENCE,
	asn_DEF_EtsiTs103097HeaderInfoExtensions_tags_1,
	sizeof(asn_DEF_EtsiTs103097HeaderInfoExtensions_tags_1)
		/sizeof(asn_DEF_EtsiTs103097HeaderInfoExtensions_tags_1[0]), /* 1 */
	asn_DEF_EtsiTs103097HeaderInfoExtensions_tags_1,	/* Same as above */
	sizeof(asn_DEF_EtsiTs103097HeaderInfoExtensions_tags_1)
		/sizeof(asn_DEF_EtsiTs103097HeaderInfoExtensions_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_EtsiTs103097HeaderInfoExtensions_1,
	2,	/* Elements count */
	&asn_SPC_EtsiTs103097HeaderInfoExtensions_specs_1	/* Additional specs */
};

