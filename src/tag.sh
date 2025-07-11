#!/bin/sh

printf "\n";
printf "/* auto generated file, do not edit */\n";
printf "\n";

printf "#include <stdio.h>\n";
printf "#include <stdlib.h>\n";
printf "#include <stdint.h>\n";
printf "#include <string.h>\n";
printf "#define CLEW_DEBUG_NAME \"tag\"\n";

printf "#include \"debug.h\"\n";
printf "#include \"tag.h\"\n";
printf "\n";

printf "struct tag {\n";
printf "\tuint32_t tag;\n";
printf "\tconst char *name;\n";
printf "};\n";
printf "\n";

printf "static struct tag tags_v[] = {\n";
for t in `cat $1tag*.h  | grep "clew_tag_"  | grep "," | awk {'print $1'} | cut -d "," -f 1 | cut -b 10- | cut -d" " -f1 | sort -V | uniq`; do
  printf "\t{ clew_tag_$t, \"$t\" },\n";
done;
printf "};\n";
printf "\n";

printf "static struct tag tags_n[] = {\n";
for t in `cat $1tag*.h  | grep "clew_tag_"  | grep "," | awk {'print $1'} | cut -d "," -f 1 | cut -b 10- | cut -d" " -f1 | sort -V | uniq`; do
  printf "\t{ clew_tag_$t, \"$t\" },\n";
done;
printf "};\n";
printf "\n";

printf "static struct tag tags_country_v[] = {\n";
paste -d, $1tag-country-code-2.h $1tag-country-name.h | while IFS=, read c n; do printf "\t{ $c $n },\n"; done;
paste -d, $1tag-country-code-3.h $1tag-country-name.h | while IFS=, read c n; do printf "\t{ $c $n },\n"; done;
paste -d, $1tag-country-numeric.h $1tag-country-name.h | while IFS=, read c n; do printf "\t{ $c $n },\n"; done;
printf "};\n";
printf "\n";

printf "static struct tag tags_language_v[] = {\n";
paste -d, $1tag-language-code-2.h $1tag-language-name.h | while IFS=, read c n; do printf "\t{ $c $n },\n"; done;
printf "};\n";
printf "\n";

printf "static int compare_tag_name (const void *a, const void *b)\n";
printf "{\n";
printf "\tstruct tag *key = (struct tag *) a;\n";
printf "\tstruct tag *tag = (struct tag *) b;\n";
printf "\treturn strcasecmp(key->name, tag->name);\n";
printf "};\n";
printf "\n";

printf "uint32_t clew_tag_value (const char *tag)\n";
printf "{\n";
printf "\tstruct tag k;\n";
printf "\tstruct tag *t;\n";
printf "\tif (tag == NULL) {\n";
printf "\t\treturn clew_tag_unknown;\n";
printf "\t}\n";
printf "\tif (strncmp(tag, \"clew_tag_\", strlen(\"clew_tag_\")) == 0) {\n";
printf "\t\tk.name = tag + strlen(\"clew_tag_\");\n";
printf "\t} else {\n";
printf "\t\tk.name = tag;\n";
printf "\t}\n";
printf "\tt = (struct tag *) bsearch(&k, tags_n, sizeof(tags_n) / sizeof(tags_n[0]), sizeof(tags_n[0]), compare_tag_name);\n";
printf "\treturn (t != NULL) ? t->tag : clew_tag_unknown;\n";
printf "}\n";
printf "\n";

printf "static int compare_tag_value (const void *a, const void *b)\n";
printf "{\n";
printf "\tstruct tag *key = (struct tag *) a;\n";
printf "\tstruct tag *tag = (struct tag *) b;\n";
printf "\tif (key->tag < tag->tag) { return -1; }\n";
printf "\tif (key->tag > tag->tag) { return 1; }\n";
printf "\treturn 0;\n";
printf "};\n";
printf "\n";

printf "const char * clew_tag_string (uint32_t tag)\n";
printf "{\n";
printf "\tstruct tag k;\n";
printf "\tstruct tag *t;\n";
printf "\tk.tag = tag;\n";
printf "\tt = (struct tag *) bsearch(&k, tags_v, sizeof(tags_v) / sizeof(tags_v[0]), sizeof(tags_v[0]), compare_tag_value);\n";
printf "\treturn (t != NULL) ? t->name : \"unknown\";\n";
printf "}\n";
printf "\n";

printf "const char * clew_tag_country_string (uint32_t tag)\n";
printf "{\n";
printf "\tstruct tag k;\n";
printf "\tstruct tag *t;\n";
printf "\tk.tag = tag;\n";
printf "\tt = (struct tag *) bsearch(&k, tags_country_v, sizeof(tags_country_v) / sizeof(tags_country_v[0]), sizeof(tags_country_v[0]), compare_tag_value);\n";
printf "\treturn (t != NULL) ? t->name : \"unknown\";\n";
printf "}\n";
printf "\n";

printf "const char * clew_tag_language_string (uint32_t tag)\n";
printf "{\n";
printf "\tstruct tag k;\n";
printf "\tstruct tag *t;\n";
printf "\tk.tag = tag;\n";
printf "\tt = (struct tag *) bsearch(&k, tags_language_v, sizeof(tags_language_v) / sizeof(tags_language_v[0]), sizeof(tags_language_v[0]), compare_tag_value);\n";
printf "\treturn (t != NULL) ? t->name : \"unknown\";\n";
printf "}\n";
printf "\n";

printf "static int compare_tag (const void *a, const void *b)\n";
printf "{\n";
printf "\tuint32_t key = *(uint32_t *) a;\n";
printf "\tuint32_t tag = *(uint32_t *) b;\n";
printf "\tif (key < tag) { return -1; }\n";
printf "\tif (key > tag) { return 1; }\n";
printf "\treturn 0;\n";
printf "};\n";
printf "\n";

groups=`cat $1tag*.h | grep "clew_tag_group_" | grep "," | cut -d "_" -f 4- | cut -d "," -f 1 | sort -V | uniq`

for s in $groups; do
	printf "static uint32_t tags_$s[] = {\n";
	for t in `cat $1tag*.h  | grep "clew_tag_$s\_" | grep -v "clew_tag_$s\_no," | grep "," | awk {'print $1'} | cut -d "," -f 1 | cut -b 10- | cut -d " " -f 1 | sort -V | uniq`; do
  		printf "\tclew_tag_$t,\n";
	done;
  	printf "\tclew_tag_unknown,\n";
	printf "};\n";
	printf "\n";

	if [ `cat $1tag*.h  | grep "clew_tag_$s\_"  | grep "," | awk {'print $1'} | cut -d "," -f 1 | cut -b 10- | cut -d" " -f1 | sort -V | uniq | wc -l` -lt 32 ]; then

		printf "int clew_tag_is_group_$s (uint32_t tag)\n";
		printf "{\n";
		printf "\tsize_t i;\n";
		printf "\tif (tag < tags_$s[0]) {\n";
		printf "\t\treturn 0;\n";
		printf "\t}\n";
		printf "\ti = (sizeof(tags_$s) / sizeof(tags_$s[0])) - 1;\n";
		printf "\twhile (i--) {\n";
		printf "\t\tif (tag == tags_$s[i]) {\n";
		printf "\t\t\treturn 1;\n";
		printf "\t\t}\n";
		printf "\t\tif (tag > tags_$s[i]) {\n";
		printf "\t\t\treturn 0;\n";
		printf "\t\t}\n";
		printf "\t}\n";
		printf "\treturn 0;\n";
		printf "}\n";
		printf "\n";

	else

		printf "int clew_tag_is_group_$s (uint32_t tag)\n";
		printf "{\n";
		printf "\treturn (bsearch(&tag, tags_$s, (sizeof(tags_$s) / sizeof(tags_$s[0])) - 1, sizeof(tags_$s[0]), compare_tag) != NULL) ? 1 : 0;\n";
		printf "}\n";
		printf "\n";

	fi

	printf "uint32_t * clew_tags_group_$s (void)\n";
	printf "{\n";
	printf "\treturn tags_$s;\n";
	printf "}\n";
	printf "\n";

done;

printf "static uint32_t tag_groups[] = {\n";
for t in ${groups}; do
	printf "\tclew_tag_group_$t,\n";
done;
printf "\tclew_tag_unknown,\n";
printf "};\n";
printf "\n";

printf "uint32_t * clew_tag_groups (void)\n";
printf "{\n";
printf "\treturn tag_groups;\n";
printf "}\n";
printf "\n";

printf "struct tag_group_s {\n";
printf "\tconst char *group;\n";
printf "\tuint32_t * (*function) (void);\n";
printf "};\n";
printf "\n";

printf "struct tag_group_v {\n";
printf "\tuint32_t group;\n";
printf "\tuint32_t * (*function) (void);\n";
printf "};\n";
printf "\n";

printf "static struct tag_group_s tag_groups_s[] = {\n";
for s in $groups; do
	printf "\t{ \"$s\", clew_tags_group_$s },\n";
done;
printf "};\n";
printf "\n";

printf "static struct tag_group_v tag_groups_v[] = {\n";
for s in $groups; do
	printf "\t{ clew_tag_group_$s, clew_tags_group_$s },\n";
done;
printf "};\n";
printf "\n";

printf "static int compare_tag_group_string (const void *a, const void *b)\n";
printf "{\n";
printf "\tstruct tag_group_s *key = (struct tag_group_s *) a;\n";
printf "\tstruct tag_group_s *tag = (struct tag_group_s *) b;\n";
printf "\treturn strcasecmp(key->group, tag->group);\n";
printf "};\n";
printf "\n";

printf "static int compare_tag_group_value (const void *a, const void *b)\n";
printf "{\n";
printf "\tstruct tag_group_v *key = (struct tag_group_v *) a;\n";
printf "\tstruct tag_group_v *tag = (struct tag_group_v *) b;\n";
printf "\tif (key->group < tag->group) { return - 1; }\n";
printf "\tif (key->group > tag->group) { return 1; }\n";
printf "\treturn 0;\n";
printf "};\n";
printf "\n";

printf "uint32_t * clew_tags_group_string (const char *group)\n";
printf "{\n";
printf "\tstruct tag_group_s k;\n";
printf "\tstruct tag_group_s *g;\n";
printf "\tif (group == NULL) {\n";
printf "\t\treturn NULL;\n";
printf "\t}\n";
printf "\tif (strncmp(group, \"clew_tag_group_\", strlen(\"clew_tag_group_\")) == 0) {\n";
printf "\t\tk.group = group + strlen(\"clew_tag_group_\");\n";
printf "\t} else {\n";
printf "\t\tk.group = group;\n";
printf "\t}\n";
printf "\tg = (struct tag_group_s *) bsearch(&k, tag_groups_s, sizeof(tag_groups_s) / sizeof(tag_groups_s[0]), sizeof(tag_groups_s[0]), compare_tag_group_string);\n";
printf "\treturn (g != NULL) ? g->function() : NULL;\n";
printf "}\n";
printf "\n";

printf "uint32_t * clew_tags_group_value (uint32_t group)\n";
printf "{\n";
printf "\tstruct tag_group_v k;\n";
printf "\tstruct tag_group_v *g;\n";
printf "\tk.group = group;\n";
printf "\tg = (struct tag_group_v *) bsearch(&k, tag_groups_v, sizeof(tag_groups_v) / sizeof(tag_groups_v[0]), sizeof(tag_groups_v[0]), compare_tag_group_value);\n";
printf "\treturn (g != NULL) ? g->function() : NULL;\n";
printf "}\n";
printf "\n";

printf "void clew_tag_init (void)\n";
printf "{\n";
printf "\tclew_debugf(\"init tag\");\n";
printf "\tclew_debugf(\"  sorting tag values\");\n";
printf "\tqsort(tags_v, sizeof(tags_v) / sizeof(tags_v[0]), sizeof(tags_v[0]), compare_tag_value);\n";
printf "\tclew_debugf(\"  sorting tag names\");\n";
printf "\tqsort(tags_n, sizeof(tags_n) / sizeof(tags_n[0]), sizeof(tags_n[0]), compare_tag_name);\n";
printf "\tclew_debugf(\"  sorting tag country values\");\n";
printf "\tqsort(tags_country_v, sizeof(tags_country_v) / sizeof(tags_country_v[0]), sizeof(tags_country_v[0]), compare_tag_value);\n";
printf "\tclew_debugf(\"  sorting tag language values\");\n";
printf "\tqsort(tags_language_v, sizeof(tags_language_v) / sizeof(tags_language_v[0]), sizeof(tags_language_v[0]), compare_tag_value);\n";
printf "\tclew_debugf(\"  sorting tag group values\");\n";
printf "\tqsort(tag_groups, (sizeof(tag_groups) / sizeof(tag_groups[0])) - 1, sizeof(tag_groups[0]), compare_tag);\n";
printf "\tclew_debugf(\"  sorting tag group strings\");\n";
printf "\tqsort(tag_groups_s, sizeof(tag_groups_s) / sizeof(tag_groups_s[0]), sizeof(tag_groups_s[0]), compare_tag_group_string);\n";
printf "\tclew_debugf(\"  sorting tag group values\");\n";
printf "\tqsort(tag_groups_v, sizeof(tag_groups_v) / sizeof(tag_groups_v[0]), sizeof(tag_groups_v[0]), compare_tag_group_value);\n";
for s in $groups; do
	printf "\tclew_debugf(\"    sorting tag $s values\");\n";
	printf "\tqsort(tags_$s, (sizeof(tags_$s) / sizeof(tags_$s[0])) - 1, sizeof(tags_$s[0]), compare_tag);\n";
done;
printf "}\n";
printf "\n";

printf "void clew_tag_fini (void)\n";
printf "{\n";
printf "\tclew_debugf(\"fini tag\");\n";
printf "}\n";
