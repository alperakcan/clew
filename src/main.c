
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <getopt.h>

#define CLEW_DEBUG_NAME                 "clew"
#include "debug.h"
#include "input.h"
#include "bound.h"
#include "point.h"
#include "bitmap.h"
#include "stack.h"
#include "expression.h"
#include "tag.h"

#define OPTION_HELP                     'h'

#define OPTION_INPUT                    'i'
#define OPTION_OUTPUT                   'o'

#define OPTION_FILTER                   'f'
#define OPTION_POINTS                   'p'

#define OPTION_CLIP_PATH                0x200
#define OPTION_CLIP_BOUND               0x201
#define OPTION_CLIP_OFFSET              'c'
#define OPTION_CLIP_STRATEGY            0x203

#define OPTION_KEEP_TAGS                'k'
#define OPTION_KEEP_TAGS_NODE           0x301
#define OPTION_KEEP_TAGS_WAY            0x302
#define OPTION_KEEP_TAGS_RELATION       0x303

#define OPTION_DROP_TAGS                0x400
#define OPTION_DROP_TAGS_NODE           0x401
#define OPTION_DROP_TAGS_WAY            0x402
#define OPTION_DROP_TAGS_RELATION       0x403

#define OPTION_KEEP_NODES               'n'
#define OPTION_KEEP_WAYS                'w'
#define OPTION_KEEP_RELATIONS           'r'

#define OPTION_DROP_NODES               0x504
#define OPTION_DROP_WAYS                0x505
#define OPTION_DROP_RELATIONS           0x506

static const char *g_short_options     = "+i:o:c:f:p:k:n:w:r:v:h";
static struct option g_long_options[] = {
        { "help",               no_argument,            0,      OPTION_HELP                     },
        { "input",              required_argument,      0,      OPTION_INPUT                    },
        { "output",             required_argument,      0,      OPTION_OUTPUT                   },
        { "clip-path",          required_argument,      0,      OPTION_CLIP_PATH                },
        { "clip-bound",         required_argument,      0,      OPTION_CLIP_BOUND               },
        { "clip-offset",        required_argument,      0,      OPTION_CLIP_OFFSET              },
        { "clip-strategy",      required_argument,      0,      OPTION_CLIP_STRATEGY            },
        { "filter",             required_argument,      0,      OPTION_FILTER                   },
        { "points",             required_argument,      0,      OPTION_POINTS                   },
        { "keep-tags",          required_argument,      0,      OPTION_KEEP_TAGS                },
        { "keep-tags-node",     required_argument,      0,      OPTION_KEEP_TAGS_NODE           },
        { "keep-tags-way",      required_argument,      0,      OPTION_KEEP_TAGS_WAY            },
        { "keep-tags-relation", required_argument,      0,      OPTION_KEEP_TAGS_RELATION       },
        { "drop-tags",          required_argument,      0,      OPTION_DROP_TAGS                },
        { "drop-tags-node",     required_argument,      0,      OPTION_DROP_TAGS_NODE           },
        { "drop-tags-way",      required_argument,      0,      OPTION_DROP_TAGS_WAY            },
        { "drop-tags-relation", required_argument,      0,      OPTION_DROP_TAGS_RELATION       },
        { "keep-nodes",         required_argument,      0,      OPTION_KEEP_NODES               },
        { "keep-ways",          required_argument,      0,      OPTION_KEEP_WAYS                },
        { "keep-relations",     required_argument,      0,      OPTION_KEEP_RELATIONS           },
        { "drop-nodes",         required_argument,      0,      OPTION_DROP_NODES               },
        { "drop-ways",          required_argument,      0,      OPTION_DROP_WAYS                },
        { "drop-relations",     required_argument,      0,      OPTION_DROP_RELATIONS           },
        { 0,                    0,                      0,      0                               }
};

enum {
        CLEW_STATE_INITIAL                      = 0,
        CLEW_STATE_SELECT                       = 1,
        CLEW_STATE_EXTRACT                      = 2
#define CLEW_STATE_INITIAL                      CLEW_STATE_INITIAL
#define CLEW_STATE_SELECT                       CLEW_STATE_SELECT
#define CLEW_STATE_EXTRACT                      CLEW_STATE_EXTRACT
};

enum {
        CLEW_CLIP_STRATEGY_UNKNOWN              = 0,
        CLEW_CLIP_STRATEGY_SIMPLE               = 1,
        CLEW_CLIP_STRATEGY_COMPLETE_WAYS        = 2,
        CLEW_CLIP_STRATEGY_SMART                = 3
#define CLEW_CLIP_STRATEGY_UNKNOWN              CLEW_CLIP_STRATEGY_UNKNOWN
#define CLEW_CLIP_STRATEGY_SIMPLE               CLEW_CLIP_STRATEGY_SIMPLE
#define CLEW_CLIP_STRATEGY_COMPLETE_WAYS        CLEW_CLIP_STRATEGY_COMPLETE_WAYS
#define CLEW_CLIP_STRATEGY_SMART                CLEW_CLIP_STRATEGY_SMART
};

enum {
        CLEW_READ_STATE_UNKNOWN                 = 0,
        CLEW_READ_STATE_BOUNDS                  = 1,
        CLEW_READ_STATE_NODE                    = 2,
        CLEW_READ_STATE_WAY                     = 3,
        CLEW_READ_STATE_RELATION                = 4,
        CLEW_READ_STATE_TAG                     = 5,
        CLEW_READ_STATE_ND                      = 6,
        CLEW_READ_STATE_MEMBER                  = 7
#define CLEW_READ_STATE_UNKNOWN                 CLEW_READ_STATE_UNKNOWN
#define CLEW_READ_STATE_BOUNDS                  CLEW_READ_STATE_BOUNDS
#define CLEW_READ_STATE_NODE                    CLEW_READ_STATE_NODE
#define CLEW_READ_STATE_WAY                     CLEW_READ_STATE_WAY
#define CLEW_READ_STATE_RELATION                CLEW_READ_STATE_RELATION
#define CLEW_READ_STATE_TAG                     CLEW_READ_STATE_TAG
#define CLEW_READ_STATE_ND                      CLEW_READ_STATE_ND
#define CLEW_READ_STATE_MEMBER                  CLEW_READ_STATE_MEMBER
};

struct clew_options {
        struct clew_stack inputs;
        const char *output;
        struct clew_stack clip_path;
        int clip_strategy;
        struct clew_stack points;
        struct clew_expression *filter;
        struct clew_expression *keep_tags;
        struct clew_expression *keep_tags_node;
        struct clew_expression *keep_tags_way;
        struct clew_expression *keep_tags_relation;
        struct clew_expression *drop_tags;
        struct clew_expression *drop_tags_node;
        struct clew_expression *drop_tags_way;
        struct clew_expression *drop_tags_relation;
        int keep_nodes;
        int keep_ways;
        int keep_relations;
};

struct clew_node {
        uint64_t id;
        int32_t lon;
        int32_t lat;
        uint32_t *tags;
};

struct clew_way {
        uint64_t id;
        uint32_t *tags;
        uint64_t *refs;
};

struct clew {
        struct clew_options options;

        int state;

        struct clew_bitmap node_ids;
        struct clew_bitmap way_ids;
        struct clew_bitmap relation_ids;

        struct clew_stack nodes;
        struct clew_stack ways;
        struct clew_stack relations;

        struct clew_stack read_state;

        uint64_t read_id;
        int64_t read_lon;
        int64_t read_lat;

        struct clew_stack read_tags;
        #define READ_TAG_K_LENGTH       512
        #define READ_TAG_V_LENGTH       512
        #define READ_TAG_S_LENGTH       (READ_TAG_K_LENGTH + READ_TAG_V_LENGTH)
        char read_tag_k[512];
        char read_tag_v[512];
        char read_tag_s[1024];

        struct clew_stack read_refs;

        uint64_t read_node_start;
        uint64_t read_way_start;
        uint64_t read_relation_start;
};

static int tags_expression_match_has (void *context, uint32_t tag);

static void parse_tag_fix_layer (char *k, char *v);
static void parse_tag_fix (char *k, char *v);

static int input_callback_select_bounds_start (struct clew_input *input, void *context);
static int input_callback_select_bounds_end (struct clew_input *input, void *context);
static int input_callback_select_node_start (struct clew_input *input, void *context);
static int input_callback_select_node_end (struct clew_input *input, void *context);
static int input_callback_select_way_start (struct clew_input *input, void *context);
static int input_callback_select_way_end (struct clew_input *input, void *context);
static int input_callback_select_relation_start (struct clew_input *input, void *context);
static int input_callback_select_relation_end (struct clew_input *input, void *context);
static int input_callback_select_tag_start (struct clew_input *input, void *context);
static int input_callback_select_tag_end (struct clew_input *input, void *context);
static int input_callback_select_nd_start (struct clew_input *input, void *context);
static int input_callback_select_nd_end (struct clew_input *input, void *context);
static int input_callback_select_member_start (struct clew_input *input, void *context);
static int input_callback_select_member_end (struct clew_input *input, void *context);
static int input_callback_select_minlon (struct clew_input *input, void *context, int32_t lon);
static int input_callback_select_minlat (struct clew_input *input, void *context, int32_t lat);
static int input_callback_select_maxlon (struct clew_input *input, void *context, int32_t lon);
static int input_callback_select_maxlat (struct clew_input *input, void *context, int32_t lat);
static int input_callback_select_id (struct clew_input *input, void *context, uint64_t id);
static int input_callback_select_lon (struct clew_input *input, void *context, int32_t lon);
static int input_callback_select_lat (struct clew_input *input, void *context, int32_t lat);
static int input_callback_select_ref (struct clew_input *input, void *context, uint64_t ref);
static int input_callback_select_type (struct clew_input *input, void *context, const char *type);
static int input_callback_select_role (struct clew_input *input, void *context, const char *role);
static int input_callback_select_k (struct clew_input *input, void *context, const char *k);
static int input_callback_select_v (struct clew_input *input, void *context, const char *v);
static int input_callback_select_error (struct clew_input *input, void *context, unsigned int reason);

static int input_callback_extract_bounds_start (struct clew_input *input, void *context);
static int input_callback_extract_bounds_end (struct clew_input *input, void *context);
static int input_callback_extract_node_start (struct clew_input *input, void *context);
static int input_callback_extract_node_end (struct clew_input *input, void *context);
static int input_callback_extract_way_start (struct clew_input *input, void *context);
static int input_callback_extract_way_end (struct clew_input *input, void *context);
static int input_callback_extract_relation_start (struct clew_input *input, void *context);
static int input_callback_extract_relation_end (struct clew_input *input, void *context);
static int input_callback_extract_tag_start (struct clew_input *input, void *context);
static int input_callback_extract_tag_end (struct clew_input *input, void *context);
static int input_callback_extract_nd_start (struct clew_input *input, void *context);
static int input_callback_extract_nd_end (struct clew_input *input, void *context);
static int input_callback_extract_member_start (struct clew_input *input, void *context);
static int input_callback_extract_member_end (struct clew_input *input, void *context);
static int input_callback_extract_minlon (struct clew_input *input, void *context, int32_t lon);
static int input_callback_extract_minlat (struct clew_input *input, void *context, int32_t lat);
static int input_callback_extract_maxlon (struct clew_input *input, void *context, int32_t lon);
static int input_callback_extract_maxlat (struct clew_input *input, void *context, int32_t lat);
static int input_callback_extract_id (struct clew_input *input, void *context, uint64_t id);
static int input_callback_extract_lon (struct clew_input *input, void *context, int32_t lon);
static int input_callback_extract_lat (struct clew_input *input, void *context, int32_t lat);
static int input_callback_extract_ref (struct clew_input *input, void *context, uint64_t ref);
static int input_callback_extract_type (struct clew_input *input, void *context, const char *type);
static int input_callback_extract_role (struct clew_input *input, void *context, const char *role);
static int input_callback_extract_k (struct clew_input *input, void *context, const char *k);
static int input_callback_extract_v (struct clew_input *input, void *context, const char *v);
static int input_callback_extract_error (struct clew_input *input, void *context, unsigned int reason);

static void clew_node_destroy (struct clew_node *node);

static const char * clew_clip_strategy_string (int strategy);
static int clew_clip_strategy_value (const char *strategy);

static void print_help (const char *pname)
{
	fprintf(stdout, "%s usage:\n", pname);
	fprintf(stdout, "\n");
	fprintf(stdout, "  --input              / -i : input path\n");
	fprintf(stdout, "  --output             / -o: output path\n");
	fprintf(stdout, "  --clip-path              : clip path, ex: lon1,lat1 lon2,lat2 ... (default: \"\")\n");
	fprintf(stdout, "  --clip-bound             : clip bound, ex: minlon,minlat,maxlon,maxlat (default: \"\")\n");
	fprintf(stdout, "  --clip-offset        / -c: clip offset in meters (default: 0)\n");
	fprintf(stdout, "  --clip-strategy          : clip strategy; simple, complete_ways, smart (default: complete_ways)\n");
	fprintf(stdout, "  --filter             / -f: filter expression (default: \"\")\n");
	fprintf(stdout, "  --points             / -p: points to visit, ex: lon1,lat1 lon2,lat2 ... (default: \"\")\n");
	fprintf(stdout, "  --keep-nodes         / -n: filter nodes (default: 0)\n");
	fprintf(stdout, "  --keep-ways          / -w: filter ways (default: 0)\n");
	fprintf(stdout, "  --keep-relations     / -r: filter relations (default: 0)\n");
	fprintf(stdout, "  --keep-tags          / -k: keep tag (default: \"\")\n");
	fprintf(stdout, "  --keep-tags-node         : keep node tag (default: \"\")\n");
	fprintf(stdout, "  --keep-tags-way          : keep way tag (default: \"\")\n");
	fprintf(stdout, "  --keep-tags-relation     : keep relation tag (default: \"\")\n");
	fprintf(stdout, "\n");
        fprintf(stdout, "clip strategies;\n");
	fprintf(stdout, "\n");
        fprintf(stdout, "  simple: Runs in a single pass. The extract will contain all nodes inside the region and all ways referencing those nodes "
                        "as well as all relations referencing any nodes or ways already included. Ways crossing the region boundary will not be "
                        "reference-complete. Relations will not be reference-complete. This strategy is fast, because it reads the input only once, "
                        "but the result is not enough for most use cases. It is the only strategy that will work when reading from a socket or pipe.\n");
	fprintf(stdout, "\n");
        fprintf(stdout, "  complete_ways: Runs in two passes. The extract will contain all nodes inside the region and all ways referencing those "
                        "nodes as well as all nodes referenced by those ways. The extract will also contain all relations referenced by nodes inside "
                        "the region or ways already included and, recursively, their parent relations. The ways are reference-complete, but the "
                        "relations are not.\n");
	fprintf(stdout, "\n");
        fprintf(stdout, "  smart: Runs in three passes. The extract will contain all nodes inside the region and all ways referencing those nodes as "
                        "well as all nodes referenced by those ways. The extract will also contain all relations referenced by nodes inside the region "
                        "or ways already included and, recursively, their parent relations. The extract will also contain all nodes and ways (and the "
                        "nodes they reference) referenced by relations tagged “type=multipolygon” directly referencing any nodes in the region or ways "
                        "referencing nodes in the region. The ways are reference-complete, and all multipolygon relations referencing nodes in the "
                        "regions or ways that have nodes in the region are reference-complete. Other relations are not.\n");
	fprintf(stdout, "\n");
        fprintf(stdout, "highway tags;\n");
	fprintf(stdout, "\n");
	fprintf(stdout, "  roads:\n");
        fprintf(stdout, "    highway_motorway      : A restricted access major divided highway, normally with 2 or more running lanes plus emergency hard shoulder. Equivalent to the Freeway, Autobahn, etc..\n");
        fprintf(stdout, "    highway_trunk         : The most important roads in a country's system that aren't motorways. (Need not necessarily be a divided highway.\n");
        fprintf(stdout, "    highway_primary       : The next most important roads in a country's system. (Often link larger towns.)\n");
        fprintf(stdout, "    highway_secondary     : The next most important roads in a country's system. (Often link towns.)\n");
        fprintf(stdout, "    highway_tertiary      : The next most important roads in a country's system. (Often link smaller towns and villages)\n");
        fprintf(stdout, "    highway_unclassified  : The least important through roads in a country's system – i.e. minor roads of a lower classification than tertiary, but which serve a purpose other than access to properties. (Often link villages and hamlets.)\n");
        fprintf(stdout, "    highway_residential   : Roads which serve as an access to housing, without function of connecting settlements. Often lined with housing.\n");
	fprintf(stdout, "\n");
	fprintf(stdout, "  link roads:\n");
        fprintf(stdout, "    highway_motorway_link : The link roads (sliproads/ramps) leading to/from a motorway from/to a motorway or lower class highway. Normally with the same motorway restrictions.\n");
        fprintf(stdout, "    highway_trunk_link    : The link roads (sliproads/ramps) leading to/from a trunk road from/to a trunk road or lower class highway.\n");
        fprintf(stdout, "    highway_primary_link  : The link roads (sliproads/ramps) leading to/from a primary road from/to a primary road or lower class highway.\n");
        fprintf(stdout, "    highway_secondary_link: The link roads (sliproads/ramps) leading to/from a secondary road from/to a secondary road or lower class highway.\n");
        fprintf(stdout, "    highway_tertiary_link : The link roads (sliproads/ramps) leading to/from a tertiary road from/to a tertiary road or lower class highway.\n");
	fprintf(stdout, "\n");
	fprintf(stdout, "  special road types:\n");
        fprintf(stdout, "    highway_living_street : For living streets, which are residential streets where pedestrians have legal priority over cars, speeds are kept very low.\n");
        fprintf(stdout, "    highway_service       : For access roads to, or within an industrial estate, camp site, business park, car park, alleys, etc.\n");
        fprintf(stdout, "    highway_pedestrian    : For roads used mainly/exclusively for pedestrians in shopping and some residential areas which may allow access by motorised vehicles only for very limited periods of the day.\n");
        fprintf(stdout, "    highway_track         : Roads for mostly agricultural or forestry uses.\n");
        fprintf(stdout, "    highway_bus_guideway  : A busway where the vehicle guided by the way (though not a railway) and is not suitable for other traffic.\n");
        fprintf(stdout, "    highway_escape        : For runaway truck ramps, runaway truck lanes, emergency escape ramps, or truck arrester beds.\n");
        fprintf(stdout, "    highway_raceway       : A course or track for (motor) racing\n");
        fprintf(stdout, "    highway_road          : A road/way/street/motorway/etc. of unknown type. It can stand for anything ranging from a footpath to a motorway.\n");
        fprintf(stdout, "    highway_bus_way       : A dedicated roadway for bus rapid transit systems\n");
	fprintf(stdout, "\n");
	fprintf(stdout, "  paths:\n");
        fprintf(stdout, "    highway_footway       : For designated footpaths; i.e., mainly/exclusively for pedestrians. This includes walking tracks and gravel paths.\n");
        fprintf(stdout, "    highway_bridleway     : For horse riders. Pedestrians are usually also permitted, cyclists may be permitted depending on local rules/laws. Motor vehicles are forbidden.\n");
        fprintf(stdout, "    highway_steps         : For flights of steps (stairs) on footways.\n");
        fprintf(stdout, "    highway_corridor      : For a hallway inside of a building.\n");
        fprintf(stdout, "    highway_path          : A non-specific path.\n");
        fprintf(stdout, "    highway_via_ferrata   : A via ferrata is a route equipped with fixed cables, stemples, ladders, and bridges in order to increase ease and security for climbers.\n");
}

static int tags_expression_match_has (void *context, uint32_t tag)
{
        uint64_t pos;
        struct clew_stack *tags = context;
        pos = clew_stack_search_uint32(tags, tag);
        return (pos == UINT64_MAX) ? 0 : 1;
}

static void parse_tag_fix_layer (char *k, char *v)
{
        int layer;
        (void) k;
        layer = atoi(v);
        if (layer == 0) {
                snprintf(v, READ_TAG_V_LENGTH, "layer_ground");
        } else if (layer > 0) {
                snprintf(v, READ_TAG_V_LENGTH, "layer_above_%d", layer);
        } else {
                snprintf(v, READ_TAG_V_LENGTH, "layer_below_%d", -layer);
        }
}

static void parse_tag_fix (char *k, char *v)
{
        char *c;
        for (c = k; c && *c; c++) {
                if (*c == '-') {
                        *c = '_';
                }
        }
        for (c = v; c && *c; c++) {
                if (*c == '-') {
                        *c = '_';
                }
        }
        if (strcasecmp(k, "layer") == 0) {
                parse_tag_fix_layer(k, v);
        }
}

static int input_callback_select_bounds_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_UNKNOWN) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_UNKNOWN);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_BOUNDS);

        return 0;
bail:   return -1;
}

static int input_callback_select_bounds_end (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_BOUNDS) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_BOUNDS);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        return 0;
bail:   return -1;
}

static int input_callback_select_node_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew->read_node_start == 0) {
                clew_infof("      reading nodes");
        }
        clew->read_node_start++;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_UNKNOWN) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_UNKNOWN);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_NODE);

        clew_stack_reset(&clew->read_tags);

        if (clew->options.keep_nodes == 0) {
                goto skip;
        }

        return 0;
skip:   return 1;
bail:   return -1;
}

static int input_callback_select_node_end (struct clew_input *input, void *context)
{
        int rc;
        int match;
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_NODE) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        match = 0;
        if (clew_stack_count(&clew->read_tags) > 0) {
                clew_stack_sort_uint32(&clew->read_tags);
                match = clew_expression_match(clew->options.filter, &clew->read_tags, NULL, NULL, NULL, tags_expression_match_has);
        }
        if (match == 0) {
                goto out;
        }

        rc = clew_bitmap_mark(&clew->node_ids, clew->read_id);
        if (rc < 0) {
                clew_errorf("can not push node id");
                goto bail;
        }

out:    return 0;
bail:   return -1;
}

static int input_callback_select_way_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew->read_way_start == 0) {
                clew_infof("      reading ways");
        }
        clew->read_way_start++;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_UNKNOWN) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_UNKNOWN);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_WAY);

        clew_stack_reset(&clew->read_tags);
        clew_stack_reset(&clew->read_refs);

        if (clew->options.keep_ways == 0) {
                goto skip;
        }

        return 0;
skip:   return 1;
bail:   return -1;
}

static int input_callback_select_way_end (struct clew_input *input, void *context)
{
        int rc;
        int match;

        uint64_t i;
        uint64_t il;
        uint64_t ref;

        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_WAY) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_WAY);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        match = 0;
        if (clew_stack_count(&clew->read_tags) > 0) {
                clew_stack_sort_uint32(&clew->read_tags);
                match = clew_expression_match(clew->options.filter, &clew->read_tags, NULL, NULL, NULL, tags_expression_match_has);
        }
        if (match == 0) {
                goto out;
        }

        rc = clew_bitmap_mark(&clew->way_ids, clew->read_id);
        if (rc < 0) {
                clew_errorf("can not push way id");
                goto bail;
        }

        for (i = 0, il = clew_stack_count(&clew->read_refs); i < il; i++) {
                ref = clew_stack_at_uint64(&clew->read_refs, i);
                rc = clew_bitmap_mark(&clew->node_ids, ref);
                if (rc < 0) {
                        clew_errorf("can not push node id");
                        goto bail;
                }
        }

out:    return 0;
bail:   return -1;
}

static int input_callback_select_relation_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew->read_relation_start == 0) {
                clew_infof("      reading relations");
        }
        clew->read_relation_start++;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_UNKNOWN) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_UNKNOWN);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_RELATION);

        clew_stack_reset(&clew->read_tags);
        clew_stack_reset(&clew->read_refs);

        if (clew->options.keep_relations == 0) {
                goto skip;
        }

        return 0;
skip:   return 1;
bail:   return -1;
}

static int input_callback_select_relation_end (struct clew_input *input, void *context)
{
        int rc;
        int match;
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_RELATION) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_RELATION);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        match = 0;
        if (clew_stack_count(&clew->read_tags) > 0) {
                clew_stack_sort_uint32(&clew->read_tags);
                match = clew_expression_match(clew->options.filter, &clew->read_tags, NULL, NULL, NULL, tags_expression_match_has);
        }
        if (match == 0) {
                goto out;
        }

        rc = clew_bitmap_mark(&clew->relation_ids, clew->read_id);
        if (rc < 0) {
                clew_errorf("can not push relation id");
                goto bail;
        }

out:    return 0;
bail:   return -1;
}

static int input_callback_select_tag_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        switch (clew_stack_peek_uint32(&clew->read_state)) {
                case CLEW_READ_STATE_NODE:
                case CLEW_READ_STATE_WAY:
                case CLEW_READ_STATE_RELATION:
                        break;
                default:
                       clew_errorf("read_state is invalid, %d != %d || %d || %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE, CLEW_READ_STATE_WAY, CLEW_READ_STATE_RELATION);
                        goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_TAG);

        clew->read_tag_k[0] = '\0';
        clew->read_tag_v[0] = '\0';

        return 0;
bail:   return -1;
}

static int input_callback_select_tag_end (struct clew_input *input, void *context)
{
        int rc;
        int kl;
        int vl;
        int sl;
        uint32_t tag;

        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_TAG) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_TAG);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        kl = strlen(clew->read_tag_k);
        if (kl <= 0) {
                clew_debugf("k is invalid: %s = %s", clew->read_tag_k, clew->read_tag_v);
                goto out;
        }
        vl = strlen(clew->read_tag_v);
        if (vl <= 0) {
                clew_debugf("v is invalid: %s = %s", clew->read_tag_k, clew->read_tag_v);
                goto out;
        }
        if (kl + 1 + vl >= (int) sizeof(clew->read_tag_s)) {
                clew_errorf("tag string too long: %s_%s", clew->read_tag_k, clew->read_tag_v);
                goto out;
        }

        parse_tag_fix(clew->read_tag_k, clew->read_tag_v);

        sl = 0;
        clew->read_tag_s[sl] = '\0';
        memcpy(clew->read_tag_s + sl, clew->read_tag_k, kl); sl += kl;
        memcpy(clew->read_tag_s + sl, "_", 1);               sl += 1;
        memcpy(clew->read_tag_s + sl, clew->read_tag_v, vl); sl += vl;
        clew->read_tag_s[sl] = '\0';

        tag = clew_tag_value(clew->read_tag_s);
        if (tag == clew_tag_unknown) {
                //clew_todof("tag is invalid, '%s' = '%s'", clew->read_tag_k, clew->read_tag_v);
                goto out;
        }

        rc = clew_stack_push_uint32(&clew->read_tags, tag);
        if (rc != 0) {
                clew_errorf("can not add tag");
                goto bail;
        }

out:    return 0;
bail:   return -1;
}

static int input_callback_select_nd_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_WAY) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_WAY);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_ND);

        return 0;
bail:   return -1;
}

static int input_callback_select_nd_end (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_ND) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_ND);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        return 0;
bail:   return -1;
}

static int input_callback_select_member_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_RELATION) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_RELATION);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_MEMBER);

        return 0;
bail:   return -1;
}

static int input_callback_select_member_end (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_MEMBER) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_MEMBER);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        return 0;
bail:   return -1;
}

static int input_callback_select_minlon (struct clew_input *input, void *context, int32_t lon)
{
        struct clew *clew = context;

        (void) input;
        (void) lon;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_BOUNDS) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_BOUNDS);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_select_minlat (struct clew_input *input, void *context, int32_t lat)
{
        struct clew *clew = context;

        (void) input;
        (void) lat;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_BOUNDS) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_BOUNDS);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_select_maxlon (struct clew_input *input, void *context, int32_t lon)
{
        struct clew *clew = context;

        (void) input;
        (void) lon;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_BOUNDS) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_BOUNDS);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_select_maxlat (struct clew_input *input, void *context, int32_t lat)
{
        struct clew *clew = context;

        (void) input;
        (void) lat;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_BOUNDS) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_BOUNDS);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_select_id (struct clew_input *input, void *context, uint64_t id)
{
        struct clew *clew = context;

        (void) input;

        switch (clew_stack_peek_uint32(&clew->read_state)) {
                case CLEW_READ_STATE_NODE:
                case CLEW_READ_STATE_WAY:
                case CLEW_READ_STATE_RELATION:
                        break;
                default:
                       clew_errorf("read_state is invalid, %d != %d || %d || %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE, CLEW_READ_STATE_WAY, CLEW_READ_STATE_RELATION);
                        goto bail;
        }

        clew->read_id = id;

        return 0;
bail:   return -1;
}

static int input_callback_select_lon (struct clew_input *input, void *context, int32_t lon)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_NODE) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE);
                goto bail;
        }

        clew->read_lon = lon;

        return 0;
bail:   return -1;
}

static int input_callback_select_lat (struct clew_input *input, void *context, int32_t lat)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_NODE) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE);
                goto bail;
        }

        clew->read_lat = lat;

        return 0;
bail:   return -1;
}

static int input_callback_select_ref (struct clew_input *input, void *context, uint64_t ref)
{
        int rc;
        struct clew *clew = context;

        (void) input;

        switch (clew_stack_peek_uint32(&clew->read_state)) {
                case CLEW_READ_STATE_ND:
                case CLEW_READ_STATE_MEMBER:
                case CLEW_READ_STATE_RELATION:
                        break;
                default:
                       clew_errorf("read_state is invalid, %d != %d || %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_ND, CLEW_READ_STATE_MEMBER);
                        goto bail;
        }

        rc = clew_stack_push_uint64(&clew->read_refs, ref);
        if (rc < 0) {
                clew_errorf("can not push ref");
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_select_type (struct clew_input *input, void *context, const char *type)
{
        struct clew *clew = context;

        (void) input;
        (void) type;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_MEMBER) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_MEMBER);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_select_role (struct clew_input *input, void *context, const char *role)
{
        struct clew *clew = context;

        (void) input;
        (void) role;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_MEMBER) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_MEMBER);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_select_k (struct clew_input *input, void *context, const char *k)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_TAG) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_TAG);
                goto bail;
        }

        strncpy(clew->read_tag_k, k, sizeof(clew->read_tag_k) - 1);
        clew->read_tag_k[sizeof(clew->read_tag_k) - 1] = '\0';

        return 0;
bail:   return -1;
}

static int input_callback_select_v (struct clew_input *input, void *context, const char *v)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_TAG) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_TAG);
                goto bail;
        }

        strncpy(clew->read_tag_v, v, sizeof(clew->read_tag_v) - 1);
        clew->read_tag_v[sizeof(clew->read_tag_v) - 1] = '\0';

        return 0;
bail:   return -1;
}

static int input_callback_select_error (struct clew_input *input, void *context, unsigned int reason)
{
        struct clew *clew = context;
        (void) input;
        (void) clew;
        (void) reason;
        return 0;
}

static int input_callback_extract_bounds_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_UNKNOWN) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_UNKNOWN);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_BOUNDS);

        return 0;
bail:   return -1;
}

static int input_callback_extract_bounds_end (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_BOUNDS) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_BOUNDS);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        return 0;
bail:   return -1;
}

static int input_callback_extract_node_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew->read_node_start == 0) {
                clew_infof("      reading nodes");
        }
        clew->read_node_start++;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_UNKNOWN) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_UNKNOWN);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_NODE);

        clew_stack_reset(&clew->read_tags);

        return 0;
bail:   return -1;
}

static int input_callback_extract_node_end (struct clew_input *input, void *context)
{
        int rc;
        struct clew_node *node;

        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_NODE) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        rc = clew_bitmap_marked(&clew->node_ids, clew->read_id);
        if (rc < 0) {
                clew_errorf("can not read bitmap");
                goto bail;
        } else if (rc == 0) {
                goto out;
        }

        node = malloc(sizeof(struct clew_node));
        if (node == NULL) {
                clew_errorf("can not allocate memory");
                goto bail;
        }
        memset(node, 0, sizeof(struct clew_node));
        node->id   = clew->read_id;
        node->lon  = clew->read_lon;
        node->lat  = clew->read_lat;
        node->tags = NULL;
        rc = clew_stack_push_ptr(&clew->nodes, node);
        if (rc < 0) {
                clew_errorf("can not push node");
                clew_node_destroy(node);
                goto bail;
        }

out:    return 0;
bail:   return -1;
}

static int input_callback_extract_way_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew->read_way_start == 0) {
                clew_infof("      reading ways");
        }
        clew->read_way_start++;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_UNKNOWN) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_UNKNOWN);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_WAY);

        clew_stack_reset(&clew->read_tags);
        clew_stack_reset(&clew->read_refs);

        if (clew->options.keep_ways == 0) {
                goto skip;
        }

        return 0;
skip:   return 1;
bail:   return -1;
}

static int input_callback_extract_way_end (struct clew_input *input, void *context)
{
        int rc;
        int match;

        uint64_t i;
        uint64_t il;
        uint64_t ref;

        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_WAY) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_WAY);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        match = 0;
        if (clew_stack_count(&clew->read_tags) > 0) {
                clew_stack_sort_uint32(&clew->read_tags);
                match = clew_expression_match(clew->options.filter, &clew->read_tags, NULL, NULL, NULL, tags_expression_match_has);
        }
        if (match == 0) {
                goto out;
        }

        rc = clew_bitmap_mark(&clew->way_ids, clew->read_id);
        if (rc < 0) {
                clew_errorf("can not push way id");
                goto bail;
        }

        for (i = 0, il = clew_stack_count(&clew->read_refs); i < il; i++) {
                ref = clew_stack_at_uint64(&clew->read_refs, i);
                rc = clew_bitmap_mark(&clew->node_ids, ref);
                if (rc < 0) {
                        clew_errorf("can not push node id");
                        goto bail;
                }
        }

out:    return 0;
bail:   return -1;
}

static int input_callback_extract_relation_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew->read_relation_start == 0) {
                clew_infof("      reading relations");
        }
        clew->read_relation_start++;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_UNKNOWN) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_UNKNOWN);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_RELATION);

        clew_stack_reset(&clew->read_tags);
        clew_stack_reset(&clew->read_refs);

        if (clew->options.keep_relations == 0) {
                goto skip;
        }

        return 0;
skip:   return 1;
bail:   return -1;
}

static int input_callback_extract_relation_end (struct clew_input *input, void *context)
{
        int rc;
        int match;
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_RELATION) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_RELATION);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        match = 0;
        if (clew_stack_count(&clew->read_tags) > 0) {
                clew_stack_sort_uint32(&clew->read_tags);
                match = clew_expression_match(clew->options.filter, &clew->read_tags, NULL, NULL, NULL, tags_expression_match_has);
        }
        if (match == 0) {
                goto out;
        }

        rc = clew_bitmap_mark(&clew->relation_ids, clew->read_id);
        if (rc < 0) {
                clew_errorf("can not push relation id");
                goto bail;
        }

out:    return 0;
bail:   return -1;
}

static int input_callback_extract_tag_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        switch (clew_stack_peek_uint32(&clew->read_state)) {
                case CLEW_READ_STATE_NODE:
                case CLEW_READ_STATE_WAY:
                case CLEW_READ_STATE_RELATION:
                        break;
                default:
                       clew_errorf("read_state is invalid, %d != %d || %d || %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE, CLEW_READ_STATE_WAY, CLEW_READ_STATE_RELATION);
                        goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_TAG);

        clew->read_tag_k[0] = '\0';
        clew->read_tag_v[0] = '\0';

        return 0;
bail:   return -1;
}

static int input_callback_extract_tag_end (struct clew_input *input, void *context)
{
        int rc;
        int kl;
        int vl;
        int sl;
        uint32_t tag;

        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_TAG) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_TAG);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        kl = strlen(clew->read_tag_k);
        if (kl <= 0) {
                clew_debugf("k is invalid: %s = %s", clew->read_tag_k, clew->read_tag_v);
                goto out;
        }
        vl = strlen(clew->read_tag_v);
        if (vl <= 0) {
                clew_debugf("v is invalid: %s = %s", clew->read_tag_k, clew->read_tag_v);
                goto out;
        }
        if (kl + 1 + vl >= (int) sizeof(clew->read_tag_s)) {
                clew_errorf("tag string too long: %s_%s", clew->read_tag_k, clew->read_tag_v);
                goto out;
        }

        parse_tag_fix(clew->read_tag_k, clew->read_tag_v);

        sl = 0;
        clew->read_tag_s[sl] = '\0';
        memcpy(clew->read_tag_s + sl, clew->read_tag_k, kl); sl += kl;
        memcpy(clew->read_tag_s + sl, "_", 1);               sl += 1;
        memcpy(clew->read_tag_s + sl, clew->read_tag_v, vl); sl += vl;
        clew->read_tag_s[sl] = '\0';

        tag = clew_tag_value(clew->read_tag_s);
        if (tag == clew_tag_unknown) {
                //clew_todof("tag is invalid, '%s' = '%s'", clew->read_tag_k, clew->read_tag_v);
                goto out;
        }

        rc = clew_stack_push_uint32(&clew->read_tags, tag);
        if (rc != 0) {
                clew_errorf("can not add tag");
                goto bail;
        }

out:    return 0;
bail:   return -1;
}

static int input_callback_extract_nd_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_WAY) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_WAY);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_ND);

        return 0;
bail:   return -1;
}

static int input_callback_extract_nd_end (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_ND) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_ND);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        return 0;
bail:   return -1;
}

static int input_callback_extract_member_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_RELATION) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_RELATION);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_MEMBER);

        return 0;
bail:   return -1;
}

static int input_callback_extract_member_end (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_MEMBER) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_MEMBER);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        return 0;
bail:   return -1;
}

static int input_callback_extract_minlon (struct clew_input *input, void *context, int32_t lon)
{
        struct clew *clew = context;

        (void) input;
        (void) lon;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_BOUNDS) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_BOUNDS);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_extract_minlat (struct clew_input *input, void *context, int32_t lat)
{
        struct clew *clew = context;

        (void) input;
        (void) lat;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_BOUNDS) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_BOUNDS);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_extract_maxlon (struct clew_input *input, void *context, int32_t lon)
{
        struct clew *clew = context;

        (void) input;
        (void) lon;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_BOUNDS) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_BOUNDS);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_extract_maxlat (struct clew_input *input, void *context, int32_t lat)
{
        struct clew *clew = context;

        (void) input;
        (void) lat;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_BOUNDS) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_BOUNDS);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_extract_id (struct clew_input *input, void *context, uint64_t id)
{
        struct clew *clew = context;

        (void) input;

        switch (clew_stack_peek_uint32(&clew->read_state)) {
                case CLEW_READ_STATE_NODE:
                case CLEW_READ_STATE_WAY:
                case CLEW_READ_STATE_RELATION:
                        break;
                default:
                       clew_errorf("read_state is invalid, %d != %d || %d || %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE, CLEW_READ_STATE_WAY, CLEW_READ_STATE_RELATION);
                        goto bail;
        }

        clew->read_id = id;

        return 0;
bail:   return -1;
}

static int input_callback_extract_lon (struct clew_input *input, void *context, int32_t lon)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_NODE) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE);
                goto bail;
        }

        clew->read_lon = lon;

        return 0;
bail:   return -1;
}

static int input_callback_extract_lat (struct clew_input *input, void *context, int32_t lat)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_NODE) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE);
                goto bail;
        }

        clew->read_lat = lat;

        return 0;
bail:   return -1;
}

static int input_callback_extract_ref (struct clew_input *input, void *context, uint64_t ref)
{
        int rc;
        struct clew *clew = context;

        (void) input;

        switch (clew_stack_peek_uint32(&clew->read_state)) {
                case CLEW_READ_STATE_ND:
                case CLEW_READ_STATE_MEMBER:
                case CLEW_READ_STATE_RELATION:
                        break;
                default:
                       clew_errorf("read_state is invalid, %d != %d || %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_ND, CLEW_READ_STATE_MEMBER);
                        goto bail;
        }

        rc = clew_stack_push_uint64(&clew->read_refs, ref);
        if (rc < 0) {
                clew_errorf("can not push ref");
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_extract_type (struct clew_input *input, void *context, const char *type)
{
        struct clew *clew = context;

        (void) input;
        (void) type;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_MEMBER) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_MEMBER);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_extract_role (struct clew_input *input, void *context, const char *role)
{
        struct clew *clew = context;

        (void) input;
        (void) role;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_MEMBER) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_MEMBER);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_extract_k (struct clew_input *input, void *context, const char *k)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_TAG) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_TAG);
                goto bail;
        }

        strncpy(clew->read_tag_k, k, sizeof(clew->read_tag_k) - 1);
        clew->read_tag_k[sizeof(clew->read_tag_k) - 1] = '\0';

        return 0;
bail:   return -1;
}

static int input_callback_extract_v (struct clew_input *input, void *context, const char *v)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_TAG) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_TAG);
                goto bail;
        }

        strncpy(clew->read_tag_v, v, sizeof(clew->read_tag_v) - 1);
        clew->read_tag_v[sizeof(clew->read_tag_v) - 1] = '\0';

        return 0;
bail:   return -1;
}

static int input_callback_extract_error (struct clew_input *input, void *context, unsigned int reason)
{
        struct clew *clew = context;
        (void) input;
        (void) clew;
        (void) reason;
        return 0;
}

static void node_stack_destroy_element (void *context, void *elem)
{
        (void) context;
        clew_node_destroy(*(struct clew_node **) elem);
}

static void clew_node_destroy (struct clew_node *node)
{
        if (node == NULL) {
                return;
        }
        if (node->tags != NULL) {
                free(node->tags);
        }
        free(node);
}

static const char * clew_clip_strategy_string (int strategy)
{
        if (strategy == CLEW_CLIP_STRATEGY_SIMPLE)              return "simple";
        if (strategy == CLEW_CLIP_STRATEGY_COMPLETE_WAYS)       return "complete-ways";
        if (strategy == CLEW_CLIP_STRATEGY_SMART)               return "smart";
        return "unknown";
}

static int clew_clip_strategy_value (const char *strategy)
{
        if (strategy == NULL)                           return CLEW_CLIP_STRATEGY_UNKNOWN;
        if (strcasecmp(strategy, "simple") == 0)        return CLEW_CLIP_STRATEGY_SIMPLE;
        if (strcasecmp(strategy, "complete-ways") == 0) return CLEW_CLIP_STRATEGY_COMPLETE_WAYS;
        if (strcasecmp(strategy, "complete_ways") == 0) return CLEW_CLIP_STRATEGY_COMPLETE_WAYS;
        if (strcasecmp(strategy, "smart") == 0)         return CLEW_CLIP_STRATEGY_SMART;
        return CLEW_CLIP_STRATEGY_UNKNOWN;
}

int main (int argc, char *argv[])
{
        int c;
        int option_index;

        int rs;
        int rc;

        uint64_t i;
        uint64_t il;

        struct clew_input *input;
        struct clew_input_init_options input_init_options;

        struct clew *clew;

        rs = 0;
        clew = NULL;

        clew_debug_init();
        clew_tag_init();

        clew = malloc(sizeof(struct clew));
        if (clew == NULL) {
                clew_errorf("can not allocate memory");
                goto bail;
        }
        memset(clew, 0, sizeof(struct clew));

        clew->options.inputs                    = clew_stack_init(sizeof(const char *));
        clew->options.output                    = NULL;
        clew->options.clip_path                 = clew_stack_init(sizeof(int32_t));
        clew->options.clip_strategy             = CLEW_CLIP_STRATEGY_SMART;
        clew->options.filter                    = NULL;
        clew->options.points                    = clew_stack_init(sizeof(int32_t));
        clew->options.keep_tags                 = NULL;
        clew->options.keep_tags_node            = NULL;
        clew->options.keep_tags_way             = NULL;
        clew->options.keep_tags_relation        = NULL;
        clew->options.drop_tags                 = NULL;
        clew->options.drop_tags_node            = NULL;
        clew->options.drop_tags_way             = NULL;
        clew->options.drop_tags_relation        = NULL;
        clew->options.keep_nodes                = 1;
        clew->options.keep_ways                 = 1;
        clew->options.keep_relations            = 1;

        clew->state             = CLEW_STATE_INITIAL;
        clew->read_state        = clew_stack_init(sizeof(uint32_t));
        clew->read_tags         = clew_stack_init(sizeof(uint32_t));
        clew->read_refs         = clew_stack_init(sizeof(uint64_t));
        clew->node_ids          = clew_bitmap_init(64 * 1024  * 1024);
        clew->way_ids           = clew_bitmap_init(64 * 1024  * 1024);
        clew->relation_ids      = clew_bitmap_init(64 * 1024 * 1024);
        clew->nodes             = clew_stack_init4(sizeof(struct clew_node *), 64 * 1024, node_stack_destroy_element, NULL);
        clew->ways              = clew_stack_init2_ptr(64 * 1024);
        clew->relations         = clew_stack_init2_ptr(64 * 1024);

        optind = 1;
        while (1) {
                c = getopt_long(argc, argv, g_short_options, g_long_options, &option_index);
                if (c == -1) {
                        break;
                }
                switch (c) {
                        case OPTION_HELP:
                                print_help(argv[0]);
                                goto out;
                }
        }

        optind = 1;
        while (1) {
                c = getopt_long(argc, argv, g_short_options, g_long_options, &option_index);
                if (c == -1) {
                        break;
                }
                switch (c) {
                        case OPTION_INPUT:
                                rc = clew_stack_push(&clew->options.inputs, &optarg);
                                if (rc < 0) {
                                        clew_errorf("can not add input to stack");
                                        goto bail;
                                }
                                break;
                        case OPTION_OUTPUT:
                                clew->options.output = optarg;
                                break;
                        case OPTION_FILTER:
                                if (clew->options.filter != NULL) {
                                        clew_errorf("filter already exists");
                                        goto bail;
                                }
                                clew->options.filter = clew_expression_create(optarg);
                                if (clew->options.filter == NULL) {
                                        clew_errorf("can not create expression: %s", optarg);
                                        goto bail;
                                }
                                break;
                        case OPTION_POINTS: {
                                int rc;

                                double x;
                                double y;
                                const char *ptr;

                                ptr = optarg;
                                while (ptr && *ptr == ' ') {
                                        ptr++;
                                }
                                while (ptr && *ptr) {
                                        rc = sscanf(ptr, "%lf,%lf", &x, &y);
                                        if (rc != 2) {
                                                clew_errorf("malformed format");
                                                goto bail;
                                        }
                                        clew_stack_push_int32(&clew->options.points, x * 1e7);
                                        clew_stack_push_int32(&clew->options.points, y * 1e7);
                                        ptr = strchr(ptr, ' ');
                                        if (ptr == NULL) {
                                                break;
                                        }
                                        while (ptr && *ptr == ' ') {
                                                ptr++;
                                        }
                                }
                        }       break;
                        case OPTION_KEEP_TAGS:
                                if (clew->options.keep_tags != NULL) {
                                        clew_errorf("keep_tags already exists");
                                        goto bail;
                                }
                                clew->options.keep_tags = clew_expression_create(optarg);
                                if (clew->options.keep_tags == NULL) {
                                        clew_errorf("can not create expression: %s", optarg);
                                        goto bail;
                                }
                                break;
                        case OPTION_KEEP_TAGS_NODE:
                                if (clew->options.keep_tags_node != NULL) {
                                        clew_errorf("keep_tags_node already exists");
                                        goto bail;
                                }
                                clew->options.keep_tags_node = clew_expression_create(optarg);
                                if (clew->options.keep_tags_node == NULL) {
                                        clew_errorf("can not create expression: %s", optarg);
                                        goto bail;
                                }
                                break;
                        case OPTION_KEEP_TAGS_WAY:
                                if (clew->options.keep_tags_way != NULL) {
                                        clew_errorf("keep_tags_way already exists");
                                        goto bail;
                                }
                                clew->options.keep_tags_way = clew_expression_create(optarg);
                                if (clew->options.keep_tags_way == NULL) {
                                        clew_errorf("can not create expression: %s", optarg);
                                        goto bail;
                                }
                                break;
                        case OPTION_KEEP_TAGS_RELATION:
                                if (clew->options.keep_tags_relation != NULL) {
                                        clew_errorf("keep_tags_relation already exists");
                                        goto bail;
                                }
                                clew->options.keep_tags_relation = clew_expression_create(optarg);
                                if (clew->options.keep_tags_relation == NULL) {
                                        clew_errorf("can not create expression: %s", optarg);
                                        goto bail;
                                }
                                break;
                        case OPTION_DROP_TAGS:
                                if (clew->options.drop_tags != NULL) {
                                        clew_errorf("drop_tags already exists");
                                        goto bail;
                                }
                                clew->options.drop_tags = clew_expression_create(optarg);
                                if (clew->options.drop_tags == NULL) {
                                        clew_errorf("can not create expression: %s", optarg);
                                        goto bail;
                                }
                                break;
                        case OPTION_DROP_TAGS_NODE:
                                if (clew->options.drop_tags_node != NULL) {
                                        clew_errorf("drop_tags_node already exists");
                                        goto bail;
                                }
                                clew->options.drop_tags_node = clew_expression_create(optarg);
                                if (clew->options.drop_tags_node == NULL) {
                                        clew_errorf("can not create expression: %s", optarg);
                                        goto bail;
                                }
                                break;
                        case OPTION_DROP_TAGS_WAY:
                                if (clew->options.drop_tags_way != NULL) {
                                        clew_errorf("drop_tags_way already exists");
                                        goto bail;
                                }
                                clew->options.drop_tags_way = clew_expression_create(optarg);
                                if (clew->options.drop_tags_way == NULL) {
                                        clew_errorf("can not create expression: %s", optarg);
                                        goto bail;
                                }
                                break;
                        case OPTION_DROP_TAGS_RELATION:
                                if (clew->options.drop_tags_relation != NULL) {
                                        clew_errorf("drop_tags_relation already exists");
                                        goto bail;
                                }
                                clew->options.drop_tags_relation = clew_expression_create(optarg);
                                if (clew->options.drop_tags_relation == NULL) {
                                        clew_errorf("can not create expression: %s", optarg);
                                        goto bail;
                                }
                                break;
                        case OPTION_KEEP_NODES:
                                clew->options.keep_nodes = !!atoi(optarg);
                                break;
                        case OPTION_KEEP_WAYS:
                                clew->options.keep_ways = !!atoi(optarg);
                                break;
                        case OPTION_KEEP_RELATIONS:
                                clew->options.keep_relations = !!atoi(optarg);
                                break;
                        case OPTION_DROP_NODES:
                                clew->options.keep_nodes = !atoi(optarg);
                                break;
                        case OPTION_DROP_WAYS:
                                clew->options.keep_ways = !atoi(optarg);
                                break;
                        case OPTION_DROP_RELATIONS:
                                clew->options.keep_relations = !atoi(optarg);
                                break;
                }
        }

        optind = 1;
        while (1) {
                c = getopt_long(argc, argv, g_short_options, g_long_options, &option_index);
                if (c == -1) {
                        break;
                }
                switch (c) {
                        case OPTION_CLIP_PATH: {
                                int rc;

                                double x;
                                double y;
                                const char *ptr;

                                int32_t lon0;
                                int32_t lat0;
                                int32_t lonn;
                                int32_t latn;
                                uint64_t count;

                                ptr = optarg;
                                while (ptr && *ptr == ' ') {
                                        ptr++;
                                }
                                while (ptr && *ptr) {
                                        rc = sscanf(ptr, "%lf,%lf", &x, &y);
                                        if (rc != 2) {
                                                clew_errorf("malformed format");
                                                goto bail;
                                        }
                                        clew_stack_push_int32(&clew->options.clip_path, x * 1e7);
                                        clew_stack_push_int32(&clew->options.clip_path, y * 1e7);
                                        ptr = strchr(ptr, ' ');
                                        if (ptr == NULL) {
                                                break;
                                        }
                                        while (ptr && *ptr == ' ') {
                                                ptr++;
                                        }
                                }

                                if (clew_stack_count(&clew->options.clip_path) < 4) {
                                        clew_errorf("malformed format, must have at least 2 points");
                                        goto bail;
                                }

                                lon0 = clew_stack_at_int32(&clew->options.clip_path, 0);
                                lat0 = clew_stack_at_int32(&clew->options.clip_path, 1);

                                count = clew_stack_count(&clew->options.clip_path);
                                lonn = clew_stack_at_int32(&clew->options.clip_path, count - 2);
                                latn = clew_stack_at_int32(&clew->options.clip_path, count - 1);

                                if (lon0 != lonn || lat0 != latn) {
                                        clew_stack_push_int32(&clew->options.clip_path, lon0);
                                        clew_stack_push_int32(&clew->options.clip_path, lat0);
                                }
                        }       break;
                        case OPTION_CLIP_BOUND: {
                                int rc;

                                double minlon;
                                double minlat;
                                double maxlon;
                                double maxlat;

                                if (clew_stack_count(&clew->options.clip_path) != 0) {
                                        clew_errorf("clip path is already set");
                                        goto bail;
                                }

                                rc = sscanf(optarg, "%lf,%lf,%lf,%lf", &minlon, &minlat, &maxlon, &maxlat);
                                if (rc != 4) {
                                        clew_errorf("malformed format");
                                        goto bail;
                                }

                                clew_stack_push_int32(&clew->options.clip_path, minlon * 1e7);
                                clew_stack_push_int32(&clew->options.clip_path, minlat * 1e7);

                                clew_stack_push_int32(&clew->options.clip_path, maxlon * 1e7);
                                clew_stack_push_int32(&clew->options.clip_path, minlat * 1e7);

                                clew_stack_push_int32(&clew->options.clip_path, maxlon * 1e7);
                                clew_stack_push_int32(&clew->options.clip_path, maxlat * 1e7);

                                clew_stack_push_int32(&clew->options.clip_path, minlon * 1e7);
                                clew_stack_push_int32(&clew->options.clip_path, maxlat * 1e7);

                                clew_stack_push_int32(&clew->options.clip_path, minlon * 1e7);
                                clew_stack_push_int32(&clew->options.clip_path, minlat * 1e7);

                        }       break;
                        case OPTION_CLIP_OFFSET: {
                                int i;
                                int il;
                                double offset;
                                struct clew_bound bound;
                                struct clew_point point;
                                struct clew_point npoint;
                                struct clew_point epoint;
                                struct clew_point spoint;
                                struct clew_point wpoint;

                                if (clew_stack_count(&clew->options.clip_path) != 0) {
                                        clew_errorf("clip path is already set");
                                        goto bail;
                                }

                                offset = atof(optarg);
                                if (offset <= 0) {
                                        clew_errorf("malformed offset, see help");
                                        goto bail;
                                }

                                bound = clew_bound_null();
                                for (i = 0, il = clew_stack_count(&clew->options.points); i < il; i += 2) {
                                        point = clew_point_init(clew_stack_at_int32(&clew->options.points, i + 0), clew_stack_at_int32(&clew->options.points, i + 1));
                                        npoint = clew_point_derived_position(&point, offset, 0);
                                        epoint = clew_point_derived_position(&point, offset, 90);
                                        spoint = clew_point_derived_position(&point, offset, 180);
                                        wpoint = clew_point_derived_position(&point, offset, 270);
                                        bound = clew_bound_union_point(&bound, &npoint);
                                        bound = clew_bound_union_point(&bound, &epoint);
                                        bound = clew_bound_union_point(&bound, &spoint);
                                        bound = clew_bound_union_point(&bound, &wpoint);
                                }

                                clew_stack_push_int32(&clew->options.clip_path, bound.minlon);
                                clew_stack_push_int32(&clew->options.clip_path, bound.minlat);

                                clew_stack_push_int32(&clew->options.clip_path, bound.maxlon);
                                clew_stack_push_int32(&clew->options.clip_path, bound.minlat);

                                clew_stack_push_int32(&clew->options.clip_path, bound.maxlon);
                                clew_stack_push_int32(&clew->options.clip_path, bound.maxlat);

                                clew_stack_push_int32(&clew->options.clip_path, bound.minlon);
                                clew_stack_push_int32(&clew->options.clip_path, bound.maxlat);

                                clew_stack_push_int32(&clew->options.clip_path, bound.minlon);
                                clew_stack_push_int32(&clew->options.clip_path, bound.minlat);

                        }       break;
                        case OPTION_CLIP_STRATEGY:
                                clew->options.clip_strategy = clew_clip_strategy_value(optarg);
                                if (clew->options.clip_strategy == CLEW_CLIP_STRATEGY_UNKNOWN) {
                                        clew_errorf("strategy is invalid, see help");
                                        goto bail;
                                }
                }
        }

        if (clew_stack_count(&clew->options.inputs) <= 0) {
                clew_errorf("inputs is invalid, see help");
                goto bail;
        }
        if (clew->options.output == NULL) {
                clew_errorf("output is invalid, see help");
                goto bail;
        }
        if (clew->options.filter == NULL) {
                clew_errorf("filter is invalid, see help");
                goto bail;
        }

        clew_infof("clew");
        clew_infof("  inputs             :");
        for (i = 0, il = clew_stack_count(&clew->options.inputs); i < il; i++) {
                clew_infof("    %s", *(char **) clew_stack_at(&clew->options.inputs, i));
        }
        clew_infof("  output             : %s", clew->options.output);
        clew_infof("  points             : %ld", clew_stack_count(&clew->options.points) / 2);
        for (i = 0, il = clew_stack_count(&clew->options.points); i < il; i += 2) {
                clew_infof("    %12.7f,%12.7f", clew_stack_at_int32(&clew->options.points, i + 0) / 1e7, clew_stack_at_int32(&clew->options.points, i + 1) / 1e7);
        }
        clew_infof("  clip-path          : %ld", clew_stack_count(&clew->options.clip_path) / 2);
        for (i = 0, il = clew_stack_count(&clew->options.clip_path); i < il; i += 2) {
                clew_infof("    %12.7f,%12.7f", clew_stack_at_int32(&clew->options.clip_path, i + 0) / 1e7, clew_stack_at_int32(&clew->options.clip_path, i + 1) / 1e7);
        }
        clew_infof("  clip-strategy      : '%s'", clew_clip_strategy_string(clew->options.clip_strategy));
        clew_infof("  filter             : '%s'", clew_expression_orig(clew->options.filter));
        clew_infof("  keep-tags          : '%s'", clew_expression_orig(clew->options.keep_tags));
        clew_infof("  keep-tags-node     : '%s'", clew_expression_orig(clew->options.keep_tags_node));
        clew_infof("  keep-tags-way      : '%s'", clew_expression_orig(clew->options.keep_tags_way));
        clew_infof("  keep-tags-relation : '%s'", clew_expression_orig(clew->options.keep_tags_relation));
        clew_infof("  drop-tags          : '%s'", clew_expression_orig(clew->options.drop_tags));
        clew_infof("  drop-tags-node     : '%s'", clew_expression_orig(clew->options.drop_tags_node));
        clew_infof("  drop-tags-way      : '%s'", clew_expression_orig(clew->options.drop_tags_way));
        clew_infof("  drop-tags-relation : '%s'", clew_expression_orig(clew->options.drop_tags_relation));
        clew_infof("  keep-nodes         : %d", clew->options.keep_nodes);
        clew_infof("  keep-keep_ways     : %d", clew->options.keep_ways);
        clew_infof("  keep-keep_relations: %d", clew->options.keep_relations);

        clew_infof("selecting");
        clew->state = CLEW_STATE_SELECT;

        clew_bitmap_reset(&clew->node_ids);
        clew_bitmap_reset(&clew->way_ids);
        clew_bitmap_reset(&clew->relation_ids);

        clew_stack_reset(&clew->read_state);
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_UNKNOWN);

        clew->read_node_start     = 0;
        clew->read_way_start      = 0;
        clew->read_relation_start = 0;

        clew_infof("  reading inputs");
        for (i = 0, il = clew_stack_count(&clew->options.inputs); i < il; i++) {
                clew_infof("    %ld: %s", i, *(char **) clew_stack_at(&clew->options.inputs, i));

                clew_input_init_options_default(&input_init_options);
                input_init_options.path                         = *(char **) clew_stack_at(&clew->options.inputs, i);
                input_init_options.callback_bounds_start        = input_callback_select_bounds_start;
                input_init_options.callback_bounds_end          = input_callback_select_bounds_end;
                input_init_options.callback_node_start          = input_callback_select_node_start;
                input_init_options.callback_node_end            = input_callback_select_node_end;
                input_init_options.callback_way_start           = input_callback_select_way_start;
                input_init_options.callback_way_end             = input_callback_select_way_end;
                input_init_options.callback_relation_start      = input_callback_select_relation_start;
                input_init_options.callback_relation_end        = input_callback_select_relation_end;
                input_init_options.callback_tag_start           = input_callback_select_tag_start;
                input_init_options.callback_tag_end             = input_callback_select_tag_end;
                input_init_options.callback_nd_start            = input_callback_select_nd_start;
                input_init_options.callback_nd_end              = input_callback_select_nd_end;
                input_init_options.callback_member_start        = input_callback_select_member_start;
                input_init_options.callback_member_end          = input_callback_select_member_end;
                input_init_options.callback_minlon              = input_callback_select_minlon;
                input_init_options.callback_minlat              = input_callback_select_minlat;
                input_init_options.callback_maxlon              = input_callback_select_maxlon;
                input_init_options.callback_maxlat              = input_callback_select_maxlat;
                input_init_options.callback_id                  = input_callback_select_id;
                input_init_options.callback_lat                 = input_callback_select_lat;
                input_init_options.callback_lon                 = input_callback_select_lon;
                input_init_options.callback_ref                 = input_callback_select_ref;
                input_init_options.callback_type                = input_callback_select_type;
                input_init_options.callback_role                = input_callback_select_role;
                input_init_options.callback_k                   = input_callback_select_k;
                input_init_options.callback_v                   = input_callback_select_v;
                input_init_options.callback_error               = input_callback_select_error;
                input_init_options.callback_context             = clew;

                input = clew_input_create(&input_init_options);
                if (input == NULL) {
                        clew_errorf("can not create input for path: %s", input_init_options.path);
                        goto bail;
                }
                while (clew_input_read(input) == 0) {
                        rc = clew_input_get_error(input);
                        if (rc != 0) {
                                clew_errorf("input error occured, error: %d", rc);
                                goto bail;
                        }
                }

                if (input != NULL) {
                        clew_input_destroy(input);
                }
        }

        clew_infof("  processed");
        clew_infof("    inputs   : %ld", clew_stack_count(&clew->options.inputs));
        clew_infof("    nodes    : %ld", clew->read_node_start);
        clew_infof("    ways     : %ld", clew->read_way_start);
        clew_infof("    relations: %ld", clew->read_relation_start);

        clew_infof("  selected");
        clew_infof("    nodes    : %ld", clew_bitmap_count(&clew->node_ids));
        clew_infof("    ways     : %ld", clew_bitmap_count(&clew->way_ids));
        clew_infof("    relations: %ld", clew_bitmap_count(&clew->relation_ids));

        clew_infof("extracting");
        clew->state = CLEW_STATE_EXTRACT;

        clew_stack_reset(&clew->read_state);
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_UNKNOWN);

        clew->read_node_start     = 0;
        clew->read_way_start      = 0;
        clew->read_relation_start = 0;

        clew_infof("  reading inputs");
        for (i = 0, il = clew_stack_count(&clew->options.inputs); i < il; i++) {
                clew_infof("    %ld: %s", i, *(char **) clew_stack_at(&clew->options.inputs, i));

                clew_input_init_options_default(&input_init_options);
                input_init_options.path                         = *(char **) clew_stack_at(&clew->options.inputs, i);
                input_init_options.callback_bounds_start        = input_callback_extract_bounds_start;
                input_init_options.callback_bounds_end          = input_callback_extract_bounds_end;
                input_init_options.callback_node_start          = input_callback_extract_node_start;
                input_init_options.callback_node_end            = input_callback_extract_node_end;
                input_init_options.callback_way_start           = input_callback_extract_way_start;
                input_init_options.callback_way_end             = input_callback_extract_way_end;
                input_init_options.callback_relation_start      = input_callback_extract_relation_start;
                input_init_options.callback_relation_end        = input_callback_extract_relation_end;
                input_init_options.callback_tag_start           = input_callback_extract_tag_start;
                input_init_options.callback_tag_end             = input_callback_extract_tag_end;
                input_init_options.callback_nd_start            = input_callback_extract_nd_start;
                input_init_options.callback_nd_end              = input_callback_extract_nd_end;
                input_init_options.callback_member_start        = input_callback_extract_member_start;
                input_init_options.callback_member_end          = input_callback_extract_member_end;
                input_init_options.callback_minlon              = input_callback_extract_minlon;
                input_init_options.callback_minlat              = input_callback_extract_minlat;
                input_init_options.callback_maxlon              = input_callback_extract_maxlon;
                input_init_options.callback_maxlat              = input_callback_extract_maxlat;
                input_init_options.callback_id                  = input_callback_extract_id;
                input_init_options.callback_lat                 = input_callback_extract_lat;
                input_init_options.callback_lon                 = input_callback_extract_lon;
                input_init_options.callback_ref                 = input_callback_extract_ref;
                input_init_options.callback_type                = input_callback_extract_type;
                input_init_options.callback_role                = input_callback_extract_role;
                input_init_options.callback_k                   = input_callback_extract_k;
                input_init_options.callback_v                   = input_callback_extract_v;
                input_init_options.callback_error               = input_callback_extract_error;
                input_init_options.callback_context             = clew;

                input = clew_input_create(&input_init_options);
                if (input == NULL) {
                        clew_errorf("can not create input for path: %s", input_init_options.path);
                        goto bail;
                }
                while (clew_input_read(input) == 0) {
                        rc = clew_input_get_error(input);
                        if (rc != 0) {
                                clew_errorf("input error occured, error: %d", rc);
                                goto bail;
                        }
                }

                if (input != NULL) {
                        clew_input_destroy(input);
                }
        }

        clew_infof("  processed");
        clew_infof("    inputs   : %ld", clew_stack_count(&clew->options.inputs));
        clew_infof("    nodes    : %ld", clew->read_node_start);
        clew_infof("    ways     : %ld", clew->read_way_start);
        clew_infof("    relations: %ld", clew->read_relation_start);

        clew_infof("  extracted");
        clew_infof("    nodes    : %ld", clew_bitmap_count(&clew->node_ids));
        clew_infof("    ways     : %ld", clew_bitmap_count(&clew->way_ids));
        clew_infof("    relations: %ld", clew_bitmap_count(&clew->relation_ids));

        clew_node_destroy(NULL);

out:
        if (clew != NULL) {
                clew_stack_uninit(&clew->options.inputs);
                clew_stack_uninit(&clew->options.clip_path);
                clew_stack_uninit(&clew->options.points);
                clew_expression_destroy(clew->options.keep_tags);
                clew_expression_destroy(clew->options.keep_tags_node);
                clew_expression_destroy(clew->options.keep_tags_way);
                clew_expression_destroy(clew->options.keep_tags_relation);
                clew_expression_destroy(clew->options.drop_tags);
                clew_expression_destroy(clew->options.drop_tags_node);
                clew_expression_destroy(clew->options.drop_tags_way);
                clew_expression_destroy(clew->options.drop_tags_relation);
                clew_stack_uninit(&clew->read_state);
                clew_bitmap_uninit(&clew->node_ids);
                clew_bitmap_uninit(&clew->way_ids);
                clew_bitmap_uninit(&clew->relation_ids);
                clew_stack_uninit(&clew->nodes);
                clew_stack_uninit(&clew->ways);
                clew_stack_uninit(&clew->relations);
                clew_stack_uninit(&clew->read_tags);
                clew_stack_uninit(&clew->read_refs);
                clew_expression_destroy(clew->options.filter);
                free(clew);
        }

        clew_tag_fini();
        clew_debug_fini();
        return rs;

bail:   rs = -1;
        goto out;
}
