
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <getopt.h>

#include <ctype.h>
#include <time.h>

#define CLEW_DEBUG_NAME                 "main"
#include "debug.h"
#include "input.h"
#include "bound.h"
#include "point.h"
#include "bitmap.h"
#include "stack.h"
#include "khash.h"
#include "pqueue.h"
#include "expression.h"
#include "tag.h"

#define OPTION_HELP                     'h'
#define OPTION_DEBUG                    'd'

#define OPTION_INPUT                    'i'
#define OPTION_OUTPUT                   'o'

#define OPTION_COMMAND                  'c'

#define OPTION_FILTER                   'f'
#define OPTION_POINTS                   'p'

#define OPTION_CLIP_PATH                0x200
#define OPTION_CLIP_BOUND               0x201
#define OPTION_CLIP_OFFSET              0x202
#define OPTION_CLIP_STRATEGY            0x203

#define OPTION_KEEP_TAGS                'k'
#define OPTION_KEEP_TAGS_NODE           0x301
#define OPTION_KEEP_TAGS_WAY            0x302
#define OPTION_KEEP_TAGS_RELATION       0x303

#define OPTION_KEEP_NODES               'n'
#define OPTION_KEEP_WAYS                'w'
#define OPTION_KEEP_RELATIONS           'r'

static const char *g_short_options     = "+i:o:m:f:p:k:n:w:r:d:h";
static struct option g_long_options[] = {
        { "help",               no_argument,            0,      OPTION_HELP                     },
        { "input",              required_argument,      0,      OPTION_INPUT                    },
        { "output",             required_argument,      0,      OPTION_OUTPUT                   },
        { "command",            required_argument,      0,      OPTION_COMMAND                  },
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
        { "keep-nodes",         required_argument,      0,      OPTION_KEEP_NODES               },
        { "keep-ways",          required_argument,      0,      OPTION_KEEP_WAYS                },
        { "keep-relations",     required_argument,      0,      OPTION_KEEP_RELATIONS           },
        { 0,                    0,                      0,      0                               }
};

#define mesh_node_hash_func(key)		kh_int64_hash_func(key)
#define mesh_node_hash_equal(a, b)		(a == b)
KHASH_INIT(mesh_nodes, uint64_t, struct clew_mesh_node *, 1, mesh_node_hash_func, mesh_node_hash_equal);

enum {
        CLEW_STATE_INITIAL                      = 0,
        CLEW_STATE_SELECT                       = 1,
        CLEW_STATE_EXTRACT                      = 2,
        CLEW_STATE_BUILD_MESH                   = 3,
        CLEW_STATE_SOLVE_ROUTES                 = 4
#define CLEW_STATE_INITIAL                      CLEW_STATE_INITIAL
#define CLEW_STATE_SELECT                       CLEW_STATE_SELECT
#define CLEW_STATE_EXTRACT                      CLEW_STATE_EXTRACT
#define CLEW_STATE_BUILD_MESH                   CLEW_STATE_BUILD_MESH
#define CLEW_STATE_SOLVE_ROUTES                 CLEW_STATE_SOLVE_ROUTES
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
        CLEW_READ_STATE_BOUNDS                  = 1 << 0,
        CLEW_READ_STATE_NODE                    = 1 << 1,
        CLEW_READ_STATE_WAY                     = 1 << 2,
        CLEW_READ_STATE_RELATION                = 1 << 3,
        CLEW_READ_STATE_TAG                     = 1 << 4,
        CLEW_READ_STATE_ND                      = 1 << 5,
        CLEW_READ_STATE_MEMBER                  = 1 << 6
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
        int keep_nodes;
        int keep_ways;
        int keep_relations;
};

struct clew_node {
        uint64_t id;
        int32_t lon;
        int32_t lat;
        uint32_t ntags;
        uint32_t *tags;
};

struct clew_way {
        uint64_t id;
        uint32_t ntags;
        uint32_t *tags;
        uint32_t nrefs;
        uint64_t *refs;
};

struct clew_relation {
        uint64_t id;
        uint32_t ntags;
        uint32_t *tags;
        uint32_t nrefs;
        uint64_t *refs;
};

struct clew_mesh_way_type {
	uint32_t tag;
	uint32_t oneway;
	uint32_t maxspeed;
};

struct clew_mesh_way {
        struct clew_way *way;
        uint32_t tag;
        uint32_t oneway;
        uint32_t maxspeed;
};

struct clew_mesh_node_neighbour {
        struct clew_mesh_node *mesh_node;
        double distance;
        double duration;
        double cost;
};

struct clew_mesh_node {
        struct clew_node *node;
        struct clew_stack mesh_ways;
        struct clew_stack mesh_neighbours;

        double pqueue_cost;
        uint64_t pqueue_pos;
        struct clew_mesh_node *pqueue_prev;

        double pqueue_duration;
        double pqueue_distance;
};

struct clew_mesh_point {
        int32_t lon;
        int32_t lat;
        double nearest_distance;
        struct clew_mesh_node *nearest_node;
        int _solved;
};

struct clew_mesh_solution {
        struct clew_mesh_point *source;
        struct clew_mesh_point *destination;
        struct clew_stack mesh_nodes;

        double duration;
        double distance;
        double cost;
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

        struct clew_stack mesh_ways;
        khash_t(mesh_nodes) *mesh_nodes;

        struct clew_stack mesh_points;
        struct clew_stack mesh_solutions;

        struct clew_stack read_state;
        int read_keep;

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

static const struct clew_mesh_way_type clew_mesh_way_types[] = {
        { clew_tag_highway_motorway,            clew_tag_oneway_yes,    clew_tag_maxspeed_140 },
        { clew_tag_highway_motorway_link,       clew_tag_oneway_yes,    clew_tag_maxspeed_110 },
        { clew_tag_highway_trunk,               clew_tag_oneway_yes,    clew_tag_maxspeed_110 },
        { clew_tag_highway_trunk_link,          clew_tag_oneway_yes,    clew_tag_maxspeed_90 },
        { clew_tag_highway_primary,             clew_tag_oneway_no,     clew_tag_maxspeed_90 },
        { clew_tag_highway_primary_link,        clew_tag_oneway_no,     clew_tag_maxspeed_70 },
        { clew_tag_highway_secondary,           clew_tag_oneway_no,     clew_tag_maxspeed_70 },
        { clew_tag_highway_secondary_link,      clew_tag_oneway_no,     clew_tag_maxspeed_50 },
        { clew_tag_highway_tertiary,            clew_tag_oneway_no,     clew_tag_maxspeed_50 },
        { clew_tag_highway_tertiary_link,       clew_tag_oneway_no,     clew_tag_maxspeed_30 },
        { clew_tag_highway_unclassified,        clew_tag_oneway_no,     clew_tag_maxspeed_30 },
        { clew_tag_highway_road,                clew_tag_oneway_no,     clew_tag_maxspeed_30 },
        { clew_tag_highway_residential,         clew_tag_oneway_no,     clew_tag_maxspeed_30 },
        { clew_tag_highway_living_street,       clew_tag_oneway_no,     clew_tag_maxspeed_20 },
        { clew_tag_highway_service,             clew_tag_oneway_no,     clew_tag_maxspeed_30 },
        { clew_tag_highway_track,               clew_tag_oneway_no,     clew_tag_maxspeed_30 },
        { clew_tag_highway_path,                clew_tag_oneway_no,     clew_tag_maxspeed_30 },
        { clew_tag_highway_cycleway,            clew_tag_oneway_no,     clew_tag_maxspeed_30 },
        { clew_tag_highway_bridleway,           clew_tag_oneway_no,     clew_tag_maxspeed_30 },
};

static const char *g_filter_preset_motorcycle_scenic =
        "( "
        "highway_primary or highway_primary_link or "
        "highway_secondary or highway_secondary_link or "
        "highway_tertiary or highway_tertiary_link or "
        "highway_unclassified or "
        "highway_road or "
        "highway_residential or "
        "highway_living_street "
        ") "
        "and not ( motor_vehicle_no or motor_vehicle_private ) "
        "and not ( access_no or access_private) ";

static const char *g_filter_preset_motorcycle_scenic_plus =
        "( "
        "highway_primary or highway_primary_link or "
        "highway_secondary or highway_secondary_link or "
        "highway_tertiary or highway_tertiary_link or "
        "highway_unclassified or "
        "highway_road or "
        "highway_residential or "
        "highway_living_street or "
        "(( highway_track or highway_path ) and surface_asphalt )"
        ") "
        "and not ( motor_vehicle_no or motor_vehicle_private ) "
        "and not ( access_no or access_private) ";

static int tags_expression_match_has (void *context, uint32_t tag);

static uint32_t parse_tag_fix_length_value (const char *v_raw);
static void parse_tag_fix_length (char *k, char *v);
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

static int node_stack_compare_elements (const void *a, const void *b);
static void node_stack_destroy_element (void *context, void *elem);

static int way_stack_compare_elements (const void *a, const void *b);
static void way_stack_destroy_element (void *context, void *elem);

static int relation_stack_compare_elements (const void *a, const void *b);
static void relation_stack_destroy_element (void *context, void *elem);

static int mesh_node_pqueue_compare (const void *a, const void *b);
static void mesh_node_pqueue_setpos (void *a, uint64_t position);
static uint64_t mesh_node_pqueue_getpos (const void *a);

static void mesh_solution_stack_destroy_element (void *context, void *elem);

KHASH_SET_INIT_INT64(mesh_visited);

static int64_t clew_mesh_node_neighbours_count_depth (
        struct clew_mesh_node *mnode,
        int64_t depth,
        int64_t mdepth,
        int64_t count,
        int64_t mcount,
        khash_t(mesh_visited) *visited);
static int64_t clew_mesh_node_neighbours_count (struct clew_mesh_node *mnode, int64_t mcount);
static void clew_mesh_node_destroy (struct clew_mesh_node *mnode);

static void clew_node_destroy (struct clew_node *node);
static void clew_way_destroy (struct clew_way *way);
static void clew_relation_destroy (struct clew_relation *relation);

static const char * clew_clip_strategy_string (int strategy);
static int clew_clip_strategy_value (const char *strategy);

static void print_help (const char *pname)
{
        fprintf(stdout, "%s usage:\n", pname);
        fprintf(stdout, "\n");
        fprintf(stdout, "  --input              / -i: input path\n");
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
        fprintf(stdout, "  simple, complete_ways, smart\n");
        fprintf(stdout, "\n");
        fprintf(stdout, "  simple: runs in a single pass. the extract will contain all nodes inside the region and all ways referencing those nodes "
                        "as well as all relations referencing any nodes or ways already included. ways crossing the region boundary will not be "
                        "reference-complete. relations will not be reference-complete.\n");
        fprintf(stdout, "\n");
        fprintf(stdout, "  complete_ways: runs in two passes. the extract will contain all nodes inside the region and all ways referencing those "
                        "nodes as well as all nodes referenced by those ways. the extract will also contain all relations referenced by nodes inside "
                        "the region or ways already included and, recursively, their parent relations. the ways are reference-complete, but the "
                        "relations are not.\n");
        fprintf(stdout, "\n");
        fprintf(stdout, "  smart: runs in three passes. the extract will contain all nodes inside the region and all ways referencing those nodes as "
                        "well as all nodes referenced by those ways. the extract will also contain all relations referenced by nodes inside the region "
                        "or ways already included and, recursively, their parent relations. the extract will also contain all nodes and ways (and the "
                        "nodes they reference) referenced by relations tagged “type=multipolygon” directly referencing any nodes in the region or ways "
                        "referencing nodes in the region. the ways are reference-complete, and all multipolygon relations referencing nodes in the "
                        "regions or ways that have nodes in the region are reference-complete. other relations are not.\n");
        fprintf(stdout, "\n");
        fprintf(stdout, "filter;\n");
        fprintf(stdout, "  logical expression using predefined tags and operators: and, or, not, (, ). the filter applies to each route segment's tags "
                        "to determine inclusion.\n");
        fprintf(stdout, "    - tags follow the format: key_value (e.g., highway_trunk, surface_dirt)\n");
        fprintf(stdout, "    - wildcards supported for key groups: key_* (e.g., highway_* matches all highway types)\n");
        fprintf(stdout, "\n");
        fprintf(stdout, "  example:\n");
        fprintf(stdout, "    (highway_trail or highway_track) and not surface_asphalt\n");
        fprintf(stdout, "\n");
        fprintf(stdout, "  presets;\n");
        fprintf(stdout, "    motorcycle-scenic, motorcycle-scenic+\n");
        fprintf(stdout, "\n");
        fprintf(stdout, "    motorcycle-scenic: for relaxed motorcycle travel on asphalt roads.it avoids tolls, tunnels, and rough or unpaved paths.\n"
                        "      ideal for:\n"
                        "        - scenic riding\n"
                        "        - visiting villages and cafés\n"
                        "        - comfortable road touring with occasional stops\n"
                        "      filter:\n"
                        "        %s\n", g_filter_preset_motorcycle_scenic);
        fprintf(stdout, "\n");
        fprintf(stdout, "    motorcycle-scenic+: for relaxed motorcycle travel on asphalt roads. it avoids tolls, tunnels, and rough or unpaved paths.\n"
                        "      ideal for:\n"
                        "        - scenic riding\n"
                        "        - visiting villages and cafés\n"
                        "        - comfortable road touring with occasional stops\n"
                        "      filter:\n"
                        "        %s\n", g_filter_preset_motorcycle_scenic_plus);
        fprintf(stdout, "\n");
        fprintf(stdout, "highway tags;\n");
        fprintf(stdout, "\n");
        fprintf(stdout, "  roads:\n");
        fprintf(stdout, "    highway_motorway      : a restricted access major divided highway, normally with 2 or more running lanes plus emergency hard shoulder. equivalent to the freeway, autobahn, etc..\n");
        fprintf(stdout, "    highway_trunk         : the most important roads in a country's system that aren't motorways. (need not necessarily be a divided highway.\n");
        fprintf(stdout, "    highway_primary       : the next most important roads in a country's system. (often link larger towns.)\n");
        fprintf(stdout, "    highway_secondary     : the next most important roads in a country's system. (often link towns.)\n");
        fprintf(stdout, "    highway_tertiary      : the next most important roads in a country's system. (often link smaller towns and villages)\n");
        fprintf(stdout, "    highway_unclassified  : the least important through roads in a country's system i.e. minor roads of a lower classification than tertiary, but which serve a purpose other than access to properties. (often link villages and hamlets.)\n");
        fprintf(stdout, "    highway_residential   : roads which serve as an access to housing, without function of connecting settlements. often lined with housing.\n");
        fprintf(stdout, "\n");
        fprintf(stdout, "  link roads:\n");
        fprintf(stdout, "    highway_motorway_link : the link roads (sliproads/ramps) leading to/from a motorway from/to a motorway or lower class highway. normally with the same motorway restrictions.\n");
        fprintf(stdout, "    highway_trunk_link    : the link roads (sliproads/ramps) leading to/from a trunk road from/to a trunk road or lower class highway.\n");
        fprintf(stdout, "    highway_primary_link  : the link roads (sliproads/ramps) leading to/from a primary road from/to a primary road or lower class highway.\n");
        fprintf(stdout, "    highway_secondary_link: the link roads (sliproads/ramps) leading to/from a secondary road from/to a secondary road or lower class highway.\n");
        fprintf(stdout, "    highway_tertiary_link : the link roads (sliproads/ramps) leading to/from a tertiary road from/to a tertiary road or lower class highway.\n");
        fprintf(stdout, "\n");
        fprintf(stdout, "  special road types:\n");
        fprintf(stdout, "    highway_living_street : for living streets, which are residential streets where pedestrians have legal priority over cars, speeds are kept very low.\n");
        fprintf(stdout, "    highway_service       : for access roads to, or within an industrial estate, camp site, business park, car park, alleys, etc.\n");
        fprintf(stdout, "    highway_pedestrian    : for roads used mainly/exclusively for pedestrians in shopping and some residential areas which may allow access by motorised vehicles only for very limited periods of the day.\n");
        fprintf(stdout, "    highway_track         : roads for mostly agricultural or forestry uses.\n");
        fprintf(stdout, "    highway_bus_guideway  : a busway where the vehicle guided by the way (though not a railway) and is not suitable for other traffic.\n");
        fprintf(stdout, "    highway_escape        : for runaway truck ramps, runaway truck lanes, emergency escape ramps, or truck arrester beds.\n");
        fprintf(stdout, "    highway_raceway       : a course or track for (motor) racing\n");
        fprintf(stdout, "    highway_road          : a road/way/street/motorway/etc. of unknown type. it can stand for anything ranging from a footpath to a motorway.\n");
        fprintf(stdout, "    highway_bus_way       : a dedicated roadway for bus rapid transit systems\n");
        fprintf(stdout, "\n");
        fprintf(stdout, "  paths:\n");
        fprintf(stdout, "    highway_footway       : for designated footpaths; i.e., mainly/exclusively for pedestrians. this includes walking tracks and gravel paths.\n");
        fprintf(stdout, "    highway_bridleway     : for horse riders. pedestrians are usually also permitted, cyclists may be permitted depending on local rules/laws. motor vehicles are forbidden.\n");
        fprintf(stdout, "    highway_steps         : for flights of steps (stairs) on footways.\n");
        fprintf(stdout, "    highway_corridor      : for a hallway inside of a building.\n");
        fprintf(stdout, "    highway_path          : a non-specific path.\n");
        fprintf(stdout, "    highway_via_ferrata   : a via ferrata is a route equipped with fixed cables, stemples, ladders, and bridges in order to increase ease and security for climbers.\n");
}

static int tags_expression_match_has (void *context, uint32_t tag)
{
        uint64_t pos;
        struct clew_stack *tags = context;
        pos = clew_stack_search_uint32(tags, tag);
        return (pos == UINT64_MAX) ? 0 : 1;
}

static uint32_t parse_tag_fix_length_value (const char *v_raw)
{
        const char *v = v_raw;
        char *endptr = NULL;
        double meters = 0.0;

        const char *apos = strchr(v, '\'');  // typewriter apostrophe
        const char *quote = strchr(v, '"');  // typewriter quote

        // 1. Handle feet + inches like 16'3" or 6' or 3"
        if (apos || quote) {
                double feet = 0.0, inches = 0.0;

                if (apos) {
                        feet = strtod(v, &endptr);
                        if (endptr != apos) return UINT32_MAX; // invalid format
                }

                if (quote) {
                        if (apos) {
                                inches = strtod(apos + 1, &endptr);
                                if (endptr != quote) return UINT32_MAX;
                        } else {
                                inches = strtod(v, &endptr);
                                if (endptr != quote) return UINT32_MAX;
                        }
                }

                meters = (feet * 0.3048) + (inches * 0.0254);
                return (uint32_t) round((meters < 0.0) ? 0.0 : meters);
        }

        // 2. Handle suffix units: "2 m", "0.6 mi", etc.
        double value = strtod(v, &endptr);

        if (endptr == v) return UINT32_MAX; // no valid number found

        while (*endptr && isspace(*endptr)) ++endptr;

        if (strncmp(endptr, "m", 1) == 0 || *endptr == '\0') {
                // meters (default)
                meters = value;
        } else if (strncmp(endptr, "mi", 2) == 0) {
                meters = value * 1609.344;
        } else {
                // unsupported unit
                return UINT32_MAX;
        }

        return (uint32_t) round((meters < 0.0) ? 0.0 : meters);
}

static void parse_tag_fix_length (char *k, char *v)
{
        int length;
        uint32_t tag;
        (void) k;
        length = parse_tag_fix_length_value(v);
        if (length == 0) {
                tag = clew_tag_length_0;
        } else if (length >= 1 && length <= 100) {
                tag = clew_tag_length_1 + (length - 1);
        } else if (length >= 110 && length <= 1000) {
                tag = clew_tag_length_110 + ((length - 110) / 10);
        } else if (length >= 1010 && length <= 5000) {
                tag = clew_tag_length_1010 + ((length - 1010) / 10);
        } else if (length >= 5025 && length <= 10000) {
                tag = clew_tag_length_5025 + ((length - 5025) / 25);
        } else if (length >= 10050 && length <= 50000) {
                tag = clew_tag_length_10050 + ((length - 10050) / 50);
        } else if (length >= 50100 && length <= 100000) {
                tag = clew_tag_length_50100 + ((length - 50100) / 100);
        } else if (length >= 100250 && length <= 500000) {
                tag = clew_tag_length_100250 + ((length - 100250) / 250);
        } else if (length >= 500500 && length <= 1000000) {
                tag = clew_tag_length_500500 + ((length - 500500) / 500);
        } else {
                tag = clew_tag_length_unknown;
        }
        snprintf(v, READ_TAG_V_LENGTH, "length_%d", tag - clew_tag_length_0);
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
        } else if (strcasecmp(k, "length") == 0) {
                parse_tag_fix_length(k, v);
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

        if ((clew->read_keep & CLEW_READ_STATE_NODE) == 0) {
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

        if ((clew->read_keep & CLEW_READ_STATE_WAY) == 0) {
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

        if ((clew->read_keep & CLEW_READ_STATE_RELATION) == 0) {
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

        if ((clew_stack_peek_uint32(&clew->read_state) & (CLEW_READ_STATE_NODE | CLEW_READ_STATE_WAY | CLEW_READ_STATE_RELATION)) == 0) {
                clew_errorf("read_state is invalid, %d != %d | %d | %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE, CLEW_READ_STATE_WAY, CLEW_READ_STATE_RELATION);
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

        if ((clew_stack_peek_uint32(&clew->read_state) & (CLEW_READ_STATE_NODE | CLEW_READ_STATE_WAY | CLEW_READ_STATE_RELATION)) == 0) {
                clew_errorf("read_state is invalid, %d != %d | %d | %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE, CLEW_READ_STATE_WAY, CLEW_READ_STATE_RELATION);
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

        if ((clew_stack_peek_uint32(&clew->read_state) & (CLEW_READ_STATE_ND | CLEW_READ_STATE_MEMBER | CLEW_READ_STATE_RELATION)) == 0) {
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

        node->id  = clew->read_id;
        node->lon = clew->read_lon;
        node->lat = clew->read_lat;

        node->ntags = clew_stack_count(&clew->read_tags);
        if (node->ntags > 0) {
                node->tags  = malloc(sizeof(uint32_t) * node->ntags);
                if (node->tags == NULL) {
                        clew_errorf("can not allocate memory");
                        goto bail;
                }
                memcpy(node->tags, clew_stack_buffer(&clew->read_tags), sizeof(uint32_t) * node->ntags);
        }

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

        return 0;
bail:   return -1;
}

static int input_callback_extract_way_end (struct clew_input *input, void *context)
{
        int rc;
        struct clew_way *way;

        struct clew *clew = context;

        (void) input;

        way = NULL;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_WAY) {
                clew_errorf("read_state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_WAY);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        rc = clew_bitmap_marked(&clew->way_ids, clew->read_id);
        if (rc < 0) {
                clew_errorf("can not read bitmap");
                goto bail;
        } else if (rc == 0) {
                goto out;
        }

        way = malloc(sizeof(struct clew_way));
        if (way == NULL) {
                clew_errorf("can not allocate memory");
                goto bail;
        }
        memset(way, 0, sizeof(struct clew_way));

        way->id = clew->read_id;

        way->ntags = clew_stack_count(&clew->read_tags);
        if (way->ntags > 0) {
                way->tags  = malloc(sizeof(uint32_t) * way->ntags);
                if (way->tags == NULL) {
                        clew_errorf("can not allocate memory");
                        goto bail;
                }
                memcpy(way->tags, clew_stack_buffer(&clew->read_tags), sizeof(uint32_t) * way->ntags);
        }

        way->nrefs = clew_stack_count(&clew->read_refs);
        if (way->nrefs > 0) {
                way->refs  = malloc(sizeof(uint64_t) * way->nrefs);
                if (way->refs == NULL) {
                        clew_errorf("can not allocate memory");
                        goto bail;
                }
                memcpy(way->refs, clew_stack_buffer(&clew->read_refs), sizeof(uint64_t) * way->nrefs);
        }

        rc = clew_stack_push_ptr(&clew->ways, way);
        if (rc < 0) {
                clew_errorf("can not push way");
                goto bail;
        }

out:    return 0;
bail:   if (way != NULL) {
                clew_way_destroy(way);
        }
        return -1;
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

        if ((clew->read_keep & CLEW_READ_STATE_RELATION) == 0) {
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
        int read_state;
        struct clew *clew = context;

        (void) input;

        read_state = clew_stack_peek_uint32(&clew->read_state);
        if ((read_state & (CLEW_READ_STATE_NODE | CLEW_READ_STATE_WAY | CLEW_READ_STATE_RELATION)) == 0) {
                clew_errorf("read_state is invalid, %d != %d | %d | %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE, CLEW_READ_STATE_WAY, CLEW_READ_STATE_RELATION);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_TAG);

        clew->read_tag_k[0] = '\0';
        clew->read_tag_v[0] = '\0';

        if ((clew->read_keep & read_state) == 0) {
                goto skip;
        }

        return 0;
skip:   return 1;
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

        if ((clew_stack_peek_uint32(&clew->read_state) & (CLEW_READ_STATE_NODE | CLEW_READ_STATE_WAY | CLEW_READ_STATE_RELATION)) == 0) {
                clew_errorf("read_state is invalid, %d != %d | %d | %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE, CLEW_READ_STATE_WAY, CLEW_READ_STATE_RELATION);
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

        if ((clew_stack_peek_uint32(&clew->read_state) & (CLEW_READ_STATE_ND | CLEW_READ_STATE_MEMBER | CLEW_READ_STATE_RELATION)) == 0) {
                clew_errorf("read_state is invalid, %d != %d | %d | %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_ND, CLEW_READ_STATE_MEMBER, CLEW_READ_STATE_RELATION);
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

static int node_stack_compare_elements (const void *a, const void *b)
{
        const struct clew_node *t1 = *(const struct clew_node * const *)a;
        const struct clew_node *t2 = *(const struct clew_node * const *)b;
        if (t1->id < t2->id) return -1;
        if (t1->id > t2->id) return 1;
        return 0;
}

static void node_stack_destroy_element (void *context, void *elem)
{
        (void) context;
        clew_node_destroy(*(struct clew_node **) elem);
}

static int way_stack_compare_elements (const void *a, const void *b)
{
        const struct clew_way *t1 = *(const struct clew_way * const *)a;
        const struct clew_way *t2 = *(const struct clew_way * const *)b;
        if (t1->id < t2->id) return -1;
        if (t1->id > t2->id) return 1;
        return 0;
}

static void way_stack_destroy_element (void *context, void *elem)
{
        (void) context;
        clew_way_destroy(*(struct clew_way **) elem);
}

static int relation_stack_compare_elements (const void *a, const void *b)
{
        const struct clew_relation *t1 = *(const struct clew_relation * const *)a;
        const struct clew_relation *t2 = *(const struct clew_relation * const *)b;
        if (t1->id < t2->id) return -1;
        if (t1->id > t2->id) return 1;
        return 0;
}

static void relation_stack_destroy_element (void *context, void *elem)
{
        (void) context;
        clew_relation_destroy(*(struct clew_relation **) elem);
}

static int mesh_node_pqueue_compare (const void *a, const void *b)
{
        const struct clew_mesh_node *t1 = (const struct clew_mesh_node *) a;
        const struct clew_mesh_node *t2 = (const struct clew_mesh_node *) b;
        if (t1->pqueue_cost > t2->pqueue_cost) return 1;
        //if (t1->pqueue_cost < t2->pqueue_cost) return -1;
        return 0;
}

static void mesh_node_pqueue_setpos (void *a, uint64_t position)
{
        struct clew_mesh_node *t1 = (struct clew_mesh_node *) a;
        t1->pqueue_pos = position;
}

static uint64_t mesh_node_pqueue_getpos (const void *a)
{
        const struct clew_mesh_node *t1 = (const struct clew_mesh_node *) a;
        return t1->pqueue_pos;
}

static void mesh_solution_stack_destroy_element (void *context, void *elem)
{
        struct clew_mesh_solution *msolution = (struct clew_mesh_solution *) elem;
        (void) context;
        clew_stack_uninit(&msolution->mesh_nodes);
}

static int64_t clew_mesh_node_neighbours_count_depth (
        struct clew_mesh_node *mnode,
        int64_t depth,
        int64_t mdepth,
        int64_t count,
        int64_t mcount,
        khash_t(mesh_visited) *visited)
{
        if (mnode == NULL || (mdepth > 0 && depth >= mdepth)) {
                return 0;
        }

        khint_t k = kh_get(mesh_visited, visited, mnode->node->id);
        if (k != kh_end(visited)) {
                return 0;
        }

        int ret;
        k = kh_put(mesh_visited, visited, mnode->node->id, &ret);
        (void) k;
        (void) ret;

        int64_t total = 0;
        uint64_t nl = clew_stack_count(&mnode->mesh_neighbours);

        total += nl;
        if (mcount > 0 && count + total >= mcount) {
                return total;
        }

        for (uint64_t n = 0; n < nl; n++) {
                struct clew_mesh_node_neighbour *rneig = (struct clew_mesh_node_neighbour *) clew_stack_at(&mnode->mesh_neighbours, n);
                if (rneig && rneig->mesh_node) {
                        total += clew_mesh_node_neighbours_count_depth(rneig->mesh_node, depth + 1, mdepth, count + total, mcount, visited);
                        if (mcount > 0 && count + total >= mcount) {
                                return total;
                        }
                }
        }

        return total;
}

static int64_t clew_mesh_node_neighbours_count (struct clew_mesh_node *mnode, int64_t mcount)
{
        const int64_t mdepth = 16;
        khash_t(mesh_visited) *visited = kh_init(mesh_visited);
        int64_t result = clew_mesh_node_neighbours_count_depth(mnode, 0, mdepth, 0, mcount, visited);
        kh_destroy(mesh_visited, visited);
        return result;
}

static void clew_mesh_node_destroy (struct clew_mesh_node *mnode)
{
        clew_stack_uninit(&mnode->mesh_ways);
        clew_stack_uninit(&mnode->mesh_neighbours);
        free(mnode);
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

static void clew_way_destroy (struct clew_way *way)
{
        if (way == NULL) {
                return;
        }
        if (way->tags != NULL) {
                free(way->tags);
        }
        if (way->refs != NULL) {
                free(way->refs);
        }
        free(way);
}

static void clew_relation_destroy (struct clew_relation *relation)
{
        if (relation == NULL) {
                return;
        }
        if (relation->tags != NULL) {
                free(relation->tags);
        }
        if (relation->refs != NULL) {
                free(relation->refs);
        }
        free(relation);
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

        uint64_t j;
        uint64_t jl;

        uint64_t w;
        uint64_t wl;

        uint64_t t;
        uint64_t tl;

        uint64_t r;
        uint64_t rl;

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
        clew->options.keep_nodes                = 1;
        clew->options.keep_ways                 = 1;
        clew->options.keep_relations            = 1;

        clew->state             = CLEW_STATE_INITIAL;
        clew->read_state        = clew_stack_init(sizeof(uint32_t));
        clew->read_tags         = clew_stack_init(sizeof(uint32_t));
        clew->read_refs         = clew_stack_init(sizeof(uint64_t));
        clew->node_ids          = clew_bitmap_init(64 * 1024);
        clew->way_ids           = clew_bitmap_init(64 * 1024);
        clew->relation_ids      = clew_bitmap_init(64 * 1024);
        clew->nodes             = clew_stack_init4(sizeof(struct clew_node *), 64 * 1024, node_stack_destroy_element, NULL);
        clew->ways              = clew_stack_init4(sizeof(struct clew_way *), 64 * 1024, way_stack_destroy_element, NULL);
        clew->relations         = clew_stack_init4(sizeof(struct clew_relation *), 64 * 1024, relation_stack_destroy_element, NULL);
        clew->mesh_ways         = clew_stack_init2(sizeof(struct clew_mesh_way), 64 * 1024);
        clew->mesh_nodes        = kh_init(mesh_nodes);
        clew->mesh_points       = clew_stack_init(sizeof(struct clew_mesh_point));
        clew->mesh_solutions    = clew_stack_init4(sizeof(struct clew_mesh_solution), 64, mesh_solution_stack_destroy_element, NULL);

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
                                if (strcasecmp(optarg, "motorcycle_scenic") == 0 ||
                                    strcasecmp(optarg, "motorcycle-scenic") == 0) {
                                        clew->options.filter = clew_expression_create(g_filter_preset_motorcycle_scenic);
                                } else if (strcasecmp(optarg, "motorcycle_scenic+") == 0 ||
                                           strcasecmp(optarg, "motorcycle-scenic+") == 0 ||
                                           strcasecmp(optarg, "motorcycle_scenic_plus") == 0 ||
                                           strcasecmp(optarg, "motorcycle-scenic-plus") == 0) {
                                        clew->options.filter = clew_expression_create(g_filter_preset_motorcycle_scenic_plus);
                                } else {
                                        clew->options.filter = clew_expression_create(optarg);
                                }
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
                        case OPTION_KEEP_NODES:
                                clew->options.keep_nodes = !!atoi(optarg);
                                break;
                        case OPTION_KEEP_WAYS:
                                clew->options.keep_ways = !!atoi(optarg);
                                break;
                        case OPTION_KEEP_RELATIONS:
                                clew->options.keep_relations = !!atoi(optarg);
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
        clew->read_keep |= clew->options.keep_nodes     ? CLEW_READ_STATE_NODE     : 0;
        clew->read_keep |= clew->options.keep_ways      ? CLEW_READ_STATE_WAY      : 0;
        clew->read_keep |= clew->options.keep_relations ? CLEW_READ_STATE_RELATION : 0;

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
                rc = clew_input_get_error(input);
                if (rc != 0) {
                        clew_errorf("input error occured, error: %d", rc);
                        clew_input_destroy(input);
                        goto bail;
                }
                clew_input_destroy(input);
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
                rc = clew_input_get_error(input);
                if (rc != 0) {
                        clew_errorf("input error occured, error: %d", rc);
                        clew_input_destroy(input);
                        goto bail;
                }
                clew_input_destroy(input);
        }

        clew_infof("  processed");
        clew_infof("    inputs   : %ld", clew_stack_count(&clew->options.inputs));
        clew_infof("    nodes    : %ld", clew->read_node_start);
        clew_infof("    ways     : %ld", clew->read_way_start);
        clew_infof("    relations: %ld", clew->read_relation_start);

        clew_infof("  sorting");
        clew_stack_sort(&clew->nodes, node_stack_compare_elements);
        clew_stack_sort(&clew->ways, way_stack_compare_elements);
        clew_stack_sort(&clew->relations, relation_stack_compare_elements);

        clew_infof("  extracted");
        clew_infof("    nodes    : %ld", clew_bitmap_count(&clew->node_ids));
        clew_infof("    ways     : %ld", clew_bitmap_count(&clew->way_ids));
        clew_infof("    relations: %ld", clew_bitmap_count(&clew->relation_ids));

        clew_infof("building mesh");
        clew->state = CLEW_STATE_BUILD_MESH;

        clew_infof("  building mesh ways");
        for (w = 0, wl = clew_stack_count(&clew->ways); w < wl; w++) {
                struct clew_way *way;
                struct clew_mesh_way mway;

                mway.way      = NULL;
                mway.tag      = clew_tag_unknown;
                mway.oneway   = clew_tag_oneway_no;
                mway.maxspeed = clew_tag_maxspeed_20;

                way = clew_stack_at_ptr(&clew->ways, w);

                for (i = 0, il = sizeof(clew_mesh_way_types) / sizeof(clew_mesh_way_types[0]); i < il; i++) {
                        for (t = 0, tl = way->ntags; t < tl; t++) {
                                if (clew_mesh_way_types[i].tag == way->tags[t]) {
                                        mway.tag      = clew_mesh_way_types[i].tag;
                                        mway.oneway   = clew_mesh_way_types[i].oneway;
                                        mway.maxspeed = clew_mesh_way_types[i].maxspeed;
                                        break;
                                }
                        }
                        if (t < tl) {
                                break;
                        }
                }
                if (mway.tag == clew_tag_unknown) {
                        clew_errorf("way: %ld with invalid tags", way->id);
                        for (t = 0, tl = way->ntags; t < tl; t++) {
                                clew_errorf("  %d, %s", way->tags[t], clew_tag_string(way->tags[t]));
                        }
                        continue;
                }

                for (t = 0, tl = way->ntags; t < tl; t++) {
                        if (way->tags[t] == clew_tag_oneway_no ||
                            way->tags[t] == clew_tag_oneway_yes ||
                            way->tags[t] == clew_tag_oneway__1) {
                                mway.oneway = way->tags[t];
                                break;
                        }
                }

                for (t = 0, tl = way->ntags; t < tl; t++) {
                        if (clew_tag_is_group_maxspeed(way->tags[t])) {
                                mway.maxspeed = way->tags[t];
                                break;
                        }
                }

                mway.way = way;

                rc = clew_stack_push(&clew->mesh_ways, &mway);
                if (rc < 0) {
                        clew_errorf("can not push mesh way");
                        goto bail;
                }
        }
        clew_infof("    ways: %ld", clew_stack_count(&clew->mesh_ways));

        clew_infof("  building mesh nodes");
        for (w = 0, wl = clew_stack_count(&clew->mesh_ways); w < wl; w++) {
                struct clew_way *way;
                struct clew_mesh_way *mway;

                struct clew_node *node;
                struct clew_node _knode;
                struct clew_node *knode;

                khiter_t k;
                struct clew_mesh_node *mnode;
                struct clew_mesh_node *pmnode;
                struct clew_mesh_node _kmnode;
                struct clew_mesh_node *kmnode;

                struct clew_mesh_node_neighbour *mnodeneigh;
                struct clew_mesh_node_neighbour _mnodeneigh;

                mway = clew_stack_at(&clew->mesh_ways, w);
                way  = mway->way;

                pmnode = NULL;
                for (r = 0, rl = way->nrefs; r < rl; r++) {
                        knode = &_knode;
                        knode->id = way->refs[r];
                        node = *(struct clew_node **) clew_stack_search(&clew->nodes, &knode, node_stack_compare_elements);

                        kmnode = &_kmnode;
                        kmnode->node = &_knode;
                        kmnode->node->id = node->id;
                        k = kh_get(mesh_nodes, clew->mesh_nodes, kmnode->node->id);
                        if (k == kh_end(clew->mesh_nodes)) {
                                mnode = malloc(sizeof(struct clew_mesh_node));
                                if (mnode == NULL) {
                                        clew_errorf("can not allocate memory");
                                        goto bail;
                                }
                                mnode->node = node;
                                mnode->mesh_ways       = clew_stack_init2(sizeof(struct clew_mesh_way *), 2);
                                mnode->mesh_neighbours = clew_stack_init2(sizeof(struct clew_mesh_node_neighbour), 2);
                                k = kh_put(mesh_nodes, clew->mesh_nodes, mnode->node->id, &rc);
                                if (rc < 0) {
                                        clew_errorf("can not push mesh node");
                                        clew_mesh_node_destroy(mnode);
                                        goto bail;
                                }
                                kh_value(clew->mesh_nodes, k) = mnode;
                        } else {
                                mnode = kh_val(clew->mesh_nodes, k);
                        }

                        rc = clew_stack_push(&mnode->mesh_ways, &mway);
                        if (rc < 0) {
                                clew_errorf("can not push mesh way");
                                goto bail;
                        }

                        if (pmnode != NULL) {
                                struct clew_point a = clew_point_init(pmnode->node->lon, pmnode->node->lat);
                                struct clew_point b = clew_point_init(mnode->node->lon, mnode->node->lat);
                                double distance = clew_point_distance_euclidean(&a, &b);
                                double duration = (distance * 3.60) / ((double) (mway->maxspeed - clew_tag_maxspeed_0));

                                if (mway->oneway == clew_tag_oneway__1) {
                                        mnodeneigh = &_mnodeneigh;
                                        mnodeneigh->distance  = distance;
                                        mnodeneigh->duration  = duration;
                                        mnodeneigh->cost      = duration;
                                        mnodeneigh->mesh_node = pmnode;
                                        rc = clew_stack_push(&mnode->mesh_neighbours, mnodeneigh);
                                        if (rc < 0) {
                                                clew_errorf("can not push mesh node neighbour");
                                                goto bail;
                                        }
                                } else if (mway->oneway == clew_tag_oneway_yes) {
                                        mnodeneigh = &_mnodeneigh;
                                        mnodeneigh->distance  = distance;
                                        mnodeneigh->duration  = duration;
                                        mnodeneigh->cost      = duration;
                                        mnodeneigh->mesh_node = mnode;
                                        rc = clew_stack_push(&pmnode->mesh_neighbours, mnodeneigh);
                                        if (rc < 0) {
                                                clew_errorf("can not push mesh node neighbour");
                                                goto bail;
                                        }
                                } else if (mway->oneway == clew_tag_oneway_no) {
                                        mnodeneigh = &_mnodeneigh;
                                        mnodeneigh->distance  = distance;
                                        mnodeneigh->duration  = duration;
                                        mnodeneigh->cost      = duration;
                                        mnodeneigh->mesh_node = mnode;
                                        rc = clew_stack_push(&pmnode->mesh_neighbours, mnodeneigh);
                                        if (rc < 0) {
                                                clew_errorf("can not push mesh node neighbour");
                                                goto bail;
                                        }

                                        mnodeneigh = &_mnodeneigh;
                                        mnodeneigh->distance  = distance;
                                        mnodeneigh->duration  = duration;
                                        mnodeneigh->cost      = duration;
                                        mnodeneigh->mesh_node = pmnode;
                                        rc = clew_stack_push(&mnode->mesh_neighbours, mnodeneigh);
                                        if (rc < 0) {
                                                clew_errorf("can not push mesh node neighbour");
                                                goto bail;
                                        }
                                }
                        }

                        pmnode = mnode;
                }
        }
        clew_infof("    nodes: %d", kh_size(clew->mesh_nodes));

        clew_stack_reset(&clew->mesh_points);
        clew_stack_reset(&clew->mesh_solutions);

        clew_infof("building points");
        for (i = 0, il = clew_stack_count(&clew->options.points); i < il; i += 2) {
                double distance;
                double sdistance;
                struct clew_mesh_node *smnode;

                khiter_t k;
                struct clew_mesh_node *mnode;

                struct clew_point npoint;
                struct clew_point spoint;
                struct clew_bound sbound;

                clew_infof("  %ld: %.7f,%.7f", i / 2, clew_stack_at_int32(&clew->options.points, i + 0) / 1e7, clew_stack_at_int32(&clew->options.points, i + 1) / 1e7);

                sdistance = INFINITY;
                spoint    = clew_point_init(clew_stack_at_int32(&clew->options.points, i + 0), clew_stack_at_int32(&clew->options.points, i + 1));
                sbound    = clew_bound_null();
                smnode    = NULL;

                int64_t node_count = kh_size(clew->mesh_nodes);
                int64_t min_neighbour_count = node_count * 0.01;
                if (min_neighbour_count < 4) {
                        min_neighbour_count = 4;
                }
                if (min_neighbour_count > 8) {
                        min_neighbour_count = 8;
                }

                for (k = kh_begin(clew->mesh_nodes); k != kh_end(clew->mesh_nodes); k++) {
                        if (!kh_exist(clew->mesh_nodes, k)) {
                                continue;
                        }
                        mnode = kh_val(clew->mesh_nodes, k);

                        npoint = clew_point_init(mnode->node->lon, mnode->node->lat);
                        if (clew_bound_invalid(&sbound) ||
                                clew_bound_contains_point(&sbound, &npoint)) {
                                distance = clew_point_distance_euclidean(&spoint, &npoint);
                                if (distance < sdistance) {
#if 1
                                        struct clew_point snpoint = clew_point_derived_position(&npoint, distance, 0);
                                        struct clew_point sepoint = clew_point_derived_position(&npoint, distance, 90);
                                        struct clew_point sspoint = clew_point_derived_position(&npoint, distance, 180);
                                        struct clew_point swpoint = clew_point_derived_position(&npoint, distance, 270);
                                        sbound = clew_bound_null();
                                        sbound = clew_bound_union_point(&sbound, &snpoint);
                                        sbound = clew_bound_union_point(&sbound, &sepoint);
                                        sbound = clew_bound_union_point(&sbound, &sspoint);
                                        sbound = clew_bound_union_point(&sbound, &swpoint);
#else
                                        #define METERS_TO_E7_LAT(m)             ((int32_t) ((m) / 0.011132))   // ~0.0000001 deg = 1.11 meters
                                        #define METERS_TO_E7_LON(m, lat)        ((int32_t) ((m) / (0.011132 * cos((lat) * 1e-7 * M_PI / 180.0))))
                                        struct clew_point cpoint = clew_point_init(mnode->node->lon, mnode->node->lat);
                                        int32_t r_lat = METERS_TO_E7_LAT(distance);
                                        int32_t r_lon = METERS_TO_E7_LON(distance, cpoint.lat);
                                        sbound = clew_bound_init(cpoint.lon - r_lon, cpoint.lat - r_lat, cpoint.lon + r_lon, cpoint.lat + r_lat);
#endif
                                        if (clew_mesh_node_neighbours_count(mnode, min_neighbour_count) >= min_neighbour_count) {
                                                smnode = mnode;
                                                sdistance = distance;
                                        }
                                }
                        }
                }
                clew_infof("    nearest: %ld", smnode->node->id);
                clew_infof("             %.7f, %.7f", smnode->node->lon * 1e-7, smnode->node->lat * 1e-7);
                clew_infof("             %.3f meters", sdistance);

                {
                        struct clew_mesh_point mpoint;
                        mpoint.lon = clew_stack_at_int32(&clew->options.points, i + 0);
                        mpoint.lat = clew_stack_at_int32(&clew->options.points, i + 1);
                        mpoint.nearest_distance = sdistance;
                        mpoint.nearest_node     = smnode;
                        mpoint._solved          = 0;
                        rc = clew_stack_push(&clew->mesh_points, &mpoint);
                        if (rc < 0) {
                                clew_errorf("can not push mesh point");
                                goto bail;
                        }
                }
        }

        clew_infof("solving routes");
        clew->state = CLEW_STATE_SOLVE_ROUTES;

        for (i = 0, il = clew_stack_count(&clew->mesh_points); i < il; i++) {
                khiter_t k;
                struct clew_mesh_node *mnode;

                double pqueue_ocost;
                struct clew_pqueue *pqueue;
                struct clew_mesh_point *mpoint = clew_stack_at(&clew->mesh_points, i);

                clew_infof("  %ld: %.7f,%.7f", i, mpoint->lon * 1e-7, mpoint->lat * 1e-7);
                clew_infof("    nearest: %ld, %.3f meters", mpoint->nearest_node->node->id, mpoint->nearest_distance);

                for (j = 0, jl = clew_stack_count(&clew->mesh_points); j < jl; j++) {
                        struct clew_mesh_point *nmpoint = clew_stack_at(&clew->mesh_points, j);
                        nmpoint->_solved = 0;
                }

                clew_infof("    building pqueue");
                pqueue = clew_pqueue_create(
                        kh_size(clew->mesh_nodes) + 2,
                        64 * 1024,
                        mesh_node_pqueue_compare,
                        mesh_node_pqueue_setpos,
                        mesh_node_pqueue_getpos
                );

                for (k = kh_begin(clew->mesh_nodes); k != kh_end(clew->mesh_nodes); k++) {
                        if (!kh_exist(clew->mesh_nodes, k)) {
                                continue;
                        }
                        mnode = kh_val(clew->mesh_nodes, k);
                        mnode->pqueue_cost      = INFINITY;
                        mnode->pqueue_pos       = 0;
                        mnode->pqueue_prev      = NULL;
                        mnode->pqueue_distance  = 0;
                        mnode->pqueue_duration  = 0;
                        rc = clew_pqueue_add(pqueue, mnode);
                        if (rc < 0) {
                                clew_errorf("can not mesh node to pqueue");
                                clew_pqueue_destroy(pqueue);
                                goto bail;
                        }
                }

                clew_infof("    solving pqueue");
                {
                        uint64_t n;
                        uint64_t nl;
                        struct clew_mesh_node *rnode;
                        struct clew_mesh_node_neighbour *rneig;
                        pqueue_ocost = mpoint->nearest_node->pqueue_cost;
                        mpoint->nearest_node->pqueue_cost = 0;
                        clew_pqueue_mod(pqueue, mpoint->nearest_node, pqueue_ocost > mpoint->nearest_node->pqueue_cost);
                        while ((rnode = clew_pqueue_pop(pqueue)) != NULL) {
                                if (rnode->pqueue_cost == INFINITY) {
                                        clew_infof("      there are unsolved points");
                                        break;
                                }
                                for (j = 0, jl = clew_stack_count(&clew->mesh_points); j < jl; j++) {
                                        struct clew_mesh_point *nmpoint = clew_stack_at(&clew->mesh_points, j);
                                        if (mpoint == nmpoint) {
                                                continue;
                                        }
                                        if (nmpoint->_solved == 1) {
                                                continue;
                                        }
                                        if (0) {
                                                static double d = INFINITY;
                                                static struct clew_mesh_node *dnode = NULL;
                                                struct clew_point b = clew_point_init(rnode->node->lon, rnode->node->lat);
                                                struct clew_point e = clew_point_init(nmpoint->nearest_node->node->lon, nmpoint->nearest_node->node->lat);
                                                double f = clew_point_distance_euclidean(&b, &e);
                                                if (f < d) {
                                                        d = f;
                                                        dnode = rnode;
                                                        clew_infof("d: %ld", dnode->node->id);
                                                }
                                        }
                                        if (rnode == nmpoint->nearest_node) {
                                                time_t ts    = (time_t) rnode->pqueue_duration;
                                                struct tm *tm = gmtime(&ts);
                                                char duration[80];
                                                strftime(duration, sizeof(duration), "%H:%M:%S", tm);

                                                clew_infof("      %2ld: distance: %10.3f, duration: %s, cost: %.3f", j, rnode->pqueue_distance, duration, rnode->pqueue_cost);

                                                uint64_t nprnode;
                                                uint64_t tprnode;
                                                struct clew_mesh_node *prnode;
                                                struct clew_mesh_solution msolution;
                                                for (tprnode = 0, prnode = rnode; prnode; prnode = prnode->pqueue_prev) {
                                                        tprnode += 1;
                                                }

                                                msolution.source      = mpoint;
                                                msolution.destination = nmpoint;
                                                msolution.mesh_nodes  = clew_stack_init(sizeof(struct clew_mesh_node *));
                                                msolution.duration    = rnode->pqueue_duration;
                                                msolution.distance    = rnode->pqueue_distance;
                                                msolution.cost        = rnode->pqueue_cost;
                                                rc = clew_stack_resize(&msolution.mesh_nodes, tprnode);
                                                if (rc < 0) {
                                                        clew_errorf("stack reserve failed");
                                                        goto bail;
                                                }
                                                for (nprnode = 0, prnode = rnode; prnode; prnode = prnode->pqueue_prev) {
                                                        rc = clew_stack_put_at(&msolution.mesh_nodes, &prnode, tprnode - nprnode - 1);
                                                        if (rc < 0) {
                                                                clew_errorf("stack put at failed, t: %ld, n: %ld", tprnode, nprnode);
                                                                goto bail;
                                                        }
                                                        nprnode += 1;
                                                }
                                                clew_stack_push(&clew->mesh_solutions, &msolution);

                                                nmpoint->_solved = 1;
                                        }
                                }
                                for (j = 0, jl = clew_stack_count(&clew->mesh_points); j < jl; j++) {
                                        struct clew_mesh_point *nmpoint = clew_stack_at(&clew->mesh_points, j);
                                        if (mpoint == nmpoint) {
                                                continue;
                                        }
                                        if (nmpoint->_solved == 0) {
                                                break;
                                        }
                                }
                                if (j >= jl) {
                                        clew_infof("      all points are solved");
                                        break;
                                }
                                for (n = 0, nl = clew_stack_count(&rnode->mesh_neighbours); n < nl; n++) {
                                        rneig = (struct clew_mesh_node_neighbour *) clew_stack_at(&rnode->mesh_neighbours, n);
                                        if (rnode->pqueue_cost + rneig->cost < rneig->mesh_node->pqueue_cost) {
                                                rneig->mesh_node->pqueue_prev = rnode;

                                                pqueue_ocost = rneig->mesh_node->pqueue_cost;
                                                rneig->mesh_node->pqueue_cost     = rnode->pqueue_cost + rneig->cost;
                                                rneig->mesh_node->pqueue_distance =  rnode->pqueue_distance + rneig->distance;
                                                rneig->mesh_node->pqueue_duration =  rnode->pqueue_duration + rneig->duration;
                                                clew_pqueue_mod(pqueue, rneig->mesh_node, pqueue_ocost > rneig->mesh_node->pqueue_cost);
                                        }
                                }
                        }
                        for (j = 0, jl = clew_stack_count(&clew->mesh_points); j < jl; j++) {
                                struct clew_mesh_point *nmpoint = clew_stack_at(&clew->mesh_points, j);
                                if (mpoint == nmpoint) {
                                        continue;
                                }
                                if (nmpoint->_solved == 0) {
                                        clew_infof("        %ld: %.7f,%.7f", j, nmpoint->lon * 1e-7, nmpoint->lat * 1e-7);
                                }
                        }
                }

                clew_pqueue_destroy(pqueue);
        }

        clew_infof("writing solutions");

        FILE *fp = fopen("output.gpx", "w+b");

        fprintf(fp, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        fprintf(fp, "<gpx version=\"1.1\" xmlns=\"http://www.topografix.com/GPX/1/1\">\n");

        for (i = 0, il = clew_stack_count(&clew->mesh_points); i < il; i++) {
                struct clew_mesh_point *mpoint = clew_stack_at(&clew->mesh_points, i);
                fprintf(fp, " <wpt lon=\"%.7f\" lat=\"%.7f\">\n", mpoint->lon * 1e-7, mpoint->lat * 1e-7);
                fprintf(fp, "  <name>point %ld</name>\n", i);
                fprintf(fp, " </wpt>\n");
        }

        for (i = 0, il = clew_stack_count(&clew->mesh_solutions); i < il; i++) {
                struct clew_mesh_solution *msolution = clew_stack_at(&clew->mesh_solutions, i);
                clew_infof("  %.7f,%.7f -> %.7f,%.7f",
                        msolution->source->lon * 1e-7, msolution->source->lat * 1e-7,
                        msolution->destination->lon * 1e-7, msolution->destination->lat * 1e-7);

                time_t ts    = (time_t) msolution->duration;
                struct tm *tm = gmtime(&ts);
                char duration[80];
                strftime(duration, sizeof(duration), "%H:%M:%S", tm);

                clew_infof("    distance: %.3f meters", msolution->distance);
                clew_infof("    duration: %s, %.3f seconds", duration, msolution->duration);
                clew_infof("    cost    : %.3f", msolution->cost);
                clew_infof("    nodes   : %ld", clew_stack_count(&msolution->mesh_nodes));

                fprintf(fp, " <trk>\n");
                fprintf(fp, "  <name>%.7f,%.7f -> %.7f,%.7f</name>\n",
                        msolution->source->lon * 1e-7, msolution->source->lat * 1e-7,
                        msolution->destination->lon * 1e-7, msolution->destination->lat * 1e-7);
                fprintf(fp, "  <desc>distance=%.3fm duration=%.1fs cost=%.3f</desc>\n",
                        msolution->distance, msolution->duration, msolution->cost);
                fprintf(fp, "  <trkseg>\n");
                for (j = 0, jl = clew_stack_count(&msolution->mesh_nodes); j < jl; j++) {
                        struct clew_mesh_node *mnode = *(struct clew_mesh_node **) clew_stack_at(&msolution->mesh_nodes, j);
                        fprintf(fp, "   <trkpt lon=\"%.7f\" lat=\"%.7f\"/>\n", mnode->node->lon * 1e-7, mnode->node->lat * 1e-7);
                }
                fprintf(fp, "  </trkseg>\n");
                fprintf(fp, " </trk>\n");
        }

        fprintf(fp, "</gpx>\n");

        fclose(fp);

out:
        if (clew != NULL) {
                struct clew_mesh_node *mnode;
                clew_stack_uninit(&clew->options.inputs);
                clew_stack_uninit(&clew->options.clip_path);
                clew_stack_uninit(&clew->options.points);
                clew_expression_destroy(clew->options.keep_tags);
                clew_expression_destroy(clew->options.keep_tags_node);
                clew_expression_destroy(clew->options.keep_tags_way);
                clew_expression_destroy(clew->options.keep_tags_relation);
                clew_stack_uninit(&clew->read_state);
                clew_bitmap_uninit(&clew->node_ids);
                clew_bitmap_uninit(&clew->way_ids);
                clew_bitmap_uninit(&clew->relation_ids);
                clew_stack_uninit(&clew->nodes);
                clew_stack_uninit(&clew->ways);
                clew_stack_uninit(&clew->relations);
                clew_stack_uninit(&clew->mesh_ways);
                kh_foreach_value(clew->mesh_nodes, mnode,
                        clew_mesh_node_destroy(mnode);
                )
                kh_destroy(mesh_nodes, clew->mesh_nodes);
                clew_stack_uninit(&clew->mesh_points);
                clew_stack_uninit(&clew->mesh_solutions);
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
