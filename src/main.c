
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <getopt.h>

#define CLEW_DEBUG_NAME                 "clew"
#include "debug.h"
#include "input.h"
#include "stack.h"
#include "tag.h"

#define OPTION_HELP                     'h'

#define OPTION_INPUT                    'i'
#define OPTION_OUTPUT                   'o'

#define OPTION_EXPRESSION               'e'

#define OPTION_KEEP_TAG                 'k'
#define OPTION_KEEP_TAG_NODE            0x201
#define OPTION_KEEP_TAG_WAY             0x202
#define OPTION_KEEP_TAG_RELATION        0x203

#define OPTION_DROP_TAG                 'd'
#define OPTION_DROP_TAG_NODE            0x301
#define OPTION_DROP_TAG_WAY             0x302
#define OPTION_DROP_TAG_RELATION        0x303

#define OPTION_KEEP_NODES               'n'
#define OPTION_KEEP_WAYS                'w'
#define OPTION_KEEP_RELATIONS           'r'

#define OPTION_DROP_NODES               0x404
#define OPTION_DROP_WAYS                0x405
#define OPTION_DROP_RELATIONS           0x406

static struct option g_long_options[] = {
        { "help",                     no_argument,        0,        OPTION_HELP                     },
        { "input",                     required_argument,        0,        OPTION_INPUT                    },
        { "output",                     required_argument,        0,        OPTION_OUTPUT                   },
        { "expression",                     required_argument,        0,        OPTION_EXPRESSION               },
        { "keep-tag",                required_argument,        0,        OPTION_KEEP_TAG                 },
        { "keep-tag-node",           required_argument,        0,        OPTION_KEEP_TAG_NODE            },
        { "keep-tag-way",            required_argument,        0,        OPTION_KEEP_TAG_WAY             },
        { "keep-tag-relation",       required_argument,        0,      OPTION_KEEP_TAG_RELATION        },
        { "drop-tag",                required_argument,        0,        OPTION_DROP_TAG                 },
        { "drop-tag-node",           required_argument,        0,        OPTION_DROP_TAG_NODE            },
        { "drop-tag-way",            required_argument,        0,        OPTION_DROP_TAG_WAY             },
        { "drop-tag-relation",       required_argument,        0,      OPTION_DROP_TAG_RELATION        },
        { "keep-nodes",              required_argument,        0,        OPTION_KEEP_NODES               },
        { "keep-ways",               required_argument,        0,        OPTION_KEEP_WAYS                },
        { "keep-relations",          required_argument,        0,        OPTION_KEEP_RELATIONS           },
        { "drop-nodes",              required_argument,        0,        OPTION_DROP_NODES               },
        { "drop-ways",               required_argument,        0,        OPTION_DROP_WAYS                },
        { "drop-relations",          required_argument,        0,        OPTION_DROP_RELATIONS           },
        { 0,                             0,                        0,        0                               }
};

enum {
        CLEW_STATE_INITIAL,
#define CLEW_STATE_INITIAL      CLEW_STATE_INITIAL
};

enum {
        CLEW_READ_STATE_UNKNOWN         = 0,
        CLEW_READ_STATE_BOUNDS          = 1,
        CLEW_READ_STATE_NODE            = 2,
        CLEW_READ_STATE_WAY             = 3,
        CLEW_READ_STATE_RELATION        = 4,
        CLEW_READ_STATE_TAG             = 5,
        CLEW_READ_STATE_ND              = 6,
        CLEW_READ_STATE_MEMBER          = 7
#define CLEW_READ_STATE_UNKNOWN         CLEW_READ_STATE_UNKNOWN
#define CLEW_READ_STATE_BOUNDS          CLEW_READ_STATE_BOUNDS
#define CLEW_READ_STATE_NODE            CLEW_READ_STATE_NODE
#define CLEW_READ_STATE_WAY             CLEW_READ_STATE_WAY
#define CLEW_READ_STATE_RELATION        CLEW_READ_STATE_RELATION
#define CLEW_READ_STATE_TAG             CLEW_READ_STATE_TAG
#define CLEW_READ_STATE_ND              CLEW_READ_STATE_ND
#define CLEW_READ_STATE_MEMBER          CLEW_READ_STATE_MEMBER
};

struct clew_options {
        struct clew_stack inputs;
        const char *output;
        const char *expression;
        struct clew_stack keep_tag_node;
        struct clew_stack keep_tag_way;
        struct clew_stack keep_tag_relation;
        struct clew_stack drop_tag_node;
        struct clew_stack drop_tag_way;
        struct clew_stack drop_tag_relation;
        int keep_nodes;
        int keep_ways;
        int keep_relations;
        int drop_nodes;
        int drop_ways;
        int drop_relations;
};

struct clew {
        struct clew_options options;

        int state;

        struct clew_stack read_state;
        struct clew_stack node_ids;
        struct clew_stack node_tags;
        struct clew_stack way_ids;
        struct clew_stack way_tags;
        struct clew_stack relation_ids;
        struct clew_stack relation_tags;
        #define TAG_K_LENGTH    512
        #define TAG_V_LENGTH    512
        #define TAG_S_LENGTH    (TAG_K_LENGTH + TAG_V_LENGTH)
        char tag_k[512];
        char tag_v[512];
        char tag_s[1024];

};

static int input_callback_bounds_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_UNKNOWN) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_UNKNOWN);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_BOUNDS);

        return 0;
bail:   return -1;
}

static int input_callback_bounds_end (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_BOUNDS) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_BOUNDS);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        return 0;
bail:   return -1;
}

static int input_callback_node_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_UNKNOWN) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_UNKNOWN);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_NODE);

        clew_stack_reset(&clew->node_tags);

        return 0;
bail:   return -1;
}

static int input_callback_node_end (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_NODE) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        return 0;
bail:   return -1;
}

static int input_callback_way_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_UNKNOWN) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_UNKNOWN);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_WAY);

        clew_stack_reset(&clew->way_tags);

        return 0;
bail:   return -1;
}

static int input_callback_way_end (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_WAY) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_WAY);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        return 0;
bail:   return -1;
}

static int input_callback_relation_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_UNKNOWN) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_UNKNOWN);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_RELATION);

        clew_stack_reset(&clew->relation_tags);

        return 0;
bail:   return -1;
}

static int input_callback_relation_end (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_RELATION) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_RELATION);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        return 0;
bail:   return -1;
}

static void parse_tag_fix_layer (char *k, char *v)
{
        int layer;
        (void) k;
        layer = atoi(v);
        if (layer == 0) {
                snprintf(v, TAG_V_LENGTH, "layer_ground");
        } else if (layer > 0) {
                snprintf(v, TAG_V_LENGTH, "layer_above_%d", layer);
        } else {
                snprintf(v, TAG_V_LENGTH, "layer_below_%d", -layer);
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

static int input_callback_tag_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_NODE &&
            clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_WAY &&
            clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_RELATION) {
                clew_errorf("read state is invalid, %d != %d || %d || %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE, CLEW_READ_STATE_WAY, CLEW_READ_STATE_RELATION);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_TAG);

        return 0;
bail:   return -1;
}

static int input_callback_tag_end (struct clew_input *input, void *context)
{
        int rc;
        uint32_t tag;
        struct clew_stack *tags;

        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_TAG) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_TAG);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        switch (clew_stack_peek_uint32(&clew->read_state)) {
                case CLEW_READ_STATE_NODE:      tags = &clew->node_tags;        break;
                case CLEW_READ_STATE_WAY:       tags = &clew->way_tags;         break;
                case CLEW_READ_STATE_RELATION:  tags = &clew->relation_tags;    break;
                default:
                        clew_errorf("read state is invalid, %d != %d || %d || %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE, CLEW_READ_STATE_WAY, CLEW_READ_STATE_RELATION);
                        goto bail;
        }

        if (strlen(clew->tag_k) <= 0) {
                clew_debugf("k is invalid: %s = %s", clew->tag_k, clew->tag_v);
                goto out;
        }
        if (strlen(clew->tag_v) <= 0) {
                clew_debugf("v is invalid: %s = %s", clew->tag_k, clew->tag_v);
                goto out;
        }

        parse_tag_fix(clew->tag_k, clew->tag_v);
        snprintf(clew->tag_s, sizeof(clew->tag_s), "%s_%s", clew->tag_k, clew->tag_v);
        tag = clew_tag_value(clew->tag_s);
        if (tag == clew_tag_unknown) {
                //clew_todof("unknown tag: '%s' = '%s'", clew->tag_k, clew->tag_v);
                goto out;
        }
        rc = clew_stack_push_uint32(tags, tag);
        if (rc != 0) {
                clew_errorf("can not add tag");
                goto bail;
        }

out:    clew->tag_k[0] = '\0';
        clew->tag_v[0] = '\0';
        return 0;
bail:   clew->tag_k[0] = '\0';
        clew->tag_v[0] = '\0';
        return -1;
}

static int input_callback_nd_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_WAY) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_WAY);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_ND);

        return 0;
bail:   return -1;
}

static int input_callback_nd_end (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_ND) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_ND);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        return 0;
bail:   return -1;
}

static int input_callback_member_start (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_RELATION) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_RELATION);
                goto bail;
        }
        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_MEMBER);

        return 0;
bail:   return -1;
}

static int input_callback_member_end (struct clew_input *input, void *context)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_MEMBER) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_MEMBER);
                goto bail;
        }
        clew_stack_pop(&clew->read_state);

        return 0;
bail:   return -1;
}

static int input_callback_minlon (struct clew_input *input, void *context, uint32_t lon)
{
        struct clew *clew = context;

        (void) input;
        (void) lon;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_BOUNDS) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_BOUNDS);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_minlat (struct clew_input *input, void *context, uint32_t lat)
{
        struct clew *clew = context;

        (void) input;
        (void) lat;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_BOUNDS) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_BOUNDS);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_maxlon (struct clew_input *input, void *context, uint32_t lon)
{
        struct clew *clew = context;

        (void) input;
        (void) lon;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_BOUNDS) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_BOUNDS);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_maxlat (struct clew_input *input, void *context, uint32_t lat)
{
        struct clew *clew = context;

        (void) input;
        (void) lat;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_BOUNDS) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_BOUNDS);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_id (struct clew_input *input, void *context, uint64_t id)
{
        struct clew *clew = context;

        (void) input;
        (void) id;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_NODE &&
            clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_WAY &&
            clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_RELATION) {
                clew_errorf("read state is invalid, %d != %d || %d || %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE, CLEW_READ_STATE_WAY, CLEW_READ_STATE_RELATION);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_lon (struct clew_input *input, void *context, uint32_t lon)
{
        struct clew *clew = context;

        (void) input;
        (void) lon;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_NODE) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_lat (struct clew_input *input, void *context, uint32_t lat)
{
        struct clew *clew = context;

        (void) input;
        (void) lat;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_NODE) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_NODE);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_ref (struct clew_input *input, void *context, uint64_t ref)
{
        struct clew *clew = context;

        (void) input;
        (void) ref;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_ND &&
            clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_MEMBER) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_ND);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_type (struct clew_input *input, void *context, const char *type)
{
        struct clew *clew = context;

        (void) input;
        (void) type;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_MEMBER) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_MEMBER);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_role (struct clew_input *input, void *context, const char *role)
{
        struct clew *clew = context;

        (void) input;
        (void) role;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_MEMBER) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_MEMBER);
                goto bail;
        }

        return 0;
bail:   return -1;
}

static int input_callback_k (struct clew_input *input, void *context, const char *k)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_TAG) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_TAG);
                goto bail;
        }

        snprintf(clew->tag_k, sizeof(clew->tag_k), "%s", k);

        return 0;
bail:   return -1;
}

static int input_callback_v (struct clew_input *input, void *context, const char *v)
{
        struct clew *clew = context;

        (void) input;

        if (clew_stack_peek_uint32(&clew->read_state) != CLEW_READ_STATE_TAG) {
                clew_errorf("read state is invalid, %d != %d", clew_stack_peek_uint32(&clew->read_state), CLEW_READ_STATE_TAG);
                goto bail;
        }

        snprintf(clew->tag_v, sizeof(clew->tag_v), "%s", v);

        return 0;
bail:   return -1;
}

static int input_callback_error (struct clew_input *input, void *context, unsigned int reason)
{
        struct clew *clew = context;
        (void) input;
        (void) clew;
        (void) reason;
        return 0;
}

int main (int argc, char *argv[])
{
        int c;
        int option_index;

        int rs;
        int rc;

        int i;
        int il;

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

        clew->options.inputs            = clew_stack_init(sizeof(const char *));
        clew->options.output            = NULL;
        clew->options.expression        = NULL;
        clew->options.keep_tag_node     = clew_stack_init(sizeof(const char *));
        clew->options.keep_tag_way      = clew_stack_init(sizeof(const char *));
        clew->options.keep_tag_relation = clew_stack_init(sizeof(const char *));
        clew->options.drop_tag_node     = clew_stack_init(sizeof(const char *));
        clew->options.drop_tag_way      = clew_stack_init(sizeof(const char *));
        clew->options.drop_tag_relation = clew_stack_init(sizeof(const char *));
        clew->options.keep_nodes        = -1;
        clew->options.keep_ways         = -1;
        clew->options.keep_relations    = -1;
        clew->options.drop_nodes        = -1;
        clew->options.drop_ways         = -1;
        clew->options.drop_relations    = -1;

        clew->state             = CLEW_STATE_INITIAL;
        clew->read_state        = clew_stack_init(sizeof(uint32_t));
        clew->node_ids          = clew_stack_init(sizeof(uint64_t));
        clew->node_tags         = clew_stack_init(sizeof(uint32_t));
        clew->way_ids           = clew_stack_init(sizeof(uint64_t));
        clew->way_tags          = clew_stack_init(sizeof(uint32_t));
        clew->relation_ids      = clew_stack_init(sizeof(uint64_t));
        clew->relation_tags     = clew_stack_init(sizeof(uint32_t));

        clew_stack_push_uint32(&clew->read_state, CLEW_READ_STATE_UNKNOWN);

        while (1) {
                c = getopt_long(argc, argv, "i:o:e:k:d:n:w:r:v:h", g_long_options, &option_index);
                if (c == -1) {
                        break;
                }
                switch (c) {
                        case OPTION_HELP:
                                break;
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
                        case OPTION_EXPRESSION:
                                clew->options.expression = optarg;
                                break;
                        case OPTION_KEEP_TAG:
                                rc  = clew_stack_push(&clew->options.keep_tag_node, &optarg);
                                rc |= clew_stack_push(&clew->options.keep_tag_way, &optarg);
                                rc |= clew_stack_push(&clew->options.keep_tag_relation, &optarg);
                                if (rc < 0) {
                                        clew_errorf("can not add tag to stack");
                                        goto bail;
                                }
                                break;
                        case OPTION_KEEP_TAG_NODE:
                                rc = clew_stack_push(&clew->options.keep_tag_node, &optarg);
                                if (rc < 0) {
                                        clew_errorf("can not add tag to stack");
                                        goto bail;
                                }
                                break;
                        case OPTION_KEEP_TAG_WAY:
                                rc = clew_stack_push(&clew->options.keep_tag_way, &optarg);
                                if (rc < 0) {
                                        clew_errorf("can not add tag to stack");
                                        goto bail;
                                }
                                break;
                        case OPTION_KEEP_TAG_RELATION:
                                rc = clew_stack_push(&clew->options.keep_tag_relation, &optarg);
                                if (rc < 0) {
                                        clew_errorf("can not add tag to stack");
                                        goto bail;
                                }
                                break;
                        case OPTION_DROP_TAG:
                                rc  = clew_stack_push(&clew->options.drop_tag_node, &optarg);
                                rc |= clew_stack_push(&clew->options.drop_tag_way, &optarg);
                                rc |= clew_stack_push(&clew->options.drop_tag_relation, &optarg);
                                if (rc < 0) {
                                        clew_errorf("can not add tag to stack");
                                        goto bail;
                                }
                                break;
                        case OPTION_DROP_TAG_NODE:
                                rc = clew_stack_push(&clew->options.drop_tag_node, &optarg);
                                if (rc < 0) {
                                        clew_errorf("can not add tag to stack");
                                        goto bail;
                                }
                                break;
                        case OPTION_DROP_TAG_WAY:
                                rc = clew_stack_push(&clew->options.drop_tag_way, &optarg);
                                if (rc < 0) {
                                        clew_errorf("can not add tag to stack");
                                        goto bail;
                                }
                                break;
                        case OPTION_DROP_TAG_RELATION:
                                rc = clew_stack_push(&clew->options.drop_tag_relation, &optarg);
                                if (rc < 0) {
                                        clew_errorf("can not add tag to stack");
                                        goto bail;
                                }
                                break;
                        case OPTION_KEEP_NODES:
                                clew->options.keep_nodes = !!atoi(optarg);
                                break;
                        case OPTION_KEEP_WAYS:
                                clew->options.keep_nodes = !!atoi(optarg);
                                break;
                        case OPTION_KEEP_RELATIONS:
                                clew->options.keep_nodes = !!atoi(optarg);
                                break;
                        case OPTION_DROP_NODES:
                                clew->options.drop_nodes = !!atoi(optarg);
                                break;
                        case OPTION_DROP_WAYS:
                                clew->options.drop_nodes = !!atoi(optarg);
                                break;
                        case OPTION_DROP_RELATIONS:
                                clew->options.drop_nodes = !!atoi(optarg);
                                break;
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
        if (clew->options.expression == NULL) {
                clew_errorf("expression is invalid, see help");
                goto bail;
        }

        for (i = 0, il = clew_stack_count(&clew->options.inputs); i < il; i++) {
                clew_infof("reading input: %s", *(char **) clew_stack_at(&clew->options.inputs, i));

                clew_input_init_options_default(&input_init_options);
                input_init_options.path                         = *(char **) clew_stack_at(&clew->options.inputs, i);
                input_init_options.callback_bounds_start        = input_callback_bounds_start;
                input_init_options.callback_bounds_end          = input_callback_bounds_end;
                input_init_options.callback_node_start          = input_callback_node_start;
                input_init_options.callback_node_end            = input_callback_node_end;
                input_init_options.callback_way_start           = input_callback_way_start;
                input_init_options.callback_way_end             = input_callback_way_end;
                input_init_options.callback_relation_start      = input_callback_relation_start;
                input_init_options.callback_relation_end        = input_callback_relation_end;
                input_init_options.callback_tag_start           = input_callback_tag_start;
                input_init_options.callback_tag_end             = input_callback_tag_end;
                input_init_options.callback_nd_start            = input_callback_nd_start;
                input_init_options.callback_nd_end              = input_callback_nd_end;
                input_init_options.callback_member_start        = input_callback_member_start;
                input_init_options.callback_member_end          = input_callback_member_end;
                input_init_options.callback_minlon              = input_callback_minlon;
                input_init_options.callback_minlat              = input_callback_minlat;
                input_init_options.callback_maxlon              = input_callback_maxlon;
                input_init_options.callback_maxlat              = input_callback_maxlat;
                input_init_options.callback_id                  = input_callback_id;
                input_init_options.callback_lat                 = input_callback_lat;
                input_init_options.callback_lon                 = input_callback_lon;
                input_init_options.callback_ref                 = input_callback_ref;
                input_init_options.callback_type                = input_callback_type;
                input_init_options.callback_role                = input_callback_role;
                input_init_options.callback_k                   = input_callback_k;
                input_init_options.callback_v                   = input_callback_v;
                input_init_options.callback_error               = input_callback_error;
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

out:
        if (clew != NULL) {
                clew_stack_uninit(&clew->options.inputs);
                clew_stack_uninit(&clew->options.keep_tag_node);
                clew_stack_uninit(&clew->options.keep_tag_way);
                clew_stack_uninit(&clew->options.keep_tag_relation);
                clew_stack_uninit(&clew->options.drop_tag_node);
                clew_stack_uninit(&clew->options.drop_tag_way);
                clew_stack_uninit(&clew->options.drop_tag_relation);
                clew_stack_uninit(&clew->read_state);
                clew_stack_uninit(&clew->node_ids);
                clew_stack_uninit(&clew->node_tags);
                clew_stack_uninit(&clew->way_ids);
                clew_stack_uninit(&clew->way_tags);
                clew_stack_uninit(&clew->relation_ids);
                clew_stack_uninit(&clew->relation_tags);
                free(clew);
        }

        clew_tag_fini();
        clew_debug_fini();
        return rs;

bail:   rs = -1;
        goto out;
}
