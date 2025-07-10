
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "debug.h"
#include "input.h"

static int input_callback_bounds_start (struct clew_input *input, void *context)
{
        (void) input;
        (void) context;
        return 0;
}

static int input_callback_bounds_end (struct clew_input *input, void *context)
{
        (void) input;
        (void) context;
        return 0;
}

static int input_callback_node_start (struct clew_input *input, void *context)
{
        (void) input;
        (void) context;
        return 0;
}

static int input_callback_node_end (struct clew_input *input, void *context)
{
        (void) input;
        (void) context;
        return 0;
}

static int input_callback_way_start (struct clew_input *input, void *context)
{
        (void) input;
        (void) context;
        return 0;
}

static int input_callback_way_end (struct clew_input *input, void *context)
{
        (void) input;
        (void) context;
        return 0;
}

static int input_callback_relation_start (struct clew_input *input, void *context)
{
        (void) input;
        (void) context;
        return 0;
}

static int input_callback_relation_end (struct clew_input *input, void *context)
{
        (void) input;
        (void) context;
        return 0;
}

static int input_callback_tag_start (struct clew_input *input, void *context)
{
        (void) input;
        (void) context;
        return 0;
}

static int input_callback_tag_end (struct clew_input *input, void *context)
{
        (void) input;
        (void) context;
        return 0;
}

static int input_callback_nd_start (struct clew_input *input, void *context)
{
        (void) input;
        (void) context;
        return 0;
}

static int input_callback_nd_end (struct clew_input *input, void *context)
{
        (void) input;
        (void) context;
        return 0;
}

static int input_callback_member_start (struct clew_input *input, void *context)
{
        (void) input;
        (void) context;
        return 0;
}

static int input_callback_member_end (struct clew_input *input, void *context)
{
        (void) input;
        (void) context;
        return 0;
}

static int input_callback_minlon (struct clew_input *input, void *context, uint32_t lon)
{
        (void) input;
        (void) context;
        (void) lon;
        return 0;
}

static int input_callback_minlat (struct clew_input *input, void *context, uint32_t lat)
{
        (void) input;
        (void) context;
        (void) lat;
        return 0;
}

static int input_callback_maxlon (struct clew_input *input, void *context, uint32_t lon)
{
        (void) input;
        (void) context;
        (void) lon;
        return 0;
}

static int input_callback_maxlat (struct clew_input *input, void *context, uint32_t lat)
{
        (void) input;
        (void) context;
        (void) lat;
        return 0;
}

static int input_callback_id (struct clew_input *input, void *context, uint64_t id)
{
        (void) input;
        (void) context;
        (void) id;
        //clew_errorf("id: %ld", id);
        return 0;
}

static int input_callback_lat (struct clew_input *input, void *context, uint32_t lat)
{
        (void) input;
        (void) context;
        (void) lat;
        return 0;
}

static int input_callback_lon (struct clew_input *input, void *context, uint32_t lon)
{
        (void) input;
        (void) context;
        (void) lon;
        return 0;
}

static int input_callback_ref (struct clew_input *input, void *context, uint64_t ref)
{
        (void) input;
        (void) context;
        (void) ref;
        //clew_errorf("ref: %ld", ref);
        return 0;
}

static int input_callback_type (struct clew_input *input, void *context, const char *type)
{
        (void) input;
        (void) context;
        (void) type;
        return 0;
}

static int input_callback_role (struct clew_input *input, void *context, const char *role)
{
        (void) input;
        (void) context;
        (void) role;
        return 0;
}

static int input_callback_k (struct clew_input *input, void *context, const char *k)
{
        (void) input;
        (void) context;
        (void) k;
        return 0;
}

static int input_callback_v (struct clew_input *input, void *context, const char *v)
{
        (void) input;
        (void) context;
        (void) v;
        return 0;
}

static int input_callback_error (struct clew_input *input, void *context, unsigned int reason)
{
        (void) input;
        (void) context;
        (void) reason;
        return 0;
}

int main (int argc, char *argv[])
{
        struct clew_input *input;
        struct clew_input_init_options input_init_options;

        (void) argc;
        (void) argv;

        clew_input_init_options_default(&input_init_options);
        input_init_options.path                         = argv[1];
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
        input_init_options.callback_context             = NULL;

        input = clew_input_create(&input_init_options);
        while (clew_input_read(input) == 0) {

        }
        clew_input_destroy(input);

        return 0;
}
