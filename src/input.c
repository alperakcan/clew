
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "debug.h"
#include "input.h"
#include "input-backend.h"
#include "input-osm-pbf.h"

struct clew_input {
        struct clew_input_backend *backend;

	int (*callback_bounds_start) (struct clew_input *input, void *context);
	int (*callback_bounds_end) (struct clew_input *input, void *context);

	int (*callback_node_start) (struct clew_input *input, void *context);
	int (*callback_node_end) (struct clew_input *input, void *context);

	int (*callback_way_start) (struct clew_input *input, void *context);
	int (*callback_way_end) (struct clew_input *input, void *context);

	int (*callback_relation_start) (struct clew_input *input, void *context);
	int (*callback_relation_end) (struct clew_input *input, void *context);

	int (*callback_tag_start) (struct clew_input *input, void *context);
	int (*callback_tag_end) (struct clew_input *input, void *context);

	int (*callback_nd_start) (struct clew_input *input, void *context);
	int (*callback_nd_end) (struct clew_input *input, void *context);

	int (*callback_member_start) (struct clew_input *input, void *context);
	int (*callback_member_end) (struct clew_input *input, void *context);

	int (*callback_minlon) (struct clew_input *input, void *context, int32_t lon);
	int (*callback_minlat) (struct clew_input *input, void *context, int32_t lat);
	int (*callback_maxlon) (struct clew_input *input, void *context, int32_t lon);
	int (*callback_maxlat) (struct clew_input *input, void *context, int32_t lat);

	int (*callback_id) (struct clew_input *input, void *context, uint64_t id);

	int (*callback_lat) (struct clew_input *input, void *context, int32_t lat);
	int (*callback_lon) (struct clew_input *input, void *context, int32_t lon);

	int (*callback_ref) (struct clew_input *input, void *context, uint64_t ref);

	int (*callback_type) (struct clew_input *input, void *context, const char *type);
	int (*callback_role) (struct clew_input *input, void *context, const char *role);

	int (*callback_k) (struct clew_input *input, void *context, const char *k);
	int (*callback_v) (struct clew_input *input, void *context, const char *v);

        int error;
        int (*callback_error) (struct clew_input *input, void *context, unsigned int reason);

        void *callback_context;
};

struct {
        const char *name;
        struct clew_input_backend * (*creator) (struct clew_input_backend_init_options *options);
} g_backends[] = {
        { "osm-pbf", clew_input_osm_pbf_create }
};

static int clew_input_backend_callback_bounds_start (struct clew_input_backend *backend, void *context)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_bounds_start != NULL) {
                rc = input->callback_bounds_start(input, input->callback_context);
                if (rc < 0) {
                        clew_errorf("input callback_bounds_start failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_bounds_end (struct clew_input_backend *backend, void *context)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_bounds_end != NULL) {
                rc = input->callback_bounds_end(input, input->callback_context);
                if (rc < 0) {
                        clew_errorf("input callback_bounds_end failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_node_start (struct clew_input_backend *backend, void *context)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        rc = 0;
        if (input->callback_node_start != NULL) {
                rc = input->callback_node_start(input, input->callback_context);
                if (rc < 0) {
                        clew_errorf("input callback_node_start failed");
                        goto bail;
                }
        }

        return rc;
bail:   return -1;
}

static int clew_input_backend_callback_node_end (struct clew_input_backend *backend, void *context)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_node_end != NULL) {
                rc = input->callback_node_end(input, input->callback_context);
                if (rc < 0) {
                        clew_errorf("input callback_node_end failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_way_start (struct clew_input_backend *backend, void *context)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        rc = 0;
        if (input->callback_way_start != NULL) {
                rc = input->callback_way_start(input, input->callback_context);
                if (rc < 0) {
                        clew_errorf("input callback_way_start failed");
                        goto bail;
                }
        }

        return rc;
bail:   return -1;
}

static int clew_input_backend_callback_way_end (struct clew_input_backend *backend, void *context)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_way_end != NULL) {
                rc = input->callback_way_end(input, input->callback_context);
                if (rc < 0) {
                        clew_errorf("input callback_way_end failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_relation_start (struct clew_input_backend *backend, void *context)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        rc = 0;
        if (input->callback_relation_start != NULL) {
                rc = input->callback_relation_start(input, input->callback_context);
                if (rc < 0) {
                        clew_errorf("input callback_relation_start failed");
                        goto bail;
                }
        }

        return rc;
bail:   return -1;
}

static int clew_input_backend_callback_relation_end (struct clew_input_backend *backend, void *context)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_relation_end != NULL) {
                rc = input->callback_relation_end(input, input->callback_context);
                if (rc < 0) {
                        clew_errorf("input callback_relation_end failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_tag_start (struct clew_input_backend *backend, void *context)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_tag_start != NULL) {
                rc = input->callback_tag_start(input, input->callback_context);
                if (rc < 0) {
                        clew_errorf("input callback_tag_start failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_tag_end (struct clew_input_backend *backend, void *context)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_tag_end != NULL) {
                rc = input->callback_tag_end(input, input->callback_context);
                if (rc < 0) {
                        clew_errorf("input callback_tag_end failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_nd_start (struct clew_input_backend *backend, void *context)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_nd_start != NULL) {
                rc = input->callback_nd_start(input, input->callback_context);
                if (rc < 0) {
                        clew_errorf("input callback_nd_start failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_nd_end (struct clew_input_backend *backend, void *context)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_nd_end != NULL) {
                rc = input->callback_nd_end(input, input->callback_context);
                if (rc < 0) {
                        clew_errorf("input callback_nd_end failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_member_start (struct clew_input_backend *backend, void *context)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_member_start != NULL) {
                rc = input->callback_member_start(input, input->callback_context);
                if (rc < 0) {
                        clew_errorf("input callback_member_start failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_member_end (struct clew_input_backend *backend, void *context)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_member_end != NULL) {
                rc = input->callback_member_end(input, input->callback_context);
                if (rc < 0) {
                        clew_errorf("input callback_member_end failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_minlon (struct clew_input_backend *backend, void *context, int32_t minlon)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_minlon != NULL) {
                rc = input->callback_minlon(input, input->callback_context, minlon);
                if (rc < 0) {
                        clew_errorf("input callback_minlon failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_minlat (struct clew_input_backend *backend, void *context, int32_t minlat)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_minlat != NULL) {
                rc = input->callback_minlat(input, input->callback_context, minlat);
                if (rc < 0) {
                        clew_errorf("input callback_minlat failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_maxlon (struct clew_input_backend *backend, void *context, int32_t maxlon)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_maxlon != NULL) {
                rc = input->callback_maxlon(input, input->callback_context, maxlon);
                if (rc < 0) {
                        clew_errorf("input callback_maxlon failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_maxlat (struct clew_input_backend *backend, void *context, int32_t maxlat)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_maxlat != NULL) {
                rc = input->callback_maxlat(input, input->callback_context, maxlat);
                if (rc < 0) {
                        clew_errorf("input callback_maxlat failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_id (struct clew_input_backend *backend, void *context, uint64_t id)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        rc = 0;
        if (input->callback_id != NULL) {
                rc = input->callback_id(input, input->callback_context, id);
                if (rc < 0) {
                        clew_errorf("input callback_id failed");
                        goto bail;
                }
        }

        return rc;
bail:   return -1;
}

static int clew_input_backend_callback_lon (struct clew_input_backend *backend, void *context, int32_t lon)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_lon != NULL) {
                rc = input->callback_lon(input, input->callback_context, lon);
                if (rc < 0) {
                        clew_errorf("input callback_lon failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_lat (struct clew_input_backend *backend, void *context, int32_t lat)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_lat != NULL) {
                rc = input->callback_lat(input, input->callback_context, lat);
                if (rc < 0) {
                        clew_errorf("input callback_lat failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_ref (struct clew_input_backend *backend, void *context, uint64_t ref)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_ref != NULL) {
                rc = input->callback_ref(input, input->callback_context, ref);
                if (rc < 0) {
                        clew_errorf("input callback_ref failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_type (struct clew_input_backend *backend, void *context, const char *type)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_type != NULL) {
                rc = input->callback_type(input, input->callback_context, type);
                if (rc < 0) {
                        clew_errorf("input callback_type failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_role (struct clew_input_backend *backend, void *context, const char *role)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_role != NULL) {
                rc = input->callback_role(input, input->callback_context, role);
                if (rc < 0) {
                        clew_errorf("input callback_role failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_k (struct clew_input_backend *backend, void *context, const char *k)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_k != NULL) {
                rc = input->callback_k(input, input->callback_context, k);
                if (rc < 0) {
                        clew_errorf("input callback_k failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_v (struct clew_input_backend *backend, void *context, const char *v)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->callback_v != NULL) {
                rc = input->callback_v(input, input->callback_context, v);
                if (rc < 0) {
                        clew_errorf("input callback_v failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

static int clew_input_backend_callback_error (struct clew_input_backend *backend, void *context, unsigned int reason)
{
        int rc;
        struct clew_input *input = (struct clew_input *) context;

        if (backend == NULL) {
                clew_errorf("backend is invalid");
                goto bail;
        }
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        input->error = reason;

        if (input->callback_error != NULL) {
                rc = input->callback_error(input, input->callback_context, reason);
                if (rc < 0) {
                        clew_errorf("input callback_error failed");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

int clew_input_init_options_default (struct clew_input_init_options *options)
{
        if (options == NULL) {
                clew_errorf("options is invalid");
                goto bail;
        }
        memset(options, 0, sizeof(struct clew_input_init_options));
        return 0;
bail:   return -1;
}

struct clew_input * clew_input_create (struct clew_input_init_options *options)
{
        int i;
        struct clew_input *input;
        struct clew_input_backend_init_options backend_options;

        input = NULL;

        if (options == NULL) {
                clew_errorf("options is invalid");
                goto bail;
        }

        input = malloc(sizeof(struct clew_input));
        if (input == NULL) {
                clew_errorf("can not allocate memory");
                goto bail;
        }
        memset(input, 0, sizeof(struct clew_input));

	input->callback_bounds_start    = options->callback_bounds_start;
	input->callback_bounds_end      = options->callback_bounds_end;
	input->callback_node_start      = options->callback_node_start;
	input->callback_node_end        = options->callback_node_end;
	input->callback_way_start       = options->callback_way_start;
	input->callback_way_end         = options->callback_way_end;
	input->callback_relation_start  = options->callback_relation_start;
	input->callback_relation_end    = options->callback_relation_end;
	input->callback_tag_start       = options->callback_tag_start;
	input->callback_tag_end         = options->callback_tag_end;
	input->callback_nd_start        = options->callback_nd_start;
	input->callback_nd_end          = options->callback_nd_end;
	input->callback_member_start    = options->callback_member_start;
	input->callback_member_end      = options->callback_member_end;
	input->callback_minlon          = options->callback_minlon;
	input->callback_minlat          = options->callback_minlat;
	input->callback_maxlon          = options->callback_maxlon;
	input->callback_maxlat          = options->callback_maxlat;
	input->callback_id              = options->callback_id;
	input->callback_lat             = options->callback_lat;
	input->callback_lon             = options->callback_lon;
	input->callback_ref             = options->callback_ref;
	input->callback_type            = options->callback_type;
	input->callback_role            = options->callback_role;
	input->callback_k               = options->callback_k;
	input->callback_v               = options->callback_v;
        input->callback_error           = options->callback_error;
        input->callback_context         = options->callback_context;

        memset(&backend_options, 0, sizeof(struct clew_input_backend_init_options));
        backend_options.path                    = options->path;
	backend_options.callback_bounds_start   = clew_input_backend_callback_bounds_start;
	backend_options.callback_bounds_end     = clew_input_backend_callback_bounds_end;
	backend_options.callback_node_start     = clew_input_backend_callback_node_start;
	backend_options.callback_node_end       = clew_input_backend_callback_node_end;
	backend_options.callback_way_start      = clew_input_backend_callback_way_start;
	backend_options.callback_way_end        = clew_input_backend_callback_way_end;
	backend_options.callback_relation_start = clew_input_backend_callback_relation_start;
	backend_options.callback_relation_end   = clew_input_backend_callback_relation_end;
	backend_options.callback_tag_start      = clew_input_backend_callback_tag_start;
	backend_options.callback_tag_end        = clew_input_backend_callback_tag_end;
	backend_options.callback_nd_start       = clew_input_backend_callback_nd_start;
	backend_options.callback_nd_end         = clew_input_backend_callback_nd_end;
	backend_options.callback_member_start   = clew_input_backend_callback_member_start;
	backend_options.callback_member_end     = clew_input_backend_callback_member_end;
	backend_options.callback_minlon         = clew_input_backend_callback_minlon;
	backend_options.callback_minlat         = clew_input_backend_callback_minlat;
	backend_options.callback_maxlon         = clew_input_backend_callback_maxlon;
	backend_options.callback_maxlat         = clew_input_backend_callback_maxlat;
	backend_options.callback_id             = clew_input_backend_callback_id;
	backend_options.callback_lat            = clew_input_backend_callback_lat;
	backend_options.callback_lon            = clew_input_backend_callback_lon;
	backend_options.callback_ref            = clew_input_backend_callback_ref;
	backend_options.callback_type           = clew_input_backend_callback_type;
	backend_options.callback_role           = clew_input_backend_callback_role;
	backend_options.callback_k              = clew_input_backend_callback_k;
	backend_options.callback_v              = clew_input_backend_callback_v;
        backend_options.callback_error          = clew_input_backend_callback_error;
        backend_options.callback_context        = input;

        for (i = 0; i < (int) (sizeof(g_backends) / sizeof(g_backends[0])); i++) {
                if (g_backends[i].creator == NULL) {
                        continue;
                }
                input->backend = g_backends[i].creator(&backend_options);
                if (input->backend != NULL) {
                        break;
                }
        }
        if (input->backend == NULL) {
                clew_errorf("can not crteate backend for path");
                goto bail;
        }

        return input;
bail:   if (input != NULL) {
                clew_input_destroy(input);
        }
        return NULL;
}

void clew_input_destroy (struct clew_input *input)
{
        if (input == NULL) {
                return;
        }
        if (input->backend != NULL) {
                if (input->backend->destroy != NULL) {
                        input->backend->destroy(input->backend);
                }
        }
        free(input);
}

int clew_input_read (struct clew_input *input)
{
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }
        if (input->backend == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }
        if (input->backend->read == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }
        return input->backend->read(input->backend);
bail:   return -1;
}

int clew_input_reset (struct clew_input *input)
{
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }
        if (input->backend == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }
        if (input->backend->reset == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }
        input->error = 0;
        return input->backend->reset(input->backend);
bail:   return -1;
}

int clew_input_get_error (struct clew_input *input)
{
        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }
        return 0;
bail:   return -1;
}
