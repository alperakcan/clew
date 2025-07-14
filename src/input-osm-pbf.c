
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <zlib.h>

#include "debug.h"
#include "input.h"
#include "input-backend.h"
#include "input-osm-pbf.h"

#include "input-osm-pbf-fileformat.pb-c.h"
#include "input-osm-pbf-osmformat.pb-c.h"

enum {
        STATE_READ_HEADER,
        STATE_READ_BLOB,
        STATE_READ_OSM,
        STATE_FINISHED,
        STATE_ERROR
};

struct clew_input_osm_pbf {
        struct clew_input_backend backend;

        char *path;

        int (*callback_bounds_start) (struct clew_input_backend *backend, void *context);
	int (*callback_bounds_end) (struct clew_input_backend *backend, void *context);

	int (*callback_node_start) (struct clew_input_backend *backend, void *context);
	int (*callback_node_end) (struct clew_input_backend *backend, void *context);

	int (*callback_way_start) (struct clew_input_backend *backend, void *context);
	int (*callback_way_end) (struct clew_input_backend *backend, void *context);

	int (*callback_relation_start) (struct clew_input_backend *backend, void *context);
	int (*callback_relation_end) (struct clew_input_backend *backend, void *context);

	int (*callback_tag_start) (struct clew_input_backend *backend, void *context);
	int (*callback_tag_end) (struct clew_input_backend *backend, void *context);

	int (*callback_nd_start) (struct clew_input_backend *backend, void *context);
	int (*callback_nd_end) (struct clew_input_backend *backend, void *context);

	int (*callback_member_start) (struct clew_input_backend *backend, void *context);
	int (*callback_member_end) (struct clew_input_backend *backend, void *context);

	int (*callback_minlon) (struct clew_input_backend *backend, void *context, int32_t lon);
	int (*callback_minlat) (struct clew_input_backend *backend, void *context, int32_t lat);
	int (*callback_maxlon) (struct clew_input_backend *backend, void *context, int32_t lon);
	int (*callback_maxlat) (struct clew_input_backend *backend, void *context, int32_t lat);

	int (*callback_id) (struct clew_input_backend *backend, void *context, uint64_t id);

	int (*callback_lat) (struct clew_input_backend *backend, void *context, int32_t lat);
	int (*callback_lon) (struct clew_input_backend *backend, void *context, int32_t lon);

	int (*callback_ref) (struct clew_input_backend *backend, void *context, uint64_t ref);

	int (*callback_type) (struct clew_input_backend *backend, void *context, const char *type);
	int (*callback_role) (struct clew_input_backend *backend, void *context, const char *role);

	int (*callback_k) (struct clew_input_backend *backend, void *context, const char *k);
	int (*callback_v) (struct clew_input_backend *backend, void *context, const char *v);

        int (*callback_error) (struct clew_input_backend *backend, void *context, unsigned int reason);

        void *callback_context;

        FILE *fp;
        unsigned char *buffer;
        int state;

        OSMPBF__BlobHeader *header;
        OSMPBF__Blob *blob;
        unsigned char *data;

        char keybuff[1024];
        char valbuff[1024];
        char rolebuff[1024];
};

static int string_ends_with (const char *string, const char *end)
{
        int lstring;
        int lend;

        if (string == NULL) {
                goto bail;
        }
        if (end == NULL) {
                goto bail;
        }

        lstring = (int) strlen(string);
        lend    = (int) strlen(end);

        if (lend > lstring) {
                return 0;
        }
        if (strncasecmp(string + lstring - lend, end, lend) == 0) {
                return 1;
        }
        return 0;
bail:   return -1;
}

#define MAX_HEADER_LENGTH	(1024 * 64)
#define MAX_BLOB_LENGTH		(1024 * 1024 * 32)

#define SANITY_CHECK_LENGTH(length, max_length) \
	if (length > max_length) { \
		clew_errorf("invalid block size: %d, max: %d", length, max_length); \
		return NULL; \
	}

static OSMPBF__BlobHeader * read_header (FILE *f)
{
	int len;
	unsigned char lenb[4];
	unsigned char *buffer;
	static OSMPBF__BlobHeader *blob_header;
	if (fread(lenb, 4, 1, f) != 1) {
		return NULL;
	}
	len = (lenb[0] << 24) | (lenb[1] << 16) | (lenb[2] << 8) | lenb[3];
	SANITY_CHECK_LENGTH(len, MAX_HEADER_LENGTH)
	buffer = (unsigned char *) malloc(len);
	if (buffer == NULL) {
		clew_errorf("can not allocate memory");
		return NULL;
	}
	if (fread(buffer, len, 1, f) != 1) {
		free(buffer);
		return NULL;
	}
	blob_header = osmpbf__blob_header__unpack(NULL, len, buffer);
	free(buffer);
	return blob_header;
}

static OSMPBF__Blob * read_blob (OSMPBF__BlobHeader *blob_header, FILE *f, unsigned char *buffer)
{
	int len;
	len = blob_header->datasize;
	SANITY_CHECK_LENGTH(len, MAX_BLOB_LENGTH)
	if (fread(buffer, len, 1, f) != 1) {
		return NULL;
	}
	return osmpbf__blob__unpack(NULL, len, buffer);
}

static unsigned char * uncompress_blob (OSMPBF__Blob *blob)
{
	int zerr;
	z_stream strm;
	unsigned char *ret;
	ret = (unsigned char *) malloc(blob->raw_size);
	if (!ret) {
		clew_errorf("can not allocate memory");
		return NULL;
	}
	strm.zalloc = Z_NULL;
	strm.zfree = Z_NULL;
	strm.opaque = Z_NULL;
	strm.avail_in = blob->zlib_data.len;
	strm.next_in = blob->zlib_data.data;
	strm.avail_out = blob->raw_size;
	strm.next_out = ret;
	zerr = inflateInit(&strm);
	if (zerr != Z_OK) {
		free(ret);
		return NULL;
	}
	zerr = inflate(&strm, Z_NO_FLUSH);
	if (zerr != Z_STREAM_END) {
		free(ret);
		return NULL;
	}
	inflateEnd(&strm);
	return ret;
}

static int get_string (char *buffer, size_t buffer_size, OSMPBF__PrimitiveBlock *primitive_block, uint32_t id)
{
	int len;
	char *data;
	len = primitive_block->stringtable->s[id].len;
	data = (char *) primitive_block->stringtable->s[id].data;
	if (primitive_block->stringtable->s[id].len >= buffer_size) {
		buffer[0] = '\0';
		return 0;
	}
	strncpy(buffer, data, len);
	buffer[len] = '\0';
	return 1;
}

static int clew_input_osm_pbf_read (struct clew_input_backend *backend)
{
        struct clew_input_osm_pbf *input = (struct clew_input_osm_pbf *) backend;

        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->state == STATE_READ_HEADER) {
                if (input->header != NULL) {
                        osmpbf__blob_header__free_unpacked(input->header, NULL);
                        input->header = NULL;
                }
                if (input->blob != NULL) {
                        osmpbf__blob__free_unpacked(input->blob, NULL);
                        input->blob = NULL;
                }
                if (input->data != NULL) {
                        free(input->data);
                        input->data  = NULL;
                }

                input->header = read_header(input->fp);
                if (input->header == NULL) {
                        input->state = STATE_FINISHED;
                        goto finish;
                } else {
                        input->state = STATE_READ_BLOB;
                }
        } else if (input->state == STATE_READ_BLOB) {
                if (input->blob != NULL) {
                        osmpbf__blob__free_unpacked(input->blob, NULL);
                        input->blob = NULL;
                }
                if (input->data != NULL) {
                        free(input->data);
                        input->data  = NULL;
                }

                input->blob = read_blob(input->header, input->fp, input->buffer);
                if (input->blob == NULL) {
                        clew_errorf("can not read blob");
                        goto bail;
                }
                input->data = uncompress_blob(input->blob);
                if (input->data == NULL) {
                        clew_errorf("can not read blob");
                        goto bail;
                }
                input->state = STATE_READ_OSM;
        } else if (input->state == STATE_READ_OSM) {
		if (strcmp(input->header->type, "OSMHeader") == 0) {
                        int rc;
                        OSMPBF__HeaderBlock *header_block;
                        header_block = osmpbf__header_block__unpack(NULL, input->blob->raw_size, input->data);
                        if (input->callback_bounds_start != NULL) {
                                rc = input->callback_bounds_start(&input->backend, input->callback_context);
                                if (rc < 0) {
                                        clew_errorf("input callback_bounds_start failed");
                                        osmpbf__header_block__free_unpacked(header_block, NULL);
                                        goto bail;
                                }
                        }
                        if (header_block->bbox != NULL) {
                                if (input->callback_minlon != NULL) {
                                        rc = input->callback_minlon(&input->backend, input->callback_context, header_block->bbox->left / 100);
                                        if (rc < 0) {
                                                clew_errorf("input callback_minlon failed");
                                                osmpbf__header_block__free_unpacked(header_block, NULL);
                                                goto bail;
                                        }
                                }
                                if (input->callback_minlat != NULL) {
                                        rc = input->callback_minlat(&input->backend, input->callback_context, header_block->bbox->top / 100);
                                        if (rc < 0) {
                                                clew_errorf("input callback_minlat failed");
                                                osmpbf__header_block__free_unpacked(header_block, NULL);
                                                goto bail;
                                        }
                                }
                                if (input->callback_maxlon != NULL) {
                                        rc = input->callback_maxlon(&input->backend, input->callback_context, header_block->bbox->right / 100);
                                        if (rc < 0) {
                                                clew_errorf("input callback_maxlon failed");
                                                osmpbf__header_block__free_unpacked(header_block, NULL);
                                                goto bail;
                                        }
                                }
                                if (input->callback_maxlat != NULL) {
                                        rc = input->callback_maxlat(&input->backend, input->callback_context, header_block->bbox->bottom / 100);
                                        if (rc < 0) {
                                                clew_errorf("input callback_maxlat failed");
                                                osmpbf__header_block__free_unpacked(header_block, NULL);
                                                goto bail;
                                        }
                                }
                        }
                        if (input->callback_bounds_end != NULL) {
                                rc = input->callback_bounds_end(&input->backend, input->callback_context);
                                if (rc < 0) {
                                        clew_errorf("input callback_bounds_end failed");
                                        osmpbf__header_block__free_unpacked(header_block, NULL);
                                        goto bail;
                                }
                        }
                        osmpbf__header_block__free_unpacked(header_block, NULL);
                        input->state = STATE_READ_HEADER;
		} else if (strcmp(input->header->type, "OSMData") == 0) {
                        int rc;
                        uint64_t i;
                        uint64_t j;
                        uint64_t k;
                        uint64_t id;
                        int32_t lat;
                        int32_t lon;
                        uint64_t ref;
                        OSMPBF__PrimitiveBlock *primitive_block;
                        primitive_block = osmpbf__primitive_block__unpack(NULL, input->blob->raw_size, input->data);
                        for (i = 0; i < primitive_block->n_primitivegroup; i++) {
                                OSMPBF__PrimitiveGroup *primitive_group = primitive_block->primitivegroup[i];
                                for (j = 0, id = 0, lat = 0, lon = 0, k = 0; primitive_group->dense != NULL && j < primitive_group->dense->n_id ; j++) {
                                        id += primitive_group->dense->id[j];
                                        lat += primitive_group->dense->lat[j];
                                        lon += primitive_group->dense->lon[j];
                                        if (input->callback_node_start) {
                                                rc = input->callback_node_start(&input->backend, input->callback_context);
                                                if (rc < 0) {
                                                        clew_errorf("input callback_node_start failed");
                                                        osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                        goto bail;
                                                } else if (rc == 1) {
                                                        goto skip_node;
                                                }
                                        }
                                        if (input->callback_id) {
                                                rc = input->callback_id(&input->backend, input->callback_context, id);
                                                if (rc < 0) {
                                                        clew_errorf("input callback_id failed");
                                                        osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                        goto bail;
                                                } else if (rc == 1) {
                                                        goto skip_node;
                                                }
                                        }
                                        if (input->callback_lon) {
                                                rc = input->callback_lon(&input->backend, input->callback_context, lon);
                                                if (rc < 0) {
                                                        clew_errorf("input callback_lon failed");
                                                        osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                        goto bail;
                                                }
                                        }
                                        if (input->callback_lat) {
                                                rc = input->callback_lat(&input->backend, input->callback_context, lat);
                                                if (rc < 0) {
                                                        clew_errorf("input callback_lat failed");
                                                        osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                        goto bail;
                                                }
                                        }
                                        if (primitive_group->dense->keys_vals) {
                                                while (primitive_group->dense->keys_vals[k]) {
                                                        get_string(input->keybuff, sizeof(input->keybuff), primitive_block, primitive_group->dense->keys_vals[k + 0]);
                                                        get_string(input->valbuff, sizeof(input->valbuff), primitive_block, primitive_group->dense->keys_vals[k + 1]);
                                                        if (input->callback_tag_start) {
                                                                rc = input->callback_tag_start(&input->backend, input->callback_context);
                                                                if (rc < 0) {
                                                                        clew_errorf("input callback_tag_start failed");
                                                                        osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                        goto bail;
                                                                }
                                                        }
                                                        if (input->callback_k) {
                                                                rc = input->callback_k(&input->backend, input->callback_context, input->keybuff);
                                                                if (rc < 0) {
                                                                        clew_errorf("input callback_k failed");
                                                                        osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                        goto bail;
                                                                }
                                                        }
                                                        if (input->callback_v) {
                                                                rc = input->callback_v(&input->backend, input->callback_context, input->valbuff);
                                                                if (rc < 0) {
                                                                        clew_errorf("input callback_v failed");
                                                                        osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                        goto bail;
                                                                }
                                                        }
                                                        if (input->callback_tag_end) {
                                                                rc = input->callback_tag_end(&input->backend, input->callback_context);
                                                                if (rc < 0) {
                                                                        clew_errorf("input callback_tag_end failed");
                                                                        osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                        goto bail;
                                                                }
                                                        }
                                                        k += 2;
                                                }
                                                k++;
                                        }
skip_node:
                                        if (input->callback_node_end) {
                                                rc = input->callback_node_end(&input->backend, input->callback_context);
                                                if (rc < 0) {
                                                        clew_errorf("input callback_node_end failed");
                                                        osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                        goto bail;
                                                }
                                        }
                                }
                                for (j = 0, k = 0; j < primitive_group->n_ways; j++) {
                                        if (input->callback_way_start) {
                                                rc = input->callback_way_start(&input->backend, input->callback_context);
                                                if (rc < 0) {
                                                        clew_errorf("input callback_way_start failed");
                                                        osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                        goto bail;
                                                } else if (rc == 1) {
                                                        goto skip_way;
                                                }
                                        }
                                        if (input->callback_id) {
                                                rc = input->callback_id(&input->backend, input->callback_context, primitive_group->ways[j]->id);
                                                if (rc < 0) {
                                                        clew_errorf("input callback_id failed");
                                                        osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                        goto bail;
                                                } else if (rc == 1) {
                                                        goto skip_way;
                                                }
                                        }
                                        for (k = 0, ref = 0; k < primitive_group->ways[j]->n_refs; k++) {
                                                ref += primitive_group->ways[j]->refs[k];
                                                if (input->callback_nd_start) {
                                                        rc = input->callback_nd_start(&input->backend, input->callback_context);
                                                        if (rc < 0) {
                                                                clew_errorf("input callback_nd_start failed");
                                                                osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                goto bail;
                                                        }
                                                }
                                                if (input->callback_ref) {
                                                        rc = input->callback_ref(&input->backend, input->callback_context, ref);
                                                        if (rc < 0) {
                                                                clew_errorf("input callback_ref failed");
                                                                osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                goto bail;
                                                        }
                                                }
                                                if (input->callback_nd_end) {
                                                        rc = input->callback_nd_end(&input->backend, input->callback_context);
                                                        if (rc < 0) {
                                                                clew_errorf("input callback_nd_end failed");
                                                                osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                goto bail;
                                                        }
                                                }
                                        }
                                        for (k = 0; k < primitive_group->ways[j]->n_keys; k++) {
                                                get_string(input->keybuff, sizeof(input->keybuff), primitive_block, primitive_group->ways[j]->keys[k]);
                                                get_string(input->valbuff, sizeof(input->valbuff), primitive_block, primitive_group->ways[j]->vals[k]);
                                                if (input->callback_tag_start) {
                                                        rc = input->callback_tag_start(&input->backend, input->callback_context);
                                                        if (rc < 0) {
                                                                clew_errorf("input callback_tag_start failed");
                                                                osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                goto bail;
                                                        }
                                                }
                                                if (input->callback_k) {
                                                        rc = input->callback_k(&input->backend, input->callback_context, input->keybuff);
                                                        if (rc < 0) {
                                                                clew_errorf("input callback_k failed");
                                                                osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                goto bail;
                                                        }
                                                }
                                                if (input->callback_v) {
                                                        rc = input->callback_v(&input->backend, input->callback_context, input->valbuff);
                                                        if (rc < 0) {
                                                                clew_errorf("input callback_v failed");
                                                                osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                goto bail;
                                                        }
                                                }
                                                if (input->callback_tag_end) {
                                                        rc = input->callback_tag_end(&input->backend, input->callback_context);
                                                        if (rc < 0) {
                                                                clew_errorf("input callback_tag_end failed");
                                                                osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                goto bail;
                                                        }
                                                }
                                        }
skip_way:
                                        if (input->callback_way_end) {
                                                rc = input->callback_way_end(&input->backend, input->callback_context);
                                                if (rc < 0) {
                                                        clew_errorf("input callback_way_end failed");
                                                        osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                        goto bail;
                                                }
                                        }
                                }
                                for (j = 0; j < primitive_group->n_relations; j++) {
                                        if (input->callback_relation_start) {
                                                rc = input->callback_relation_start(&input->backend, input->callback_context);
                                                if (rc < 0) {
                                                        clew_errorf("input callback_relation_start failed");
                                                        osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                        goto bail;
                                                } else if (rc == 1) {
                                                        goto skip_relation;
                                                }
                                        }
                                        if (input->callback_id) {
                                                rc = input->callback_id(&input->backend, input->callback_context, primitive_group->relations[j]->id);
                                                if (rc < 0) {
                                                        clew_errorf("input callback_id failed");
                                                        osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                        goto bail;
                                                } else if (rc == 1) {
                                                        goto skip_relation;
                                                }
                                        }
                                        for (k = 0, ref = 0; k < primitive_group->relations[j]->n_roles_sid; k++) {
                                                ref += primitive_group->relations[j]->memids[k];
                                                get_string(input->rolebuff, sizeof(input->rolebuff), primitive_block, primitive_group->relations[j]->roles_sid[k]);
                                                if (input->callback_member_start) {
                                                        rc = input->callback_member_start(&input->backend, input->callback_context);
                                                        if (rc < 0) {
                                                                clew_errorf("input callback_member_start failed");
                                                                osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                goto bail;
                                                        }
                                                }
                                                if (input->callback_type != NULL) {
                                                        rc = input->callback_type(&input->backend, input->callback_context, (primitive_group->relations[j]->types[k] == 0) ? "node" : (primitive_group->relations[j]->types[k] == 1) ? "way" : (primitive_group->relations[j]->types[k] == 2) ? "relation" : "unknown");
                                                        if (rc < 0) {
                                                                clew_errorf("input callback_type failed");
                                                                osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                goto bail;
                                                        }
                                                }
                                                if (input->callback_ref != NULL) {
                                                        rc = input->callback_ref(&input->backend, input->callback_context, ref);
                                                        if (rc < 0) {
                                                                clew_errorf("input callback_ref failed");
                                                                osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                goto bail;
                                                        }
                                                }
                                                if (input->callback_role != NULL) {
                                                        rc = input->callback_role(&input->backend, input->callback_context, input->rolebuff);
                                                        if (rc < 0) {
                                                                clew_errorf("input callback_role failed");
                                                                osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                goto bail;
                                                        }
                                                }
                                                if (input->callback_member_end) {
                                                        rc = input->callback_member_end(&input->backend, input->callback_context);
                                                        if (rc < 0) {
                                                                clew_errorf("input callback_member_end failed");
                                                                osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                goto bail;
                                                        }
                                                }
                                        }
                                        for (k = 0; k < primitive_group->relations[j]->n_keys; k++) {
                                                get_string(input->keybuff, sizeof(input->keybuff), primitive_block, primitive_group->relations[j]->keys[k]);
                                                get_string(input->valbuff, sizeof(input->valbuff), primitive_block, primitive_group->relations[j]->vals[k]);
                                                if (input->callback_tag_start) {
                                                        rc = input->callback_tag_start(&input->backend, input->callback_context);
                                                        if (rc < 0) {
                                                                clew_errorf("input callback_tag_start failed");
                                                                osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                goto bail;
                                                        }
                                                }
                                                if (input->callback_k) {
                                                        rc = input->callback_k(&input->backend, input->callback_context, input->keybuff);
                                                        if (rc < 0) {
                                                                clew_errorf("input callback_k failed");
                                                                osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                goto bail;
                                                        }
                                                }
                                                if (input->callback_v) {
                                                        rc = input->callback_v(&input->backend, input->callback_context, input->valbuff);
                                                        if (rc < 0) {
                                                                clew_errorf("input callback_v failed");
                                                                osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                goto bail;
                                                        }
                                                }
                                                if (input->callback_tag_end) {
                                                        rc = input->callback_tag_end(&input->backend, input->callback_context);
                                                        if (rc < 0) {
                                                                clew_errorf("input callback_tag_end failed");
                                                                osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                                goto bail;
                                                        }
                                                }
                                        }
skip_relation:
                                        if (input->callback_relation_end) {
                                                rc = input->callback_relation_end(&input->backend, input->callback_context);
                                                if (rc < 0) {
                                                        clew_errorf("input callback_relation_end failed");
                                                        osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                                                        goto bail;
                                                }
                                        }
                                }
                        }
                        osmpbf__primitive_block__free_unpacked(primitive_block, NULL);
                        input->state = STATE_READ_HEADER;
		} else {
			clew_errorf("unknown header type type '%s'", input->header->type);
			goto bail;
		}
        } else if (input->state == STATE_FINISHED) {
                goto bail;
        } else if (input->state == STATE_ERROR) {
                goto bail;
        } else {
                clew_errorf("state is invalid");
                goto bail;
        }

        return 0;
finish: input->state = STATE_FINISHED;
        return 1;
bail:   input->state = STATE_ERROR;
        if (input->callback_error) {
                input->callback_error(&input->backend, input->callback_context, CLEW_INPUT_ERROR_UNKNOWN);
        }
        return -1;
}

static int clew_input_osm_pbf_reset (struct clew_input_backend *backend)
{
        struct clew_input_osm_pbf *input = (struct clew_input_osm_pbf *) backend;

        if (input == NULL) {
                clew_errorf("input is invalid");
                goto bail;
        }

        if (input->header != NULL) {
                osmpbf__blob_header__free_unpacked(input->header, NULL);
                input->header = NULL;
        }
        if (input->blob != NULL) {
                osmpbf__blob__free_unpacked(input->blob, NULL);
                input->blob = NULL;
        }
        if (input->data != NULL) {
                free(input->data);
                input->data  = NULL;
        }
        if (input->fp != NULL) {
                fseek(input->fp, 0, SEEK_SET);
        }
        input->state = STATE_READ_HEADER;

        return 0;
bail:   return -1;
}

static void clew_input_osm_pbf_destroy (struct clew_input_backend *backend)
{
        struct clew_input_osm_pbf *input = (struct clew_input_osm_pbf *) backend;

        if (input == NULL) {
                return;
        }

        clew_input_osm_pbf_reset(&input->backend);

        if (input->buffer != NULL) {
                free(input->buffer);
        }
        if (input->fp != NULL) {
                fclose(input->fp);
        }
        if (input->path != NULL) {
                free(input->path);
        }
        free(input);
}

struct clew_input_backend * clew_input_osm_pbf_create (struct clew_input_backend_init_options *options)
{
        struct clew_input_osm_pbf *input;

        input = NULL;

        if (options == NULL) {
                clew_errorf("options is invalid");
                goto bail;
        }
        if (options->path == NULL) {
                clew_errorf("options path is invalid");
                goto bail;
        }

        input = malloc(sizeof(struct clew_input_osm_pbf));
        if (input == NULL) {
                clew_errorf("can not allocate memory");
                goto bail;
        }
        memset(input, 0, sizeof(struct clew_input_osm_pbf));

        input->path = strdup(options->path);
        if (input->path == NULL) {
                clew_errorf("can not allocate memory");
                goto bail;
        }

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

        input->backend.read     = clew_input_osm_pbf_read;
        input->backend.reset    = clew_input_osm_pbf_reset;
        input->backend.destroy  = clew_input_osm_pbf_destroy;

        if (string_ends_with(input->path, "osm.pbf") != 1) {
                clew_errorf("path suffix is invalid");
                goto bail;
        }

        input->fp = fopen(input->path, "rb");
        if (input->fp == NULL) {
                clew_errorf("can not open path for reading");
                goto bail;
        }
	input->buffer = (unsigned char *) malloc(MAX_BLOB_LENGTH);
	if (input->buffer == NULL) {
		clew_errorf("can not allocate memory");
		goto bail;
	}

        input->state = STATE_READ_HEADER;

        return &input->backend;
bail:   if (input != NULL) {
                clew_input_osm_pbf_destroy(&input->backend);
        }
        return NULL;
}
