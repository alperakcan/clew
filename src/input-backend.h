
struct clew_input_backend;

struct clew_input_backend_init_options {
        const char *path;

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

	int (*callback_minlon) (struct clew_input_backend *backend, void *context, uint32_t lon);
	int (*callback_minlat) (struct clew_input_backend *backend, void *context, uint32_t lat);
	int (*callback_maxlon) (struct clew_input_backend *backend, void *context, uint32_t lon);
	int (*callback_maxlat) (struct clew_input_backend *backend, void *context, uint32_t lat);

	int (*callback_id) (struct clew_input_backend *backend, void *context, uint64_t id);

	int (*callback_lat) (struct clew_input_backend *backend, void *context, uint32_t lat);
	int (*callback_lon) (struct clew_input_backend *backend, void *context, uint32_t lon);

	int (*callback_ref) (struct clew_input_backend *backend, void *context, uint64_t ref);

	int (*callback_type) (struct clew_input_backend *backend, void *context, const char *type);
	int (*callback_role) (struct clew_input_backend *backend, void *context, const char *role);

	int (*callback_k) (struct clew_input_backend *backend, void *context, const char *k);
	int (*callback_v) (struct clew_input_backend *backend, void *context, const char *v);

        int (*callback_error) (struct clew_input_backend *backend, void *context, unsigned int reason);

        void *callback_context;
};

struct clew_input_backend {
        int (*read) (struct clew_input_backend *backend);
        int (*reset) (struct clew_input_backend *backend);
        void (*destroy) (struct clew_input_backend *backend);
};
