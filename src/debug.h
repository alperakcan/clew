
#if !defined(CLEW_DEBUG_H)
#define CLEW_DEBUG_H

#if !defined(CLEW_DEBUG_NAME)
#define CLEW_DEBUG_NAME        "unknown"
#endif

enum {
        CLEW_DEBUG_LEVEL_INVALID      = 0,
        CLEW_DEBUG_LEVEL_SILENT       = 1,
        CLEW_DEBUG_LEVEL_ERROR        = 2,
        CLEW_DEBUG_LEVEL_WARNING      = 3,
        CLEW_DEBUG_LEVEL_NOTICE       = 4,
        CLEW_DEBUG_LEVEL_INFO         = 5,
        CLEW_DEBUG_LEVEL_DEBUG        = 6,
        CLEW_DEBUG_LEVEL_TRACE        = 7
#define CLEW_DEBUG_LEVEL_INVALID      CLEW_DEBUG_LEVEL_INVALID
#define CLEW_DEBUG_LEVEL_SILENT       CLEW_DEBUG_LEVEL_SILENT
#define CLEW_DEBUG_LEVEL_ERROR        CLEW_DEBUG_LEVEL_ERROR
#define CLEW_DEBUG_LEVEL_WARNING      CLEW_DEBUG_LEVEL_WARNING
#define CLEW_DEBUG_LEVEL_NOTICE       CLEW_DEBUG_LEVEL_NOTICE
#define CLEW_DEBUG_LEVEL_INFO         CLEW_DEBUG_LEVEL_INFO
#define CLEW_DEBUG_LEVEL_DEBUG        CLEW_DEBUG_LEVEL_DEBUG
#define CLEW_DEBUG_LEVEL_TRACE        CLEW_DEBUG_LEVEL_TRACE
};

extern unsigned int clew_debug_level;

#define clew_enterf() { \
        clew_tracef("enter"); \
}

#define clew_leavef() { \
        clew_tracef("leave"); \
}

#define clew_tracef(a...) { \
        if (clew_debug_level >= CLEW_DEBUG_LEVEL_TRACE) { \
                clew_debug_printf(CLEW_DEBUG_LEVEL_TRACE, CLEW_DEBUG_NAME, __FUNCTION__, __FILE__, __LINE__, a); \
        } \
}

#define clew_debugf(a...) { \
        if (clew_debug_level >= CLEW_DEBUG_LEVEL_DEBUG) { \
                clew_debug_printf(CLEW_DEBUG_LEVEL_DEBUG, CLEW_DEBUG_NAME, __FUNCTION__, __FILE__, __LINE__, a); \
        } \
}

#define clew_warningf(a...) { \
        if (clew_debug_level >= CLEW_DEBUG_LEVEL_WARNING) { \
                clew_debug_printf(CLEW_DEBUG_LEVEL_WARNING, CLEW_DEBUG_NAME, __FUNCTION__, __FILE__, __LINE__, a); \
        } \
}

#define clew_noticef(a...) { \
        if (clew_debug_level >= CLEW_DEBUG_LEVEL_NOTICE) { \
                clew_debug_printf(CLEW_DEBUG_LEVEL_NOTICE, CLEW_DEBUG_NAME, __FUNCTION__, __FILE__, __LINE__, a); \
        } \
}

#define clew_infof(a...) { \
        if (clew_debug_level >= CLEW_DEBUG_LEVEL_INFO) { \
                clew_debug_printf(CLEW_DEBUG_LEVEL_INFO, CLEW_DEBUG_NAME, __FUNCTION__, __FILE__, __LINE__, a); \
        } \
}

#define clew_errorf(a...) { \
        if (clew_debug_level >= CLEW_DEBUG_LEVEL_ERROR) { \
                clew_debug_printf(CLEW_DEBUG_LEVEL_ERROR, CLEW_DEBUG_NAME, __FUNCTION__, __FILE__, __LINE__, a); \
        } \
}

int clew_debug_init (void);
void clew_debug_fini (void);

const char * clew_debug_level_to_string (unsigned int level);
unsigned int clew_debug_level_from_string (const char *string);
int clew_debug_set_level (unsigned int level);
int clew_debug_printf (unsigned int level, const char *name, const char *function, const char *file, int line, const char *fmt, ...) __attribute__((format(printf, 6, 7)));

void clew_debug_lock (void);
void clew_debug_unlock (void);

#endif
