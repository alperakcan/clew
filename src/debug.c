
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>

#define CLEW_DEBUG_NAME       "debug"
#include "debug.h"

unsigned int clew_debug_level           = CLEW_DEBUG_LEVEL_INFO;
static int debug_initialized            = 0;
static char *debug_buffer               = NULL;
static int debug_buffer_size            = 0;
static struct timeval debug_timeval     = { 0, 0 };
static pthread_mutex_t debug_mutex      = PTHREAD_MUTEX_INITIALIZER;

#define clew_timeval_sub(a, b, result)                                \
        do {                                                            \
                (result)->tv_sec = (a)->tv_sec - (b)->tv_sec;           \
                (result)->tv_usec = (a)->tv_usec - (b)->tv_usec;        \
                if ((result)->tv_usec < 0) {                            \
                        --(result)->tv_sec;                             \
                        (result)->tv_usec += 1000000;                   \
                }                                                       \
        } while (0)

int clew_debug_init (void)
{
        clew_debug_lock();
        if (debug_initialized) {
                clew_debug_unlock();
                return 0;
        }

        debug_buffer      = NULL;
        debug_buffer_size = 0;
        debug_initialized = 1;
        gettimeofday(&debug_timeval, NULL);
        clew_debug_unlock();
        return 0;
}

void clew_debug_fini (void)
{
        clew_debug_lock();
        if (debug_buffer != NULL) {
                free(debug_buffer);
        }
        debug_buffer      = NULL;
        debug_buffer_size = 0;
        debug_initialized = 0;
        clew_debug_unlock();
}

void clew_debug_lock (void)
{
        pthread_mutex_lock(&debug_mutex);
}

void clew_debug_unlock (void)
{
        pthread_mutex_unlock(&debug_mutex);
}

const char * clew_debug_level_to_string (unsigned int level)
{
        switch (level) {
                case CLEW_DEBUG_LEVEL_SILENT:  return "silent";
                case CLEW_DEBUG_LEVEL_ERROR:   return "error";
                case CLEW_DEBUG_LEVEL_WARNING: return "warning";
                case CLEW_DEBUG_LEVEL_NOTICE:  return "notice";
                case CLEW_DEBUG_LEVEL_INFO:    return "info";
                case CLEW_DEBUG_LEVEL_DEBUG:   return "debug";
                case CLEW_DEBUG_LEVEL_TRACE:   return "trace";
        }
        return "invalid";
}

unsigned int clew_debug_level_from_string (const char *string)
{
        if (string == NULL) {
                return CLEW_DEBUG_LEVEL_INVALID;
        }
        if (strcmp(string, "silent") == 0 || strcmp(string, "s") == 0) {
                return CLEW_DEBUG_LEVEL_SILENT;
        }
        if (strcmp(string, "error") == 0 || strcmp(string, "e") == 0) {
                return CLEW_DEBUG_LEVEL_ERROR;
        }
        if (strcmp(string, "warning") == 0 || strcmp(string, "w") == 0) {
                return CLEW_DEBUG_LEVEL_WARNING;
        }
        if (strcmp(string, "notice") == 0 || strcmp(string, "n") == 0) {
                return CLEW_DEBUG_LEVEL_NOTICE;
        }
        if (strcmp(string, "info") == 0 || strcmp(string, "i") == 0) {
                return CLEW_DEBUG_LEVEL_INFO;
        }
        if (strcmp(string, "info+") == 0 || strcmp(string, "i+") == 0) {
                return CLEW_DEBUG_LEVEL_INFO;
        }
        if (strcmp(string, "infoplus") == 0 || strcmp(string, "ip") == 0) {
                return CLEW_DEBUG_LEVEL_INFO;
        }
        if (strcmp(string, "debug") == 0 || strcmp(string, "d") == 0) {
                return CLEW_DEBUG_LEVEL_DEBUG;
        }
        if (strcmp(string, "trace") == 0 || strcmp(string, "t") == 0) {
                return CLEW_DEBUG_LEVEL_TRACE;
        }
        return CLEW_DEBUG_LEVEL_INVALID;
}

int clew_debug_set_level (unsigned int level)
{
        clew_debug_level = level;
        return clew_debug_level;
}

int clew_debug_printf (unsigned int level, const char *name, const char *function, const char *file, int line, const char *fmt, ...)
{
        int rc;
        va_list ap;

        time_t timeval_tv_sec;
        struct timeval timeval;
        struct tm *tm;
        int milliseconds;
        char date[80];

        clew_debug_init();

        clew_debug_lock();

        va_start(ap, fmt);
        rc = vsnprintf(debug_buffer, debug_buffer_size, fmt, ap);
        va_end(ap);
        if (rc < 0) {
                clew_debug_unlock();
                goto bail;
        }
        if (debug_buffer_size == 0 ||
            rc >= debug_buffer_size) {
                free(debug_buffer);
                debug_buffer = (char *) malloc(rc + 1);
                if (debug_buffer == NULL) {
                        goto bail;
                }
                debug_buffer_size = rc + 1;
                va_start(ap, fmt);
                rc = vsnprintf(debug_buffer, debug_buffer_size, fmt, ap);
                va_end(ap);
                if (rc < 0) {
                        clew_debug_unlock();
                        goto bail;
                }
        }

        gettimeofday(&timeval, NULL);
        clew_timeval_sub(&timeval, &debug_timeval, &timeval);

        milliseconds = (int) ((timeval.tv_usec / 1000.0) + 0.5);
        if (milliseconds >= 1000) {
                milliseconds -= 1000;
                timeval.tv_sec++;
        }
        timeval_tv_sec = timeval.tv_sec;
        tm = gmtime(&timeval_tv_sec);
        strftime(date, sizeof(date), "%H:%M:%S", tm);

        if (level == CLEW_DEBUG_LEVEL_INFO_PLUS) {
                fprintf(stderr, "clew:%s.%03d:%-5s:%-5s: %s (%s %s:%d)\n", date, milliseconds, name, clew_debug_level_to_string(level), debug_buffer, function, file, line);
        } else {
                fprintf(stderr, "clew:%s.%03d:%-5s:%-5s: %s\n", date, milliseconds, name, clew_debug_level_to_string(level), debug_buffer);
        }
        fflush(stderr);

        clew_debug_unlock();

        return 0;
bail:   va_end(ap);
        return -1;
}
