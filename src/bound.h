
#include <stdint.h>
#include "point.h"

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(MIN)
#define MIN(a, b)                       (((a) < (b)) ? (a) : (b))
#endif
#if !defined(MAX)
#define MAX(a, b)                       (((a) > (b)) ? (a) : (b))
#endif

#if !defined(CLEW_BOUND_FORMAT)
#define CLEW_BOUND_FORMAT               "%.7f, %.7f, %.7f, %.7f"
#endif

#if !defined(CLEW_BOUND_FORMAT_ARG)
#define CLEW_BOUND_FORMAT_ARG(b)        (b).minlon, (b).minlat, (b).maxlon, (b).maxlat
#endif

struct clew_bound {
        int32_t minlon;
        int32_t minlat;
        int32_t maxlon;
        int32_t maxlat;
};

static inline __attribute__ ((warn_unused_result)) struct clew_bound clew_bound_null (void)
{
        struct clew_bound bound;
        bound.minlon = INT32_MAX;
        bound.minlat = INT32_MAX;
        bound.maxlon = INT32_MIN;
        bound.maxlat = INT32_MIN;
        return bound;
}

static inline __attribute__ ((warn_unused_result)) struct clew_bound clew_bound_full (void)
{
        struct clew_bound bound;
        bound.minlon = INT32_MIN;
        bound.minlat = INT32_MIN;
        bound.maxlon = INT32_MAX;
        bound.maxlat = INT32_MAX;
        return bound;
}

static inline __attribute__ ((warn_unused_result)) int clew_bound_valid (const struct clew_bound *bound)
{
        return (bound->maxlon >= bound->minlon && bound->maxlat >= bound->minlat);
}

static inline __attribute__ ((warn_unused_result)) int clew_bound_invalid (const struct clew_bound *bound)
{
        return (bound->maxlon < bound->minlon || bound->maxlat < bound->minlat);
}

static inline __attribute__ ((warn_unused_result)) struct clew_point clew_bound_center (const struct clew_bound *bound)
{
        if (clew_bound_invalid(bound)) {
                return clew_point_null();
        }
        return clew_point_init((bound->maxlon + bound->minlon) / 2.0, (bound->maxlat + bound->minlat) / 2.0);
}

static inline __attribute__ ((warn_unused_result)) struct clew_bound clew_bound_init (const int32_t minlon, const int32_t minlat, const int32_t maxlon, const int32_t maxlat)
{
        struct clew_bound bound;
        bound.minlon = minlon;
        bound.minlat = minlat;
        bound.maxlon = maxlon;
        bound.maxlat = maxlat;
        return bound;
}

static inline __attribute__ ((warn_unused_result)) struct clew_bound clew_bound_intersect (const struct clew_bound *a, const struct clew_bound *b)
{
        int32_t x31;
        int32_t x32;
        int32_t y31;
        int32_t y32;
        x31 = MAX(a->minlon, b->minlon);
        x32 = MIN(a->maxlon, b->maxlon);
        y31 = MAX(a->minlat, b->minlat);
        y32 = MIN(a->maxlat, b->maxlat);
        if ((x31 > x32) || (y31 > y32)) {
                return clew_bound_null();
        }
        return clew_bound_init(x31, y31, x32, y32);
}

static inline __attribute__ ((warn_unused_result)) int clew_bound_difference (const struct clew_bound *a, const struct clew_bound *b, struct clew_bound r[4])
{
        struct clew_bound intersect;
        struct clew_bound *rectangle;

        intersect = clew_bound_intersect(a, b);
        if (clew_bound_invalid(&intersect)) {
                /*
                 * aaaa
                 * aaaa
                 * aaaa  bbbb
                 * aaaa  bbbb
                 *       bbbb
                 */
                r[0] = *a;
                return 1;
        }

        /*
         * 0011122
         * 0011122
         * 00iii22
         * 00iii22
         * 0033322
         * 0033322
         */
        rectangle = &r[0];

        rectangle->minlon = a->minlon;
        rectangle->minlat = a->minlat;
        rectangle->maxlon = intersect.minlon;
        rectangle->maxlat = a->maxlat;
        if ((rectangle->maxlon > rectangle->minlon) &&
            (rectangle->maxlat > rectangle->minlat)) {
                rectangle += 1;
        } else {
        }

        rectangle->minlon = intersect.minlon;
        rectangle->minlat = a->minlat;
        rectangle->maxlon = intersect.maxlon;
        rectangle->maxlat = intersect.minlat;
        if ((rectangle->maxlon > rectangle->minlon) &&
            (rectangle->maxlat > rectangle->minlat)) {
                rectangle += 1;
        }

        rectangle->minlon = intersect.maxlon;
        rectangle->minlat = a->minlat;
        rectangle->maxlon = a->maxlon;
        rectangle->maxlat = a->maxlat;
        if ((rectangle->maxlon > rectangle->minlon) &&
            (rectangle->maxlat > rectangle->minlat)) {
                rectangle += 1;
        }

        rectangle->minlon = intersect.minlon;
        rectangle->minlat = intersect.maxlat;
        rectangle->maxlon = intersect.maxlon;
        rectangle->maxlat = a->maxlat;
        if ((rectangle->maxlon > rectangle->minlon) &&
            (rectangle->maxlat > rectangle->minlat)) {
                rectangle += 1;
        }

        return (int) (rectangle - r);
}

static inline __attribute__ ((warn_unused_result)) int clew_bound_intersects (const struct clew_bound *a, const struct clew_bound *b)
{
        if (MAX(a->minlon, b->minlon) > MIN(a->maxlon, b->maxlon)) {
                return 0;
        }
        if (MAX(a->minlat, b->minlat) > MIN(a->maxlat, b->maxlat)) {
                return 0;
        }
        return 1;
}

static inline __attribute__ ((warn_unused_result)) int clew_bound_contains (const struct clew_bound *a, const struct clew_bound *b)
{
        if (clew_bound_intersects(a, b) == 0) {
                return 0;
        }
        if ((b->minlon >= a->minlon) && (b->maxlon <= a->maxlon) &&
            (b->minlat >= a->minlat) && (b->maxlat <= a->maxlat)) {
                /* full */
                return 1;
        }
        /* partial */
        return 2;
}

static inline __attribute__ ((warn_unused_result)) int clew_bound_contains_xy (const struct clew_bound *a, int32_t x, int32_t y)
{
        if ((a->minlon <= x) && (x <= a->maxlon) &&
            (a->minlat <= y) && (y <= a->maxlat)) {
                return 1;
        }
        return 0;
}

static inline __attribute__ ((warn_unused_result)) int clew_bound_contains_point (const struct clew_bound *a, const struct clew_point *p)
{
        if ((a->minlon <= p->lon) && (p->lon <= a->maxlon) &&
            (a->minlat <= p->lat) && (p->lat <= a->maxlat)) {
                return 1;
        }
        return 0;
}

static inline __attribute__ ((warn_unused_result)) struct clew_bound clew_bound_union (const struct clew_bound *a, const struct clew_bound *b)
{
        struct clew_bound u;
        u.minlon = MIN(a->minlon, b->minlon);
        u.minlat = MIN(a->minlat, b->minlat);
        u.maxlon = MAX(a->maxlon, b->maxlon);
        u.maxlat = MAX(a->maxlat, b->maxlat);
        return u;
}

static inline __attribute__ ((warn_unused_result)) struct clew_bound clew_bound_union_xy (const struct clew_bound *a, int32_t x, int32_t y)
{
        struct clew_bound u;
        u.minlon = MIN(a->minlon, x);
        u.minlat = MIN(a->minlat, y);
        u.maxlon = MAX(a->maxlon, x);
        u.maxlat = MAX(a->maxlat, y);
        return u;
}

static inline __attribute__ ((warn_unused_result)) struct clew_bound clew_bound_union_point (const struct clew_bound *a, const struct clew_point *point)
{
        struct clew_bound u;
        u.minlon = MIN(a->minlon, point->lon);
        u.minlat = MIN(a->minlat, point->lat);
        u.maxlon = MAX(a->maxlon, point->lon);
        u.maxlat = MAX(a->maxlat, point->lat);
        return u;
}

static inline __attribute__ ((warn_unused_result)) struct clew_bound clew_bound_enlarge (const struct clew_bound *a, int32_t enlarge)
{
        struct clew_bound e;
        e.minlon = MIN(a->minlon - enlarge, a->maxlon + enlarge);
        e.maxlon = MAX(a->minlon - enlarge, a->maxlon + enlarge);
        e.minlat = MIN(a->minlat - enlarge, a->maxlat + enlarge);
        e.maxlat = MAX(a->minlat - enlarge, a->maxlat + enlarge);
        return e;
}

#ifdef __cplusplus
}
#endif
