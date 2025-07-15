
#if !defined(CLEW_POINT_H)
#define CLEW_POINT_H

#include <stdint.h>
#include <math.h>

#if !defined(CLEW_POINT_FORMAT)
#define CLEW_POINT_FORMAT               "%.7f, %.7f"
#endif

#if !defined(CLEW_POINT_FORMAT_ARG)
#define CLEW_POINT_FORMAT_ARG(b)        (b).lon, (b).lat
#endif

struct clew_point {
        int32_t lon;
        int32_t lat;
};

static inline __attribute__ ((warn_unused_result)) struct clew_point clew_point_init (int32_t lon, int32_t lat)
{
        return (struct clew_point) { .lon = lon, .lat = lat };
}

static inline __attribute__ ((warn_unused_result)) struct clew_point clew_point_null (void)
{
        return clew_point_init(INT32_MIN, INT32_MIN);
}

static inline __attribute__ ((warn_unused_result)) int clew_point_valid (const struct clew_point *point)
{
        return ((point->lon != INT32_MIN) && (point->lat != INT32_MIN));
}

static inline __attribute__ ((warn_unused_result)) int clew_point_invalid (const struct clew_point *point)
{
        return ((point->lon == INT32_MIN) || (point->lat == INT32_MIN));
}

static inline void clew_point_copy (struct clew_point *dst, const struct clew_point *src)
{
        *dst = *src;
}

static inline __attribute__ ((warn_unused_result)) int clew_point_equal (const struct clew_point *point, const struct clew_point *with)
{
        return ((point->lon == with->lon) && (point->lat == with->lat));
}

static inline __attribute__ ((warn_unused_result)) int clew_point_equal_xy (const struct clew_point *point, int32_t lon, int32_t lat)
{
        return ((point->lon == lon) && (point->lat == lat));
}

static inline __attribute__ ((warn_unused_result)) double clew_point_distance (const struct clew_point *a, const struct clew_point *b)
{
        double dx, dy;
        dx = b->lon - a->lon;
        dy = b->lat - a->lat;
        return sqrt(dx * dx + dy * dy);
}

static inline __attribute__ ((warn_unused_result)) double clew_point_segment_distance (const struct clew_point *p, const struct clew_point *s1, const struct clew_point *s2)
{
        double x = s1->lon;
        double y = s1->lat;
        double dx = s2->lon - x;
        double dy = s2->lat - y;
        if (dx != 0 || dy != 0) {
                double t = ((p->lon - x) * dx + (p->lat - y) * dy) / (dx * dx + dy * dy);
                if (t > 1) {
                        x = s2->lon;
                        y = s2->lat;
                } else if (t < 0) {
                        x = s1->lon;
                        y = s1->lat;
                } else {
                        x += dx * t;
                        y += dy * t;
                }
        }
        dx = p->lon - x;
        dy = p->lat - y;
        return sqrt(dx * dx + dy * dy);
}

static inline __attribute__ ((warn_unused_result)) double clew_point_distance_euclidean (const struct clew_point *a, const struct clew_point *b)
{
        static const double earthRadius = 6378137.0;
        double dLat = ((b->lat - a->lat) * 1e-7) * M_PI / 180.00;
        double dLng = ((b->lon - a->lon) * 1e-7) * M_PI / 180.00;
        double sindLat = sin(dLat / 2);
        double sindLng = sin(dLng / 2);
        double _a = pow(sindLat, 2) + pow(sindLng, 2) * cos((a->lat * M_PI / 180.00)) * cos((b->lat * M_PI / 180.00));
        double _c = 2 * atan2(sqrt(_a), sqrt(1 - _a));
        double dist = earthRadius * _c;
        return dist;
}

static inline struct clew_point clew_point_derived_position (const struct clew_point *a, double range, double bearing)
{
        static const double earthRadius = 6378137.0;
        static const double degreesToRadians = M_PI / 180.0;
        static const double radiansToDegrees = 180.0 / M_PI;

        double latA = a->lat * 1e-7 * degreesToRadians;
        double lonA = a->lon * 1e-7 * degreesToRadians;
        double angularDistance = range / earthRadius;
        double trueCourse = bearing * degreesToRadians;

        double lat = asin(
            sin(latA) * cos(angularDistance) +
            cos(latA) * sin(angularDistance) * cos(trueCourse));

        double dlon = atan2(
            sin(trueCourse) * sin(angularDistance) * cos(latA),
            cos(angularDistance) - sin(latA) * sin(lat));

        double lon = fmod(lonA + dlon + M_PI, M_PI * 2);
        if (lon < 0) {
                lon += 2 * M_PI;
        }
        lon -= M_PI;

        return (struct clew_point) {
                .lon = (int32_t) (lon * radiansToDegrees * 1e7),
                .lat = (int32_t) (lat * radiansToDegrees * 1e7)
        };
}

#endif
