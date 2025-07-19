
#if !defined(CLEW_POINT_H)
#define CLEW_POINT_H

#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

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
        static const double degreesToRadians = M_PI / 180.0;
        double aLat = (a->lat * 1e-7) * degreesToRadians;
        double bLat = (b->lat * 1e-7) * degreesToRadians;
        double dLat = bLat - aLat;
        double dLng = ((b->lon - a->lon) * 1e-7) * degreesToRadians;
        double sindLat = sin(dLat / 2);
        double sindLng = sin(dLng / 2);
        double _a = sindLat * sindLat + sindLng * sindLng * cos(aLat) * cos(bLat);
        double _c = 2 * atan2(sqrt(_a), sqrt(1 - _a));
        double dist = earthRadius * _c;
        return dist;
}

static inline struct clew_point clew_point_derived_position (const struct clew_point *a, double range, double bearing)
{
    static const double R = 6378137.0; // WGS-84 Earth radius
    static const double degreesToRadians = M_PI / 180.0;
    static const double radiansToDegrees = 180.0 / M_PI;

    double lat1 = a->lat * 1e-7 * degreesToRadians;
    double lon1 = a->lon * 1e-7 * degreesToRadians;
    double brng = bearing * degreesToRadians;
    double d = range;

    double dLat = d * cos(brng) / R;
    double lat2 = lat1 + dLat;

    // Handle nearly constant latitude for E/W movement
    double dPhi = log(tan(lat2 / 2 + M_PI / 4) / tan(lat1 / 2 + M_PI / 4));
    double q = (fabs(dPhi) > 1e-12) ? (dLat / dPhi) : cos(lat1); // avoid div-by-zero
    double dLon = d * sin(brng) / (R * q);
    double lon2 = lon1 + dLon;

    // Normalize
    if (lon2 > M_PI) lon2 -= 2 * M_PI;
    if (lon2 < -M_PI) lon2 += 2 * M_PI;

    return (struct clew_point) {
        .lon = (int32_t)(lon2 * radiansToDegrees * 1e7),
        .lat = (int32_t)(lat2 * radiansToDegrees * 1e7)
    };
}

#ifdef __cplusplus
}
#endif

#endif
