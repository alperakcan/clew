
#include <math.h>

#include "bound.h"
#include "projection-mercator.h"

#define CLEW_PROJECTION_WGS84_SCALE_MULTIPLIER        (10000000.00)

#define CLEW_PROJECTION_WGS84_SCALE(v)                ((int32_t) (((double) v) * CLEW_PROJECTION_WGS84_SCALE_MULTIPLIER))
#define CLEW_PROJECTION_WGS84_SCALE1(v0)              CLEW_PROJECTION_WGS84_SCALE(v0)
#define CLEW_PROJECTION_WGS84_SCALE2(v0, v1)          CLEW_PROJECTION_WGS84_SCALE(v0), CLEW_PROJECTION_WGS84_SCALE(v1)
#define CLEW_PROJECTION_WGS84_SCALE3(v0, v1, v2)      CLEW_PROJECTION_WGS84_SCALE(v0), CLEW_PROJECTION_WGS84_SCALE(v1), CLEW_PROJECTION_WGS84_SCALE(v2)
#define CLEW_PROJECTION_WGS84_SCALE4(v0, v1, v2, v3)  CLEW_PROJECTION_WGS84_SCALE(v0), CLEW_PROJECTION_WGS84_SCALE(v1), CLEW_PROJECTION_WGS84_SCALE(v2), CLEW_PROJECTION_WGS84_SCALE(v3)

#define CLEW_PROJECTION_WGS84_INVERT(v)               ((double) (((double) v) / CLEW_PROJECTION_WGS84_SCALE_MULTIPLIER))
#define CLEW_PROJECTION_WGS84_INVERT1(v0)             CLEW_PROJECTION_WGS84_INVERT(v0)
#define CLEW_PROJECTION_WGS84_INVERT2(v0, v1)         CLEW_PROJECTION_WGS84_INVERT(v0), CLEW_PROJECTION_WGS84_INVERT(v1)
#define CLEW_PROJECTION_WGS84_INVERT3(v0, v1, v2)     CLEW_PROJECTION_WGS84_INVERT(v0), CLEW_PROJECTION_WGS84_INVERT(v1), CLEW_PROJECTION_WGS84_INVERT(v2)
#define CLEW_PROJECTION_WGS84_INVERT4(v0, v1, v2, v3) CLEW_PROJECTION_WGS84_INVERT(v0), CLEW_PROJECTION_WGS84_INVERT(v1), CLEW_PROJECTION_WGS84_INVERT(v2), CLEW_PROJECTION_WGS84_INVERT(v3)

static struct clew_bound g_bound;

static const double g_earth_mean_radius     = 6378137;
static const double g_earth_bound_lon_east  = +180.00;
static const double g_earth_bound_lon_west  = -180.00;
static const double g_earth_bound_lat_north = +83.00;
static const double g_earth_bound_lat_south = -83.00;

static inline double mercator_lon_to_x (double lon)
{
	return lon * g_earth_mean_radius * M_PI / 180.00;
}

static inline double mercator_lat_to_y (double lat)
{
	return log(tan(M_PI_4 + lat * M_PI / 360.00)) * g_earth_mean_radius;
}

static inline double mercator_x_to_lon (double x)
{
	return x / g_earth_mean_radius / M_PI * 180.00;
}

static inline double mercator_y_to_lat (double y)
{
	return atan(exp(y / g_earth_mean_radius)) / M_PI * 360.00 - 90.00;
}

int32_t clew_projection_mercator_convert_lon (int32_t lon)
{
	if (lon > CLEW_PROJECTION_WGS84_SCALE(g_earth_bound_lon_east)) {
		lon = CLEW_PROJECTION_WGS84_SCALE(g_earth_bound_lon_east);
	}
	if (lon < CLEW_PROJECTION_WGS84_SCALE(g_earth_bound_lon_west)) {
		lon = CLEW_PROJECTION_WGS84_SCALE(g_earth_bound_lon_west);
	}
	return mercator_lon_to_x(CLEW_PROJECTION_WGS84_INVERT(lon)) + (g_bound.maxlon / 2.00);
}

int32_t clew_projection_mercator_convert_lat (int32_t lat)
{
	if (lat > CLEW_PROJECTION_WGS84_SCALE(g_earth_bound_lat_north)) {
		lat = CLEW_PROJECTION_WGS84_SCALE(g_earth_bound_lat_north);
	}
	if (lat < CLEW_PROJECTION_WGS84_SCALE(g_earth_bound_lat_south)) {
		lat = CLEW_PROJECTION_WGS84_SCALE(g_earth_bound_lat_south);
	}
	return (g_bound.maxlat / 2.00) - mercator_lat_to_y(CLEW_PROJECTION_WGS84_INVERT(lat));
}

int32_t clew_projection_mercator_invert_lon (int32_t lon)
{
	return CLEW_PROJECTION_WGS84_SCALE(mercator_x_to_lon(lon - (g_bound.maxlon / 2.00)));
}

int32_t clew_projection_mercator_invert_lat (int32_t lat)
{
	return CLEW_PROJECTION_WGS84_SCALE(mercator_y_to_lat((g_bound.maxlat / 2.00) - lat));
}

void clew_projection_mercator_init (void)
{
	g_bound.minlon = mercator_lon_to_x(g_earth_bound_lon_west);
	g_bound.maxlon = mercator_lon_to_x(g_earth_bound_lon_east);
	g_bound.minlat = mercator_lat_to_y(g_earth_bound_lat_south);
	g_bound.maxlat = mercator_lat_to_y(g_earth_bound_lat_north);
	g_bound.minlon = g_bound.minlon + g_bound.maxlon;
	g_bound.maxlon = g_bound.maxlon + g_bound.maxlon;
	g_bound.minlat = g_bound.minlat + g_bound.maxlat;
	g_bound.maxlat = g_bound.maxlat + g_bound.maxlat;
}

void clew_projection_mercator_fini (void)
{

}
