
#if !defined(CLEW_PROJECTION_MERCATOR_H)
#define CLEW_PROJECTION_MERCATOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int32_t clew_projection_mercator_convert_lon (int32_t lon);
int32_t clew_projection_mercator_convert_lat (int32_t lat);

int32_t clew_projection_mercator_invert_lon (int32_t lon);
int32_t clew_projection_mercator_invert_lat (int32_t lat);

#ifdef __cplusplus
}
#endif

#endif
