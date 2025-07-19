
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)

struct clew_bitmap {
        uint64_t grow;

        uint64_t avail;
        uint8_t *buffer;
};

static inline struct clew_bitmap clew_bitmap_init (uint64_t grow) {
        return (struct clew_bitmap) {
                .grow = grow,
                .avail = 0,
                .buffer = NULL
        };
}

static inline void clew_bitmap_uninit (struct clew_bitmap *bitmap)
{
        if (bitmap == NULL) {
                return;
        }
        if (bitmap->buffer != NULL) {
                free(bitmap->buffer);
        }
        bitmap->buffer = NULL;
        bitmap->avail  = 0;
}

static inline uint64_t clew_bitmap_calculate_resize (uint64_t current, uint64_t request, uint64_t grow)
{
        if (current == 0) {
                current = 2;
        }
        if (current >= request) {
                return current;
        }
        if (request >= grow) {
                return (request + (grow - 1)) & ~(uint64_t) (grow - 1);
        }
        request--;
        request |= request >> 1;
        request |= request >> 2;
        request |= request >> 4;
        request |= request >> 8;
        request |= request >> 16;
        request |= request >> 32;
        request++;
        return (request < 8) ? 8 : request;
}

static inline int clew_bitmap_reserve (struct clew_bitmap *bitmap, uint64_t reserve)
{
        uint64_t os;
        uint64_t ns;
        unsigned char *buffer;

        if (bitmap->avail >= reserve) {
                return 0;
        }

        reserve = clew_bitmap_calculate_resize(bitmap->avail, reserve, bitmap->grow);

        os = (bitmap->avail + 7) / 8;
        ns = (reserve + 7) / 8;

        if (bitmap->avail >= reserve) {
                return 0;
        }

        buffer = (unsigned char *) realloc(bitmap->buffer, ns);
        if (unlikely(buffer == NULL)) {
                buffer = (unsigned char *) malloc(ns);
                if (unlikely(buffer == NULL)) {
                        return -1;
                }
                if (bitmap->buffer) {
                        memcpy(buffer, bitmap->buffer, os);
                        free(bitmap->buffer);
                }
        }
        memset(buffer + os, 0, ns - os);

        bitmap->buffer = buffer;
        bitmap->avail  = reserve;
        return 0;
}

static inline void clew_bitmap_reset (struct clew_bitmap *bitmap)
{
        memset(bitmap->buffer, 0, (bitmap->avail + 7) / 8);
}

static inline int clew_bitmap_mark (struct clew_bitmap *bitmap, uint64_t at)
{
        int rc;
        rc = clew_bitmap_reserve(bitmap, at + 1);
        if (unlikely(rc != 0)) {
                return -1;
        }
	bitmap->buffer[at / 8] |= 1 << (at % 8);
        return 0;
}

static inline int clew_bitmap_unmark (struct clew_bitmap *bitmap, uint64_t at)
{
        if (unlikely(at >= bitmap->avail)) {
                return -1;
        }
        bitmap->buffer[at / 8] &= ~(1 << (at % 8));
        return 0;
}

static inline int clew_bitmap_marked (const struct clew_bitmap *bitmap, uint64_t at)
{
        if (unlikely(at >= bitmap->avail)) {
                return 0;
        }
	return !!(bitmap->buffer[at / 8] & (1 << (at % 8)));
}

static inline uint64_t clew_bitmap_count (const struct clew_bitmap *bitmap)
{
        static const uint8_t bitcount[256] = {
                0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
                1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
        };

        uint64_t count = 0;
        uint64_t bytes = (bitmap->avail + 7) / 8;
        for (uint64_t i = 0; i < bytes; i++) {
                count += bitcount[bitmap->buffer[i]];
        }

        uint64_t extra = bitmap->avail % 8;
        if (extra != 0) {
                uint8_t mask = (1 << extra) - 1;
                count -= bitcount[bitmap->buffer[bytes - 1] & ~mask];
        }

        return count;
}

#if 0

static inline int clew_bitmap_foreach (const struct clew_bitmap *bitmap, int (*callback) (void *context, uint64_t at), void *context)
{
        int rc;
        uint64_t i;
        uint64_t il;

        uint64_t bitcount;
        uint64_t byte_index;

        if (unlikely(bitmap == NULL)) {
                goto bail;
        }
        if (unlikely(callback == NULL)) {
                goto bail;
        }

        for (i = 0, il = bitmap->avail; i < il; i++) {
                rc = clew_bitmap_marked(bitmap, i);
                if (rc == 1) {
                        rc = callback(context, i);
                        if (unlikely(rc < 0)) {
                                clew_errorf("bitmap logic error");
                                goto bail;
                        } else if (rc == 1) {
                                break;
                        }
                } else if (unlikely(rc < 0)) {
                        clew_errorf("bitmap logic error");
                        goto bail;
                }
        }

        return 0;
bail:   return -1;
}

#else

static inline int clew_bitmap_foreach(const struct clew_bitmap *bitmap, int (*callback)(void *context, uint64_t at), void *context)
{
        static const uint8_t ctz[256] = {
                8, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
                4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
                5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
                4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
                6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
                4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
                5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
                4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
                7, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
                4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
                5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
                4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
                6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
                4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
                5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
                4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0
        };

        uint64_t bytes;

        if (unlikely(bitmap == NULL)) {
                goto bail;
        }
        if (unlikely(callback == NULL)) {
                goto bail;
        }

        bytes = (bitmap->avail + 7) / 8;
        for (uint64_t i = 0; i < bytes; ++i) {
                uint8_t byte = bitmap->buffer[i];

                if (i == bytes - 1 && (bitmap->avail % 8)) {
                        byte &= (1U << (bitmap->avail % 8)) - 1;
                }

                while (byte) {
                        int bit = ctz[byte];
                        uint64_t bit_index = i * 8 + bit;

                        int rc = callback(context, bit_index);
                        if (unlikely(rc < 0)) {
                                goto bail;
                        } else if (rc == 1) {
                                goto out;
                        }

                        byte &= byte - 1;
                }
        }

out:    return 0;
bail:   return -1;
}

#endif

#ifdef __cplusplus
}
#endif
