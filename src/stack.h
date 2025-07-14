
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)

struct clew_stack {
        uint64_t size;
        uint64_t grow;

        uint64_t count;
        uint64_t avail;
        uint8_t *buffer;
};

static inline struct clew_stack clew_stack_init2 (uint64_t size, uint64_t grow) {
        return (struct clew_stack) {
                .size = size,
                .grow = grow,
                .count = 0,
                .avail = 0,
                .buffer = NULL
        };
}

static inline struct clew_stack clew_stack_init (uint64_t size) {
        return clew_stack_init2(size, 4096);
}

static inline void clew_stack_uninit (struct clew_stack *stack)
{
        if (stack == NULL) {
                return;
        }
        if (stack->buffer != NULL) {
                free(stack->buffer);
        }
        stack->buffer = NULL;
        stack->count  = 0;
        stack->avail  = 0;
}

static inline uint64_t clew_stack_calculate_resize (uint64_t current, uint64_t request, uint64_t grow)
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

static inline int clew_stack_reserve (struct clew_stack *stack, uint64_t reserve)
{
        unsigned char *buffer;
        if (stack->avail >= reserve) {
                return 0;
        }
        reserve = clew_stack_calculate_resize(stack->avail, reserve, stack->grow);
        buffer = (unsigned char *) realloc(stack->buffer, reserve * stack->size);
        if (unlikely(buffer == NULL)) {
                buffer = (unsigned char *) malloc(reserve * stack->size);
                if (unlikely(buffer == NULL)) {
                        return -1;
                }
                memcpy(buffer, stack->buffer, stack->count * stack->size);
                free(stack->buffer);
        }
        stack->buffer = buffer;
        stack->avail  = reserve;
        return 0;
}

static inline void clew_stack_reset (struct clew_stack *stack)
{
        stack->count = 0;
}

static inline uint64_t clew_stack_count (const struct clew_stack *stack)
{
        return stack->count;
}

static inline uint64_t clew_stack_size (const struct clew_stack *stack)
{
        return stack->size;
}

static inline int clew_stack_empty (const struct clew_stack *stack)
{
        return (clew_stack_count(stack) > 0) ? 0 : 1;
}

static inline int clew_stack_push (struct clew_stack *stack, const void *elem)
{
        int rc;
        rc = clew_stack_reserve(stack, stack->count + 1);
        if (unlikely(rc != 0)) {
                return -1;
        }
        memcpy(stack->buffer + stack->count++ * stack->size, elem, stack->size);
        return 0;
}

static inline int clew_stack_push_sorted (struct clew_stack *stack, const void *elem, int (*compare)(const void *, const void *))
{
        size_t lo = 0, hi = stack->count;
        while (lo < hi) {
                size_t mid = lo + (hi - lo) / 2;
                void *mid_elem = stack->buffer + mid * stack->size;
                int cmp = compare(elem, mid_elem);
                if (cmp < 0)
                        hi = mid;
                else
                        lo = mid + 1;
        }

        // Insert at position `lo`
        int rc = clew_stack_reserve(stack, stack->count + 1);
        if (rc != 0) return -1;

        void *dst = stack->buffer + (lo + 1) * stack->size;
        void *src = stack->buffer + lo * stack->size;
        memmove(dst, src, (stack->count - lo) * stack->size);
        memcpy(src, elem, stack->size);
        stack->count += 1;
        return 0;
}

static inline int clew_stack_push_sorted_unique (struct clew_stack *stack, const void *elem, int (*compare)(const void *, const void *))
{
        size_t lo = 0, hi = stack->count;
        while (lo < hi) {
                size_t mid = lo + (hi - lo) / 2;
                void *mid_elem = stack->buffer + mid * stack->size;
                int cmp = compare(elem, mid_elem);
                if (cmp == 0)
                        return 0; // Already exists, do not insert
                if (cmp < 0)
                        hi = mid;
                else
                        lo = mid + 1;
        }

        // Insert at position `lo`
        int rc = clew_stack_reserve(stack, stack->count + 1);
        if (rc != 0) return -1;

        // Shift elements
        void *dst = stack->buffer + (lo + 1) * stack->size;
        void *src = stack->buffer + lo * stack->size;
        memmove(dst, src, (stack->count - lo) * stack->size);
        memcpy(src, elem, stack->size);
        stack->count += 1;
        return 1; // Inserted
}

static inline void * clew_stack_pop (struct clew_stack *stack)
{
        if (stack->count < 1) {
                return NULL;
        }
        return stack->buffer + --stack->count * stack->size;
}

static inline void clew_stack_del (struct clew_stack *stack, uint64_t at)
{
        void *dst;
        void *src;
        uint64_t size;
        if (at >= stack->count) {
                return;
        }
        dst = stack->buffer + at * stack->size;
        src = stack->buffer + (at + 1) * stack->size;
        size = (stack->count - at - 1) * stack->size;
        if (size != 0) {
                memmove(dst, src, size);
        }
        stack->count -= 1;
}

static inline void * clew_stack_at (const struct clew_stack *stack, int64_t at)
{
        if (likely(at >= 0)) {
                if (unlikely(at >= (int64_t) stack->count)) {
                        return NULL;
                }
        } else {
                if (unlikely(-at >= (int64_t) stack->count)) {
                        return NULL;
                }
                at = stack->count + at;
        }
        return stack->buffer + at * stack->size;
}

static inline void * clew_stack_peek (const struct clew_stack *stack)
{
        if (stack->count < 1) {
                return NULL;
        }
        return stack->buffer + (stack->count - 1) * stack->size;
}

static inline void * clew_stack_back (struct clew_stack *stack)
{
        return clew_stack_peek(stack);
}

static inline void * clew_stack_front (struct clew_stack *stack)
{
        return clew_stack_at(stack, 0);
}

static inline void clew_stack_sort (const struct clew_stack *stack, int (*compare) (const void *a, const void *b))
{
        qsort(stack->buffer, stack->count, stack->size, compare);
}

static inline void * clew_stack_search (const struct clew_stack *stack, const void *key, int (*compare) (const void *a, const void *b))
{
        int r;
        uint64_t i;
        if (stack->count <= 7) {
                for (i = 0; i < stack->count; i++) {
                        r = compare(key, stack->buffer + i * stack->size);
                        if (r == 0) {
                                return stack->buffer + i * stack->size;
                        } else if (r < 0) {
                                break;
                        }
                }
                return NULL;
        } else {
                return bsearch(key, stack->buffer, stack->count, stack->size, compare);
        }
}

#define CLEW_STACK_API_FOR_TYPE(name, type) \
        static inline int clew_stack_compare_ ## name (const void *a, const void *b)                            \
        {                                                                                                       \
                const type *t1 = (const type *) a;                                                              \
                const type *t2 = (const type *) b;                                                              \
                return *t1 - *t2;                                                                               \
        }                                                                                                       \
                                                                                                                \
        static inline int clew_stack_push_ ## name (struct clew_stack *stack, type elem)                        \
        {                                                                                                       \
                return clew_stack_push(stack, &elem);                                                           \
        }                                                                                                       \
                                                                                                                \
        static inline int clew_stack_push_ ## name ## _sorted (struct clew_stack *stack, type elem)             \
        {                                                                                                       \
                return clew_stack_push_sorted(stack, &elem, clew_stack_compare_ ## name);                       \
        }                                                                                                       \
                                                                                                                \
        static inline int clew_stack_push_ ## name ## _sorted_unique (struct clew_stack *stack, type elem)      \
        {                                                                                                       \
                return clew_stack_push_sorted_unique(stack, &elem, clew_stack_compare_ ## name);                \
        }                                                                                                       \
                                                                                                                \
        static inline type clew_stack_pop_ ## name (struct clew_stack *stack)                                   \
        {                                                                                                       \
                return *(type *) clew_stack_pop(stack);                                                         \
        }                                                                                                       \
                                                                                                                \
        static inline type clew_stack_at_ ## name (const struct clew_stack *stack, uint64_t at)                 \
        {                                                                                                       \
                void *elem;                                                                                     \
                elem = clew_stack_at(stack, at);                                                                \
                return elem ? *(type *) elem : 0;                                                               \
        }                                                                                                       \
                                                                                                                \
        static inline type clew_stack_peek_ ## name (const struct clew_stack *stack)                            \
        {                                                                                                       \
                void *elem;                                                                                     \
                elem = clew_stack_peek(stack);                                                                  \
                return elem ? (*(type *) elem) : 0;                                                             \
        }                                                                                                       \
                                                                                                                \
        static inline void clew_stack_sort_ ## name (const struct clew_stack *stack)                            \
        {                                                                                                       \
                clew_stack_sort(stack, clew_stack_compare_ ## name);                                            \
        }                                                                                                       \
                                                                                                                \
        static inline uint64_t clew_stack_search_ ## name (const struct clew_stack *stack, type key)            \
        {                                                                                                       \
                uint8_t *elem;                                                                                  \
                elem = clew_stack_search(stack, &key, clew_stack_compare_ ## name);                             \
                return (elem == NULL) ? UINT64_MAX : ((elem - stack->buffer) / stack->size);                    \
        }

CLEW_STACK_API_FOR_TYPE(int32, int32_t)
CLEW_STACK_API_FOR_TYPE(int64, int64_t)
CLEW_STACK_API_FOR_TYPE(uint32, uint32_t)
CLEW_STACK_API_FOR_TYPE(uint64, uint64_t)
