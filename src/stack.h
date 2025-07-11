
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

static inline struct clew_stack clew_stack_init (uint64_t size) {
        return (struct clew_stack) {
                .size = size,
                .grow = 4096,
                .count = 0,
                .avail = 0,
                .buffer = NULL
        };
}

static inline struct clew_stack clew_stack_init2 (uint64_t size, uint64_t grow) {
        return (struct clew_stack) {
                .size = size,
                .grow = grow,
                .count = 0,
                .avail = 0,
                .buffer = NULL
        };
}

static inline void clew_stack_uninit (struct clew_stack *stack)
{
        if (stack == NULL) {
                return;
        }
        if (stack->buffer != NULL) {
                free(stack->buffer);
        }
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

static inline int clew_stack_push_uint32 (struct clew_stack *stack, uint32_t elem)
{
        return clew_stack_push(stack, &elem);
}

static inline void * clew_stack_pop (struct clew_stack *stack)
{
        if (stack->count < 1) {
                return NULL;
        }
        return stack->buffer + --stack->count * stack->size;
}

static inline uint32_t clew_stack_pop_uint32 (struct clew_stack *stack)
{
        return *(uint32_t *) clew_stack_pop(stack);
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

static inline void * clew_stack_at (const struct clew_stack *stack, uint64_t at)
{
        if (unlikely(at >= stack->count)) {
                return NULL;
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

static inline uint32_t clew_stack_peek_uint32 (const struct clew_stack *stack)
{
        void *elem;
        elem = clew_stack_peek(stack);
        return elem ? (*(uint32_t *) elem) : UINT32_MAX;
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
        return bsearch(key, stack->buffer, stack->count, stack->size, compare);
}
