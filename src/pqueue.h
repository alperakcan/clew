
#include <stdint.h>

struct clew_pqueue;

struct clew_pqueue * clew_pqueue_create (
        uint64_t size, uint64_t step,
        int (*compare) (const void *a, const void *b),
        void (*setpos) (void *entry, uint64_t position),
        uint64_t (*getpos) (const void *entry));
void clew_pqueue_destroy (struct clew_pqueue *head);

uint64_t clew_pqueue_count (struct clew_pqueue *head);

int clew_pqueue_add (struct clew_pqueue *head, void *entry);
int clew_pqueue_mod (struct clew_pqueue *head, void *entry, int compare);
int clew_pqueue_del (struct clew_pqueue *head, void *entry);
int clew_pqueue_verify (struct clew_pqueue *head);

void * clew_pqueue_peek (struct clew_pqueue *head);
void * clew_pqueue_pop (struct clew_pqueue *head);
int clew_pqueue_search (struct clew_pqueue *head, void *key, int (*callback) (void *context, void *entry), void *context);
int clew_pqueue_traverse (struct clew_pqueue *head, int (*callback) (void *context, void *entry), void *context);
