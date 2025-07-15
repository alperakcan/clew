
struct clew_pqueue_head;

struct clew_pqueue_head * clew_pqueue_create (
        unsigned int size, unsigned int step,
        int (*compare) (void *a, void *b),
        void (*setpos) (void *entry, unsigned int position),
        unsigned int (*getpos) (void *entry));
void clew_pqueue_destroy (struct clew_pqueue_head *head);

unsigned int clew_pqueue_count (struct clew_pqueue_head *head);

int clew_pqueue_add (struct clew_pqueue_head *head, void *entry);
int clew_pqueue_mod (struct clew_pqueue_head *head, void *entry, int compare);
int clew_pqueue_del (struct clew_pqueue_head *head, void *entry);
int clew_pqueue_verify (struct clew_pqueue_head *head);

void * clew_pqueue_peek (struct clew_pqueue_head *head);
void * clew_pqueue_pop (struct clew_pqueue_head *head);
int clew_pqueue_search (struct clew_pqueue_head *head, void *key, int (*callback) (void *context, void *entry), void *context);
int clew_pqueue_traverse (struct clew_pqueue_head *head, int (*callback) (void *context, void *entry), void *context);
