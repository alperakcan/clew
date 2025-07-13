
#include <stdint.h>

struct clew_expression;

struct clew_expression * clew_expression_create (const char *expression);
void clew_expression_destroy (struct clew_expression *expression);

const char * clew_expression_orig (struct clew_expression *expression);
const char * clew_expression_text (struct clew_expression *expression);

int clew_expression_match (
		const struct clew_expression *expression,
		void *context,
		int (*_and) (int first, int second),
		int (*_or) (int first, int second),
		int (*_not) (int first),
		int (*_has) (void *context, uint32_t tag));

int clew_expression_has (const struct clew_expression *expression, uint32_t tag);
int clew_expression_count (const struct clew_expression *expression);
