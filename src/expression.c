
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>

#define CLEW_DEBUG_NAME "search"

#include "debug.h"
#include "tag.h"
#include "expression.h"

#define likely(x)		__builtin_expect(!!(x), 1)
#define unlikely(x)		__builtin_expect(!!(x), 0)

#define MIN(a, b)		(((a) < (b)) ? (a) : (b))
#define MAX(a, b)		(((a) > (b)) ? (a) : (b))

#define TAG_SIZE_MAX		(256)
#define EXPRESSION_SIZE_MAX	(32 * 1024)

struct clew_expression {
	char *text;
	unsigned long long count;
	unsigned long long ors;
	unsigned long long ands;
	unsigned long long nots;
	uint32_t postfix[0];
};

static struct clew_expression * clew_expression_create_actual (const char *from)
{
	int i;
	size_t w;
	char *in;
	char *str;
	char *ptr;
	unsigned long long ors;
	unsigned long long ands;
	unsigned long long nots;
	uint32_t *opr;
	uint32_t *pfx;
	uint32_t *cur;
	uint32_t *current;
	uint32_t *operators;
	uint32_t *postfix;
	struct clew_expression *expression;
	in = NULL;
	current = NULL;
	operators = NULL;
	expression = NULL;
	in = strdup(from);
	if (unlikely(in == NULL)) {
		clew_errorf("can not allocate memory");
		goto bail;
	}
	for (w = 1, str = in; *str; str++) {
		if (*str == ' ') {
			w += 1;
		}
	}
	current = (uint32_t *) calloc(w + 1, sizeof(uint32_t));
	if (unlikely(current == NULL)) {
		clew_errorf("can not allocate memory");
		goto bail;
	}
	ors = 0;
	ands = 0;
	nots = 0;
	for (cur = current, ptr = in, str = in; *ptr; ptr++) {
		if (*ptr != ' ') {
			continue;
		}
		*ptr = '\0';
		if (strcmp(str, "and") == 0) {
			clew_debugf("and");
			ands += 1;
			*cur = clew_tag_internal_expression_and;
		} else if (strcmp(str, "or") == 0) {
			clew_debugf("or");
			ors += 1;
			*cur = clew_tag_internal_expression_or;
		} else if (strcmp(str, "not") == 0) {
			clew_debugf("not");
			nots += 1;
			*cur = clew_tag_internal_expression_not;
		} else if (strcmp(str, "(") == 0) {
			clew_debugf("(");
			*cur = clew_tag_internal_expression_group_start;
		} else if (strcmp(str, ")") == 0) {
			clew_debugf(")");
			*cur = clew_tag_internal_expression_group_end;
		} else {
			*cur = clew_tag_value(str);
			clew_debugf("%s %d", str, *cur);
		}
		if (unlikely(*cur == clew_tag_unknown)) {
			clew_errorf("invalid value: '%s'", str);
			goto bail;
		}
		str = ptr + 1;
		cur++;
	}
	*cur = clew_tag_unknown;
	clew_debugf("current:");
	for (cur = current; *cur != clew_tag_unknown; cur++) {
		clew_debugf("  %s", clew_tag_string(*cur));
	}
	operators = (uint32_t *) malloc(sizeof(uint32_t) * (w + 1));
	if (unlikely(operators == NULL)) {
		clew_errorf("can not allocate memory");
		goto bail;
	}
	*operators = clew_tag_unknown;
	expression = (struct clew_expression *) malloc(sizeof(struct clew_expression) + (sizeof(uint32_t) * (w + 1)));
	if (unlikely(expression == NULL)) {
		clew_errorf("can not allocate memory");
		goto bail;
	}
	expression->text = strdup(from);
	if (expression->text == NULL) {
		clew_errorf("can not allocate memory");
		goto bail;
	}
	expression->ors = ors;
	expression->ands = ands;
	expression->nots = nots;
	expression->count = w + 1;
	*expression->postfix = clew_tag_unknown;
	postfix = expression->postfix;
	for (cur = current, opr = operators, pfx = postfix; *cur != clew_tag_unknown; cur++) {
		clew_debugf("current: %s", clew_tag_string(*cur));
		clew_debugf("  operators:");
		for (opr = operators; *opr != clew_tag_unknown; opr++) {
			clew_debugf("    %d - %s", *opr, clew_tag_string(*opr));
		}
		clew_debugf("  postfix:");
		for (pfx = postfix; *pfx != clew_tag_unknown; pfx++) {
			clew_debugf("    %d - %s", *pfx, clew_tag_string(*pfx));
		}
		if ((*cur != clew_tag_internal_expression_and) &&
		    (*cur != clew_tag_internal_expression_or) &&
		    (*cur != clew_tag_internal_expression_not) &&
		    (*cur != clew_tag_internal_expression_group_start) &&
		    (*cur != clew_tag_internal_expression_group_end)) {
			/* print operands as they arrive */
			*pfx++ = *cur;
			*pfx = clew_tag_unknown;
		} else if ((opr == operators) ||
			   ((*(opr - 1) == clew_tag_internal_expression_group_start) &&
			    (*cur != clew_tag_internal_expression_group_end))) {
			/* if the stack is empty or contains a left parenthesis on top,
			 *  push the incoming operators onto the stack */
			*opr++ = *cur;
			*opr = clew_tag_unknown;
		} else if (*cur == clew_tag_internal_expression_group_start) {
			/* if the incoming symbol is a left parenthesis, push it on the stack */
			*opr++ = *cur;
			*opr = clew_tag_unknown;
		} else if (*cur == clew_tag_internal_expression_group_end) {
			/* if the incoming symbol is a right parenthesis, pop the stack and print
			 * the operatorss until you see a left parenthesis. Discard the pair of
			 * parentheses */
			clew_debugf("opr: %s", clew_tag_string(*opr));
			opr--;
			while (opr >= operators) {
				clew_debugf("opr: %s", clew_tag_string(*opr));
				if (*opr == clew_tag_internal_expression_group_start) {
					*opr-- = clew_tag_unknown;
					break;
				}
				*pfx++ = *opr;
				*pfx = clew_tag_unknown;
				*opr-- = clew_tag_unknown;
			}
		} else if (*cur < *(opr - 1)) {
			/* if the incoming symbol has higher precedence than the top of the stack,
			 * push it on the stack */
			*opr++ = *cur;
			*opr = clew_tag_unknown;
		} else if (*cur == *(opr - 1)) {
			/* if the incoming symbol has equal precedence with the top of the stack,
			 * use association. If the association is left to right, pop and print the
			 * top of the stack and then push the incoming operators. If the association
			 * is right to left, push the incoming operators */
			*pfx++ = *--opr;
			*opr++ = *cur;
			*pfx = clew_tag_unknown;
			*opr = clew_tag_unknown;
		} else if (*cur > *(opr - 1)) {
			/* if the incoming symbol has lower precedence than the symbol on the top of
			 * the stack, pop the stack and print the top operator. Then test the incoming
			 * operator against the new top of stack */
			*pfx++ = *--opr;
			cur--;
			*pfx = clew_tag_unknown;
			*opr = clew_tag_unknown;
		}
	}
	/* at the end of the expression, pop and print all operators on the stack.
	 * no parentheses should remain */
	clew_debugf("end:");
	clew_debugf("  operator:");
	for (opr = operators; *opr != clew_tag_unknown; opr++) {
		clew_debugf("    %d - %s", *opr, clew_tag_string(*opr));
	}
	clew_debugf("  postfix:");
	for (pfx = postfix; *pfx != clew_tag_unknown; pfx++) {
		clew_debugf("    %d - %s", *pfx, clew_tag_string(*pfx));
	}
	while (--opr >= operators) {
		*pfx++ = *opr;
	}
	*pfx = clew_tag_unknown;
	expression->count = pfx - postfix;
	clew_debugf("postfix:");
	clew_debugf("  count: %lld", expression->count);
	for (pfx = postfix; *pfx != clew_tag_unknown; pfx++) {
		if ((*pfx == clew_tag_internal_expression_group_start) ||
		    (*pfx == clew_tag_internal_expression_group_end)) {
			clew_errorf("invalid expression: '%s'", from);
			goto bail;
		}
		clew_debugf("  %s", clew_tag_string(*pfx));
	}
	/* validate */
	for (i = 0, pfx = postfix; *pfx != clew_tag_unknown; pfx++) {
		switch (*pfx) {
			case clew_tag_internal_expression_and:
				i -= 2;
				if (i < 0) {
					clew_errorf("expression is not valid: %d", i);
					goto bail;
				}
				i += 1;
				break;
			case clew_tag_internal_expression_or:
				i -= 2;
				if (i < 0) {
					clew_errorf("expression is not valid: %d", i);
					goto bail;
				}
				i += 1;
				break;
			case clew_tag_internal_expression_not:
				i -= 1;
				if (i < 0) {
					clew_errorf("expression is not valid: %d", i);
					goto bail;
				}
				i += 1;
				break;
			case clew_tag_internal_expression_any:
				break;
			default:
				i += 1;
				break;
		}
	}
	if (strlen(from) > 0 && i != 1) {
		clew_errorf("expression is not valid: %d", i);
		goto bail;
	}
	free(in);
	free(operators);
	free(current);
	return expression;
bail:	if (expression != NULL) {
		if (expression->text != NULL) {
			free(expression->text);
		}
		free(expression);
	}
	if (operators != NULL) {
		free(operators);
	}
	if (current != NULL) {
		free(current);
	}
	if (in != NULL) {
		free(in);
	}
	return NULL;
}

const char * clew_expression_text (struct clew_expression *expression)
{
	if (expression == NULL) {
		return NULL;
	}
	return expression->text;
}

struct clew_expression * clew_expression_create (const char *expression)
{
	int wild;
	char *ptr;
	char *tmp;
	char *end;
	char *str;
	const char *exp;
	struct clew_expression *rc;
	if (unlikely(expression == NULL)) {
		clew_errorf("expression is null");
		return NULL;
	}
#if 0
	size_t l;
	l = strlen(expression);
	if (unlikely(l == 0)) {
		clew_errorf("expression is empty");
		return NULL;
	}
#endif
	str = (char *) malloc(EXPRESSION_SIZE_MAX);
	if (unlikely(str == NULL)) {
		clew_errorf("can not allocate memory");
		return NULL;
	}
	ptr = str;
	end = ptr + EXPRESSION_SIZE_MAX;
	*ptr = '\0';
	/* add spaces for each operator */
	wild = 0;
	for (exp = expression; *exp && ptr < end; exp++) {
		if (*exp == '*') {
			wild = 1;
		}
		if ((*exp == '(') || *exp == ')') {
			*ptr++ = ' ';
		}
		*ptr++ = *exp;
		if ((*exp == '(') || *exp == ')') {
			*ptr++ = ' ';
		}
	}
	if (ptr >= end) {
		clew_errorf("not enough memory");
		free(str);
		return NULL;
	}
	*ptr = '\0';
	/* resolve '*' */
	char *spc;
	if (wild != 0) {
		tmp = malloc(EXPRESSION_SIZE_MAX);
		if (unlikely(tmp == NULL)) {
			clew_errorf("can not allocate memory");
			free(str);
			return NULL;
		}
		*tmp = '\0';
wild_restart:
		ptr = str;
		end = ptr + EXPRESSION_SIZE_MAX;
		clew_debugf("str: '%s'", str);
		for (ptr = str, spc = str; *ptr; ptr++) {
			if (*ptr == ' ') {
				spc = ptr + 1;
				continue;
			}
			if (ptr > str && *(ptr - 1) == '_' && *ptr == '*' && (*(ptr + 1) == ' ' || *(ptr + 1) == '\0')) {
				int l;
				snprintf(tmp, MIN(EXPRESSION_SIZE_MAX, spc - str + 1), "%s", str);
				l = strlen(tmp);
				clew_debugf("tmp: '%s', spc: '%s'", tmp, spc);
				*(ptr - 1) = '\0';
				if (clew_tags_group_string(spc) != NULL) {
#if 0
					uint32_t *tag;
					snprintf(tmp + l, EXPRESSION_SIZE_MAX - l, "%s", " ( ");
					l = strlen(tmp);
					for (tag = clew_tags_group_string(spc); *tag != clew_tag_unknown; tag++) {
						snprintf(tmp + l, EXPRESSION_SIZE_MAX - l, "%s%s", (tag != clew_tags_group_string(spc)) ? " or " : " ", clew_tag_string(*tag)); l = strlen(tmp);
					}
					snprintf(tmp + l, EXPRESSION_SIZE_MAX - l, "%s", " ) ");
					l = strlen(tmp);
#else
					snprintf(tmp + l, EXPRESSION_SIZE_MAX - l, "internal_expression_any group_%s ", spc);
					l = strlen(tmp);
#endif
				} else {
					clew_errorf("invalid search: %s", spc);
					free(tmp);
					free(str);
					return NULL;
				}
				snprintf(tmp + l, EXPRESSION_SIZE_MAX - l, "%s", ptr + 1);
				l = strlen(tmp);
				free(str);
				str = strdup(tmp);
				if (str == NULL) {
					clew_errorf("can not allocate memory");
					free(tmp);
					return NULL;
				}
				goto wild_restart;
			}
		}
		if (ptr >= end) {
			clew_errorf("not enough memory");
			free(tmp);
			free(str);
			return NULL;
		}
		*ptr = '\0';
		free(tmp);
		clew_debugf("%s", str);
	}
	/* remove double spaces */
	for (ptr = str, tmp = str; *tmp; tmp++) {
		*ptr++ = *tmp;
		if (isspace(*tmp)) {
			while (isspace(*tmp)) {
				tmp++;
			}
			tmp--;
		}
	}
	*ptr = '\0';
	/* remove trailing space */
	while (ptr > str && isspace(*--ptr)) {
		*ptr = '\0';
	}
	/* end with space, required for correct word count */
	*++ptr = ' ';
	*++ptr = '\0';
	ptr = str;
	/* skip leading space */
	while (isspace(*ptr)) {
		ptr++;
	}
	clew_debugf("expression: '%s'", expression);
	clew_debugf("str: '%s'", ptr);
	rc = clew_expression_create_actual(ptr);
	if (unlikely(rc == NULL)) {
		clew_errorf("can not create expression");
	}
	//clew_expression_create_actual("A and ( B or C and D ) or ( E ) ");
	//clew_expression_create_actual("A and B or C ");
	//clew_expression_create_actual("A and B and C ");
	free(str);
	return rc;
}

void clew_expression_destroy (struct clew_expression *expression)
{
	if (unlikely(expression == NULL)) {
		clew_errorf("expression is null");
		return;
	}
	free(expression->text);
	free(expression);
}

static inline int search_expression_match_and (int first, int second)
{
	return first && second;
}

static inline int search_expression_match_or (int first, int second)
{
	return first || second;
}

static inline int search_expression_match_not (int first)
{
	return !first;
}

int clew_expression_has (const struct clew_expression *expression, uint32_t tag)
{
	const uint32_t *postfix;
	if (unlikely(expression == NULL)) {
		clew_errorf("expression is null");
		return -1;
	}
	for (postfix = expression->postfix; *postfix != clew_tag_unknown; postfix++) {
		if (*postfix == tag) {
			return 1;
		}
	}
	return 0;
}

int clew_expression_match (
		const struct clew_expression *expression,
		void *context,
		int (*_and) (int first, int second),
		int (*_or) (int first, int second),
		int (*_not) (int first),
		int (*_has) (void *context, uint32_t tag))
{
	int rc;
	unsigned long long a;
	unsigned long long o;
	unsigned long long n;
	const uint32_t *postfix;

	int first;
	int second;
	int *pstack;
	int *stack;
	int _stack[64];

        rc = -1;
        stack = NULL;

	if (unlikely(expression == NULL)) {
		clew_errorf("expression is null");
		goto bail;
	}

	o = expression->ors;
	a = expression->ands;
	n = expression->nots;
	postfix = expression->postfix;

	if (expression->count == 0) {
		return 0;
	}
	if (expression->count <= sizeof(_stack) / sizeof(_stack[0])) {
		stack = _stack;
	} else {
		stack = (int *) malloc(sizeof(_stack[0]) * expression->count);
		if (unlikely(stack == NULL)) {
			clew_errorf("can not allocate memory");
			goto bail;
		}
	}

	if (_and == NULL) {
		_and = search_expression_match_and;
	}
	if (_or == NULL) {
		_or = search_expression_match_or;
	}
	if (_not == NULL) {
		_not = search_expression_match_not;
	}
	if (_has == NULL) {
                clew_errorf("has is invalid");
                goto bail;
	}

	clew_debugf("match begin");
	for (pstack = stack; *postfix != clew_tag_unknown; postfix++) {
		clew_debugf("  check: %s", clew_tag_string(*postfix));
		switch (*postfix) {
			/* if the item is an operator (and,or,not) then
			 *   pop two numbers off the stack
			 *   make a calculation:  the second number
			 *   popped-operator-first number
			 *   push the result on the stack
			 */
			case clew_tag_internal_expression_and:
				first = *--pstack;
				second = *--pstack;
				*pstack++ = _and(first, second);
				a -= 1;
				break;
			case clew_tag_internal_expression_or:
				first = *--pstack;
				second = *--pstack;
				*pstack++ = _or(first, second);
				o -= 1;
				break;
			case clew_tag_internal_expression_not:
				first = *--pstack;
				*pstack++ = _not(first);
				n -= 1;
				break;
			case clew_tag_internal_expression_any:
			{
				const uint32_t *tags;
				tags = clew_tags_group_value(*++postfix);
				while (*tags != clew_tag_unknown) {
					if (_has(context, *tags)) {
						break;
					}
					tags++;
				}
				*pstack++ = (*tags != clew_tag_unknown);
				break;
			}
			default:
				*pstack++ = !!_has(context, *postfix);
				break;
		}
		if (n == 0) {
			if (a == 0) {
				if (*(pstack - 1) != 0) {
					goto out;
				}
			} else if (o == 0) {
				if (*(pstack - 1) == 0) {
					goto out;
				}
			}
		}
	}

	if (unlikely(pstack - stack != 1)) {
		clew_errorf("invalid, count: %lld", expression->count);
		rc = -1;
		goto bail;
	}

out:	clew_debugf("  count: %lld, out: %ld", expression->count, postfix - expression->postfix);
	rc = *(pstack - 1);
bail:	if (stack != NULL && stack != _stack) {
		free(stack);
	}
	return rc;
}

int clew_expression_compare (const struct clew_expression *first, const struct clew_expression *second)
{
	if (first == NULL && second == NULL) {
		return 0;
	} else if (first == NULL) {
		return -1;
	} else if (second == NULL) {
		return 1;
	} else if (first->count < second->count) {
		return -1;
	} else if (first->count > second->count) {
		return 1;
	}
	return memcmp(first->postfix, second->postfix, first->count * sizeof(uint32_t));
}
