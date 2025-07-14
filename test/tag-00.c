
#include <stdio.h>
#include <stdlib.h>
#include "tag.h"

int main (int argc, char *argv[])
{
        (void) argc;
        (void) argv;

        clew_tag_init();
        fprintf(stdout, "check: %s\n", argv[1]);
        fprintf(stdout, "  tag: %d\n", clew_tag_value(argv[1]));
        fprintf(stdout, "  tag: %s\n", clew_tag_string(clew_tag_value(argv[1])));
        return 0;
}
