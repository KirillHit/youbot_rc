#ifndef ERRORS
#define ERRORS

#include <errno.h>
#include "gui.h"

void errset(const char *msg, int code);
void errgui(struct nk_context *ctx);
void errinit(int wnkwidth, int wnkheight);

#endif //ERRORS
