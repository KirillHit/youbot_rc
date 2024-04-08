#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "error.h"
#include "gui.h"

static int errcode; 
static const char *errmsg;
static struct nk_rect errrect;


void errset(const char *msg, int code)
{
    errmsg = msg;
    errcode = code;   
}

void errgui(struct nk_context *ctx)
{
    if (*errmsg) {
        nk_popup_begin(ctx, NK_POPUP_STATIC, "err", 0, errrect);
        nk_layout_row_dynamic(ctx, ROWHEIGHT, 1);
        if (errcode)
            nk_labelf(ctx, NK_TEXT_LEFT, "%s (%d):  %s", errmsg, errcode, strerror(errcode));
        else
            nk_label(ctx, errmsg, NK_TEXT_LEFT);
        if (nk_button_label(ctx, "Ок")) {
            errset("", 0);
            nk_popup_close(ctx);
        }
        nk_popup_end(ctx);
    }
}


void errinit(int wnkwidth, int wnkheight)
{
    float errwidth = 380.f; //280
    float errheight = 65.f; //70
    float errx = (wnkwidth-errwidth)/2;
    float erry = (wnkheight-errheight)/2;
    errrect = (struct nk_rect){errx, erry, errwidth, errheight};
    errset("", 0);
}



