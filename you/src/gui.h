#ifndef GUIH
#define GUIH

#define NK_INCLUDE_FIXED_TYPES
#define NK_INCLUDE_DEFAULT_ALLOCATOR
#define NK_INCLUDE_STANDARD_VARARGS
#include "nuklear.h"
#include "nuklear_xlib.h"
#include <stdio.h>
#include <stdbool.h>
#define ROWHEIGHT 20

extern int wnkwidth;
extern int wnkheight;

typedef struct XWindow XWindow;
struct XWindow {
    Display *dpy;
    Window root;
    Visual *vis;
    Colormap cmap;
    XWindowAttributes attr;
    XSetWindowAttributes swa;
    Window win;
    int screen;
    XFont *font;
    unsigned int width;
    unsigned int height;
    Atom wm_delete_window;
};

void gui(struct nk_context *ctx);
struct nk_context *createctx();


#endif //GUIH
