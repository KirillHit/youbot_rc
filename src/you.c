#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <limits.h>
#include <locale.h>  

#include <X11/Xutil.h>         

#include "youbot_rc/gui.h"    
#include "youbot_rc/error.h"   

extern struct XWindow xw;

int main(void)
{
    bool run = true;
    long t1;
    long dt;
    struct nk_context *ctx = createctx();
    errinit(wnkwidth, wnkheight);
    XColor black;
    black.red = black.green = black.blue = 0;
    Pixmap bitmapNoData;
    static char noData[] = { 0,0,0,0,0,0,0,0 };
    
    bitmapNoData = XCreateBitmapFromData(xw.dpy, xw.win, noData, 8, 8);
    
    while(run)
    {
        XEvent evt;
        nk_input_begin(ctx);
        while (XPending(xw.dpy)) {
            XNextEvent(xw.dpy, &evt);
            if (evt.type == ClientMessage) 
                run = false;
            if (XFilterEvent(&evt, xw.win)) continue;
            nk_xlib_handle_event(xw.dpy, xw.screen, xw.win, &evt);
        }
        gui(ctx);
        nk_input_end(ctx);
        
        XClearWindow(xw.dpy, xw.win);
        nk_xlib_render(xw.win, nk_rgb(169, 169, 169));
        XFlush(xw.dpy);
        
    }
    XUngrabPointer(xw.dpy,CurrentTime);
    XCloseDisplay(xw.dpy);    
    return 0;
}
