#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "youbot_rc/gui.h"
#include "youbot_rc/error.h"

#define NK_IMPLEMENTATION
#include "youbot_rc/nuklear.h"
#define NK_XLIB_IMPLEMENTATION
#include "youbot_rc/nuklear_xlib.h"

struct XWindow xw;
int wnkheight;
int wnkwidth;

static int errcode;
static const char *errmsg;
static struct nk_rect errrect;

//Установка заголовка на русском языке

static void set_title(XWindow *xw, const char *title)
{
    int len = strlen(title);

#if defined(X_HAVE_UTF8_STRING)
    Xutf8SetWMProperties(xw->dpy, xw->win, title, title,
        NULL, 0, NULL, NULL, NULL);
#else
    XmbSetWMProperties(xw->dpy, xw->win, title, title,
        NULL, 0, NULL, NULL, NULL);
#endif

    Atom net_wm_name = XInternAtom(xw->dpy, "_NET_WM_NAME", False);
    Atom net_wm_icon = XInternAtom(xw->dpy, "_NET_WM_ICON", False);
    Atom utf8_string = XInternAtom(xw->dpy, "UTF8_STRING", False);

    XChangeProperty(xw->dpy, xw->win, net_wm_name, utf8_string, 8,
        PropModeReplace, (unsigned char*)title, len);

    XChangeProperty(xw->dpy, xw->win, net_wm_icon, utf8_string, 8,
        PropModeReplace, (unsigned char*)title, len);

    XFlush(xw->dpy);
}

//Создание окна

struct nk_context *createctx() {
    setlocale(LC_ALL, "ru_RU.UTF-8");
    memset(&xw, 0, sizeof xw);
    xw.dpy = XOpenDisplay(NULL);
    xw.root = DefaultRootWindow(xw.dpy);
    xw.screen = XDefaultScreen(xw.dpy);
    xw.vis = XDefaultVisual(xw.dpy, xw.screen);
    xw.cmap = XCreateColormap(xw.dpy, xw.root, xw.vis, AllocNone);

    Atom actual_type;
    int actual_format;
    long nitems;
    long bytes;
    long *data;
    int status;
    status = XGetWindowProperty(
        xw.dpy,
        xw.root,
        XInternAtom(xw.dpy, "_NET_WORKAREA", True),
        0,
        ~0L,
        False,
        AnyPropertyType,
        &actual_type,
        &actual_format,
        &nitems,
        &bytes,
        (unsigned char**)&data);
    if (status != Success)
        exit(EXIT_FAILURE);
    int wndwidth = data[2];
    int wndheight = data[3];
    wnkwidth = wndwidth-5;
    wnkheight = wndheight-20;

    xw.swa.colormap = xw.cmap;
    xw.swa.event_mask =
        ExposureMask|KeyPressMask|KeyReleaseMask|
        ButtonPress|ButtonReleaseMask|ButtonMotionMask|
        Button1MotionMask|Button3MotionMask|Button4MotionMask|
        Button5MotionMask|PointerMotionMask|KeymapStateMask;
    xw.win = XCreateWindow(xw.dpy, xw.root, data[0], data[1],
        wndwidth, wndheight, 0,
        XDefaultDepth(xw.dpy, xw.screen), InputOutput,
        xw.vis, CWEventMask|CWColormap, &xw.swa);

    set_title(&xw, "Youbot remote control");
    XMapWindow(xw.dpy, xw.win);
    xw.wm_delete_window = XInternAtom(xw.dpy, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(xw.dpy, xw.win, &xw.wm_delete_window, 1);
    xw.width = wndwidth;
    xw.height = wndheight;

    XSizeHints hints;
    hints.flags = PMinSize|PMaxSize;
    hints.min_width = xw.width;
    hints.min_height = xw.height;
    hints.max_width = xw.width;
    hints.max_height = xw.height;
    XSetWMNormalHints(xw.dpy, xw.win, &hints);

    xw.font = nk_xfont_create(xw.dpy, "-*-fixed-medium-r-*-*-12-*-*-*-*-*-iso10646-*");
    return nk_xlib_init(xw.font, xw.dpy, xw.screen, xw.win, xw.width, xw.height);
}

//GUI
        
void gui(struct nk_context* ctx) {
        static char mesbuf_1[512];         //Размеры буфера указаны наобум, можно менять 
        static char mesbuf_2[512];
        static char mesbuf_3[512];
        static char mesbuf_hand_1[512];
        static char mesbuf_hand_2[512];
        static char mesbuf_hand_3[512];
        static char mesbuf_hand_4[512];
        static char mesbuf_hand_5[512];
        static int meslen_1;
        static int meslen_2;
        static int meslen_3;
        static int meslen_hand_1;
        static int meslen_hand_2;
        static int meslen_hand_3;
        static int meslen_hand_4;
        static int meslen_hand_5;
        static int checkbox_1 = 1, checkbox_2 = 1, checkbox_3 = 1;
        
        if (nk_begin(ctx, "main", nk_rect(0, 0, wnkwidth, wnkheight),0)) {
        
            //Меню верхнее 
               
        nk_menubar_begin(ctx);
        nk_layout_row_begin(ctx, NK_STATIC, ROWHEIGHT, 6);
        nk_layout_row_push(ctx, 80);
        if (nk_menu_begin_label(ctx, "Button 1", NK_TEXT_LEFT, nk_vec2(100, 40))){
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_menu_item_label(ctx, "Button 1.1", NK_TEXT_LEFT);
            nk_menu_end(ctx);
        }
        
        nk_layout_row_push(ctx, 80);
        if (nk_menu_begin_label(ctx, "Button 2", NK_TEXT_LEFT, nk_vec2(100, 80))){
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_menu_item_label(ctx, "Button 2.1", NK_TEXT_LEFT);
            nk_menu_item_label(ctx, "Button 2.2", NK_TEXT_LEFT);
            nk_menu_end(ctx);   
        }
        
        nk_layout_row_push(ctx, 80);
        if (nk_menu_begin_label(ctx, "Button 3", NK_TEXT_LEFT, nk_vec2(100, 40))){
             nk_layout_row_dynamic(ctx, 20, 1);
             if (nk_menu_item_label(ctx, "Button 3.1", NK_TEXT_LEFT));
            nk_menu_end(ctx);
        }
        
        nk_layout_row_push(ctx, 80);
        if (nk_menu_begin_label(ctx, "Button 4", NK_TEXT_LEFT, nk_vec2(100, 80))){
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_checkbox_label(ctx, "Checkbox 1", &checkbox_1);
            nk_checkbox_label(ctx, "Checkbox 2", &checkbox_2);
            nk_checkbox_label(ctx, "Checkbox 3", &checkbox_3);
            nk_menu_end(ctx);
        }
        
        nk_layout_row_push(ctx, 80);
        if (nk_menu_begin_label(ctx, "Button 5", NK_TEXT_LEFT, nk_vec2(100, 80))){
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_menu_item_label(ctx, "Button 5.1", NK_TEXT_LEFT);
            nk_menu_item_label(ctx, "Button 5.2", NK_TEXT_LEFT);
            nk_menu_end(ctx);
        }
        
        errgui(ctx);
        nk_menubar_end(ctx);

        //Поля для скоростей с вводом(автоматически записывается в указанный буфер)
        
        nk_layout_row_static(ctx, wnkheight/20, wnkwidth/6, 1);
        
        nk_layout_row_dynamic(ctx, wnkheight/20, 2);
        nk_label(ctx, "Linear speed x", NK_TEXT_CENTERED);
        nk_edit_string(ctx, NK_EDIT_BOX, mesbuf_1, &meslen_1, 512, nk_filter_default);
        nk_layout_row_dynamic(ctx, wnkheight/20, 2);
        nk_label(ctx, "Linear speed y", NK_TEXT_CENTERED);
        nk_edit_string(ctx, NK_EDIT_BOX, mesbuf_2, &meslen_2, 512, nk_filter_default);
        nk_layout_row_dynamic(ctx, wnkheight/20, 2);
        nk_label(ctx, "Angle speed z", NK_TEXT_CENTERED);
        nk_edit_string(ctx, NK_EDIT_BOX, mesbuf_3, &meslen_3, 512, nk_filter_default);
        
        //Кнопки на сжатие-разжатие клешни
        
        nk_label(ctx, "", NK_TEXT_CENTERED);
        nk_layout_row_dynamic(ctx, 20, 5);
        nk_button_label(ctx, "Squeeze claw");
        nk_label(ctx,"", NK_TEXT_CENTERED);
        nk_label(ctx,"", NK_TEXT_CENTERED);
        nk_label(ctx,"", NK_TEXT_CENTERED);
        nk_button_label(ctx, "Unclench claw");
        
        nk_label(ctx, "", NK_TEXT_CENTERED);
        
        //Поля ввода для управления рукой
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_label(ctx, "Управление рукой", NK_TEXT_CENTERED);
        
        nk_layout_row_static(ctx, wnkheight/20, wnkwidth/6, 1); 
        
        nk_layout_row_dynamic(ctx, wnkheight/20, 2);
        nk_label(ctx, "Axis 1", NK_TEXT_CENTERED);
        nk_edit_string(ctx, NK_EDIT_BOX, mesbuf_hand_1, &meslen_hand_1, 512, nk_filter_default);
        nk_layout_row_dynamic(ctx, wnkheight/20, 2);
        nk_label(ctx, "Axis 2", NK_TEXT_CENTERED);
        nk_edit_string(ctx, NK_EDIT_BOX, mesbuf_hand_2, &meslen_hand_2, 512, nk_filter_default);
        nk_layout_row_dynamic(ctx, wnkheight/20, 2);
        nk_label(ctx, "Axis 3", NK_TEXT_CENTERED);
        nk_edit_string(ctx, NK_EDIT_BOX, mesbuf_hand_3, &meslen_hand_3, 512, nk_filter_default);
        nk_layout_row_dynamic(ctx, wnkheight/20, 2);
        nk_label(ctx, "Axis 4", NK_TEXT_CENTERED);
        nk_edit_string(ctx, NK_EDIT_BOX, mesbuf_hand_4, &meslen_hand_4, 512, nk_filter_default);
        nk_layout_row_dynamic(ctx, wnkheight/20, 2);
        nk_label(ctx, "Axis 5", NK_TEXT_CENTERED);
        nk_edit_string(ctx, NK_EDIT_BOX, mesbuf_hand_5, &meslen_hand_5, 512, nk_filter_default);
        
        nk_end(ctx);
        }
        if (ctx->current)
            nk_end(ctx);
}
