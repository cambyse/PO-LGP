
#include <X11/Xlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <Core/util.h>

/* first include the standard headers that we're likely to need */
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xresource.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void testKeyInput(){
  int screen_num, width, height;
  unsigned long background, border;
  Window win;
  XEvent ev;
  Display *dpy;

  /* First connect to the display server */
  dpy = XOpenDisplay(NULL);
  if (!dpy) {fprintf(stderr, "unable to connect to display\n"); }

  /* these are macros that pull useful data out of the display object */
  /* we use these bits of info enough to want them in their own variables */
  screen_num = DefaultScreen(dpy);
  background = BlackPixel(dpy, screen_num);
  border = WhitePixel(dpy, screen_num);

  width = 40; /* start with a small window */
  height = 40;

  win = XCreateSimpleWindow(dpy, DefaultRootWindow(dpy), /* display, parent */
                            0,0, /* x, y: the window manager will place the window elsewhere */
                            width, height, /* width, height */
                            2, border, /* border width & colour, unless you have a window manager */
                            background); /* background colour */

  /* tell the display server what kind of events we would like to see */
  XSelectInput(dpy, win, ButtonPressMask|StructureNotifyMask|KeyPressMask|KeyReleaseMask|KeymapStateMask);

  /* okay, put the window on the screen, please */
  XMapWindow(dpy, win);

  /* as each event that we asked about occurs, we respond.  In this
   * case we note if the window's shape changed, and exit if a button
   * is pressed inside the window */
  while(1){
    XNextEvent(dpy, &ev);
    switch(ev.type){
      case KeymapNotify:
        XRefreshKeyboardMapping(&ev.xmapping);
        break;
      case KeyPress: break; /* ignore these */
      case KeyRelease:
      {
        char string[25];
        int len;
        KeySym keysym;
        len = XLookupString(&ev.xkey, string, 25, &keysym, NULL);
        cout <<"K:" <<string<<' ' <<ev.xkey.keycode <<endl;
      }
        break;
      case ButtonPress:
        XCloseDisplay(dpy);
        return;
    }
  }
}

int mywait(){
  cout <<"wait..." <<flush;
  int screen_num, width, height;
  unsigned long background, border;
  Window win;
  XEvent ev;
  Display *dpy;

  /* First connect to the display server */
  dpy = XOpenDisplay(NULL);
  if (!dpy) {fprintf(stderr, "unable to connect to display\n"); }

  /* these are macros that pull useful data out of the display object */
  /* we use these bits of info enough to want them in their own variables */
  screen_num = DefaultScreen(dpy);
  background = BlackPixel(dpy, screen_num);
  border = WhitePixel(dpy, screen_num);

  width = 40; /* start with a small window */
  height = 40;

  win = XCreateSimpleWindow(dpy, DefaultRootWindow(dpy), /* display, parent */
                            10,10, /* x, y: the window manager will place the window elsewhere */
                            200,200, /* width, height */
                            2, 0x000000, /* border width & colour, unless you have a window manager */
                            0xaaaaaa); /* background colour */
  //  w = XCreateSimpleWindow(d, RootWindow(d, s), 10, 10, 200, 200, 10,
  //                          0x000000, 0xaab0b0);

  /* tell the display server what kind of events we would like to see */
  XSelectInput(dpy, win, KeyPressMask|KeyReleaseMask);

  /* okay, put the window on the screen, please */
  XMapWindow(dpy, win);

  mlr::String str="PRESS KEY";
  GC gc = XCreateGC(dpy, win, 0, NULL);
  XSetFont(dpy, gc,  XLoadFont(dpy,"-adobe-helvetica-medium-r-*-*-*-100-*-*-*-*-*-*"));
  XSetBackground(dpy, gc, 0x8080aa);
  XSetForeground(dpy, gc, 0x000000);
  XClearWindow(dpy,win);
  XFillRectangle(dpy, win, gc, 20, 20, 10, 10);
  XDrawString(dpy, win, gc, 50, 50, str.p, str.N);
  XFlush(dpy);

  XNextEvent(dpy, &ev);
  if(ev.type==KeyPress){ //KeyRelease:
    char string[4];
    KeySym keysym;
    XLookupString(&ev.xkey, string, 4, &keysym, NULL);
    cout <<string<<' ' <<ev.xkey.keycode <<endl;
  }
}


void testDisplay(){
  Display *d;
  Window w;
  int s;

  /* open connection with the server */
  d = XOpenDisplay(NULL);
  CHECK(d, "Cannot open display");

  s = DefaultScreen(d);

  /* create window */
  w = XCreateSimpleWindow(d, RootWindow(d, s), 10, 10, 200, 200, 10,
                          0x000000, 0xaab0b0);

  /* map (show) the window */
  XMapWindow(d, w);

  // Create a "Graphics Context"
  GC gc = XCreateGC(d, w, 0, NULL);

  XSetFont(d, gc,  XLoadFont(d,"-adobe-helvetica-medium-r-*-*-*-100-*-*-*-*-*-*"));
  XSetBackground(d, gc, 0x8080aa);
  XSetForeground(d, gc, 0x000000);

  mlr::String txt;

  for(uint i=0;i<10;i++){
    mlr::wait(1.);
    txt.clear() <<"hallo " <<i;

    XClearWindow(d,w);
    XFillRectangle(d, w, gc, 20, 20, 10, 10);
    XDrawString(d, w, gc, 50, 50, txt.p, txt.N);
    //XDrawImageString(d, w, gc, 50, 70, txt.p, txt.N);

    XFlush(d);
  }
  /* close connection to server */
  XCloseDisplay(d);

  return ;
}


int mywait2(){
  mlr::String txt="PRESS KEY";
  int key=0;

  Display *disp = XOpenDisplay(NULL);
  CHECK(disp, "Cannot open display");

  Window win = XCreateSimpleWindow(disp, DefaultRootWindow(disp),
                                   10, 10, 165, 24,
                                   2, 0x000000, 0xaaaaaa);
  XSelectInput (disp, win, KeyPressMask | ExposureMask );
  XMapWindow(disp, win);

  GC gc = XCreateGC(disp, win, 0, NULL);
  XSetFont(disp, gc,  XLoadFont(disp,"-adobe-courier-bold-r-*-*-*-220-*-*-*-*-*-*"));
  XSetForeground(disp, gc, 0x606060);

  bool quit=false;
  for(;!quit;){
    XEvent ev;
    XNextEvent(disp, &ev);
    switch(ev.type){
      case Expose:      /* Expose-Event => Bild zeichnen */
        if (ev.xexpose.count == 0) {
          XDrawString(disp, win, gc, 0, 20, txt.p, txt.N);
          XFlush(disp);
        }
        break;
      case KeyPress:
        char string[4];
        XLookupString(&ev.xkey, string, 4, NULL, NULL);
        key = string[0];
//            cout <<string<<' ' <<ev.xkey.keycode <<endl;
//        printf ("Tastaturaktion\n");
        quit=true;
        break;
    }
  }

  XCloseDisplay(disp);

  return key;
}


int main(int argn, char** argv){
  int k=mlr::x11_getKey();
  cout <<k <<':' <<char(k) <<endl;
  //  testDisplay();
}

