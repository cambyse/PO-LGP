
 #include <X11/Xlib.h>
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>

#include <MT/util.h>
 
 int main(void) {
   Display *d;
   Window w;
   int s;
 
                        /* open connection with the server */
   d = XOpenDisplay(NULL);
   if (d == NULL) {
     fprintf(stderr, "Cannot open display\n");
     exit(1);
   }
 
   s = DefaultScreen(d);
 
                        /* create window */
   w = XCreateSimpleWindow(d, RootWindow(d, s), 10, 10, 200, 200, 10,
                           0x000000, 0xb0b0b0);
 
                        /* map (show) the window */
   XMapWindow(d, w);

   // Create a "Graphics Context"

   GC gc = XCreateGC(d, w, 0, NULL);

   XSetFont(d, gc,  XLoadFont(d,"-adobe-helvetica-medium-r-*-*-*-100-*-*-*-*-*-*"));
   XSetBackground(d, gc, 0x808080);
   XSetForeground(d, gc, 0x000000);


   MT::String txt;

   for(uint i=0;i<10;i++){
     MT::wait(1.);
     txt.clr() <<"hallo " <<i;
     
     XClearWindow(d,w);
     XFillRectangle(d, w, gc, 20, 20, 10, 10);
     XDrawString(d, w, gc, 50, 50, txt.p, txt.N());
     //XDrawImageString(d, w, gc, 50, 70, txt.p, txt.N());

     XFlush(d);
   }
                  /* close connection to server */
   XCloseDisplay(d);
 
   return 0;
 }

