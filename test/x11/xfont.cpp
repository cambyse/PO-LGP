/* xfont.c */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <X11/keysym.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>

#define MT_IMPLEMENTATION
#include <MT/util.h>

#define WINDOW_WIDTH 600
#define WINDOW_HEIGHT 450

/* Funktionsprototypen */
static int create_window (void);
static void process_event (XEvent report);
static void eventloop (void);
static void close_window (void);
extern int time_diff (void);
static void draw_graphics (void);
static void print_text (GC, int, int, char *, char *);
static void print_font_demo (void);

/* Globale Variablen */
static unsigned int width, height;
static Display *display;
static int screen;
static int depth;
static Window win;

/* Schriftarten */
static char *my_fonts[] = {
   "-adobe-times-bold-r-normal--18*",
   "-adobe-times-bold-r-normal--12*",
   "-b&h-lucida-medium-r-normal-*-*-180-*-*-*-*-*-*",
   "-*-times-*-*-*-*-*-240-*-*-*-*-*-*"
};
enum { ADOBE_BOLD_18, ADOBE_BOLD_12, LUCIA_NORMAL, TIMES };

/* Funktion erzeugt ein Fenster */
static int create_window (void) {
  char *window_name = "Fontdemo";
  char *icon_name   = "window";
  static XSizeHints size_hints;
  Window rootwin;

  width = WINDOW_WIDTH;
  height = WINDOW_HEIGHT;

  /* X-Sitzung öffnen */
  display = XOpenDisplay (NULL);
  /* Fehlerüberprüfung */
  if (display == NULL) {
    printf ("Verbindung zum X-Server fehlgeschlagen?!?\n");
    exit(EXIT_FAILURE);
  }
  /* Standardbildschirm eines Displays */
  screen = XDefaultScreen (display);
  /* Standardtiefe des screen - Anzahl der Ebenen */
  depth = XDefaultDepth (display, screen);
  rootwin = RootWindow (display, screen);
  win = XCreateSimpleWindow ( display, rootwin, 10, 10,
                              width, height, 5,
                              BlackPixel (display, screen),
                              WhitePixel (display, screen) );
  size_hints.flags = PSize | PMinSize | PMaxSize;
  size_hints.min_width =  width;
  size_hints.max_width =  width;
  size_hints.min_height = height;
  size_hints.max_height = height;
  XSetStandardProperties ( display, win, window_name, icon_name,
                           None, 0, 0, &size_hints );
  XSelectInput (display, win,
                ButtonPressMask | KeyPressMask | ExposureMask );
  XMapWindow (display, win);
  return 1;
}

static void close_window (void) {
  XCloseDisplay (display);
}

static void eventloop (void) {
  XEvent xev;
  int num_events;

  XFlush (display);
  num_events = XPending (display);
  while ((num_events != 0)) {
    num_events--;
    XNextEvent (display, &xev);
    process_event (xev);
  }
}

static void process_event (XEvent report) {
  switch (report.type) {

  case Expose:      /* Expose-Event => Bild zeichnen */
    if (report.xexpose.count == 0) {
      draw_graphics ();
    }
    break;
  case KeyPress:
    printf ("Tastaturaktion\n");
    break;
  case ButtonPressMask:
    /* Rechte Maustaste gedrückt? */
    if (report.xbutton.button == 3)
      print_font_demo ();
    else
      printf ("Mausaktion\n");
    break;
  default:
    break;
  }
}

static void draw_graphics (void) {
  GC mygc;                      /* neu: Grafikkontext */
  unsigned long black, white;   /* Nummern der Farben */
  int i;
  XPoint punkte[50];            /* Punkte-Schar */
  XPoint poly[4];               /* 4 Punkte fuer Polygon */

  /* Ein paar Felder belegen                   */
  /* Ein Schräge mithilfe von Punkten Zeichnen */
  for (i = 0; i < 50; i++) {
    punkte[i].x = 470 + i;
    punkte[i].y = 200 + i;
  }
  /* geschlossenes Dreieck */
  poly[0].x = 200;
  poly[0].y = 200;
  poly[1].x = 300;
  poly[1].y = 300;
  poly[2].x = 400;
  poly[2].y = 200;
  poly[3].x = 200;
  poly[3].y = 200;
  /* Fenster oben drauf anzeigen */
  XMapRaised (display, win);

  /* Farbnummern besorgen */
  black = BlackPixel (display, screen);
  white = WhitePixel (display, screen);

  /* Graphical Context erzeugen, Farben eintragen */
  mygc = XCreateGC (display, win, 0, 0);
  XSetForeground (display, mygc, black);
  XSetBackground (display, mygc, white);
  /* sicherheitshalber alles löschen */
  XClearWindow (display, win);

  /* Ein Rechteck zeichnen */
  XDrawRectangle (display, win, mygc, 10, 50, 50, 50);
  print_text (mygc, 10, 320, "1. XDrawRectangle()",
     my_fonts[ADOBE_BOLD_12]);
  /* Ein Rechteckigen Bereich mit Farbe füllen */
  XFillRectangle (display, win, mygc, 110, 50, 50, 50);
  print_text (mygc, 110, 240, "2. XFillRectangle()",
     my_fonts[ADOBE_BOLD_12]);
  /* Einen Kreis zeichnen */
  XDrawArc (display, win, mygc, 210, 50, 50, 50, 0, 360 * 64);
  print_text (mygc, 210, 320, "3. XDrawArc()",
    my_fonts[ADOBE_BOLD_12]);
  /* Eine Kreisbereich füllen */
  XFillArc (display, win, mygc, 310, 50, 50, 50, 0, 360 * 64);
  print_text (mygc, 310, 240, "4. XFillArc()",
    my_fonts[ADOBE_BOLD_12]);
  /* Eine Ellipse zeichnen */
  XDrawArc (display, win, mygc, 410, 50, 60, 40, 0, 360 * 64);
  print_text (mygc, 410, 320, "5. XDrawArc()",
    my_fonts[ADOBE_BOLD_12]);
  /* Einen Ellipsenbereich füllen */
  XFillArc (display, win, mygc, 510, 50, 60, 40, 0, 360 * 64);
  print_text (mygc, 510, 240, "6. XFillArc()",
    my_fonts[ADOBE_BOLD_12]);
  /* Einen Halbkreis zeichnen */
  XDrawArc (display, win, mygc, 10, 200, 60, 40, 90*64, 180*64);
  print_text (mygc, 10, 480, "7. XDrawArc()",
    my_fonts[ADOBE_BOLD_12]);
  /* Einen Halbkreis füllen */
  XFillArc (display,win, mygc, 110, 200, 60, 40, 90*64, 180*64);
  print_text (mygc, 110, 380, "8. XFillArc()",
    my_fonts[ADOBE_BOLD_12]);
 /* ausgefuelltes Polygon bzw. Dreieck */
  XFillPolygon(display, win, mygc, poly, 4, 0, CoordModeOrigin);
  print_text (mygc, 250, 520, "9. XFillPolygon()",
     my_fonts[ADOBE_BOLD_12]);
  /* Eine Schräge Linie mit Punkten */
  XDrawPoints (display, win, mygc, punkte, 50, CoordModeOrigin);
  print_text (mygc, 470, 380, "10. XDrawPoints()",
     my_fonts[ADOBE_BOLD_12]);
  print_text (mygc, 150, 590,
    "Mit Rechter Maustaste zum Font-Demo",
    my_fonts[ADOBE_BOLD_18] );

  XFlush (display);
  XFreeGC (display, mygc);
}

static void
print_text (GC mygc, int x, int y, char *string, char *myfonts){
  Font myfont;      /* Schriftarten */
  char *fname = myfonts;
  /* Schriftart laden */
  myfont = XLoadFont (display, fname);
 /* im Grafikkontext eintragen */
  XSetFont (display, mygc, myfont);
  XDrawString ( display, win, mygc, x, y - 200,
                string, strlen (string) );
  /* Schriftart wieder freigeben */
  XUnloadFont (display, myfont);
}

static void print_font_demo (void) {
  int fcount;                /* Zahl der Fonts von XListFont  */
  char **fontlist;           /* Zeiger auf Feld mit Fontnamen */
  Font headerfont, myfont;   /* Text-Fonts                    */
  int fontnr;                /* Font-Nummer                   */
  char *fname;               /* Text mit Font-Namen           */
  char clist[256];           /* 256 Zeichen                   */
  XFontStruct *fontinfo, *headerinfo;   /* Font-Informationen */
  int headerlength;          /* Textlaenge Header             */
  int headerwidth;           /* Textbreite Header             */
  int headerheight;          /* Hoehe des Header-Fonts        */
  int fontheight;            /* Hoehe des Zufalls-Fonts       */
  int width;             /* Breite des Textes mit Zufallsfont */
  int i;
  GC mygc;

  mygc = XCreateGC (display, win, 0, 0);
  /* Fontliste holen */
  fontlist = XListFonts (display, "*", 5000, &fcount);
  printf ("%d Schriften gefunden!\n", fcount);
  /* Fenster loeschen */
  XClearWindow (display, win);

  /* Schriftart fuer die Ueberschrift */
  headerfont =
     XLoadFont (display, "-adobe-times-bold-r-normal--18*");
  headerinfo = XQueryFont (display, headerfont);
  headerheight = (*headerinfo).ascent + (*headerinfo).descent;
  for (i = 0; i < 256; i++)
    clist[i] = i;   /* 256 ASCII-Zeichen definieren */

  /* zufaelligen Font mit modulo! */
  fontnr = (rand ()) % fcount;
  /* das ist sein Name */
  fname = fontlist[fontnr];

  /* Header schreiben:  Name des Fonts zentriert oben        */
  /*                    mit Schrifttyp = headerfont          */
  headerlength = strlen (fname);       /* Laenge des Headers */
  XSetFont (display, mygc, headerfont);/* Header-Font setzen */
  headerwidth = XTextWidth (headerinfo, fname, headerlength);

  XDrawString (
     display, win, mygc,        /* Header darstellen */
     (600 - headerwidth)/2, headerheight*3/2, /* x,y */
               fname, headerlength  );

  /* clist mit 256 Zeichen schreiben in neuer Schrift        */
  myfont = XLoadFont (display, fname);         /* Font laden */
  fontinfo = XQueryFont (display, myfont); /* Infos besorgen */
  XSetFont (display, mygc, myfont);       /* in GC eintragen */

  fontheight = (*fontinfo).ascent + (*fontinfo).descent;
  for (i = 0; i < 4; i++) {
    width = XTextWidth (fontinfo, &clist[64 * i], 64);
    XDrawString (
       display, win, mygc,
       (600 - width)/2, (3*headerheight+i*fontheight),
        &clist[64*i], 64 );
  }
  print_text (mygc, 150, 590,
     "Mit Rechter Maustaste neue Schrift laden",
     my_fonts[ADOBE_BOLD_18]);

  XUnloadFont (display, myfont);
  XFreeGC (display, mygc);
}

 int main (int argc, char **argv) {
  int quit = 0;
  int pause = 0;
  int td;

  create_window ();
  while (!quit) {
    //MT::wait();
    eventloop ();
  }
  close_window ();
  return EXIT_SUCCESS;
}
