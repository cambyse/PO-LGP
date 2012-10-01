//===========================================================================
//
// global monitor
//

#if 0 //use fltk window

struct sThreadInfoWin:public Fl_Double_Window {
  bool isOpen;
  //ofstream log;
  char outputbuf[200];
  sThreadInfoWin():Fl_Double_Window(0, 0, 600, 300, "processes") {
  }
  void draw();
};

ThreadInfoWin::ThreadInfoWin():Process("ThreadInfoX") {
  s=new sThreadInfoWin;
  s->isOpen=false;
}

ThreadInfoWin::~ThreadInfoWin() {
  if(s->isOpen) close();
  delete s;
}

void ThreadInfoWin::open() {
  //MT::open(log, "LOG.threads");
  Fl::visual(FL_DOUBLE|FL_INDEX);
  s->show();
  Fl::check();
  s->isOpen=true;
}

void ThreadInfoWin::close() {
  if(!s->isOpen) return;
  //XCloseDisplay(s->display);
  //s->log.close();
  s->isOpen=false;
}

void ThreadInfoWin::step() {
  if(!s->isOpen) open();
  s->redraw();
  Fl::wait(.1);
}

void sThreadInfoWin::draw() {
  //timer.cycleStart();
  Process *proc;
  sProcess *th;
  //Metronome *met;
  CycleTimer *ct;
  
  //-- graphical display
  uint i, y=20, x, len;
  //XClearWindow(fl_display, fl_window);
  fl_draw_box(FL_FLAT_BOX, 0, 0, w(), h(), FL_FOREGROUND_COLOR);
  fl_color(FL_BACKGROUND2_COLOR);
  fl_font(1, 10);
#define TEXT0(txt) \
  fl_draw(txt, x, y);
#define TEXT(form, val) \
  if((len=sprintf(outputbuf, form, val))){ fl_draw(outputbuf, x, y); }
#define TEXTTIME(dt) \
  if((len=sprintf(outputbuf, "%5.2f|%5.2f|%5.2f", dt, dt##Mean, dt##Max))){ fl_draw(outputbuf, x, y); }
  for_list(i, proc, globalProcesses) {
    th = proc->s;
    int state=th->threadCondition.state;
    x=5;
    TEXT("%4i", th->tid); x+=30;
    TEXT("%3i", th->threadPriority); x+=25;
    TEXT("%s", proc->name); x+=100;
    TEXT("%4i", th->timer.steps);  x+=30;
    if(state>0) { TEXT("%4i", state); } else switch (state) {
        case tsOPEN:    TEXT0("open");   break;
        case tsCLOSE:   TEXT0("close");  break;
        case tsLOOPING: TEXT0("loop");   break;
        case tsBEATING: TEXT0("beat");   break;
        case tsSYNCLOOPING: TEXT0("sync");   break;
        case tsIDLE:    TEXT0("idle");   break;
        default: TEXT0("undefined:");
      } x+=50;
    TEXTTIME(th->timer.cyclDt); x+=130;
    TEXTTIME(th->timer.busyDt); x+=130;
    y+=20;
  }
  y+=10;
  for_list(i, ct, globalCycleTimers) {
    x=5;
    TEXT("%2i", i); x+=25;
    TEXT("%s", ct->name); x+=100;
    TEXT("%4i", ct->steps); x+=30;
    TEXTTIME(ct->cyclDt); x+=130;
    TEXTTIME(ct->busyDt); x+=130;
    y+=20;
  }
#undef TEXT
#undef TEXTTIME
}

#else //use X directly

struct sThreadInfoWin {
  bool isOpen;
  Display *display;
  Window window;
  GC gc;
  //CycleTimer timer;
  //ofstream log;
  char outputbuf[200];
};

ThreadInfoWin::ThreadInfoWin():Process("ThreadInfoX") {
  s=new sThreadInfoWin;
  s->isOpen=false;
}

ThreadInfoWin::~ThreadInfoWin() {
  threadClose();
  delete s;
}

void ThreadInfoWin::open() {
  //MT::open(s->log, "LOG.threads");
  s->display = XOpenDisplay(NULL);
  if(!s->display) HALT("Cannot open display");
  s->window = XCreateSimpleWindow(s->display, DefaultRootWindow(s->display),
                                  10, 10, 600, 300, 1,
                                  0xffffff, 0x000000);
  XMapWindow(s->display, s->window);
  s->gc = XCreateGC(s->display, s->window, 0, NULL);
  XSetFont(s->display, s->gc,  XLoadFont(s->display, "-*-helvetica-*-r-*-*-*-*-*-*-*-*-*-*"));
  //-adobe-courier-medium-r-*-*-*-80-*-*-*-*-*-*"));
  XSetBackground(s->display, s->gc, 0x000000);
  XSetForeground(s->display, s->gc, 0xffffff);
  XWindowChanges change={1500, 700,  600, 300,  10, 0, 0};
  XConfigureWindow(s->display, s->window, CWX|CWY|CWWidth|CWHeight|CWBorderWidth, &change);
  XFlush(s->display);
  //timer.reset();
  s->isOpen=true;
}

void ThreadInfoWin::close() {
  if(!s->isOpen) return;
  XCloseDisplay(s->display);
  //s->log.close();
  s->isOpen=false;
}

void ThreadInfoWin::step() {
  if(!s->isOpen) open();
  //timer.cycleStart();
  Process *pr;
  sProcess *th;
  //Metronome *met;
  CycleTimer *ct;

  //-- graphical display
  uint i, y=20, x, len;
  XClearWindow(s->display, s->window);
#define TEXT0(txt) \
  if((len=strlen(txt))){ XDrawString(s->display, s->window, s->gc, x, y, txt, len); }
#define TEXT(form, val) \
  if((len=sprintf(s->outputbuf, form, val))){ XDrawString(s->display, s->window, s->gc, x, y, s->outputbuf, len); }
#define TEXTTIME(dt) \
  if((len=sprintf(s->outputbuf, "%5.2f|%5.2f|%5.2f", dt, dt##Mean, dt##Max))){ XDrawString(s->display, s->window, s->gc, x, y, s->outputbuf, len); }
  birosInfo().readAccess(this);
  for_list(i, pr, birosInfo().processes) {
    th = pr->s;
    int state=th->threadCondition.state;
    x=5;
    TEXT("%4i", th->tid); x+=25;
    TEXT("%3i", th->threadPriority); x+=25;
    TEXT("%s" , pr->name.p); x+=100;
    TEXT("%4i", th->timer.steps);  x+=30;
    if(state>0) { TEXT("%4i", state); } else switch (state) {
        case tsCLOSE:   TEXT0("close");  break;
        case tsLOOPING: TEXT0("loop");   break;
        case tsBEATING: TEXT0("beat");   break;
        case tsIDLE:    TEXT0("idle");   break;
        default: TEXT0("undefined:");
      } x+=50;
    TEXTTIME(th->timer.cyclDt); x+=130;
    TEXTTIME(th->timer.busyDt); x+=130;
    y+=20;
  }
  birosInfo().deAccess(this);
  y+=10;
  for_list(i, ct, globalCycleTimers) {
    x=5;
    TEXT("%2i", i); x+=25;
    TEXT("%s", ct->name); x+=100;
    TEXT("%4i", ct->steps); x+=30;
    TEXTTIME(ct->cyclDt); x+=130;
    TEXTTIME(ct->busyDt); x+=130;
    y+=20;
  }
#undef TEXT
#undef TEXTTIME
  XFlush(s->display);

  //-- log file
  //for_list(i, th, globalThreads)     s->log <<th->threadName <<' ' <<th->timer.busyDt <<' ' <<th->timer.cyclDt <<' ';
  //for_list(i, ct, globalCycleTimers) s->log <<ct->name <<' ' <<ct->busyDt <<' ' <<ct->cyclDt <<' ';
  //s->log <<endl;

  //timer.cycleDone();
}


#endif
