#include "threads.h"
#include "process.h"

#include <errno.h>
//#include <signal.h>
//#include <sched.h>
#include <string.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <sys/resource.h>
#include <iostream>
#include <X11/Xlib.h>
//#include <curses.h>
#include <FL/Fl.H>
#include <FL/fl_draw.H>
#include <FL/Fl_Window.H>

#include "util.h"
#include "array.h"

//===========================================================================
//
// helpers
//

static MT::Array<StepThread*> globalThreads;
static MT::Array<Metronome *> globalMetronomes;
static MT::Array<CycleTimer*> globalCycleTimers;

void reportNice(){
  pid_t tid = syscall(SYS_gettid);
  int priority = getpriority(PRIO_PROCESS, tid);
  std::cout <<"tid=" <<tid <<" nice=" <<priority <<std::endl;
}

bool setNice(int nice){
  pid_t tid = syscall(SYS_gettid);
  //int old_nice = getpriority(PRIO_PROCESS, tid);
  int ret = setpriority(PRIO_PROCESS, tid, nice);
  if(ret) MT_MSG("cannot set nice to "<<nice <<" (might require sudo), error=" <<ret <<' ' <<strerror(ret));
  //std::cout <<"tid=" <<tid <<" old nice=" <<old_nice <<" wanted nice=" <<nice <<std::flush;
  //nice = getpriority(PRIO_PROCESS, tid);
  //std::cout <<" new nice=" <<nice <<std::endl;
  if(ret) return false;
  return true;
}

void setRRscheduling(int priority){
  pid_t tid = syscall(SYS_gettid);
  MT_MSG(" tid=" <<tid <<" old sched=" <<sched_getscheduler(tid));
  sched_param sp; sp.sched_priority=priority;
  int rc = sched_setscheduler(tid, SCHED_RR, &sp);
  if (rc) switch(errno){
    case ESRCH:
      HALT("The process whose ID is"<<tid<<"could not be found.");
      break;
    case EPERM:
      MT_MSG("ERROR: Not enough privileges! Priority unchanged! Run with sudo to use this feature!");
      break;
    case EINVAL:
    default: HALT(errno<<strerror(errno));
  }
  timespec interval;
  rc=sched_rr_get_interval(tid, &interval);
  std::cout <<"RR scheduling interval = "<<interval.tv_sec <<"sec " <<1e-6*interval.tv_nsec <<"msec" <<std::endl;
  CHECK(!rc,"sched_rr_get_interval failed:" <<errno<<strerror(errno));
  MT_MSG("Scheduling policy changed: new sched="
		  <<sched_getscheduler(tid) <<" new priority=" <<priority);
}

void updateTimeIndicators(double& dt,double& dtMean,double& dtMax,const timespec& now,const timespec& last,uint step){
  dt=double(now.tv_sec-last.tv_sec-1)*1000. +
     double(1000000000l+now.tv_nsec-last.tv_nsec)/1000000.;
  if(dt<0.) dt=0.;
  double rate=.01;  if(step<100) rate=1./(1+step);
  dtMean = (1.-rate)*dtMean    + rate*dt;
  if(dt>dtMax || !(step%100)) dtMax = dt;
}


//===========================================================================
//
// Lock
//

Lock::Lock(){
//   pthread_rwlockattr_t   att;
  int rc;
//   rc = pthread_rwlockattr_init(&att);  if(rc) HALT("pthread failed with err "<<rc);
//   rc = pthread_rwlockattr_setpshared(&att, PTHREAD_PROCESS_SHARED);  if(rc) HALT("pthread failed with err "<<rc);
//   rc = pthread_rwlock_init(&lock, &att);  if(rc) HALT("pthread failed with err "<<rc);
  rc = pthread_rwlock_init(&lock, NULL);  if(rc) HALT("pthread failed with err "<<rc);
  state=0;
}

Lock::~Lock(){
  CHECK(!state,"");
  int rc = pthread_rwlock_destroy(&lock);  if(rc) HALT("pthread failed with err "<<rc);
}
  
void Lock::readLock(const char* _msg){
  int rc = pthread_rwlock_rdlock(&lock);  if(rc) HALT("pthread failed with err "<<rc);
  if(_msg) msg=_msg; else msg=NULL;
  //CHECK(state>=0,"");
  state++;
}
  
void Lock::writeLock(const char* _msg){
  int rc = pthread_rwlock_wrlock(&lock);  if(rc) HALT("pthread failed with err "<<rc <<" '" <<strerror(rc) <<"'");
  if(_msg) msg=_msg; else msg=NULL;
  //CHECK(!state,"");
  state=-1;
}
  
void Lock::unlock(){
  int rc = pthread_rwlock_unlock(&lock);  if(rc) HALT("pthread failed with err "<<rc);
  msg=NULL;
  //CHECK(state,"");
  if(state>0) state--; else state=0;
}


//===========================================================================
//
// ConditionVariable
//

ConditionVariable::ConditionVariable(){
  state=0;
  int rc;
  rc = pthread_mutex_init(&mutex,NULL);  if(rc) HALT("pthread failed with err "<<rc);
  rc = pthread_cond_init(&cond,NULL);    if(rc) HALT("pthread failed with err "<<rc);
}

ConditionVariable::~ConditionVariable(){
  int rc;
  rc = pthread_cond_destroy(&cond);    if(rc) HALT("pthread failed with err "<<rc);
  rc = pthread_mutex_destroy(&mutex);  if(rc) HALT("pthread failed with err "<<rc);
}

int ConditionVariable::getState(){
  int rc,i;
  rc = pthread_mutex_lock(&mutex);     if(rc) HALT("pthread failed with err "<<rc);
  i=state;
  rc = pthread_mutex_unlock(&mutex);   if(rc) HALT("pthread failed with err "<<rc);
  return i;
}

void ConditionVariable::setState(int i){
  int rc;
  rc = pthread_mutex_lock(&mutex);     if(rc) HALT("pthread failed with err "<<rc);
  state=i;
  rc = pthread_cond_broadcast(&cond);  if(rc) HALT("pthread failed with err "<<rc);
  rc = pthread_mutex_unlock(&mutex);   if(rc) HALT("pthread failed with err "<<rc);
}

void ConditionVariable::signal(){
  int rc;
  rc = pthread_mutex_lock(&mutex);     if(rc) HALT("pthread failed with err "<<rc);
  rc = pthread_cond_broadcast(&cond);  if(rc) HALT("pthread failed with err "<<rc);
  rc = pthread_mutex_unlock(&mutex);   if(rc) HALT("pthread failed with err "<<rc);
}

void ConditionVariable::waitForSignal(){
  int rc;
  rc = pthread_mutex_lock(&mutex);  if(rc) HALT("pthread failed with err "<<rc);
  rc = pthread_cond_wait(&cond, &mutex);  if(rc) HALT("pthread failed with err "<<rc);
  rc = pthread_mutex_unlock(&mutex);  if(rc) HALT("pthread failed with err "<<rc);
}

void ConditionVariable::waitForStateEq(int i){
  int rc;
  rc = pthread_mutex_lock(&mutex);  if(rc) HALT("pthread failed with err "<<rc);
  while(state!=i){
    rc = pthread_cond_wait(&cond, &mutex);  if(rc) HALT("pthread failed with err "<<rc);
  }
  rc = pthread_mutex_unlock(&mutex);  if(rc) HALT("pthread failed with err "<<rc);
}

void ConditionVariable::waitForStateNotEq(int i){
  int rc;
  rc = pthread_mutex_lock(&mutex);  if(rc) HALT("pthread failed with err "<<rc);
  while(state==i){
    rc = pthread_cond_wait(&cond, &mutex);  if(rc) HALT("pthread failed with err "<<rc);
  }
  rc = pthread_mutex_unlock(&mutex);  if(rc) HALT("pthread failed with err "<<rc);
}
  
void ConditionVariable::waitUntil(double absTime){
  NIY;
/*  int rc;
  timespec ts;
  ts.tv_sec  = tp.tv_sec;
  ts.tv_nsec = tp.tv_usec * 1000;
  ts.tv_sec += WAIT_TIME_SECONDS;

  rc = pthread_mutex_lock(&mutex);  if(rc) HALT("pthread failed with err "<<rc);
  rc = pthread_cond_timedwait(&cond, &mutex);  if(rc) HALT("pthread failed with err "<<rc);
  rc = pthread_mutex_unlock(&mutex);  if(rc) HALT("pthread failed with err "<<rc);
  */
}


//===========================================================================
//
// Metronome
//

Metronome::Metronome(const char* _name,long _targetDt){
  globalMetronomes.memMove=true;
  globalMetronomes.append(this);
  name=_name;
  targetDt=_targetDt;
  reset();
}

Metronome::~Metronome(){
  globalMetronomes.removeValue(this);
}

void Metronome::reset(){
  clock_gettime(CLOCK_MONOTONIC,&ticTime);
  lastTime=ticTime;
  tics=0;
}

void Metronome::waitForTic(){
  //compute target time
  ticTime.tv_nsec+=1000000l*targetDt;
  if(ticTime.tv_nsec>1000000000l){
    ticTime.tv_sec+=1;
    ticTime.tv_nsec-=1000000000l;
  }
  /*if(now.tv_sec>ticTime.tv_sec || (now.tv_sec==ticTime.tv_sec && now.tv_nsec>ticTime.tv_nsec)){
    //time has passed already
    ticTime=now;
  }else{*/
    //wait for target time
    int rc = clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&ticTime,NULL);
    if(rc){
      if(rc==0){ MT_MSG("clock_nanosleep() interrupted by signal") }
      else{ MT_MSG("clock_nanosleep() failed " <<rc); }
    }
  //}
  
  tics++;
}
  
double Metronome::getTimeSinceTic(){
  timespec now;
  clock_gettime(CLOCK_MONOTONIC,&now);
  return double(now.tv_sec-ticTime.tv_sec) + 1e-9*(now.tv_nsec-ticTime.tv_nsec);
}

//===========================================================================
//
// StepThread
//

CycleTimer::CycleTimer(const char* _name){
  reset();
  name=_name;
  globalCycleTimers.memMove=true;
  if(name) globalCycleTimers.append(this);
}

CycleTimer::~CycleTimer(){
  if(name) globalCycleTimers.removeValue(this);
}

void CycleTimer::reset(){
  steps=0;
  busyDt=busyDtMean=busyDtMax=1.;
  cyclDt=cyclDtMean=cyclDtMax=1.;
  clock_gettime(CLOCK_MONOTONIC,&lastTime);
}

void CycleTimer::cycleStart(){
  clock_gettime(CLOCK_MONOTONIC,&now);
  updateTimeIndicators(cyclDt,cyclDtMean,cyclDtMax,now,lastTime,steps);
  lastTime=now;
}

void CycleTimer::cycleDone(){
  clock_gettime(CLOCK_MONOTONIC,&now);
  updateTimeIndicators(busyDt,busyDtMean,busyDtMax,now,lastTime,steps);
  steps++;
}

//===========================================================================
//
// StepThread
//

StepThread::StepThread(const char* name){
  globalThreads.memMove=true;
  globalThreads.append(this);
  skips=0;
  threadName=name;
  threadCondition.setState(tsCLOSE);
  tid=0;
  threadPriority=0;
  thread=NULL;
  broadCastDone=false;
  syncCondition=NULL;
}

StepThread::~StepThread(){
  if(globalThreads.N)
    globalThreads.removeValue(this);
}

void StepThread::threadOpen(int priority){
#ifndef MT_NO_THREADS
  CHECK(threadCondition.state==tsCLOSE,"never open while not closed!");
  threadCondition.setState(tsOPEN);
  threadPriority = priority;
  int rc;
  pthread_attr_t atts;
  rc = pthread_attr_init(&atts); if(rc) HALT("pthread failed with err "<<rc);
  /*if(priority){ //doesn't work - but setpriority does work!!
    rc = pthread_attr_setschedpolicy(&atts, SCHED_RR);  if(rc) HALT("pthread failed with err "<<rc <<strerror(rc));
    sched_param  param;
    rc = pthread_attr_getschedparam(&atts, &param);  if(rc) HALT("pthread failed with err "<<rc);
    std::cout <<"standard priority = " <<param.sched_priority <<std::endl;
    param.sched_priority += priority;
    std::cout <<"modified priority = " <<param.sched_priority <<std::endl;
    rc = pthread_attr_setschedparam(&atts, &param);  if(rc) HALT("pthread failed with err "<<rc <<strerror(rc));
  }*/
  rc = pthread_create(&thread, &atts, staticThreadMain, this);  if(rc) HALT("pthread failed with err "<<rc);
  threadCondition.waitForStateEq(tsIDLE);
#else
  std::cout <<" +++ opening 'thread' in SERIAL mode '" <<threadName <<'\'' <<std::endl;
  open();
#endif
}

void StepThread::threadClose(){
#ifndef MT_NO_THREADS
  if(!thread && threadCondition.state==tsCLOSE) return;
  int rc;
  if(threadCondition.state<=tsLOOPING) threadLoopStop();
  threadCondition.waitForStateEq(tsIDLE);
  threadCondition.setState(tsCLOSE);
  rc = pthread_join(thread, NULL);     if(rc) HALT("pthread failed with err "<<rc);
  thread=NULL;
#else
  close();
  std::cout <<" +++ closing 'thread' in SERIAL mode '" <<threadName <<'\'' <<std::endl;
#endif
}

void StepThread::threadStep(bool wait){
#ifndef MT_NO_THREADS
  if(wait) threadWait();
  CHECK(threadCondition.state==tsIDLE,"never step while thread is busy!");
  threadCondition.setState(1);
#else
  step();
#endif
}

void StepThread::threadStepOrSkip(uint maxSkips){
#ifndef MT_NO_THREADS
  if(threadCondition.state!=tsIDLE){
    skips++;
    //if(skips>maxSkips) HALT("skips>maxSkips: "<<skips<<'<'<<maxSkips);
    if(maxSkips && skips>=maxSkips) MT_MSG("WARNING: skips>=maxSkips="<<skips);
    return;
  }
  skips=0;
  threadCondition.setState(1);
#else
  step();
#endif
}

void StepThread::threadSteps(uint steps){
#ifndef MT_NO_THREADS
  CHECK(threadCondition.state==tsIDLE,"never step while thread is busy!");
  threadCondition.setState(steps);
#else
  for(uint k=0;k<steps;k++) step();
#endif
}

void StepThread::threadWait(){
#ifndef MT_NO_THREADS
  threadCondition.waitForStateEq(tsIDLE);
#endif
}

bool StepThread::threadIsReady(){
#ifndef MT_NO_THREADS
  if(threadCondition.state==tsIDLE) return true;
  return false;
#else
  return true;
#endif
}

void StepThread::threadLoop(){
#ifndef MT_NO_THREADS
  if(threadCondition.state==tsCLOSE) threadOpen();
  CHECK(threadCondition.state==tsIDLE,"thread '"<<threadName <<"': never start loop while thread is busy!");
  threadCondition.setState(tsLOOPING);
#else
  HALT("can't loop in no-threads mode!");
#endif
}

void StepThread::threadLoopWithBeat(double sec){
#ifndef MT_NO_THREADS
  metronome=new Metronome("threadTiccer",1000.*sec);
  if(threadCondition.state==tsCLOSE) threadOpen();
  CHECK(threadCondition.state==tsIDLE,"thread '"<<threadName <<"': never start loop while thread is busy!");
  threadCondition.setState(tsBEATING);
#else
  HALT("can't loop in no-threads mode!");
#endif
}

void StepThread::threadLoopSyncWithDone(StepThread& thread){
#ifndef MT_NO_THREADS
  thread.broadCastDone=true;
  syncCondition = &thread.threadCondition;
  if(threadCondition.state==tsCLOSE) threadOpen();
  CHECK(threadCondition.state==tsIDLE,"thread '"<<threadName <<"': never start loop while thread is busy!");
  threadCondition.setState(tsSYNCLOOPING);
#else
  HALT("can't loop in no-threads mode!");
#endif
}

void StepThread::threadLoopStop(){
#ifndef MT_NO_THREADS
  CHECK(threadCondition.state<=tsLOOPING,"called stop loop although not looping!");
  int state=threadCondition.state;
  threadCondition.setState(tsIDLE);
  if(state==tsSYNCLOOPING) syncCondition->signal(); //force a signal that wakes up the sync-thread-loop
#endif
}

void* StepThread::staticThreadMain(void *_this){
  StepThread *self=(StepThread*)_this;
  std::cout <<" +++ entering staticThreadMain of '" <<self->threadName <<'\'' <<std::endl;

  self->tid = syscall(SYS_gettid);
  
  // http://linux.die.net/man/3/setpriority
  //if(self->threadPriority) setRRscheduling(self->threadPriority);
  if(self->threadPriority) setNice(self->threadPriority);
  
  self->open();
  self->threadCondition.setState(tsIDLE);
  self->timer.reset();
  for(;self->threadCondition.state!=tsCLOSE;){
    self->threadCondition.waitForStateNotEq(tsIDLE);
    int state=self->threadCondition.state;
    if(state==tsCLOSE) break;
    //the state is either positive (steps to go) or looping
    CHECK(state>0 || state<=-3,"at this point, the thread condition should be positive (steps to do) or looping!");

    if(state==tsBEATING) self->metronome->waitForTic();
    if(state==tsSYNCLOOPING) self->syncCondition->waitForSignal(); //self is a slave and waits for condition signal
    
    self->timer.cycleStart();
    self->step();
    self->timer.cycleDone();
    
    state=self->threadCondition.state; //state might have changes due to stopping or so!!
    if(state>0) self->threadCondition.setState(state-1); //count down
    if(state<0 || self->broadCastDone) self->threadCondition.signal();
  };
  self->close();
  std::cout <<" +++ exiting staticThreadMain of '" <<self->threadName <<'\'' <<std::endl;
  return NULL;
}



//===========================================================================
//
// global monitor routines
//

#if 0
struct ThreadInfoWin:public StepThread,Fl_Window{
  bool isOpen;
  //ofstream log;
  char outputbuf[200];

  ThreadInfoWin():StepThread("ThreadInfoX"),Fl_Window(0,0,600,300,"processes"){
    //clear_border();
  }
  ~ThreadInfoWin(){  }
  
  void open(){
    //MT::open(log,"LOG.threads");
    show();
    Fl::check();
    isOpen=true;
  }

  void close(){
  }
  
  void step(){
    if(!isOpen) open();
    redraw();
    Fl::wait(.1);
  }

  void draw(){
    //timer.cycleStart();
    StepThread *th;
    //Metronome *met;
    CycleTimer *ct;
    
    //-- graphical display
    uint i,y=20,x,len;
    //XClearWindow(fl_display,fl_window);
    fl_draw_box(FL_FLAT_BOX, 0, 0, w(), h(), FL_FOREGROUND_COLOR);
    fl_color(FL_BACKGROUND2_COLOR);
    fl_font(1,10);
#define TEXT0(txt) \
    fl_draw(txt, x, y);
#define TEXT(form,val) \
    if((len=sprintf(outputbuf,form,val))){ fl_draw(outputbuf, x, y); }
#define TEXTTIME(dt) \
    if((len=sprintf(outputbuf,"%5.2f|%5.2f|%5.2f",dt,dt##Mean,dt##Max))){ fl_draw(outputbuf, x, y); }
    for_list(i,th,globalThreads){
      int state=th->threadCondition.state;
      x=5;
      TEXT("%4i",th->tid); x+=30;
      TEXT("%3i",th->threadPriority); x+=25;
      TEXT("%s",th->threadName); x+=100;
      TEXT("%4i",th->timer.steps);  x+=30;
      if(state>0){ TEXT("%4i",state); }
      else switch(state){
        case StepThread::tsOPEN:    TEXT0("open");   break;
        case StepThread::tsCLOSE:   TEXT0("close");  break;
        case StepThread::tsLOOPING: TEXT0("loop");   break;
        case StepThread::tsBEATING: TEXT0("beat");   break;
        case StepThread::tsSYNCLOOPING: TEXT0("sync");   break;
        case StepThread::tsIDLE:    TEXT0("idle");   break;
        default: TEXT0("undefined:");
      } x+=50;
      TEXTTIME(th->timer.cyclDt); x+=130;
      TEXTTIME(th->timer.busyDt); x+=130;
      y+=20;
    }
    y+=10;
    for_list(i,ct,globalCycleTimers){
      x=5;
      TEXT("%2i",i); x+=25;
      TEXT("%s",ct->name); x+=100;
      TEXT("%4i",ct->steps); x+=30;
      TEXTTIME(ct->cyclDt); x+=130;
      TEXTTIME(ct->busyDt); x+=130;
      y+=20;
    }
#undef TEXT
#undef TEXTTIME
    //XFlush(display);

    //-- log file
    //for_list(i,th,globalThreads)     log <<th->threadName <<' ' <<th->timer.busyDt <<' ' <<th->timer.cyclDt <<' ';
    //for_list(i,ct,globalCycleTimers) log <<ct->name <<' ' <<ct->busyDt <<' ' <<ct->cyclDt <<' ';
    //log <<endl;
  }

};

#else

struct sThreadInfoWin{
  bool isOpen;
  Display *display;
  Window window;
  GC gc;
  //CycleTimer timer;
  //ofstream log;
  char outputbuf[200];
};

ThreadInfoWin::ThreadInfoWin():Process("ThreadInfoX"){
  s=new sThreadInfoWin;
  s->isOpen=false;
}

ThreadInfoWin::~ThreadInfoWin(){
  if(s->isOpen) close();
}

void ThreadInfoWin::open(){
  //MT::open(s->log,"LOG.threads");
  s->display = XOpenDisplay(NULL);
  if(!s->display) HALT("Cannot open display");
  s->window = XCreateSimpleWindow(s->display, DefaultRootWindow(s->display),
				  10, 10, 600, 300, 1,
				  0xffffff, 0x000000);
  XMapWindow(s->display, s->window);
  s->gc = XCreateGC(s->display, s->window, 0, NULL);
  XSetFont(s->display, s->gc,  XLoadFont(s->display,"-*-helvetica-*-r-*-*-*-*-*-*-*-*-*-*"));
  //-adobe-courier-medium-r-*-*-*-80-*-*-*-*-*-*"));
  XSetBackground(s->display, s->gc, 0x000000);
  XSetForeground(s->display, s->gc, 0xffffff);
  XWindowChanges change={1500, 700,  600, 300,  10, NULL, 0};
  XConfigureWindow(s->display, s->window, CWX|CWY|CWWidth|CWHeight|CWBorderWidth, &change);
  XFlush(s->display);
  //timer.reset();
  s->isOpen=true;
}

  void ThreadInfoWin::close(){
    if(!s->isOpen) return;
    XCloseDisplay(s->display);
    //s->log.close();
    s->isOpen=false;
  }
  
  void ThreadInfoWin::step(){
    if(!s->isOpen) open();
    //timer.cycleStart();
    StepThread *th;
    //Metronome *met;
    CycleTimer *ct;
    
    //-- graphical display
    uint i,y=20,x,len;
    XClearWindow(s->display,s->window);
#define TEXT0(txt) \
    if((len=strlen(txt))){ XDrawString(s->display, s->window, s->gc, x, y, txt, len); }
#define TEXT(form,val) \
    if((len=sprintf(s->outputbuf,form,val))){ XDrawString(s->display, s->window, s->gc, x, y, s->outputbuf, len); }
#define TEXTTIME(dt) \
    if((len=sprintf(s->outputbuf,"%5.2f|%5.2f|%5.2f",dt,dt##Mean,dt##Max))){ XDrawString(s->display, s->window, s->gc, x, y, s->outputbuf, len); }
    for_list(i,th,globalThreads){
      int state=th->threadCondition.state;
      x=5;
      TEXT("%4i",th->tid); x+=25;
      TEXT("%3i",th->threadPriority); x+=25;
      TEXT("%s",th->threadName); x+=100;
      TEXT("%4i",th->timer.steps);  x+=30;
      if(state>0){ TEXT("%4i",state); }
      else switch(state){
        case StepThread::tsOPEN:    TEXT0("open");   break;
        case StepThread::tsCLOSE:   TEXT0("close");  break;
        case StepThread::tsLOOPING: TEXT0("loop");   break;
        case StepThread::tsBEATING: TEXT0("beat");   break;
        case StepThread::tsSYNCLOOPING: TEXT0("sync");   break;
        case StepThread::tsIDLE:    TEXT0("idle");   break;
        default: TEXT0("undefined:");
      } x+=50;
      TEXTTIME(th->timer.cyclDt); x+=130;
      TEXTTIME(th->timer.busyDt); x+=130;
      y+=20;
    }
    y+=10;
    for_list(i,ct,globalCycleTimers){
      x=5;
      TEXT("%2i",i); x+=25;
      TEXT("%s",ct->name); x+=100;
      TEXT("%4i",ct->steps); x+=30;
      TEXTTIME(ct->cyclDt); x+=130;
      TEXTTIME(ct->busyDt); x+=130;
      y+=20;
    }
#undef TEXT
#undef TEXTTIME
    XFlush(s->display);

    //-- log file
    //for_list(i,th,globalThreads)     s->log <<th->threadName <<' ' <<th->timer.busyDt <<' ' <<th->timer.cyclDt <<' ';
    //for_list(i,ct,globalCycleTimers) s->log <<ct->name <<' ' <<ct->busyDt <<' ' <<ct->cyclDt <<' ';
    //s->log <<endl;

    //timer.cycleDone();
  }
#endif
    
//static ThreadInfoWin globalThreadInfoWin;

/*void threadReportAllCurses(){
  static bool init=false;
  if(!init){
    initscr();
    cbreak();
    noecho();
  }
  MT::String txt;
  threadReportAll(txt);
  mvprintw(0,0,txt.p);
  refresh();
}*/

//===========================================================================
//
// old obsolete shared memory stuff...
//


#if 0
MT::SHM global_shm;
uint pidIndex;

#ifndef MT_MSVC
void func(int x){
  //printf("received signal!\n");
}
void waitForSignal(){
  signal(SIGUSR1,func);
  pause();
}
void sendSignal(){
  signal(SIGUSR1,func);
  uint i;
  int r;
  for(i=0;i<maxClients;i++) if(shm->pids[i]!=-1 && shm->pids[i]!=getpid()){
    r=kill(shm->pids[i],SIGUSR1);
    if(r) MT_MSG("warning: couldn't send signal to pid "<<shm->pids[i]);
  }
}
#else
void waitForSignal(){
  HANDLE hEvent = CreateEvent ( NULL, TRUE, FALSE, "EventName");
  CHECK(hEvent,"failed to open event");
  DWORD dwResult = WaitForSingleObject(hEvent, INFINITE);
  CloseHandle(hEvent);
}
void sendSignal(){
  //HANDLE hEvent = OpenEvent ( EVENT_ALL_ACCESS, FALSE, "EventName");
  //CHECK(hEvent,"failed to open event - nobody's waiting?");
  HANDLE hEvent = CreateEvent ( NULL, TRUE, FALSE, "EventName");
  CHECK(hEvent,"failed to open event");
  PulseEvent( hEvent );
  CloseHandle(hEvent);
}
#endif

void initTypes(){
  //compute type sizes
  int t;
  for(t=byteT;t<nTypes;t++){
    switch(t){
    case byteT:    TypeSize[t]=sizeof(byte);    break;
    case charT:    TypeSize[t]=sizeof(char);    break;
    case uintT:    TypeSize[t]=sizeof(uint);    break;
    case intT:     TypeSize[t]=sizeof(int);     break;
    case doubleT:  TypeSize[t]=sizeof(double);  break;
    case floatT:   TypeSize[t]=sizeof(float);   break;
    case boolT:    TypeSize[t]=sizeof(bool);    break;
    default: HALT("");
    }
  }
}

BlockDescription blockDescription[nBlocks]={
  {"shm_size"     , uintT, 1, 0},
  {"pids"         , intT, maxClients, 0},
  {"changed"      , boolT, nBlocks, 0},
  {"wheels"       , intT, 2, 0},
  {"joystick"     , intT, 9, 0},
};

void initBlockDescription(BlockDescription *b,const RobotShmStructure& shm){
  b[0].p=(void*)&shm.shm_size;
  b[1].p=(void*) shm.pids;
  b[2].p=(void*) shm.changed;
  b[3].p=(void*) shm.wheels;
  b[4].p=(void*) shm.joystick;
}

void writeBlock(std::ostream& os,int blockName){
  BlockDescription& b=blockDescription[blockName];
  os <<b.name <<'=';
  if(b.n!=1) std::cout <<'[';
  for(int i=0;i<b.n;i++){
    switch(b.type){
    case byteT:    std::cout <<(int)((byte*)b.p)[i];  break;
    case charT:    std::cout <<((char*)b.p)[i];       break;
    case uintT:    std::cout <<((uint*)b.p)[i];       break;
    case intT:     std::cout <<((int*)b.p)[i];        break;
    case doubleT:  std::cout <<((double*)b.p)[i];     break;
    case floatT:   std::cout <<((float*)b.p)[i];      break;
    case boolT:    std::cout <<((bool*)b.p)[i];       break;
    default: HALT("");
    }
    if(i<b.n-1) std::cout <<' ';
  }
  if(b.n!=1) std::cout <<']';
}

void openRobotSharedMemory(){
  initTypes();

  uint S=sizeof(RobotShmStructure);
  global_shm.open("myspace",S);
  shm = (RobotShmStructure*)global_shm.p;
  initBlockDescription(blockDescription,*shm);

  uint i;
  if(global_shm.created)
    for(i=0;i<maxClients;i++) shm->pids[i]=-1;
  
  for(i=0;i<maxClients;i++) if(shm->pids[i]==-1) break;

  if(i==maxClients)
    HALT("exceeded max number of clients "<<maxClients);
  pidIndex=i;

#ifndef MT_MSVC
  shm->pids[pidIndex]=getpid();
#else
  shm->pids[pidIndex]=GetCurrentProcessId();
#endif
  shm->changed[pidsB]=true;
  sendSignal();
}

void closeRobotSharedMemory(){
  shm->pids[pidIndex]=-1;
  shm->changed[pidsB]=true;
  sendSignal();
  global_shm.close();
}

void destroyRobotSharedMemory(){
  if(!global_shm.opened) global_shm.open("myspace",sizeof(RobotShmStructure));
  global_shm.destroy();
}

#endif
