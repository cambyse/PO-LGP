#include "process.h"
#include "process_internal.h"

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <sys/resource.h>
#include <iostream>
#include <X11/Xlib.h>

#include "util.h"
#include "array.h"


//===========================================================================
//
// Global information
//

GlobalInfo global;


//===========================================================================
//
// helpers
//

MT::Array<Metronome *> globalMetronomes;
MT::Array<CycleTimer*> globalCycleTimers;

void reportNice(){
  pid_t tid = syscall(SYS_gettid);
  int priority = getpriority(PRIO_PROCESS, tid);
  std::cout <<"tid=" <<tid <<" nice=" <<priority <<std::endl;
}

bool setNice(int nice){
  pid_t tid = syscall(SYS_gettid);
  //int old_nice = getpriority(PRIO_PROCESS, tid);
  int ret = setpriority(PRIO_PROCESS, tid, nice);
  if(ret) MT_MSG("cannot set nice to " <<nice <<" (might require sudo), error=" <<ret <<' ' <<strerror(ret));
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
  if(rc) switch(errno){
      case ESRCH:
        HALT("The process whose ID is" <<tid <<"could not be found.");
        break;
      case EPERM:
        MT_MSG("ERROR: Not enough privileges! Priority unchanged! Run with sudo to use this feature!");
        break;
      case EINVAL:
      default: HALT(errno <<strerror(errno));
    }
  timespec interval;
  rc=sched_rr_get_interval(tid, &interval);
  std::cout <<"RR scheduling interval = " <<interval.tv_sec <<"sec " <<1e-6*interval.tv_nsec <<"msec" <<std::endl;
  CHECK(!rc, "sched_rr_get_interval failed:" <<errno <<strerror(errno));
  MT_MSG("Scheduling policy changed: new sched="
         <<sched_getscheduler(tid) <<" new priority=" <<priority);
}

void updateTimeIndicators(double& dt, double& dtMean, double& dtMax, const timespec& now, const timespec& last, uint step){
  dt=double(now.tv_sec-last.tv_sec-1)*1000. +
     double(1000000000l+now.tv_nsec-last.tv_nsec)/1000000.;
  if(dt<0.) dt=0.;
  double rate=.01;  if(step<100) rate=1./(1+step);
  dtMean = (1.-rate)*dtMean    + rate*dt;
  if(dt>dtMax || !(step%100)) dtMax = dt;
}


//===========================================================================
//
// Access Lock
//

Lock::Lock(){
//   pthread_rwlockattr_t   att;
  int rc;
//   rc = pthread_rwlockattr_init(&att);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
//   rc = pthread_rwlockattr_setpshared(&att, PTHREAD_PROCESS_SHARED);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
//   rc = pthread_rwlock_init(&lock, &att);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rc = pthread_rwlock_init(&lock, NULL);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  state=0;
}

Lock::~Lock(){
  CHECK(!state, "");
  int rc = pthread_rwlock_destroy(&lock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void Lock::readLock(const char* _msg){
  int rc = pthread_rwlock_rdlock(&lock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  if(_msg) msg=_msg; else msg=NULL;
  //CHECK(state>=0, "");
  state++;
}

void Lock::writeLock(const char* _msg){
  int rc = pthread_rwlock_wrlock(&lock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  if(_msg) msg=_msg; else msg=NULL;
  //CHECK(!state, "");
  state=-1;
}

void Lock::unlock(){
  int rc = pthread_rwlock_unlock(&lock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  msg=NULL;
  //CHECK(state, "");
  if(state>0) state--; else state=0;
}


//===========================================================================
//
// Mutex Lock
//

Mutex::Mutex(){
  pthread_mutexattr_t atts;
  int rc;
  rc = pthread_mutexattr_init(&atts);  if(rc) HALT("pthread failed with err " <<rc <<strerror(rc));
  rc = pthread_mutexattr_settype(&atts, PTHREAD_MUTEX_RECURSIVE_NP);  if(rc) HALT("pthread failed with err " <<rc <<strerror(rc));
  rc = pthread_mutex_init(&_lock, &atts);

  //_lock = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;

  state=0;
}

Mutex::~Mutex(){
  CHECK(!state, "");
  int rc = pthread_mutex_destroy(&_lock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void Mutex::lock(const char* _msg){
  int rc = pthread_mutex_lock(&_lock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  if(_msg) msg=_msg; else msg=NULL;
  state++;
}

void Mutex::unlock(){
  int rc = pthread_mutex_unlock(&_lock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  msg=NULL;
  state--;
}

       
//===========================================================================
//
// ConditionVariable
//

ConditionVariable::ConditionVariable(){
  state=0;
  int rc;
  rc = pthread_mutex_init(&mutex, NULL);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rc = pthread_cond_init(&cond, NULL);    if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

ConditionVariable::~ConditionVariable(){
  int rc;
  rc = pthread_cond_destroy(&cond);    if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rc = pthread_mutex_destroy(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

int ConditionVariable::getState(){
  int rc, i;
  rc = pthread_mutex_lock(&mutex);     if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  i=state;
  rc = pthread_mutex_unlock(&mutex);   if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  return i;
}

void ConditionVariable::setState(int i){
  int rc;
  rc = pthread_mutex_lock(&mutex);     if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  state=i;
  rc = pthread_cond_broadcast(&cond);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rc = pthread_mutex_unlock(&mutex);   if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void ConditionVariable::signal(){
  int rc;
  rc = pthread_mutex_lock(&mutex);     if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rc = pthread_cond_broadcast(&cond);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rc = pthread_mutex_unlock(&mutex);   if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void ConditionVariable::waitForSignal(){
  int rc;
  rc = pthread_mutex_lock(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rc = pthread_cond_wait(&cond, &mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rc = pthread_mutex_unlock(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

int ConditionVariable::waitForStateEq(int i){
  int rc;
  int stateAfter;
  rc = pthread_mutex_lock(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  while(state!=i){
    rc = pthread_cond_wait(&cond, &mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  }
  stateAfter = state;
  rc = pthread_mutex_unlock(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  return stateAfter;
}

int ConditionVariable::waitForStateNotEq(int i){
  int rc;
  int stateAfter;
  rc = pthread_mutex_lock(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  while(state==i){
    rc = pthread_cond_wait(&cond, &mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  }
  stateAfter = state;
  rc = pthread_mutex_unlock(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  return stateAfter;
}

void ConditionVariable::waitUntil(double absTime){
  NIY;
  /*  int rc;
    timespec ts;
    ts.tv_sec  = tp.tv_sec;
    ts.tv_nsec = tp.tv_usec * 1000;
    ts.tv_sec += WAIT_TIME_SECONDS;
  
    rc = pthread_mutex_lock(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
    rc = pthread_cond_timedwait(&cond, &mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
    rc = pthread_mutex_unlock(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
    */
}


//===========================================================================
//
// Metronome
//

Metronome::Metronome(const char* _name, long _targetDt){
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
  clock_gettime(CLOCK_MONOTONIC, &ticTime);
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
  int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ticTime, NULL);
  if(rc){
    if(rc==0){ MT_MSG("clock_nanosleep() interrupted by signal") }else{ MT_MSG("clock_nanosleep() failed " <<rc); }
  }
  //}
  
  tics++;
}

double Metronome::getTimeSinceTic(){
  timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return double(now.tv_sec-ticTime.tv_sec) + 1e-9*(now.tv_nsec-ticTime.tv_nsec);
}


//===========================================================================
//
// CycleTimer
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
  clock_gettime(CLOCK_MONOTONIC, &lastTime);
}

void CycleTimer::cycleStart(){
  clock_gettime(CLOCK_MONOTONIC, &now);
  updateTimeIndicators(cyclDt, cyclDtMean, cyclDtMax, now, lastTime, steps);
  lastTime=now;
}

void CycleTimer::cycleDone(){
  clock_gettime(CLOCK_MONOTONIC, &now);
  updateTimeIndicators(busyDt, busyDtMean, busyDtMax, now, lastTime, steps);
  steps++;
}


//===========================================================================
//
// Variable
//

Variable::Variable(const char *_name){
  s = new sVariable(this);
  name = _name;
  global.writeAccess(NULL);
  id = global.variableCount++;
  //s->os.open(STRING("var-" <<name <<".log"));
  global.variables.memMove=true;
  global.variables.append(this);
  global.deAccess(NULL);
};

Variable::~Variable(){
  //s->os.close();
  delete s;
  global.writeAccess(NULL);
  global.variables.removeValue(this);
  global.deAccess(NULL);
};

void Variable::readAccess(Process *p){
  //if(p) p->V.setAppend(this); //TODO: this is too expensive!!
  s->lock.readLock();
  //cout <<(p?p->name:"NULL") <<" reads  " <<name <<" state=";
}

void Variable::writeAccess(Process *p){
  //if(p) p->V.setAppend(this); //TODO: this is too expensive!!
  s->lock.writeLock();
  //cout <<(p?p->name:"NULL") <<" writes " <<name <<" state=";
}

void Variable::deAccess(Process *p){
  //cout <<(p?p->name:"NULL") <<" frees  " <<name <<" state=";
  s->lock.unlock();
}

int Variable::lockState(){
  return s->lock.state;
}

int  Variable::getCondition(){
  return s->cond.getState();
}

void Variable::setCondition(int i){
  s->cond.setState(i);
}

void Variable::waitForConditionSignal(){
  s->cond.waitForSignal();
}

void Variable::waitForConditionEq(int i){
  s->cond.waitForStateEq(i);
}

void Variable::waitForConditionNotEq(int i){
  s->cond.waitForStateNotEq(i);
}


//===========================================================================
//
// Process
//


Process::Process(const char *_name){
  s = new sProcess();
  name = _name;
  global.writeAccess(this);
  id = global.processCount++;
  global.processes.memMove=true;
  global.processes.append(this);
  global.deAccess(this);
}

Process::~Process(){
  delete s;
  global.writeAccess(this);
  global.processes.removeValue(this);
  global.deAccess(this);
}

void Process::threadOpen(int priority){
#ifndef MT_NO_THREADS
  CHECK(s->threadCondition.state==tsCLOSE, "never open while not closed!");
  s->threadCondition.setState(tsOPEN);
  s->threadPriority = priority;
  int rc;
  pthread_attr_t atts;
  rc = pthread_attr_init(&atts); if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  /*if(priority){ //doesn't work - but setpriority does work!!
    rc = pthread_attr_setschedpolicy(&atts, SCHED_RR);  if(rc) HALT("pthread failed with err " <<rc <<strerror(rc));
    sched_param  param;
    rc = pthread_attr_getschedparam(&atts, &param);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
    std::cout <<"standard priority = " <<param.sched_priority <<std::endl;
    param.sched_priority += priority;
    std::cout <<"modified priority = " <<param.sched_priority <<std::endl;
    rc = pthread_attr_setschedparam(&atts, &param);  if(rc) HALT("pthread failed with err " <<rc <<strerror(rc));
  }*/
  rc = pthread_create(&s->thread, &atts, s->staticThreadMain, this);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  s->threadCondition.waitForStateEq(tsIDLE);
#else
  std::cout <<" +++ opening 'thread' in SERIAL mode '" <<threadName <<'\'' <<std::endl;
  open();
#endif
}

void Process::threadClose(){
#ifndef MT_NO_THREADS
  if(!s->thread && s->threadCondition.state==tsCLOSE) return;
  int rc;
  if(s->threadCondition.state<=tsLOOPING) threadStop();
  s->threadCondition.waitForStateEq(tsIDLE);
  s->threadCondition.setState(tsCLOSE);
  rc = pthread_join(s->thread, NULL);     if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  s->thread=NULL;
#else
  close();
  std::cout <<" +++ closing 'thread' in SERIAL mode '" <<threadName <<'\'' <<std::endl;
#endif
}

void Process::threadStep(bool wait){
#ifndef MT_NO_THREADS
  if(wait) threadWait();
  CHECK(s->threadCondition.state==tsIDLE, "never step while thread is busy!");
  s->threadCondition.setState(1);
#else
  step();
#endif
}

void Process::threadStepOrSkip(uint maxSkips){
#ifndef MT_NO_THREADS
  if(s->threadCondition.state!=tsIDLE){
    s->skips++;
    //if(skips>maxSkips) HALT("skips>maxSkips: " <<skips<<'<' <<maxSkips);
    if(maxSkips && s->skips>=maxSkips) MT_MSG("WARNING: skips>=maxSkips=" <<s->skips);
    return;
  }
  s->skips=0;
  s->threadCondition.setState(1);
#else
  step();
#endif
}

void Process::threadSteps(uint steps){
#ifndef MT_NO_THREADS
  CHECK(s->threadCondition.state==tsIDLE, "never step while thread is busy!");
  s->threadCondition.setState(steps);
#else
  for(uint k=0; k<steps; k++) step();
#endif
}

bool Process::threadIsIdle(){
#ifndef MT_NO_THREADS
  if(s->threadCondition.state==tsIDLE) return true;
  return false;
#else
  return true;
#endif
}

void Process::threadWait(){
#ifndef MT_NO_THREADS
  s->threadCondition.waitForStateEq(tsIDLE);
#endif
}

void Process::threadLoop(){
#ifndef MT_NO_THREADS
  if(s->threadCondition.state==tsCLOSE) threadOpen();
  CHECK(s->threadCondition.state==tsIDLE, "thread '" <<name <<"': never start loop while thread is busy!");
  s->threadCondition.setState(tsLOOPING);
#else
  HALT("can't loop in no-threads mode!");
#endif
}

void Process::threadLoopWithBeat(double sec){
#ifndef MT_NO_THREADS
  s->metronome=new Metronome("threadTiccer", 1000.*sec);
  if(s->threadCondition.state==tsCLOSE) threadOpen();
  CHECK(s->threadCondition.state==tsIDLE, "thread '" <<name <<"': never start loop while thread is busy!");
  s->threadCondition.setState(tsBEATING);
#else
  HALT("can't loop in no-threads mode!");
#endif
}

void Process::threadLoopSyncWithDone(Process& proc){
#ifndef MT_NO_THREADS
  proc.s->broadcastDone=true;
  s->syncCondition = &proc.s->threadCondition;
  if(s->threadCondition.state==tsCLOSE) threadOpen();
  CHECK(s->threadCondition.state==tsIDLE, "thread '" <<name <<"': never start loop while thread is busy!");
  s->threadCondition.setState(tsSYNCLOOPING);
#else
  HALT("can't loop in no-threads mode!");
#endif
}

void Process::threadStop(){
#ifndef MT_NO_THREADS
  CHECK(s->threadCondition.state<=tsLOOPING, "called stop loop although not looping!");
  int state=s->threadCondition.state;
  s->threadCondition.setState(tsIDLE);
  if(state==tsSYNCLOOPING) s->syncCondition->signal(); //force a signal that wakes up the sync-thread-loop
#endif
}

void* sProcess::staticThreadMain(void *_self){
  Process  *proc=(Process*)_self;
  sProcess *s   =proc->s;
  std::cout <<" +++ entering staticThreadMain of '" <<proc->name <<'\'' <<std::endl;
  
  s->tid = syscall(SYS_gettid);
  
  // http://linux.die.net/man/3/setpriority
  //if(s->threadPriority) setRRscheduling(s->threadPriority);
  if(s->threadPriority) setNice(s->threadPriority);
  
  proc->open();
  s->threadCondition.setState(tsIDLE);
  s->timer.reset();
  for(; s->threadCondition.state!=tsCLOSE;){
    int state=s->threadCondition.waitForStateNotEq(tsIDLE);
    if(state==tsCLOSE) break;
    //the state is either positive (steps to go) or looping
    CHECK(state>0 || state<=-3, "at this point, the thread condition should be positive (steps to do) or looping!");
    
    if(state==tsBEATING) s->metronome->waitForTic();
    if(state==tsSYNCLOOPING) s->syncCondition->waitForSignal(); //self is a slave and waits for condition signal
    
    s->timer.cycleStart();
    proc->step();
    s->timer.cycleDone();
    
    state=s->threadCondition.state; //state might have changes due to stopping or so!!
    if(state>0) s->threadCondition.setState(state-1); //count down
    if(state<0 || s->broadcastDone) s->threadCondition.signal();
  };
  proc->close();
  std::cout <<" +++ exiting staticThreadMain of '" <<proc->name <<'\'' <<std::endl;
  return NULL;
}

//===========================================================================
//
// Group
//

void Group::set(const VariableL &_V, const ProcessL &_P){
  V = _V;
  P = _P;
}

void Group::loop(){
  Process *p; uint i;
  for_list(i, p, P) p->threadLoop();
}

void Group::stop(){
  Process *p; uint i;
  for_list(i, p, P) p->threadStop();
}

void Group::close(){
  Process *p; uint i;
  for_list(i, p, P) p->threadClose();
}


//===========================================================================
//
// Global Static information
//

void reportGlobalProcessGraph(){
  ofstream fil("proc.graph");
  uint i, j;
  Variable *v;
  Process *p;
  global.readAccess(NULL);
  for_list(i, v, global.variables){
    fil <<"Variable " <<v->name <<endl;
  }
  fil <<endl;
  for_list(i, p, global.processes){
    fil <<"Process " <<p->name <<" (";
    for_list(j, v, p->V){
      if(j) fil <<',';
      fil <<v->name;
    }
    fil <<")" <<endl;
  }
  global.deAccess(NULL);
  fil.close();
}


//===========================================================================
//
// global monitor
//

#if 0 //use fltk window

struct sThreadInfoWin:public Fl_Double_Window {
  bool isOpen;
  //ofstream log;
  char outputbuf[200];
  sThreadInfoWin():Fl_Double_Window(0, 0, 600, 300, "processes"){
  }
  void draw();
};

ThreadInfoWin::ThreadInfoWin():Process("ThreadInfoX"){
  s=new sThreadInfoWin;
  s->isOpen=false;
}

ThreadInfoWin::~ThreadInfoWin(){
  if(s->isOpen) close();
  delete s;
}

void ThreadInfoWin::open(){
  //MT::open(log, "LOG.threads");
  Fl::visual(FL_DOUBLE|FL_INDEX);
  s->show();
  Fl::check();
  s->isOpen=true;
}

void ThreadInfoWin::close(){
  if(!s->isOpen) return;
  //XCloseDisplay(s->display);
  //s->log.close();
  s->isOpen=false;
}

void ThreadInfoWin::step(){
  if(!s->isOpen) open();
  s->redraw();
  Fl::wait(.1);
}

void sThreadInfoWin::draw(){
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
  for_list(i, proc, globalProcesses){
    th = proc->s;
    int state=th->threadCondition.state;
    x=5;
    TEXT("%4i", th->tid); x+=30;
    TEXT("%3i", th->threadPriority); x+=25;
    TEXT("%s", proc->name); x+=100;
    TEXT("%4i", th->timer.steps);  x+=30;
    if(state>0){ TEXT("%4i", state); } else switch(state){
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
  for_list(i, ct, globalCycleTimers){
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

ThreadInfoWin::ThreadInfoWin():Process("ThreadInfoX"){
  s=new sThreadInfoWin;
  s->isOpen=false;
}

ThreadInfoWin::~ThreadInfoWin(){
  if(s->isOpen) close();
  delete s;
}

void ThreadInfoWin::open(){
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
  global.readAccess(this);
  for_list(i, pr, global.processes){
    th = pr->s;
    int state=th->threadCondition.state;
    x=5;
    TEXT("%4i", th->tid); x+=25;
    TEXT("%3i", th->threadPriority); x+=25;
    TEXT("%s" , pr->name); x+=100;
    TEXT("%4i", th->timer.steps);  x+=30;
    if(state>0){ TEXT("%4i", state); } else switch(state){
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
  global.deAccess(this);
  y+=10;
  for_list(i, ct, globalCycleTimers){
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
