#include <biros/biros.h>
#include <biros/biros_internal.h>
//#include "biros_logger.h"
//#include "biros_threadless.h"

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <sys/resource.h>
#include <sys/prctl.h>
#include <iostream>
#include <X11/Xlib.h>

#include <MT/util.h>
#include <MT/array.h>


//===========================================================================
//
// helpers
//

MT::Array<Metronome *> globalMetronomes;
MT::Array<CycleTimer*> globalCycleTimers;

void reportNice() {
  pid_t tid = syscall(SYS_gettid);
  int priority = getpriority(PRIO_PROCESS, tid);
  std::cout <<"tid=" <<tid <<" nice=" <<priority <<std::endl;
}

bool setNice(int nice) {
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

void setRRscheduling(int priority) {
  pid_t tid = syscall(SYS_gettid);
  MT_MSG(" tid=" <<tid <<" old sched=" <<sched_getscheduler(tid));
  sched_param sp; sp.sched_priority=priority;
  int rc = sched_setscheduler(tid, SCHED_RR, &sp);
  if(rc) switch (errno) {
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

void updateTimeIndicators(double& dt, double& dtMean, double& dtMax, const timespec& now, const timespec& last, uint step) {
  dt=double(now.tv_sec-last.tv_sec-1)*1000. +
     double(1000000000l+now.tv_nsec-last.tv_nsec)/1000000.;
  if(dt<0.) dt=0.;
  double rate=.01;  if(step<100) rate=1./(1+step);
  dtMean = (1.-rate)*dtMean    + rate*dt;
  if(dt>dtMax || !(step%100)) dtMax = dt;
}


//===========================================================================
//
// Mutex
//

Mutex::Mutex() {
  pthread_mutexattr_t atts;
  int rc;
  rc = pthread_mutexattr_init(&atts);  if(rc) HALT("pthread failed with err " <<rc <<strerror(rc));
  rc = pthread_mutexattr_settype(&atts, PTHREAD_MUTEX_RECURSIVE_NP);  if(rc) HALT("pthread failed with err " <<rc <<strerror(rc));
  rc = pthread_mutex_init(&mutex, &atts);
  //mutex = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
  state=0;
}

Mutex::~Mutex() {
  CHECK(!state, "");
  int rc = pthread_mutex_destroy(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void Mutex::lock() {
  int rc = pthread_mutex_lock(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  state=syscall(SYS_gettid);
}

void Mutex::unlock() {
  state=0;
  int rc = pthread_mutex_unlock(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}


//===========================================================================
//
// Access Lock
//

Lock::Lock() {
  int rc = pthread_rwlock_init(&lock, NULL);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  state=0;
}

Lock::~Lock() {
  CHECK(!state, "");
  int rc = pthread_rwlock_destroy(&lock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void Lock::readLock() {
  int rc = pthread_rwlock_rdlock(&lock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  stateMutex.lock();
  state++;
  stateMutex.unlock();
}

void Lock::writeLock() {
  int rc = pthread_rwlock_wrlock(&lock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  stateMutex.lock();
  state=-1;
  stateMutex.unlock();
}

void Lock::unlock() {
  stateMutex.lock();
  if(state>0) state--; else state=0;
  stateMutex.unlock();
  int rc = pthread_rwlock_unlock(&lock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}


//===========================================================================
//
// ConditionVariable
//

ConditionVariable::ConditionVariable(int initialState) {
  int rc = pthread_cond_init(&cond, NULL);    if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  state=initialState;
}

ConditionVariable::~ConditionVariable() {
  int rc = pthread_cond_destroy(&cond);    if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void ConditionVariable::setState(int i) {
  stateMutex.lock();
  state=i;
  broadcast();
  stateMutex.unlock();
}

void ConditionVariable::broadcast() {
  int rc = pthread_cond_broadcast(&cond);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void ConditionVariable::lock() {
  stateMutex.lock();
}

void ConditionVariable::unlock() {
  stateMutex.unlock();
}

int ConditionVariable::getState(bool userLock) {
  if(!userLock) stateMutex.lock(); else CHECK(stateMutex.state==syscall(SYS_gettid),"user must have locked before calling this!");
  int i=state;
  if(!userLock) stateMutex.unlock();
  return i;
}

void ConditionVariable::waitForSignal(bool userLock) {
  if(!userLock) stateMutex.lock(); else CHECK(stateMutex.state==syscall(SYS_gettid),"user must have locked before calling this!");
  int rc = pthread_cond_wait(&cond, &stateMutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  if(!userLock) stateMutex.unlock();
}

void ConditionVariable::waitForSignal(double seconds, bool userLock) {
  struct timespec timeout;
  clock_gettime(CLOCK_MONOTONIC, &timeout);
  timeout.tv_nsec+=1000000000l*seconds;
  if(timeout.tv_nsec>1000000000l) {
    timeout.tv_sec+=1;
    timeout.tv_nsec-=1000000000l;
  }
  
  if(!userLock) stateMutex.lock(); else CHECK(stateMutex.state==syscall(SYS_gettid),"user must have locked before calling this!");
  int rc = pthread_cond_timedwait(&cond, &stateMutex.mutex, &timeout);
  if(rc && rc!=ETIMEDOUT) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  if(!userLock) stateMutex.unlock();
}

void ConditionVariable::waitForStateEq(int i, bool userLock) {
  if(!userLock) stateMutex.lock(); else CHECK(stateMutex.state==syscall(SYS_gettid),"user must have locked before calling this!");
  while (state!=i) {
    int rc = pthread_cond_wait(&cond, &stateMutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  }
  if(!userLock) stateMutex.unlock();
}

void ConditionVariable::waitForStateNotEq(int i, bool userLock) {
  if(!userLock) stateMutex.lock(); else CHECK(stateMutex.state==syscall(SYS_gettid),"user must have locked before calling this!");
  while (state==i) {
    int rc = pthread_cond_wait(&cond, &stateMutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  }
  if(!userLock) stateMutex.unlock();
}

void ConditionVariable::waitForStateGreaterThan(int i, bool userLock) {
  if(!userLock) stateMutex.lock(); else CHECK(stateMutex.state==syscall(SYS_gettid),"user must have locked before calling this!");
  while (state<=i) {
    int rc = pthread_cond_wait(&cond, &stateMutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  }
  if(!userLock) stateMutex.unlock();
}


void ConditionVariable::waitUntil(double absTime, bool userLock) {
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

Metronome::Metronome(const char* _name, long _targetDt) {
  globalMetronomes.memMove=true;
  globalMetronomes.append(this);
  name=_name;
  reset(_targetDt);
}

Metronome::~Metronome() {
  globalMetronomes.removeValue(this);
}

void Metronome::reset(long _targetDt=0) {
  clock_gettime(CLOCK_MONOTONIC, &ticTime);
  lastTime=ticTime;
  tics=0;
  targetDt = _targetDt;
}

void Metronome::waitForTic() {
  //compute target time
  ticTime.tv_nsec+=1000000l*targetDt;
  while (ticTime.tv_nsec>1000000000l) {
    ticTime.tv_sec+=1;
    ticTime.tv_nsec-=1000000000l;
  }
  //wait for target time
  int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ticTime, NULL);
  if(rc) {
    if(rc==0) { MT_MSG("clock_nanosleep() interrupted by signal") } else { MT_MSG("clock_nanosleep() failed " <<rc); }
  }
  //}
  
  tics++;
}

double Metronome::getTimeSinceTic() {
  timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return double(now.tv_sec-ticTime.tv_sec) + 1e-9*(now.tv_nsec-ticTime.tv_nsec);
}


//===========================================================================
//
// CycleTimer
//

CycleTimer::CycleTimer(const char* _name) {
  reset();
  name=_name;
  globalCycleTimers.memMove=true;
  if(name) globalCycleTimers.append(this);
}

CycleTimer::~CycleTimer() {
  if(name) globalCycleTimers.removeValue(this);
}

void CycleTimer::reset() {
  steps=0;
  busyDt=busyDtMean=busyDtMax=1.;
  cyclDt=cyclDtMean=cyclDtMax=1.;
  clock_gettime(CLOCK_MONOTONIC, &lastTime);
}

void CycleTimer::cycleStart() {
  clock_gettime(CLOCK_MONOTONIC, &now);
  updateTimeIndicators(cyclDt, cyclDtMean, cyclDtMax, now, lastTime, steps);
  lastTime=now;
}

void CycleTimer::cycleDone() {
  clock_gettime(CLOCK_MONOTONIC, &now);
  updateTimeIndicators(busyDt, busyDtMean, busyDtMax, now, lastTime, steps);
  steps++;
}


//===========================================================================
//
// Variable
//

Variable::Variable(const char *_name) {
  s = new sVariable();
  name = _name;
  revision = 0U;
  id = 0U;
  //MT logValues = false;
  //MT dbDrivenReplay = false;
  //MT pthread_mutex_init(&replay_mutex, NULL);
  if(this != &birosInfo) {
    birosInfo.writeAccess(NULL);
    id = birosInfo.variables.N;
    birosInfo.variables.memMove = true;
    birosInfo.variables.append(this);
    birosInfo.deAccess(NULL);
  }
}

Variable::~Variable() {
  if(this != &birosInfo) {
    birosInfo.writeAccess(NULL);
    birosInfo.variables.removeValue(this);
    birosInfo.deAccess(NULL);
  }
  for (uint i=0; i<fields.N; i++) delete fields(i);
  
  //MT pthread_mutex_destroy(&replay_mutex);
  
  delete s;
}

int Variable::readAccess(Process *p) {
  //MT logService.queryReadAccess(this, p);
  s->lock.readLock();
  //MT logService.logReadAccess(this, p);
  return revision;
}

int Variable::writeAccess(Process *p) {
  //MT logService.queryWriteAccess(this, p);
  s->lock.writeLock();
  //MT pthread_mutex_lock(&replay_mutex); //TODO: why do I need this lock?
  revision++;
  //MT pthread_mutex_unlock(&replay_mutex);
  //MT logService.logWriteAccess(this, p);
  uint i;  Process *l;
  s->cond.setState(revision);
  for_list(i, l, listeners) if(l!=p) l->threadStep();
  return revision;
}

int Variable::deAccess(Process *p) {
  if(s->lock.state == -1) { //log a revision after write access
    //MT logService.logRevision(this);
    //MT logService.setValueIfDbDriven(this);
    //MT logService.freeWriteAccess(this);
  } else {
    //MT logService.freeReadAccess(this);
  }
  int rev=revision;
  s->lock.unlock();
  return rev;
}

int Variable::waitForRevisionGreaterThan(uint rev) {
  s->cond.lock();
  s->cond.waitForStateGreaterThan(rev, true);
  int state=s->cond.state;
  s->cond.unlock();
  return state;
}

int Variable::lockState() {
  return s->lock.state;
}

void Variable::serializeToString(MT::String &string) const {
  string.clear();
  MT::String field_string;
  field_string.clear();
  
  // go through fields
  for (uint i=0; i < fields.N; i++) {
  
    fields(i)->writeValue(field_string);
    
    // replace every occurence of "\" by "\\"
    for (uint j=0; j < field_string.N; j++) {
      char c = field_string(j);
      if('\\' == c) string << '\\';
      string << c;
    }
    
    // add seperator after field
    string << "\\,";
  }
}

void Variable::deSerializeFromString(const MT::String &string) {
  MT::String string_copy(string), field_string;
  field_string.clear();
  uint j = 0;
  for (uint i=0; i< fields.N; i++) {
    // get field strings from string (seperated by "\\,")
    bool escaped = false; // true if previous char was '\\'
    while (j < string_copy.N) {
      char c = string_copy(j++);
      if('\\' == c) {
        escaped = true;
      } else {
        if(escaped) {
          if(',' == c) {
            break;
          }
        }
        escaped = false;
        field_string << c;
      }
    }
    fields(i)->readValue(field_string);
  }
}


//===========================================================================
//
// Process
//


Process::Process(const char *_name) {
  s = new sProcess();
  name = _name;
  step_count = 0U;
  birosInfo.writeAccess(this);
  id = birosInfo.processes.N;
  birosInfo.processes.memMove=true;
  birosInfo.processes.append(this);
  birosInfo.deAccess(this);
}

Process::~Process() {
  if(s->thread || s->threadCondition.state!=tsCLOSE) threadClose();
  birosInfo.writeAccess(this);
  birosInfo.processes.removeValue(this);
  birosInfo.deAccess(this);
  delete s;
}

int Process::stepState() {
  return s->threadCondition.getState();
}

void Process::threadOpen(int priority) {
  s->threadCondition.lock();
  if(s->thread){ s->threadCondition.unlock(); return; } //this is already open -- or has just beend opened (parallel call to threadOpen)
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
  s->threadCondition.state=tsIDLE;
  s->threadCondition.unlock();
}

void Process::threadClose() {
  if(!s->thread) return; // we were here already
  s->threadCondition.setState(tsCLOSE);
  int rc;
  CHECK(s->thread, "parallel call to threadClose -> NIY");
  rc = pthread_join(s->thread, NULL);     if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  s->thread = 0;
}

void Process::threadStep(uint steps, bool wait) {
  if(!s->thread) threadOpen();
  if(wait) threadWaitIdle();
  //CHECK(s->threadCondition.state==tsIDLE, "never step while thread is busy!");
  s->threadCondition.setState(steps);
}

void Process::threadListenTo(const VariableL &signalingVars) {
  uint i;  Variable *v;
  for_list(i, v, signalingVars) threadListenTo(v);
}

void Process::threadListenTo(Variable *v) {
  v->s->lock.writeLock(); //don't want to increase revision and broadcast!
  v->listeners.setAppend(this);
  v->s->lock.unlock();
  listensTo.append(v);
}

bool Process::threadIsIdle() {
  return s->threadCondition.getState()==tsIDLE;
}

bool Process::threadIsClosed() {
  return s->threadCondition.getState()==tsCLOSE;
}

void Process::threadWaitIdle() {
  s->threadCondition.waitForStateEq(tsIDLE);
}

void Process::threadLoop() {
  if(!s->thread) threadOpen();
  s->threadCondition.setState(tsLOOPING);
}

void Process::threadLoopWithBeat(double sec) {
  if(!s->metronome)
    s->metronome=new Metronome("threadTiccer", 1000.*sec);
  else
    s->metronome->reset(1000.*sec);
  if(!s->thread) threadOpen();
  s->threadCondition.setState(tsBEATING);
}

void Process::threadStop() {
  CHECK(s->thread, "called stop to closed thread");
  s->threadCondition.setState(tsIDLE);
}

void* sProcess::staticThreadMain(void *_self) {
  Process  *proc=(Process*)_self;
  sProcess *s   =proc->s;
  //std::cout <<" +++ entering staticThreadMain of '" <<proc->name <<'\'' <<std::endl;
  
  s->tid = syscall(SYS_gettid);
  
  //http://linux.die.net/man/3/setpriority
  //if(s->threadPriority) setRRscheduling(s->threadPriority);
  
  if(s->threadPriority) setNice(s->threadPriority);
  prctl(PR_SET_NAME, proc->name.p);
  //pthread_setname_np(proc->thread, proc->name);
  
  proc->open(); //virtual initialization routine

  s->timer.reset();
  for(;;){
    bool waitForTic=false;
    s->threadCondition.lock();
    s->threadCondition.waitForStateNotEq(tsIDLE, true);
    if(s->threadCondition.state==tsCLOSE) break;
    if(s->threadCondition.state==tsBEATING) waitForTic=true;
    if(s->threadCondition.state>0) s->threadCondition.state--; //count down
    s->threadCondition.unlock();
    
    if(waitForTic) s->metronome->waitForTic();
    
    s->timer.cycleStart();
    proc->step(); //virtual step routine
    proc->step_count++;
    s->timer.cycleDone();
  };
  s->threadCondition.unlock();
  
  proc->close(); //virtual close routine
  
  //std::cout <<" +++ exiting staticThreadMain of '" <<proc->name <<'\'' <<std::endl;
  return NULL;
}


//===========================================================================
//
// Parameter
//

Parameter::Parameter() {
  birosInfo.writeAccess(NULL);
  id = birosInfo.parameters.N;
  birosInfo.parameters.memMove=true;
  birosInfo.parameters.append(this);
  birosInfo.deAccess(NULL);
};


//===========================================================================
//
// Group
//

void open(const ProcessL& P) {
  Process *p; uint i;
  for_list(i, p, P) p->threadOpen();
}

void step(const ProcessL& P) {
  Process *p; uint i;
  for_list(i, p, P) p->threadStep();
}

void loop(const ProcessL& P) {
  Process *p; uint i;
  for_list(i, p, P) p->threadLoop();
}

void loopWithBeat(const ProcessL& P, double sec) {
  Process *p; uint i;
  for_list(i, p, P) p->threadLoopWithBeat(sec);
}

void stop(const ProcessL& P) {
  Process *p; uint i;
  for_list(i, p, P) p->threadStop();
}

void wait(const ProcessL& P) {
  Process *p; uint i;
  for_list(i, p, P) p->threadWaitIdle();
}

void close(const ProcessL& P) {
  Process *p; uint i;
  for_list(i, p, P) p->threadClose();
}


//===========================================================================
//
// Global information
//

BirosInfo birosInfo;

Process *BirosInfo::getProcessFromPID() {
  pid_t tid = syscall(SYS_gettid);
  uint i;  Process *p;
  for_list(i, p, processes) {
    if(p->s->tid==tid) break;
  }
  if(i>=processes.N) return NULL;
  return p;
}

#define TEXTTIME(dt) dt<<'|'<<dt##Mean <<'|' <<dt##Max

void BirosInfo::dump() {
  cout <<" +++ VARIABLES +++" <<endl;
  uint i, j;
  Variable *v;
  Process *p;
  Parameter *par;
  FieldInfo *vi;
  readAccess(NULL);
  for_list(i, v, variables) {
    cout <<"Variable " <<v->id <<'_' <<v->name <<" lock-state=" <<v->lockState();
    if(v->fields.N) {
      cout <<'{' <<endl;
      for_list(j, vi, v->fields) {
        cout <<"   field " <<j <<' ' <<vi->name <<' ' <<vi->p <<" value=";
        vi->writeValue(cout);
        cout <<endl;
      }
      cout <<"\n}";
    }
    //<<" {" <<endl;
    cout <<endl;
  }
  cout <<endl;
  cout <<" +++ PROCESSES +++" <<endl;
  for_list(i, p, processes) {
    cout <<"Process " <<p->id <<'_' <<p->name <<" {" <<endl;
    /*<<" ("; //process doesn't contain list of variables anymore
    for_list(j, v, p->V){
      if(j) cout <<',';
      cout <<v->id <<'_' <<v->name;
    }
    cout <<") {" <<endl;*/
    cout
      <<" tid=" <<p->s->tid
      <<" priority=" <<p->s->threadPriority
      <<" steps=" <<p->s->timer.steps
      <<" cycleDt=" <<TEXTTIME(p->s->timer.cyclDt)
      <<" busyDt=" <<TEXTTIME(p->s->timer.busyDt)
      <<" state=";
    int state=p->s->threadCondition.state;
    if(state>0) cout <<state; else switch (state) {
        case tsCLOSE:   cout <<"close";  break;
        case tsLOOPING: cout <<"loop";   break;
        case tsBEATING: cout <<"beat";   break;
        case tsIDLE:    cout <<"idle";   break;
        default: cout <<"undefined:";
      }
    cout <<"\n}" <<endl;
  }
  cout <<endl;
  cout <<" +++ PARAMETERS +++" <<endl;
  for_list(i, par, parameters) {
    cout <<"Parameter " <<par->id <<'_' <<par->name <<" value=";
    par->writeValue(cout);
    cout <<" accessed by:";
    for_list(j, p, par->dependers) {
      if(j) cout <<',';
      cout <<' ' <<(p?p->name:STRING("NULL"));
    }
    cout <<endl;
  }
  deAccess(NULL);
}

#undef TEXTTIME

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
  birosInfo.readAccess(this);
  for_list(i, pr, birosInfo.processes) {
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
  birosInfo.deAccess(this);
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
