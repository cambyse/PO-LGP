#include "biros.h"
#include "biros_internal.h"
#include "views/views.h"
#include "views/specificViews.h"
#include "logging.h"
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
// global singleton
//

BirosInfo *global_birosInfo=NULL;

BirosInfo& birosInfo(){
  if(!global_birosInfo) global_birosInfo = new BirosInfo();
  return *global_birosInfo;
}


//===========================================================================
//
// helpers
//

Mutex parameterAccessGlobalMutex;

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



//===========================================================================
//
// Variable
//

Variable::Variable(const char *_name) {
  s = new sVariable();
  name = _name;
  revision = 0U;
  id = 0U;
  listeners.memMove=true;
  //MT logValues = false;
  //MT dbDrivenReplay = false;
  //MT pthread_mutex_init(&replay_mutex, NULL);
  if(global_birosInfo) { //-> birosInfo itself will not be registered!
    birosInfo().writeAccess(NULL);
    id = birosInfo().variables.N;
    birosInfo().variables.memMove = true;
    birosInfo().variables.append(this);
    birosInfo().deAccess(NULL);
  }
}

Variable::~Variable() {
  if(this != global_birosInfo) { //-> birosInfo itself will not be de-registered!
    birosInfo().writeAccess(NULL);
    birosInfo().variables.removeValue(this);
    birosInfo().deAccess(NULL);
  }
  for (uint i=0; i<fields.N; i++) delete fields(i);
  
  //MT pthread_mutex_destroy(&replay_mutex);
  
  delete s;
}

int Variable::readAccess(Process *p) {
  accessController.queryReadAccess(this, p);
  s->rwlock.readLock();
  accessController.logReadAccess(this, p);
  return revision;
}

int Variable::writeAccess(Process *p) {
  accessController.queryWriteAccess(this, p);
  s->rwlock.writeLock();
  revision++;
  accessController.logWriteAccess(this, p);
  uint i;  Process *l;
  s->cond.setState(revision);
  for_list(i, l, listeners) if(l!=p) l->threadStep();
  return revision;
}

int Variable::deAccess(Process *p) {
  if(s->rwlock.state == -1) { //log a revision after write access
    //MT logService.logRevision(this);
    //MT logService.setValueIfDbDriven(this); //this should be done within queryREADAccess, no?!
    accessController.logWriteDeAccess(this,p);
  } else {
    accessController.logReadDeAccess(this,p);
  }
  int rev=revision;
  s->rwlock.unlock();
  return rev;
}

void Variable::waitForNextWriteAccess(){
  s->cond.waitForSignal();
}

uint Variable::waitForRevisionGreaterThan(uint rev) {
  s->cond.lock();
  s->cond.waitForStateGreaterThan(rev, true);
  rev=s->cond.state;
  s->cond.unlock();
  return rev;
}

int Variable::lockState() {
  return s->rwlock.state;
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
  listensTo.memMove=true;
  name = _name;
  step_count = 0U;
  birosInfo().writeAccess(this);
  id = birosInfo().processes.N;
  birosInfo().processes.memMove=true;
  birosInfo().processes.append(this);
  birosInfo().deAccess(this);
}

Process::~Process() {
  if(s->thread || s->threadCondition.state!=tsCLOSE) threadClose();
  birosInfo().writeAccess(this);
  birosInfo().processes.removeValue(this);
  birosInfo().deAccess(this);
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
  v->s->rwlock.writeLock(); //don't want to increase revision and broadcast!
  v->listeners.setAppend(this);
  v->s->rwlock.unlock();
  listensTo.setAppend(v);
}

void Process::threadStopListenTo(Variable *v){
  v->s->rwlock.writeLock(); //don't want to increase revision and broadcast!
  v->listeners.removeValue(this);
  v->s->rwlock.unlock();
  listensTo.removeValue(v);
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
  birosInfo().writeAccess(NULL);
  id = birosInfo().parameters.N;
  birosInfo().parameters.memMove=true;
  birosInfo().parameters.append(this);
  birosInfo().deAccess(NULL);
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

Process *BirosInfo::getProcessFromPID() {
  pid_t tid = syscall(SYS_gettid);
  uint i;  Process *p;
  for_list(i, p, processes) {
    if(p->s->tid==tid) break;
  }
  if(i>=processes.N) return NULL;
  return p;
}

//TODO: move this to the b:dump control.h
void BirosInfo::dump() {
  cout <<" +++ VARIABLES +++" <<endl;
  uint i, j;
  Variable *v;
  Process *p;
  Parameter *par;
  FieldInfo *f;
  ViewInfo *vi;
  readAccess(NULL);
  for_list(i, v, variables) {
    cout <<"Variable " <<v->id <<' ' <<v->name <<" {\n  ";
    writeInfo(cout, *v, false, ' ');
    for_list(j, f, v->fields) {
      cout <<"\n  Field " <<j <<' ' <<f->name <<' ';
      writeInfo(cout, *f, false, ' ');
    }
    cout <<"\n}" <<endl;
  }
  cout <<"\n +++ PROCESSES +++" <<endl;
  for_list(i, p, processes) {
    cout <<"Process " <<p->id <<' ' <<p->name <<" {\n  ";
    writeInfo(cout, *p, false, ' ');
    cout <<"\n}" <<endl;
    /*<<" ("; //process doesn't contain list of variables anymore
    for_list(j, v, p->V){
      if(j) cout <<',';
      cout <<v->id <<'_' <<v->name;
    }
    cout <<") {" <<endl;*/
  }
  cout <<"\n +++ PARAMETERS +++" <<endl;
  for_list(i, par, parameters) {
    cout <<"Parameter " <<par->id <<' ' <<par->name <<" {\n  ";
    writeInfo(cout, *par, false, ' ');
    cout <<"\n  accessed by=";
    for_list(j, p, par->dependers) {
      if(j) cout <<',';
      cout <<' ' <<(p?p->name:STRING("NULL"));
    }
    cout <<"\n}" <<endl;
  }
  cout <<"\n +++ VIEWS +++" <<endl;
  for_list(i, vi, views) {
    cout <<"ViewInfo " <<i <<' ' <<vi->name <<' ';
    writeInfo(cout, *vi, false, ' ');
  }
  deAccess(NULL);
}


//===========================================================================
//
// implementation of helpers
//

void writeInfo(ostream& os, Process& p, bool brief, char nl){
#define TEXTTIME(dt) dt<<'|'<<dt##Mean <<'|' <<dt##Max
  if(brief){
    os <<p.s->timer.steps <<" [" <<std::setprecision(2) <<TEXTTIME(p.s->timer.busyDt)<<':' <<TEXTTIME(p.s->timer.cyclDt) <<']';
  }else{
    os <<"tid=" <<p.s->tid <<nl
       <<"priority=" <<p.s->threadPriority <<nl
       <<"steps=" <<p.s->timer.steps <<nl
       <<"busyDt=" <<TEXTTIME(p.s->timer.busyDt) <<nl
       <<"cycleDt=" <<TEXTTIME(p.s->timer.cyclDt) <<nl
       <<"state=";
    int state=p.stepState();
    if (state>0) os <<state; else switch (state) {
      case tsCLOSE:   os <<"close";  break;
      case tsLOOPING: os <<"loop";   break;
      case tsBEATING: os <<"beat";   break;
      case tsIDLE:    os <<"idle";   break;
      default: os <<"undefined:";
    }
#undef TEXTTIME
  }
}

void writeInfo(ostream& os, Variable& v, bool brief, char nl){
  if(brief){
    os <<v.revision;
  }else{
    os <<"revision=" <<v.revision <<nl
       <<"type=" <<typeid(v).name() <<nl
       <<"lock-state=" <<v.lockState();
  }
}

void writeInfo(ostream& os, FieldInfo& f, bool brief, char nl){
  if(brief){
    MT::String str;
    f.writeValue(str);
    if(str.N>20) str.resize(20,true);
    os <<str;
  }else{
    os <<"value=";
    f.writeValue(os);
    os <<nl <<"type=" <<f.userType;
  }
}

void writeInfo(ostream& os, Parameter& pa, bool brief, char nl){
  if(brief){
    MT::String str;
    pa.writeValue(str);
    if(str.N>20) str.resize(20,true);
    for(uint i=0;i<str.N;i++) if(str(i)=='\n') str(i)=' ';
    os <<str;
  }else{
    os <<"value=";
    pa.writeValue(os);
    os <<nl <<"type=" <<pa.typeName();
  }
}

void writeInfo(ostream& os, ViewInfo& vi, bool brief, char nl){
  /*os <<"type=";
  switch (vi.type) {
    case ViewInfo::fieldVT:    os <<"field";  break;
    case ViewInfo::variableVT: os <<"variable";  break;
    case ViewInfo::processVT:  os <<"process";  break;
    case ViewInfo::parameterVT:os <<"parameter";  break;
    case ViewInfo::globalVT:   os <<"global";  break;
  }*/
  os <<nl <<"applies_on=" <<vi.appliesOn_sysType <<endl;
}

