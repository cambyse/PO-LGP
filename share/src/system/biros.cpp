/**
 * @file
 * @ingroup group_biros
 */
/**
 * @addtogroup group_biros
 * @{
 */
#include "biros.h"
#include "engine.h"
#include "biros_internal.h"
#include "views/views.h"
//#include "views/specificViews.h"
//#include "logging.h"
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


Biros *global_birosInfo=NULL;

/**
 * Access biros from everywhere.
 * It is a global singleton.
 */
Biros& biros(){
  static bool currentlyCreating=false;
  if(currentlyCreating) return *((Biros*) NULL);
  if(!global_birosInfo) {
    static Mutex m;
    m.lock();
    if(!global_birosInfo) {
      currentlyCreating=true;
      global_birosInfo = new Biros();
      currentlyCreating=false;
    }
    m.unlock();
  }
  return *global_birosInfo;
}

struct BirosDestructorDemon{
  ~BirosDestructorDemon(){ if(global_birosInfo) delete global_birosInfo; global_birosInfo=NULL; }
};

//===========================================================================
//
// helpers
//

Mutex parameterAccessGlobalMutex;

void registerField(Variable *v, FieldRegistration* f){
  v->s->fields.append(f);
}

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
// Parameter
//

Parameter::Parameter() {
  biros().writeAccess(NULL);
  biros().parameters.memMove=true;
  biros().parameters.append(this);
  biros().deAccess(NULL);
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

/**
 * @brief Execute the step function of all processes in the given order.
 *
 * @note This does not use any thread functionality, i.e. the processes are not
 * executed in parallel.
 *
 * @param P list of processes.
 */
void stepInSequence(const ProcessL& P) {
  //Process *p; uint i;
  NIY//for_list(i, p, P) p->step();
}

/**
 * @brief Execute the step function of all processes in the given order but
 * treat the processes as threads.
 *
 * @param P list of processes.
 */
void stepInSequenceThreaded(const ProcessL& P) {
  Process *p; uint i;
  for_list(i, p, P) {
    p->threadStep();
    p->waitForIdle();
  }
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
  for_list(i, p, P) p->waitForIdle();
}

void close(const ProcessL& P) {
  Process *p; uint i;
  for_list(i, p, P) p->threadClose();
}

//===========================================================================
//
// Global information
//

Biros::Biros():Variable("Biros") {
  //acc = new sBirosEventController;
};

Biros::~Biros(){
  //acc -> dumpEventList();
  //delete acc;
}

Process *Biros::getProcessFromPID() {
  pid_t tid = syscall(SYS_gettid);
  uint i;  Process *p;
  for_list(i, p, processes) {
    if(p->tid==tid) break;
  }
  if(i>=processes.N) return NULL;
  return p;
}

/**
 * @brief Spit out/print all infos about the current biros state.
 */
void Biros::dump() {
  cout <<" +++ VARIABLES +++" <<endl;
  uint i, j;
  Variable *v;
  Process *p;
  Parameter *par;
  FieldRegistration *f;
  readAccess(NULL);
  for_list(i, v, variables) {
    cout <<"Variable " <<v->name <<" {\n  ";
    writeInfo(cout, *v, false, ' ');
    for_list(j, f, v->s->fields) {
      cout <<"\n  Field " <<j <<' ' <<f->name <<' ';
      writeInfo(cout, *f, false, ' ');
    }
    cout <<"\n}" <<endl;
  }
  cout <<"\n +++ PROCESSES +++" <<endl;
  for_list(i, p, processes) {
    cout <<"Process " <<p->module->name <<" {\n  ";
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
    cout <<"Parameter " <<par->name <<" {\n  ";
    writeInfo(cout, *par, false, ' ');
    cout <<"\n  accessed by=";
    for_list(j, p, par->dependers) {
      if(j) cout <<',';
      cout <<' ' <<(p?p->module->name:STRING("NULL"));
    }
    cout <<"\n}" <<endl;
  }
  deAccess(NULL);
}


//===========================================================================
//
// implementation of helpers
//

void writeInfo(ostream& os, Process& p, bool brief, char nl){
  if(brief){
    os <<p.module->step_count <<endl;
  }else{
    os <<"tid=" <<p.tid <<nl
       <<"steps=" <<p.module->step_count
       <<"state=";
    int state=p.state.getValue();
    if (state>0) os <<state; else switch (state) {
      case tsCLOSE:   os <<"close";  break;
      case tsLOOPING: os <<"loop";   break;
      case tsBEATING: os <<"beat";   break;
      case tsIDLE:    os <<"idle";   break;
      default: os <<"undefined:";
    }
  }
}

void writeInfo(ostream& os, Variable& v, bool brief, char nl){
  if(brief){
    os <<v.revision.getValue();
  }else{
    os <<"revision=" <<v.revision.getValue() <<nl
       <<"type=" <<typeid(v).name() <<nl
       <<"lock-state=" <<v.rwlock.state;
  }
}

void writeInfo(ostream& os, FieldRegistration& f, bool brief, char nl){
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

//void writeInfo(ostream& os, ViewRegistration& vi, bool brief, char nl){
  /*os <<"type=";
  switch (vi.type) {
    case ViewRegistration::fieldVT:    os <<"field";  break;
    case ViewRegistration::variableVT: os <<"variable";  break;
    case ViewRegistration::processVT:  os <<"process";  break;
    case ViewRegistration::parameterVT:os <<"parameter";  break;
    case ViewRegistration::globalVT:   os <<"global";  break;
  }*/
//  os <<nl <<"applies_on=" <<vi.appliesOn_sysType <<endl;
//}

/** @} */
