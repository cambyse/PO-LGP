#pragma once

#include "roopi.h"

#include <Kin/kinViewer.h>
#include <Control/taskControl.h>

struct CtrlTaskUpdater : Thread {
  ACCESS(ActL, acts)
  ACCESS(CtrlTaskL, ctrlTasks)
  int verbose;

  CtrlTaskUpdater(int verbose=0) : Thread("CtrlTaskUpdater", .05), verbose(verbose) {
    threadLoop();
  }
  ~CtrlTaskUpdater(){ threadClose(); }

  virtual void open(){}
  virtual void step(){
    acts.readAccess();
    for(Act *a:acts()){
      Act_CtrlTask *c = dynamic_cast<Act_CtrlTask*>(a);
      if(c && c->task && c->task->ref){
        ctrlTasks.readAccess();
        bool conv = c->task->ref->isDone();
        ctrlTasks.deAccess();
        bool sconv = (c->getStatus()==AS_converged);
        if(conv!=sconv){
          if(verbose) cout <<"setting status: " <<c->task->name <<" conv=" <<conv <<endl;
          c->setStatus(conv?AS_converged:AS_running);
        }
      }
    }
    acts.deAccess();
  }
  virtual void close(){}

};


struct Roopi_private {
  Access_typed<mlr::KinematicWorld> modelWorld;
  ACCESSname(mlr::Array<CtrlTask*>, ctrlTasks)

  mlr::String model;
  mlr::String robot;
  bool useRos;
  arr q0;

  // persistent acts
  Act_TaskController *_taskController = NULL;
  Act_CtrlTask *_holdPositionTask = NULL;
  Act_CtrlTask *_watchTask = NULL;
  Act_CtrlTask *_collTask = NULL;
  Act_Tweets *_tweets = NULL;
  Act_Thread *_ComRos=NULL;
  Act_ComPR2 *_ComPR2=NULL;
  Act_Thread *_ctrlView = NULL;
  Act_Thread *_taskUpdater = NULL;

  //-- logging
  struct LoggingModule *loggingModule = NULL;

  Roopi_private(Roopi *roopi);

  ~Roopi_private();
};

