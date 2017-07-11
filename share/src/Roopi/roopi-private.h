#pragma once

#include "roopi.h"

#include <Kin/kinViewer.h>
#include <Control/taskControl.h>

//struct CtrlTaskUpdater : Thread {
//  ACCESS(ActL, acts)
//  ACCESS(CtrlTaskL, ctrlTasks)
//  int verbose;

//  CtrlTaskUpdater(int verbose=0) : Thread("CtrlTaskUpdater", .05), verbose(verbose) {
//    threadLoop();
//  }
//  ~CtrlTaskUpdater(){ threadClose(); }

//  virtual void open(){}
//  virtual void step(){
//    acts.readAccess();
//    for(Act *a:acts()){
//      Act_CtrlTask *c = dynamic_cast<Act_CtrlTask*>(a);
//      if(c && c->task && c->task->ref){
//        ctrlTasks.readAccess();
//        bool conv = c->task->ref->isDone();
//        ctrlTasks.deAccess();
//        bool sconv = (c->getStatus()==AS_converged);
//        if(conv!=sconv){
//          if(verbose) cout <<"setting status: " <<c->task->name <<" conv=" <<conv <<endl;
//          c->setStatus(conv?AS_converged:AS_running);
//        }
//      }
//    }
//    acts.deAccess();
//  }
//  virtual void close(){}

//};


struct Roopi_private {
  Access<mlr::KinematicWorld> modelWorld;
  Access<mlr::Array<CtrlTask*>> ctrlTasks;

  mlr::String model;
  mlr::String robot;
  bool useRos;
  double hyperSpeed;
  arr q0;

  // persistent acts
  shared_ptr<Act_TaskController>_taskController;
  shared_ptr<Act_CtrlTask> _holdPositionTask;
  shared_ptr<Act_CtrlTask> _watchTask;
  shared_ptr<Act_CtrlTask> _collTask;
  shared_ptr<Act_Tweets> _tweets;
  shared_ptr<Act_Thread> _ComRos;
  shared_ptr<Act_ComPR2> _ComPR2;
  shared_ptr<Act_Thread> _ctrlView;
  shared_ptr<Act_Thread> _taskUpdater;

  //-- logging
  struct LoggingModule *loggingModule = NULL;

  Roopi_private(Roopi *roopi);

  ~Roopi_private();
};


