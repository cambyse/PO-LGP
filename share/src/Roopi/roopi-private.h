#pragma once

#include "roopi.h"

#include <Kin/kinViewer.h>
#include <Control/taskControl.h>

struct CtrlTaskUpdater : Thread {
  ACCESS(ActL, acts)
  ACCESS(CtrlTaskL, ctrlTasks)

  CtrlTaskUpdater() : Thread("CtrlTaskUpdater", .05) {}

  virtual void open(){}
  virtual void step(){
    acts.readAccess();
    for(Act *a:acts()){
      Act_CtrlTask *c = dynamic_cast<Act_CtrlTask*>(a);
      if(c && c->task){
        ctrlTasks.readAccess();
        bool conv = c->task->ref->isDone();
        ctrlTasks.deAccess();
        bool sconv = (c->getStatus()==AS_converged);
        if(conv!=sconv){
          cout <<"setting status: " <<c->task->name <<" conv=" <<conv <<endl;
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

  // persistent acts
  Act_TaskController *_tcm = NULL;
  Act_CtrlTask *_holdPositionTask = NULL;
  Act_Tweets *_tweets = NULL;
  Act *_ComRos=NULL, *_ComPR2=NULL;
  Act_Thread<OrsPoseViewer> *ctrlView = NULL;
  Act_Thread<CtrlTaskUpdater> *_updater = NULL;


  //-- logging
  struct LoggingModule *loggingModule = NULL;



  Roopi_private(Roopi *roopi);

  ~Roopi_private();
};


