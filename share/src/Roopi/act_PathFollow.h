#pragma once

#include <Core/thread.h>
#include "roopi.h"
#include <Algo/spline.h>

struct FollowPath : Thread{
  Roopi &roopi;
  const char* name;
  arr path;
  TaskMap* map;
  CtrlTask *ct;
  double executionTime;
  double startTime;
  mlr::Spline spline;

  FollowPath(Roopi& r, const char* name, const arr& path, TaskMap* map, double executionTime)
    : Thread("Roopi_FollowPath", .02),
      roopi(r), name(name), path(path), map(map), ct(NULL), executionTime(executionTime), startTime(0.), spline(path.d0, path){
  }

  void open(){
    ct = roopi.createCtrlTask(STRING("FollowPath_"<<name), map, false);
    ct->setC(ARR(1000.0));
    ct->setGains(ARR(30.0), ARR(5.0));
    startTime = mlr::realTime();
  }

  void close(){
    roopi.destroyCtrlTask(ct);
  }

  void step(){
//    tcm()->ctrlTasks.writeAccess();
//    for(CtrlTask *t:tcm()->ctrlTasks()) t->active=false;
//    for(CtrlTask *t:tasks) t->active=true; //they were 'added' to the ctrlTasks list on creation!!
//  //  tcm()->verbose=1;
//    tcm()->ctrlTasks.deAccess();

    double time = mlr::realTime() - startTime;
    double s = time/executionTime;
    if(s > 1.) {
      cout << "finished execution of trajectory" << endl;
    }

    roopi.modifyCtrlTaskReference(ct, spline.eval(s));
  }

};
