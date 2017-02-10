#include "act_PathFollow.h"
#include "act_CtrlTask.h"

#include <Algo/spline.h>
#include <Core/thread.h>
#include <Control/taskController.h>

struct sAct_FollowPath : Thread{
  Act *a;
  Act_CtrlTask ct;
  double executionTime;
  mlr::Spline spline;

  sAct_FollowPath(Act *a, const char* name, const arr& path, TaskMap* map, double executionTime)
    : Thread(name, .02), a(a), ct(&a->roopi), executionTime(executionTime) {
    ct.setMap(map);

    uint qdim = ct.y0.N;
    spline.points = path;
    spline.points.reshape(path.N/qdim, qdim);
    spline.setUniformNonperiodicBasis();

    ct.task->PD().y_target = spline.eval(0.);
    ct.task->PD().setGainsAsNatural(.1, .9);
  //  ct.task->setGains(30., 10.);
  }

  void open();
  void close();
  void step();

};

Act_FollowPath::Act_FollowPath(Roopi *r, const char* name, const arr& path, TaskMap* map, double executionTime)
  : Act(r){
  s = new sAct_FollowPath(this, name, path, map, executionTime);
}

Act_FollowPath::~Act_FollowPath(){
  s->threadClose();
  delete s;
}

void Act_FollowPath::start(){
  s->threadLoop();
}

void sAct_FollowPath::open(){
  ct.start();
  a->startTime = mlr::realTime();
}

void sAct_FollowPath::close(){
}

void sAct_FollowPath::step(){
  double s = a->time()/executionTime;
  if(s > 1.) {
    a->status.setValue(AS_done);
    cout << "finished execution of trajectory" << endl;
    s = 1.;
    threadStop();
  }

  ct.set()->PD().y_target = spline.eval(s);
  ct.set()->PD().v_target = spline.eval(s, 1)/executionTime;
}
