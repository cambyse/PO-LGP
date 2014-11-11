#include "simulator.h"
#include <Ors/ors.h>
#include <Core/util.h>
#include <Gui/opengl.h>

struct sPR2Simulator{
  ors::KinematicWorld pr2, openglCopy;
  double t, dt;
  uint step;
  sPR2Simulator():t(0.), dt(.001), step(0){}
};

PR2Simulator::PR2Simulator():Module("PR2Simulator"), s(NULL){}

PR2Simulator::~PR2Simulator(){}

void PR2Simulator::open(){
  s = new sPR2Simulator;
  s->pr2 <<FILE("model.kvg");
  s->openglCopy = s->pr2;
  s->openglCopy.gl().update(STRING("PR2 Simulator. time=" <<s->t));
  q_obs.set() = s->pr2.q;
  qdot_obs.set() = s->pr2.qdot;
  q_ref.set() = s->pr2.q;
  qdot_ref.set() = s->pr2.qdot;
}

void PR2Simulator::step(){
  arr qref = q_ref.get();
  arr qdotref = qdot_ref.get();
#if 0
  //-- joint gains
  arr qddot = 10.*(qref - s->pr2.q) + 10.*(qdotref - s->pr2.qdot);

  //-- integrator
  s->pr2.q += s->dt*s->pr2.qdot + 0.5*(s->dt*s->dt)*qddot;
  s->pr2.qdot += s->dt*qddot;
#else
  s->pr2.q = qref;
  s->pr2.qdot = qdotref;
#endif
  s->pr2.setJointState(s->pr2.q, s->pr2.qdot);

  //-- point pos/vel sensors
  q_obs.set() = s->pr2.q;
  qdot_obs.set() = s->pr2.qdot;

  //-- update display
  if(!(s->step%100)){
    s->openglCopy.gl().lock.writeLock();
    s->openglCopy.setJointState(s->pr2.q, s->pr2.qdot);
    s->openglCopy.gl().lock.unlock();
    s->openglCopy.gl().update(STRING("PR2 Simulator.  time=" <<s->t), false, false, true);
  }
  s->t += s->dt;
  s->step ++;
}

void PR2Simulator::close(){
  delete s;
  s=NULL;
}

