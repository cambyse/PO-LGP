#include "sim.h"
#include "filter.h"

#include <Gui/opengl.h>
#include <Kin/frame.h>

KinSim::KinSim(double dt) : Thread("KinSim", dt), dt(dt),
    path(this, "path"),
    currentQ(this, "currentQ"),
    nextQ(this, "nextQ"),
    switches(this, "switches"),
    world(this, "world"),
    timeToGo(this, "timeToGo"),
    percepts_input(this, "percepts_input"){

    K = world.get();

    reference.points = K.q;
    reference.points.append( K.q );
    reference.points.append( K.q );
    reference.points.reshape(3, K.q.N );
    reference.degree = 2;
    reference.setUniformNonperiodicBasis();
    log.open("z.KinSim");
    K.gl().title = "KinSim";

    currentQ.set() = K.q;
    nextQ.set() = K.q;
}

void KinSim::step(){
  //-- reference tracking

  //check for path update
  uint rev=path.readAccess();
  if(pathRev != rev){
    pathRev = rev;
    reference.points = K.getJointState();
    reference.points.append(path());
    reference.points.reshape(path().d0+1, K.getJointStateDimension());
    reference.setUniformNonperiodicBasis();
    phase=0.;
    timeToGo.set() = 1.;
    nextQ.set() = reference.points[-1];
  }
  path.deAccess();

  //check for switches update
  uint rev2=switches.readAccess();
  if(switchesRev != rev2){
    switchesRev = rev2;
    StringA cmd = switches.get();
    cout <<"CMD = " <<cmd <<endl;
    if(cmd(0)=="attach"){
      mlr::Frame *a = K.getFrameByName(cmd(1));
      mlr::Frame *b = K.getFrameByName(cmd(2));

      if(b->parent) b->unLink();
      b->linkFrom(a, true);
      (new mlr::Joint(*b)) -> type=mlr::JT_rigid;
      K.calc_q();
    }

  }
  switches.deAccess();

  //set the simulation's joint state
  arr q = reference.eval(phase);
  K.setJointState(q);
  currentQ.set() = q;
  world.set() = K;
  K.gl().update();

  log <<q <<endl;

  //progress the reference tracking
  if(phase!=1.){
    phase += dt;
    if(phase>1.) phase=1.;
    timeToGo.set() = 1.-phase;
  }


  //-- percept simulation

  PerceptSimpleL P;
  for(mlr::Frame *a:K.frames){
    if(a->ats["percept"]){
      P.append(new PerceptSimple(a->shape->geom(), a->X));
    }
  }
  if(P.N) percepts_input.set()->append(P);

}
