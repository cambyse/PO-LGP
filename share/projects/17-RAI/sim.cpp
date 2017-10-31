#include "sim.h"
#include "filter.h"

#include <Gui/opengl.h>
#include <Kin/frame.h>

KinSim::KinSim(double dt) : Thread("KinSim", dt), dt(dt),
    ref(this, "MotionReference"),
    switches(this, "switches"),
    currentQ(this, "currentQ"),
    robotBase(this, "robotBase"),
//    nextQ(this, "nextQ"),
//    world(this, "world"),
    timeToGo(this, "timeToGo"),
    percepts_input(this, "percepts_input"){

    K = Access<mlr::KinematicWorld>("world").get();

    reference.points = K.q;
    reference.points.append( K.q );
    reference.points.append( K.q );
    reference.points.reshape(3, K.q.N );
    reference.degree = 2;
    reference.setUniformNonperiodicBasis();
    log.open("z.KinSim");
    K.gl().title = "KinSim";

    //add random noise to perceptual objects:
    for(mlr::Frame *a:K.frames){
      if(a->ats["percept"]){
        a->X.pos.y += rnd.uni(-.5, .5);
      }
    }

    currentQ.set() = K.q;
}

void KinSim::step(){
  //-- publish robot base
  robotBase.set() = K.getFrameByName("base")->X;

  //-- reference tracking

  //check for path update
  if(ref.hasNewRevision()){
    ref.readAccess();
    reference.points = K.getJointState();
    reference.points.append(ref().path);
    reference.points.reshape(ref().path.d0+1, K.getJointStateDimension());
    reference.setUniformNonperiodicBasis();
    phase=0.;
    CHECK(ref().tau.N==1,"");
    planDuration = ref().tau.scalar() * ref().path.d0;
    timeToGo.set() = planDuration;
    ref.deAccess();
  }

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
  currentQ.set() = K.getJointState();
  K.gl().update();

  log <<q <<endl;

  //progress the reference tracking
  if(phase!=1.){
    phase += dt/planDuration;
    if(phase>1.) phase=1.;
    timeToGo.set() = planDuration*(1.-phase);
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
