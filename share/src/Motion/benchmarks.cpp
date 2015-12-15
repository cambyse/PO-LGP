#include "benchmarks.h"
#include "motion.h"

void setTasks(MotionProblem& MP,
              ors::Shape &endeff,
              ors::Shape& target,
              byte whichAxesToAlign,
              uint iterate,
              int timeSteps,
              double duration);

struct sPR2EndPoseProblem{
  ors::KinematicWorld world;
  MotionProblem MP;
  MotionProblemFunction MPF;
  sPR2EndPoseProblem()
    :world ("model.kvg"), MP(world), MPF(MP){}
};

PR2EndPoseProblem::PR2EndPoseProblem()
  : s(*(new sPR2EndPoseProblem())){

  for(ors::Shape *sh:s.world.shapes) sh->cont=true;

  setTasks(s.MP, *s.world.getShapeByName("endeff"), *s.world.getShapeByName("target"), 0, 1, 0, 5.);

  ConstrainedProblem::operator=( convert_KOrderMarkovFunction_ConstrainedProblem(s.MPF) );
}

arr PR2EndPoseProblem::getInitialization(){
  arr x = replicate(s.MP.x0, s.MP.T+1); //we initialize with a constant trajectory!
  rndGauss(x,.01,true); //don't initialize at a singular config
  return x;
}

void PR2EndPoseProblem::report(){
  s.MP.costReport();
  s.world.watch(true);
}

void PR2EndPoseProblem::setState(const arr& x){
  s.world.setJointState(x);
}

