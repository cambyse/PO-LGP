#include <Motion/komo.h>

struct HandPositionMap:TaskMap{
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1){
    ors::Body *arm = G.getBodyByName("left_wrist");
    arr posArm, Jarm;
    G.kinematicsPos(posArm, Jarm, arm);

    posArm -= ARR(.5,.5,1.3);

    y = posArm;
    J = Jarm;
  }

  virtual uint dim_phi(const ors::KinematicWorld& G){
    return 3;
  }

  virtual mlr::String shortTag(const ors::KinematicWorld& G){ return mlr::String("RebaMap"); }

};

//===========================================================================

struct RebaMap:TaskMap{
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1){
    ors::Joint *j1 = G.getJointByName("right_shoulder_1");
    ors::Joint *j2 = G.getJointByName("left_shoulder_1");

    y = ARR(G.q(j1->qIndex), G.q(j2->qIndex));

    if(&J){
      J = zeros(2, G.q.N);
      J(0, j1->qIndex)=1.;
      J(1, j2->qIndex)=1.;
    }
  }


  virtual uint dim_phi(const ors::KinematicWorld& G){
    return 2;
  }

  virtual mlr::String shortTag(const ors::KinematicWorld& G){ return mlr::String("RebaMap"); }

};

//===========================================================================

void moveReba(){
  KOMO komo;
  komo.setConfigFromFile();

//  komo.setHoming(-1., -1., 1e-1);
//  komo.setSquaredQVelocities();
  komo.setSquaredQAccelerations();
#if 0
  komo.setPosition(1., 1., "endeffL", "target", sumOfSqrTT, NoArr, 1e2);
#else
  komo.setTask(1., 1., new RebaMap(), sumOfSqrTT, {.5, -.5}, 1e2);
#endif
  komo.setSlowAround(1., .1, 1e3);

  komo.reset();
  komo.run();
//  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.1, true);
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  moveReba();

  return 0;
}




