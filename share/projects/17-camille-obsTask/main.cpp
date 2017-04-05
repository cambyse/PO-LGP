#include <KOMO/komo.h>
#include <string>
#include <map>

using namespace std;

struct HandPositionMap:TaskMap{
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1){
    mlr::Shape *hand = G.getShapeByName("humanL");
    arr posArm, Jarm;
    G.kinematicsPos(posArm, Jarm, hand->body);

    // posArm -= ARR(.5,.5,1.3);

    y = posArm;
    J = Jarm;
  }

  virtual uint dim_phi(const mlr::KinematicWorld& G){
    return 3;
  }

  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return mlr::String("HandPositionMap"); }

};

//===========================================================================

void move(){
  
  KOMO komo;
  komo.setConfigFromFile();

  // mlr::Body *b = komo.world.getBodyByName("/human/base");
  // b->X.addRelativeTranslation(.3,0,0);

  //  komo.setHoming(-1., -1., 1e-1);
  //  komo.setSquaredQVelocities();
  komo.setSquaredFixJointVelocities();
  komo.setSquaredFixSwitchedObjects();
  komo.setSquaredQAccelerations();
#if 0
  komo.setPosition(1., 1., "humanL", "target", OT_sumOfSqr, NoArr, 1e2);
#else
  komo.setPosition(1., 1.1, "humanL", "target", OT_sumOfSqr, NoArr, 1e2);
//  komo.setTask(.3, .5, new HandPositionMap(), OT_sumOfSqr, ARR(.5,.5,1.3), 1e2);
//  komo.setTask(.8, 1., new HandPositionMap(), OT_sumOfSqr, ARR(.8,0.,1.3), 1e2);
//  komo.setTask(.8, 1., new TaskMap_Default(posDiffTMT, komo.world, "/human/humanR", NoVector, "target", NoVector), OT_sumOfSqr, NoArr, 1e2);

//  komo.setTask(.3, 1., new TaskMap_Default(gazeAtTMT, komo.world, "eyes", NoVector, "target", NoVector), OT_sumOfSqr, NoArr, 1e2);
#endif
  komo.setSlowAround(2., .1, 1e3);

  komo.reset();
  komo.run();
//  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.1, true);
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  move();

  return 0;
}
