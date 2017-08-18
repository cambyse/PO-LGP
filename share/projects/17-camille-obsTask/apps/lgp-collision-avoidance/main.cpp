#include <KOMO/komo.h>

#include <observation_tasks.h>
#include <object_pair_collision_avoidance.h>
#include <Kin/taskMap_GJK.h>

using namespace std;

//===========================================================================

void move()
{
  KOMO komo;
  komo.setConfigFromFile();

  //komo.setSquaredFixJointVelocities();
  komo.setFixEffectiveJoints();
  komo.setFixSwitchedObjects();
  komo.setSquaredQAccelerations();

  //activate collision avoidance
  //komo.setCollisions( true, 0.05 );
  //komo.setTask( 0.0, 5.0, new TaskMap_GJK( komo.world, "handL", "obstacle", false, true ), OT_ineq, NoArr, 1e1 );
  komo.setTask( 0.0, 5.0, new ShapePairCollisionConstraint( komo.world, "handL", "obstacle", 0.05 ), OT_ineq, NoArr, 1e1 );

  //disconnect object from container
  komo.setKinematicSwitch( 2.5/*8.0*/, true, "delete", NULL, "block" );
  //connect graspRef with object
  komo.setKinematicSwitch( 2.5/*8.0*/, true, "ballZero", "handL", "block" );

  // launch komo
  komo.reset();
  komo.run();
  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.5, true);
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  //move_1();

  move();

  return 0;
}
