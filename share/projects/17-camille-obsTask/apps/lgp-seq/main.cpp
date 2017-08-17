#include <KOMO/komo.h>

#include <observation_tasks.h>
#include <object_pair_collision_avoidance.h>

using namespace std;

//===========================================================================

void move_blocks()
{
//  {
//    mlr::KinematicWorld kin;
//    kin.init( "model_shelf.g" );
//    kin.watch();
//    kin.write( std::cout );

//    mlr::wait( 30, true );
//  }

  KOMO komo;
  komo.setConfigFromFile();

  //komo.setSquaredFixJointVelocities();
  komo.setFixEffectiveJoints();
  komo.setFixSwitchedObjects();
  komo.setSquaredQAccelerations();

  ///ALL TIME TASK MAPS
  /*
  komo.setTask( 0.0, 10.0, new TaskMap_AboveBox(komo.world, "block_o", "tableC" ), OT_ineq, NoArr, 1e2);
  komo.setTask( 0.0, 10.0, new TaskMap_AboveBox(komo.world, "block_r", "tableC" ), OT_ineq, NoArr, 1e2);
  komo.setTask( 0.0, 10.0, new TaskMap_AboveBox(komo.world, "block_g", "tableC" ), OT_ineq, NoArr, 1e2);
  komo.setTask( 0.0, 10.0, new TaskMap_AboveBox(komo.world, "block_b", "tableC" ), OT_ineq, NoArr, 1e2);
  */

  ///GRASP A, PUT IT on TABLE
  //grasp A
  komo.setKinematicSwitch( 3.0, true, "delete",   NULL, "block_r" );
  komo.setKinematicSwitch( 3.0, true, "ballZero", "handL", "block_r" );

  //place on table
  komo.setPlace( 4.0, "handL", "block_r", "tableC" );

  ///GRASP O, PUT IT on A
  //grasp
  komo.setKinematicSwitch( 5.0, true, "delete",   NULL, "block_o" );
  komo.setKinematicSwitch( 5.0, true, "ballZero", "handL", "block_o" );

  //put on A
  komo.setPlace( 6.0, "handL", "block_o", "block_r" );

  ///GRASP C, PUT IT on O
  //grasp
  komo.setKinematicSwitch( 7.0, true, "delete",   NULL, "block_b" );
  komo.setKinematicSwitch( 7.0, true, "ballZero", "handL", "block_b" );

  //put on O
  komo.setPlace( 8.0, "handL", "block_b", "block_o" );

  ///GRASP B, PUT IT on C
  //grasp
  komo.setKinematicSwitch( 9.0, true, "delete",   NULL, "block_g" );
  komo.setKinematicSwitch( 9.0, true, "ballZero", "handL", "block_g" );

  //put on B
  komo.setPlace( 10.0, "handL", "block_g", "block_b" );

  // launch komo
  komo.reset();
  komo.run();
  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.1, true);
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  //move_1();

  move_blocks();

  return 0;
}
