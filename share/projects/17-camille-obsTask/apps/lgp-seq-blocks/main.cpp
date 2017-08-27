#include <KOMO/komo.h>

#include <observation_tasks.h>
#include <approx_point_to_shape.h>

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

  komo.setTask( 0.0, 10.0, new ApproxPointToShape(komo.world, "humanL", "tableC" ), OT_ineq, NoArr, 1e2);
  //komo.setTask( 0.0, 10.0, new ApproxPointToShape(komo.world, "humanL", "tableC" ), OT_ineq, NoArr, 1e2);



  ///GRASP C, PUT IT on TABLE
  //grasp C
  komo.setKinematicSwitch( 3.0, true, "delete",   NULL, "block_c" );
  komo.setKinematicSwitch( 3.0, true, "ballZero", "handL", "block_c" );

  //place on table
  komo.setPlace( 4.0, "handL", "block_c", "tableC" );

//  ///GRASP O, PUT IT on A
//  //grasp
//  komo.setKinematicSwitch( 5.0, true, "delete",   NULL, "block_o" );
//  komo.setKinematicSwitch( 5.0, true, "ballZero", "handL", "block_o" );

//  //put on A
//  komo.setPlace( 6.0, "handL", "block_o", "block_a" );

//  ///GRASP C, PUT IT on O
//  //grasp
//  komo.setKinematicSwitch( 7.0, true, "delete",   NULL, "block_c" );
//  komo.setKinematicSwitch( 7.0, true, "ballZero", "handL", "block_c" );

//  //put on O
//  komo.setPlace( 8.0, "handL", "block_c", "block_o" );

  ///GRASP B, PUT IT on C
  //grasp
  komo.setKinematicSwitch( 5.0, true, "delete",   NULL, "block_b" );
  komo.setKinematicSwitch( 5.0, true, "ballZero", "handL", "block_b" );

  //put on C
  komo.setPlace( 6.0, "handL", "block_b", "block_c" );

  ///GRASP A, PUT IT on B
  //grasp
  komo.setKinematicSwitch( 7.0, true, "delete",   NULL, "block_a" );
  komo.setKinematicSwitch( 7.0, true, "ballZero", "handL", "block_a" );

  //put on C
  komo.setPlace( 8.0, "handL", "block_a", "block_b" );

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
