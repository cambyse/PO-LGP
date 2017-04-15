#include <Motion/komo.h>

#include <observation_tasks.h>

using namespace std;

//===========================================================================

static void setRigid( double time, mlr::String const& object1Name, mlr::String const& object2Name, KOMO& komo )
{
  mlr::Shape * object1 = komo.world.getShapeByName( object1Name );
  mlr::Shape * object2 = komo.world.getShapeByName( object2Name );
  mlr::Transformation t;
  arr relPos;
  arr relPosJ;

  object1->rel.pos.write( std::cout );
  std::cout << std::endl;
  object2->rel.pos.write( std::cout );
  std::cout << std::endl;

  komo.world.kinematicsRelPos( relPos, relPosJ, object2->body, object2->rel.pos, object1->body, object1->rel.pos );
  t.pos = relPos;
  komo.setKinematicSwitch( time, true, "addRigid", object1Name, object2Name, t );
}

struct _PairCollisionConstraint:PairCollisionConstraint
{
  _PairCollisionConstraint(const mlr::KinematicWorld& G, const char* iShapeName, const char* jShapeName, double _margin=.02)
    : PairCollisionConstraint( G, iShapeName, jShapeName, _margin )
  {

  }

  mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("_PairCollisionConstraint"); }
};
//===========================================================================

void move(){

  KOMO komo;
  komo.setConfigFromFile();
  //komo.setTiming( 5, 10, 5, 2 );


  // mlr::Body *b = komo.world.getBodyByName("/human/base");
  // b->X.addRelativeTranslation(.3,0,0);

  //  komo.setHoming(-1., -1., 1e-1);
  //  komo.setSquaredQVelocities();
  komo.setSquaredFixJointVelocities();
  komo.setSquaredFixSwitchedObjects();
  komo.setSquaredQAccelerations();

  //komo.setPosition(1., 1.1, "humanL", "target", OT_sumOfSqr, NoArr, 1e2);
  //komo.setPosition(1., 1.1, "handR", "target", OT_sumOfSqr, NoArr, 1e2);


  //komo.setPosition(1., 2.0, "manhead", "target", OT_sumOfSqr, ARR(0,0,0), 1e2);  // objective to have the head on the target on this time slab
  //komo.setTask(startTime, endTime, new TaskMap_Default(posTMT, world, shape, NoVector, shapeRel, NoVector), type, target, prec);

  //arr targetArr1 = HeadPoseMap::buildTarget( mlr::Vector( 0, -0.3, 1.7 ), 80 );
  //komo.setTask(1.0, 3.0, new HeadPoseMap(), OT_sumOfSqr, targetArr1, 1e2);



  // make container and target rigid
  setRigid( 0.5, "container_1_bottom", "target", komo );
  //setRigid( 0.5, "container_1_bottom", "table", komo );

  //setRigid( 0.5, "container_1", "container_1", komo );


  // grasp container
  //komo.setGrasp( 1.0, "handL", "container_1_front" );
  //komo.setGrasp( 2.0, "handL", "target_1" ); // grasp ball

  /////ACTIVE GET SIGHT CONTAINER 0
  {
    const double time = 1.0;

    komo.setTask( time, time + 1.0, new ActiveGetSight      ( "manhead",
                                                              "handL",
                                                              "container_0",
                                                              //ARR( -0.0, -0.0, 0.0 ),    // object position in container frame
                                                              ARR( -0.0, 0.2, 0.4 ) ),  // pivot position  in container frame
                  OT_sumOfSqr, NoArr, 1e2 );
  }

  /////GRASP CONTAINER////
  {
    const double time = 3.0;
    //arrive sideways
    komo.setTask( time, time, new TaskMap_Default( vecTMT, komo.world, "handL", Vector_x ), OT_sumOfSqr, {0.,0.,1.}, 1e1 );

    //disconnect object from table
    komo.setKinematicSwitch( time, true, "delete", NULL, "container_1_left" );
    //connect graspRef with object
    komo.setKinematicSwitch( time, true, "ballZero", "handL", "container_1_left" );
  }
  /////


  /////ACTIVE GET SIGHT CONTAINER 1
  {
    const double time = 4.0;

    komo.setTask( time, time + 1.0, new ActiveGetSight      ( "manhead",
                                                              "handL",
                                                              "container_1",
                                                              //ARR( -0.0, -0.0, 0.0 ),    // object position in container frame
                                                              ARR( -0.0, 0.2, 0.4 ) ),  // pivot position  in container frame
                  OT_sumOfSqr, NoArr, 1e2 );
  }

  /////GRASP BALL INSIDE////
//  {
//    const double time = 2.0;
//    //arrive from front
//    komo.setTask( time, time, new TaskMap_Default( vecTMT, komo.world, "handR", Vector_y ), OT_sumOfSqr, {0.,0.,1.}, 1e1 );

//    //disconnect object from table
//    komo.setKinematicSwitch( time, true, "delete", NULL, "target_1" );
//    //connect graspRef with object
//    komo.setKinematicSwitch( time, true, "ballZero", "handR", "target_1" );
//  }
  /////

  /////PLACE ON TABLE
  {
    const double time = 5.0;

    komo.setPlace( time, "handL", "container_1_front", "table" );
  }

//  { //doesn't seem to work
//  komo.setTask( 1.0, 5.0, new _PairCollisionConstraint( komo.world, "manhead", "container_1_front" ), OT_sumOfSqr, NoArr, 1e2 );
//  }

//  if(komo.stepsPerPhase>2){ //velocities down and up
//    komo.setTask(time-.15, time, new TaskMap_Default(posTMT, komo.world, "handL"), OT_sumOfSqr, {0.,0.,-.1}, 1e1, 1); //move down
//    komo.setTask(time, time+.15, new TaskMap_Default(posTMT, komo.world, "occluding_object_1"), OT_sumOfSqr, {0.,0.,.1}, 1e1, 1); // move up
//  }

  // right
//  komo.setTask( 1.0, 2.0, new HeadGetSightQuat    ( ARR(  -0.5, -0.7, 1.2 ),    // object position
//                                                    ARR(  -0.3, -0.5, 1.5 ) ),  // pivot position
//                                                    OT_sumOfSqr, NoArr, 1e2 );

  // middle
//  komo.setTask( 1.0, 2.0, new HeadGetSightQuat  ( ARR(  -0.0, -0.9, 1.2 ),    // object position
//                                                  ARR(  -0.0, -0.7, 1.5 ) ),  // pivot position
//                                                  OT_sumOfSqr, NoArr, 1e2 );

  // left
//  komo.setTask( 1.0, 2.0, new HeadGetSightQuat    ( ARR(   0.5, -0.7, 1.2 ),    // object position
//                                                    ARR(   0.3, -0.5, 1.5 ) ),  // pivot position
//                                                    OT_sumOfSqr, NoArr, 1e2 );

//  komo.setTask( 1.0, 2.0, new HeadGetSight( ARR(  1.0, -0.0, 1.9 ),    // object position
//                                            ARR(  1.0, -0.0, 1.9 ) ),  // pivot position
//                OT_sumOfSqr, NoArr, 1e2 );

  //komo.setTask(.3, .5, new HandPositionMap(), OT_sumOfSqr, ARR(.5,.5,1.3), 1e2);
  //komo.setTask(.8, 1., new HandPositionMap(), OT_sumOfSqr, ARR(.8,0.,1.3), 1e2);
  //komo.setTask(.8, 1., new TaskMap_Default(posDiffTMT, komo.world, "/human/humanR", NoVector, "target", NoVector), OT_sumOfSqr, NoArr, 1e2);

//  komo.setTask(.3, 1., new TaskMap_Default(gazeAtTMT, komo.world, "eyes", NoVector, "target", NoVector), OT_sumOfSqr, NoArr, 1e2);

  //komo.setSlowAround( 5.0, .1, 1e3 );

  komo.reset();
  komo.run();
  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.1, true);
}

void move_debug(){

  {
    mlr::KinematicWorld kin;
    kin.init( "debug-kin-parsing.kin" );
    kin.watch();
    kin.write( std::cout );
  }
  
  KOMO komo;
  //komo.setConfigFromFile();
  mlr::KinematicWorld kin;
  kin.init( "debug-kin.kin" );
  kin.watch();

  komo.setModel( kin );

  // mlr::Body *b = komo.world.getBodyByName("/human/base");
  // b->X.addRelativeTranslation(.3,0,0);

  //  komo.setHoming(-1., -1., 1e-1);
  //  komo.setSquaredQVelocities();
  /*komo.setSquaredFixJointVelocities();
  komo.setSquaredFixSwitchedObjects();
  komo.setSquaredQAccelerations();
  */

  komo.setTiming(2, 20, 5., 2, false);

  komo.setHoming(-1., -1., 1e-2); //gradient bug??
  komo.setSquaredQAccelerations();
  komo.setSquaredFixJointVelocities(-1., -1., 1e3);
  komo.setSquaredFixSwitchedObjects(-1., -1., 1e3);

  //komo.setPosition(1., 1.1, "humanL", "target", OT_sumOfSqr, NoArr, 1e2);

  //komo.setPosition(1., 2.0, "manhead", "target", OT_sumOfSqr, ARR(0,0,0), 1e2);  // objective to have the head on the target on this time slab
  //komo.setTask(startTime, endTime, new TaskMap_Default(posTMT, world, shape, NoVector, shapeRel, NoVector), type, target, prec);

  //arr targetArr1 = HeadPoseMap::buildTarget( mlr::Vector( 0, -0.3, 1.7 ), 80 );
  //komo.setTask(1.0, 3.0, new HeadPoseMap(), OT_sumOfSqr, targetArr1, 1e2);

  komo.setGrasp( 1.0, "humanR", "occluding_object_1" );
//  komo.setTask( 1.0, 2.0, new HeadGetSight    ( ARR(  0.0, -1.0, 1.6 ),    // object position
//                                                ARR( -0.2, -0.6, 1.9 ) ),  // pivot position
//                                                OT_sumOfSqr, NoArr, 1e2 );


//  komo.setTask( 1.0, 2.0, new HeadGetSight( ARR(  1.0, -0.0, 1.9 ),    // object position
//                                            ARR(  1.0, -0.0, 1.9 ) ),  // pivot position
//                OT_sumOfSqr, NoArr, 1e2 );

  //komo.setTask(.3, .5, new HandPositionMap(), OT_sumOfSqr, ARR(.5,.5,1.3), 1e2);
  //komo.setTask(.8, 1., new HandPositionMap(), OT_sumOfSqr, ARR(.8,0.,1.3), 1e2);
  //komo.setTask(.8, 1., new TaskMap_Default(posDiffTMT, komo.world, "/human/humanR", NoVector, "target", NoVector), OT_sumOfSqr, NoArr, 1e2);

//  komo.setTask(.3, 1., new TaskMap_Default(gazeAtTMT, komo.world, "eyes", NoVector, "target", NoVector), OT_sumOfSqr, NoArr, 1e2);

  komo.setSlowAround( 3.0, .1, 1e3 );

  komo.reset();
  komo.run();
  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.1, true);
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  move();

  return 0;
}
