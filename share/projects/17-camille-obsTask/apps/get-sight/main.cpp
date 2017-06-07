#include <Motion/komo.h>

#include <observation_tasks.h>
#include <object_pair_collision_avoidance.h>

using namespace std;

//===========================================================================

static void setRigid( double time, mlr::String const& object1Name, mlr::String const& object2Name, KOMO& komo )
{
  mlr::Shape * s1 = komo.world.getShapeByName( object1Name );
  mlr::Shape * s2 = komo.world.getShapeByName( object2Name );
  mlr::Transformation t;
  arr relPos;
  arr relPosJ;

  s1->rel.pos.write( std::cout );
  std::cout << std::endl;
  s2->rel.pos.write( std::cout );
  std::cout << std::endl;

  komo.world.kinematicsRelPos( relPos, relPosJ, s2->body, s2->rel.pos, s1->body, s1->rel.pos );
  t.pos = relPos;
  komo.setKinematicSwitch( time, true, "addRigid", s1->name, s2->name, t );

  /*auto * object1 = komo.world.getBodyByName( object1Name );
  auto * object2 = komo.world.getBodyByName( object2Name );

  for( auto s1 : object1->shapes )
  {
    for( auto s2 : object2->shapes )
    {
//      mlr::Shape * object1 = komo.world.getShapeByName( s1->name );
//      mlr::Shape * object2 = komo.world.getShapeByName( s2->name );
      mlr::Transformation t;
      arr relPos;
      arr relPosJ;

      s1->rel.pos.write( std::cout );
      std::cout << std::endl;
      s2->rel.pos.write( std::cout );
      std::cout << std::endl;

      komo.world.kinematicsRelPos( relPos, relPosJ, s2->body, s2->rel.pos, s1->body, s1->rel.pos );

      t.pos = relPos;

      komo.setKinematicSwitch( time, true, "addRigid", s1->name, s2->name, t );
    }
  }*/
}

static double norm2( const arr & x )
{
  return sqrt( ( ( ~ x ) * x )( 0 ) );
}

static arr Jnorm( const arr & x )
{
  arr J( 1, x.N );

  // compute sqrNorm
  double norm = norm2( x );

  // compute each jacobian element
  if( norm > 0.000001 )
  {
    for( auto i = 0; i < x.N; ++i )
      J( 0, i ) = x( i ) / norm ;
  }
  else
  {
    J.setZero();
  }

  return J;
}

/*struct AllObjectsCollisionAvoidance:TaskMap
{
  AllObjectsCollisionAvoidance( double margin=.02 )
    : margin_( margin )
  {
  }

  virtual void phi( arr& y, arr& J, const mlr::KinematicWorld& G, int t )
  {
    //::PairCollisionConstraint::phi( y, J, G, t );
//    arr tmp_y = zeros( 1 );

//    for( auto )
    uint dim = dim_phi( G );

    arr tmp_y = zeros( dim );
    arr tmp_J = zeros( dim, G.q.N );

    for( auto k = 0; k < G.proxies.N; ++k )
    {
      auto p = G.proxies( k );

      tmp_y( k ) = margin_ - p->d;

      mlr::Shape *a = G.shapes(p->a);
      mlr::Shape *b = G.shapes(p->b);

      auto arel=a->body->X.rot/(p->posA-a->body->X.pos);
      auto brel=b->body->X.rot/(p->posB-b->body->X.pos);

      arr posA;
      arr posB;
      arr JposA;
      arr JposB;
      G.kinematicsPos(posA, JposA, a->body, arel);
      G.kinematicsPos(posB, JposB, b->body, brel);

      double d1 = ( p->posA - p->posB ).length();
      double d2 = norm2( posA - posB );

      arr JnormD = Jnorm( posA - posB ) * ( JposA - JposB );

      tmp_J.setMatrixBlock( -JnormD, k, 0 );
      //tmp_J =

      //std::cout << p->d << " " << d1 << " " << d2 << std::endl;
    }

    // commit results
    y = tmp_y;
    if(&J) J = tmp_J;
  }

  mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("AllObjectsCOllisionAvoidance"); }

  uint dim_phi(const mlr::KinematicWorld& G)
  {
    return ( G.shapes.N ) * ( G.shapes.N );
  }

private:
  double margin_;
};*/

//===========================================================================

void move_0(){

  KOMO komo;
  komo.setConfigFromFile();

  komo.setSquaredFixJointVelocities();
  komo.setSquaredFixSwitchedObjects();
  komo.setSquaredQAccelerations();

  const double start_time = 1.0;

  //komo.setAlign( 1.0, 7, "container_1_front" );

  /////ACTIVE GET SIGHT CONTAINER 0
  {
    const double time = start_time + 0.0;

    komo.setTask( time, time + 1.0, new ActiveGetSight      ( "manhead",
                                                              "container_0",
                                                              ARR( -0.0, 0.2, 0.4 ) ),  // pivot position  in container frame
                  OT_sumOfSqr, NoArr, 1e2 );
  }

  {
    const double time = start_time + 1.0;

//    komo.setTask( time, time + 1.0, new TakeView      ( ),
//                  OT_eq, NoArr, 1e2, 1 );

  }

  komo.setTask( 1.0, start_time + 8.0, new AxisAlignment( "container_0", ARR( 1.0, 0, 0 ) ), OT_eq, NoArr, 1e2 );
  komo.setTask( 1.0, start_time + 8.0, new OverPlaneConstraint ( komo.world,
                                                                 "container_0",
                                                                 "tableC",
                                                                 0.01
                                                                 ),  // pivot position  in container frame
                OT_ineq, NoArr, 1e2 );

  /////GRASP CONTAINER 0 ////
//  {
    const double time = start_time + 2.0 + 1.0;
    //arrive sideways
    //komo.setTask( time, time, new TaskMap_Default( vecTMT, komo.world, "handL", Vector_x ), OT_sumOfSqr, {0.,0.,1.}, 1e1 );

    //disconnect object from table
    komo.setKinematicSwitch( time, true, "delete", "tableC", "container_0_bottom" );
    //connect graspRef with object
    komo.setKinematicSwitch( time, true, "ballZero", "handL", "container_0_left" );
    //komo.setKinematicSwitch( time, true, "addRigid", "handL", "container_1_handle", NoTransformation, NoTransformation );
//  }

  /////


//  /////ACTIVE GET SIGHT CONTAINER 1
//  {
//    const double time = start_time + 3.0;

//    komo.setTask( time, time + 1.0, new ActiveGetSight      ( "manhead",
//                                                              "container_1",
//                                                              //ARR( -0.0, -0.0, 0.0 ),    // object position in container frame
//                                                              ARR( -0.0, 0.2, 0.4 ) ),  // pivot position  in container frame
//                  OT_sumOfSqr, NoArr, 1e2 );
//  }

  /////PLACE ON TABLE FOR CONTAINER 0
  {
  //  const double time = start_time + 4.5;

  //  komo.setPlace( time, "handL", "container_0_front", "tableL" );
  }

    {
       const double time = start_time + 5.0;

      komo.setHoming(time, time+1, 1e-2); //gradient bug??
    }

//  komo.setTask( 1.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_right", "container_0_left", 0.05 ), OT_ineq, NoArr, 1e2 );
//  komo.setTask( 1.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_front", "container_0_front", 0.05 ), OT_ineq, NoArr, 1e2 );
//  komo.setTask( 1.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_right", "container_0_bottom", 0.05 ), OT_ineq, NoArr, 1e2 );
//  komo.setTask( 1.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_bottom", "container_0_left", 0.05 ), OT_ineq, NoArr, 1e2 );


  komo.reset();
  komo.run();
  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.1, true);
}

void move_1(){

  KOMO komo;
  komo.setConfigFromFile();

  komo.setSquaredFixJointVelocities();
  komo.setSquaredFixSwitchedObjects();
  komo.setSquaredQAccelerations();

//  KOMO komo;
//  //komo.setConfigFromFile();
//  mlr::KinematicWorld kin;
//  kin.init( "model.g" );
//  kin.watch();

//  komo.setModel( kin );
//  komo.setTiming(7, 10, 5., 2, true);

//  komo.setHoming(-1., -1., 1e-2); //gradient bug??
//  komo.setSquaredQAccelerations();
//  komo.setSquaredFixJointVelocities(-1., -1., 1e3);
//  komo.setSquaredFixSwitchedObjects(-1., -1., 1e3);


  //komo.setPosition(1., 1.1, "humanL", "target", OT_sumOfSqr, NoArr, 1e2);
  //komo.setPosition(1., 1.1, "handR", "target", OT_sumOfSqr, NoArr, 1e2);


  //komo.setPosition(1., 2.0, "manhead", "target", OT_sumOfSqr, ARR(0,0,0), 1e2);  // objective to have the head on the target on this time slab
  //komo.setTask(startTime, endTime, new TaskMap_Default(posTMT, world, shape, NoVector, shapeRel, NoVector), type, target, prec);

  //arr targetArr1 = HeadPoseMap::buildTarget( mlr::Vector( 0, -0.3, 1.7 ), 80 );
  //komo.setTask(1.0, 3.0, new HeadPoseMap(), OT_sumOfSqr, targetArr1, 1e2);



  // make container and target rigid
  //setRigid( 0.5, "container_1_bottom", "target", komo );
  //setRigid( 0.5, "container_1_bottom", "tableC", komo );


  // grasp container
  //komo.setGrasp( 1.0, "handL", "container_1_front" );
  //komo.setGrasp( 2.0, "handL", "target_1" ); // grasp ball
  const double start_time = 1.0;

  //komo.setAlign( 1.0, 7, "container_1_front" );

  /////ACTIVE GET SIGHT CONTAINER 0
  {
    const double time = start_time + 0.0;

    komo.setTask( time, time + 1.0, new ActiveGetSight      ( "manhead",
                                                              "container_0",
                                                              ARR( -0.0, 0.2, 0.4 ) ),  // pivot position  in container frame
                  OT_sumOfSqr, NoArr, 1e2 );
  }

  {
    const double time = start_time + 1.0;

//    komo.setTask( time, time + 1.0, new TakeView      ( ),
//                  OT_eq, NoArr, 1e2, 1 );

    auto *map = new TaskMap_Transition(komo.MP->world);
    map->posCoeff = 0.;
    map->velCoeff = 1.;
    map->accCoeff = 0.;
    komo.setTask( time, time + 1.0, map, OT_sumOfSqr, NoArr, 1e2, 1 );
  }

  komo.setTask( 1.0, start_time + 8.0, new AxisAlignment( "container_1", ARR( 1.0, 0, 0 ) ), OT_eq, NoArr, 1e2 );
  komo.setTask( 1.0, start_time + 8.0, new OverPlaneConstraint ( komo.world,
                                                                 "container_1",
                                                                 "tableC",
                                                                 0.01
                                                                 ),  // pivot position  in container frame
                OT_ineq, NoArr, 1e2 );

  /////GRASP CONTAINER////
//  {
    const double time = start_time + 2.0 + 1.0;
    //arrive sideways
    //komo.setTask( time, time, new TaskMap_Default( vecTMT, komo.world, "handL", Vector_x ), OT_sumOfSqr, {0.,0.,1.}, 1e1 );

    //disconnect object from table
    komo.setKinematicSwitch( time, true, "delete", "tableC", "container_1_bottom" );
    //connect graspRef with object
    komo.setKinematicSwitch( time, true, "ballZero", "handL", "container_1_left" );
    //komo.setKinematicSwitch( time, true, "addRigid", "handL", "container_1_handle", NoTransformation, NoTransformation );
//  }

  /////


  /////ACTIVE GET SIGHT CONTAINER 1
  {
    const double time = start_time + 3.0;

    komo.setTask( time, time + 1.0, new ActiveGetSight      ( "manhead",
                                                              "container_1",
                                                              //ARR( -0.0, -0.0, 0.0 ),    // object position in container frame
                                                              ARR( -0.0, 0.2, 0.4 ) ),  // pivot position  in container frame
                  OT_sumOfSqr, NoArr, 1e2 );
  }

  /////PLACE ON TABLE FOR CONTAINER 1
  {
    //const double time = start_time + 5.0;

    //komo.setPlace( time, "handL", "container_1_front", "tableL" );
  }
  {
     const double time = start_time + 5.0;

    komo.setHoming(time, time+1, 1e-2); //gradient bug??
  }

  //// GRASP BALL
    {
      const double time = start_time + 4.0;
      //arrive sideways
      //komo.setTask( time, time, new TaskMap_Default( vecTMT, komo.world, "handL", Vector_x ), OT_sumOfSqr, {0.,0.,1.}, 1e1 );

      //disconnect object from table
      //komo.setKinematicSwitch( time + 1.0, true, "delete", NULL, "target" );
      //connect graspRef with object
      //komo.setKinematicSwitch( time + 1.0, true, "ballZero", "handR", "target" );
      //komo.setKinematicSwitch( time, true, "addRigid", "handR", "target", NoTransformation, NoTransformation );

      //komo.setPosition(time, time + 1.0, "handR", "target", OT_sumOfSqr, NoArr, 1e2);
      //komo.setGrasp(time + 1.0, "handR", "target", 0, 1e1);
    }

    /////


  /////COLLISION AVOIDANCE
//  {
//    komo.setTask( 1.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_right", "container_0_left", 0.05 ), OT_ineq, NoArr, 1e2 );
//    komo.setTask( 1.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_front", "container_0_front", 0.05 ), OT_ineq, NoArr, 1e2 );
//    komo.setTask( 1.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_right", "container_0_bottom", 0.05 ), OT_ineq, NoArr, 1e2 );
//    komo.setTask( 1.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_bottom", "container_0_left", 0.05 ), OT_ineq, NoArr, 1e2 );

////    komo.setTask( start_time + 2.0, 6.0, new ShapePairCollisionConstraint( komo.world, "container_1_bottom", "tableCC" ), OT_ineq, NoArr, 1e2 );
//  }

//  if(komo.stepsPerPhase>2){ //velocities down and up
//    komo.setTask(time-.15, time, new TaskMap_Default(posTMT, komo.world, "handL"), OT_sumOfSqr, {0.,0.,-.1}, 1e1, 1); //move down
//    komo.setTask(time, time+.15, new TaskMap_Default(posTMT, komo.world, "occluding_object_1"), OT_sumOfSqr, {0.,0.,.1}, 1e1, 1); // move up
//  }

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

  move_0();

  return 0;
}
