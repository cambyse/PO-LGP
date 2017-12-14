#include <KOMO/komo.h>

#include <observation_tasks.h>
#include <approx_point_to_shape.h>
#include <axis_bound.h>
#include <axis_distance.h>
#include <komo_factory.h>

using namespace std;

//===========================================================================

/*struct CarKinematic:TaskMap{

  CarKinematic( const std::string & object )
    : object_( object )
  {

  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1)
  {
    mlr::Frame *object = G.getFrameByName( object_.c_str() );
    arr posObject, posJObject;
    G.kinematicsPos(posObject, posJObject, object);    // get function to minimize and its jacobian in state G


    double theta = object->X.rot.getRad();
    double val = cos( theta ) * posObject( 1 ) - sin( theta ) * posObject( 0 );

    arr tmp_y = zeros( dim_ );
    tmp_y( 0 ) = val;

    arr tmp_J = zeros( dim_, posJObject.dim(1) );
    tmp_J.setMatrixBlock( cos( theta ) * posJObject.row( 1 ) - sin( theta ) * posJObject.row( 0 ), 0 , 0 );    // jacobian

    // commit results
    y = tmp_y;
    if(&J) J = tmp_J;
  }

  /*virtual void phi(arr& y, arr& J, const WorldL& Ks, double tau, int t=-1)
  {
    auto G = *Ks.first();
    mlr::Frame *object = G.getFrameByName( object_.c_str() );
    arr posObject, posJObject;
    G.kinematicsPos(posObject, posJObject, object);    // get function to minimize and its jacobian in state G

    uint n = Ks.last()->q.N;
    J.resize(y.N, Ks.N, n).setZero();
    //arr tmp_y = zeros( dim_ );
    //arr tmp_J = zeros( dim_, posJObject.dim(1) );
    //tmp_J.setMatrixBlock( - sign * posJObject.row( id_ ), 0 , 0 );    // jacobian

    // commit results
    //y = tmp_y;
    //if(&J) J = tmp_J;
  }*/

//  virtual mlr::String shortTag(const mlr::KinematicWorld& G)
//  {
//    return mlr::String("CarKinematic");
//  }

//  virtual uint dim_phi(const mlr::KinematicWorld& K)
//  {
//    return dim_;
//  }

  /*virtual uint dim_phi(const WorldL& Ks, int t)
  {
    return dim_;
  }*/

//private:
//  static const uint dim_ = 1;
//  std::string object_;
//};



//===========================================================================

void overtake()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( mlr::KinematicWorld( "model_overtake.g" ) );


  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.setTask( 0.0, -1, new AxisBound( "car_ego", -0.15, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.15, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  // min speed
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // get sight
  komo.setTask( 2.0, 3.0, new AxisBound( "car_ego", 0.0, AxisBound::Y, AxisBound::MIN ), OT_sumOfSqr );

  // overtake constraints
  komo.setPosition( 4.0, -1, "car_ego", "truck", OT_sumOfSqr, { 0.6, 0, 0 } );

  // truck speed
  arr truck_speed{ 0.03, 0, 0 };
  truck_speed( 0 ) = 0.03;
  komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );

  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
  op_speed( 0 ) = -0.03;
  komo.setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );

  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "car_op" );
  //komo.activateCollisions( "car_ego", "car_op_2" );


  komo.setCollisions( true, 0.03 );

  //mlr::wait( 30, true );

  // launch komo
  komo.reset();
  komo.run();
  //komo.checkGradients();

  Graph result = komo.getReport(true);

  //komo.plotTrajectory();
  //komo.plotVelocity();

  for(;;) komo.displayTrajectory(.05, true);
}

void attempt()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( mlr::KinematicWorld( "model_attempt.g" ) );

  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.setTask( 0.0, -1, new AxisBound( "car_ego", -0.15, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.15, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  // min speed
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // get sight
  komo.setTask( 2.0, 3.0, new AxisBound( "car_ego", 0.0, AxisBound::Y, AxisBound::MIN ), OT_sumOfSqr );

  // move back constraints
  komo.setPosition( 4.0, -1, "car_ego", "truck", OT_sumOfSqr, { -0.6, 0, 0 } );

  // truck speed
  arr truck_speed{ 0.03, 0, 0 };
  truck_speed( 0 ) = 0.03;
  komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );

  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
  op_speed( 0 ) = -0.03;
  komo.setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );

  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "car_op" );
  //komo.activateCollisions( "car_ego", "car_op_2" );


  komo.setCollisions( true, 0.03 );

  //mlr::wait( 30, true );

  // launch komo
  komo.reset();
  komo.run();
  //komo.checkGradients();

  Graph result = komo.getReport(true);

  komo.plotTrajectory();
  komo.plotVelocity();

  for(;;) komo.displayTrajectory(.05, true);
}

void cooperative()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( mlr::KinematicWorld( "model_cooperative.g" ) );

  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.setTask( 0.0, -1, new AxisBound( "car_ego", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "truck", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "truck", -0.075, AxisBound::Y, AxisBound::MIN, 0.1 ), OT_sumOfSqr );
  komo.setTask( 0.0, -1, new AxisBound( "truck",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "car_op", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_op",  0.075, AxisBound::Y, AxisBound::MIN, 0.1 ), OT_sumOfSqr );
  komo.setTask( 0.0, -1, new AxisBound( "car_op",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  // min speed
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // get sight
  komo.setTask( 2.0, 3.0, new AxisBound( "car_ego", 0.0, AxisBound::Y, AxisBound::MIN ), OT_sumOfSqr );

  // overtake constraints
  komo.setPosition( 4.0, -1, "car_ego", "truck", OT_sumOfSqr, { 0.6, 0, 0 } );

  // truck speed
  arr truck_speed{ 0.03, 0, 0 };
  truck_speed( 0 ) = 0.03;
  komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );

  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
  op_speed( 0 ) = -0.03;
  komo.setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );

  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "car_op" );
  //komo.activateCollisions( "car_ego", "car_op_2" );


  komo.setCollisions( true, 0.03 );

  //mlr::wait( 30, true );

  // launch komo
  komo.reset();
  komo.run();
  //komo.checkGradients();

  Graph result = komo.getReport(true);

  komo.plotTrajectory();
  komo.plotVelocity();

  for(;;) komo.displayTrajectory(.05, true);
}

void cooperative_2()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( mlr::KinematicWorld( "model_cooperative_2.g" ) );

  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.setTask( 0.0, -1, new AxisBound( "car_ego", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "truck", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "truck", -0.075, AxisBound::Y, AxisBound::MIN, 0.1 ), OT_sumOfSqr );
  komo.setTask( 0.0, -1, new AxisBound( "truck",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "truck_2", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "truck_2", -0.075, AxisBound::Y, AxisBound::MIN, 0.1 ), OT_sumOfSqr );
  komo.setTask( 0.0, -1, new AxisBound( "truck_2",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "car_op", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_op",  0.075, AxisBound::Y, AxisBound::MIN, 0.1 ), OT_sumOfSqr );
  komo.setTask( 0.0, -1, new AxisBound( "car_op",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  // min speed
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // get sight
  komo.setTask( 2.0, 3.0, new AxisBound( "car_ego", 0.0, AxisBound::Y, AxisBound::MIN ), OT_sumOfSqr );

  // overtake constraints
  komo.setPosition( 4.0, -1, "car_ego", "truck", OT_sumOfSqr, { 0.4, 0, 0 } );

  // truck speed
  arr truck_speed{ 0.03, 0, 0 };
  truck_speed( 0 ) = 0.03;
  komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );
  komo.setTask( 0.0, -1, new AxisBound( "truck_2",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // distance between vehicle
  //komo.setTask( 0.0, -1, new AxisDistance( "truck_2", "car_ego", 0.5, AxisDistance::Y, AxisDistance::MIN ), OT_ineq );

  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
  op_speed( 0 ) = -0.03;
  komo.setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );

  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "truck_2" );
  komo.activateCollisions( "car_ego", "car_op" );
  //komo.activateCollisions( "car_ego", "car_op_2" );


  komo.setCollisions( true, 0.05 );

  // launch komo
  komo.reset();
  komo.run();
  //komo.checkGradients();

  Graph result = komo.getReport(true);

  komo.plotTrajectory();
  komo.plotVelocity();

  for(;;) komo.displayTrajectory(.05, true);
}

void cooperative_3()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( mlr::KinematicWorld( "model_cooperative_3.g" ) );

  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.setTask( 0.0, -1, new AxisBound( "car_ego", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "truck", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "truck", -0.075, AxisBound::Y, AxisBound::MIN, 0.1 ), OT_sumOfSqr );
  komo.setTask( 0.0, -1, new AxisBound( "truck",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "truck_2", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "truck_2", -0.075, AxisBound::Y, AxisBound::MIN, 0.1 ), OT_sumOfSqr );
  komo.setTask( 0.0, -1, new AxisBound( "truck_2",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "car_op", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_op",  0.075, AxisBound::Y, AxisBound::MIN, 0.1 ), OT_sumOfSqr );
  komo.setTask( 0.0, -1, new AxisBound( "car_op",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  // min speed
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // get sight
  komo.setTask( 3.0, 3.0, new AxisBound( "car_ego", 0.0, AxisBound::Y, AxisBound::MIN ), OT_sumOfSqr );

  // overtake constraints
  komo.setPosition( 5.0, -1, "car_ego", "truck", OT_sumOfSqr, { 0.4, 0, 0 } );

  // truck speed
  arr truck_speed{ 0.03, 0, 0 };
  truck_speed( 0 ) = 0.03;
  //komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );
  komo.setTask( 0.0, -1, new AxisBound( "truck",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.025 }, 1e2, 1 );
  komo.setTask( 0.0, -1, new AxisBound( "truck",  0.00, AxisBound::X, AxisBound::MAX ), OT_ineq,  arr{ 0.04 }, 1e2, 1 );
  komo.setTask( 0.0, -1, new AxisBound( "truck_2",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // distance between vehicle
  //komo.setTask( 0.0, -1, new AxisDistance( "truck_2", "car_ego", 0.5, AxisDistance::Y, AxisDistance::MIN ), OT_ineq );

  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
  op_speed( 0 ) = -0.03;
  komo.setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );

  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "truck_2" );
  komo.activateCollisions( "car_ego", "car_op" );
  //komo.activateCollisions( "car_ego", "car_op_2" );


  komo.setCollisions( true, 0.05 );

  // launch komo
  komo.reset();
  komo.run();
  //komo.checkGradients();

  Graph result = komo.getReport(true);

  komo.plotTrajectory();
  komo.plotVelocity();

  for(;;) komo.displayTrajectory(.05, true);
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  overtake();

  //attempt();

  //cooperative();

  //cooperative_2();

  //cooperative_3();

  return 0;
}
