#include <KOMO/komo.h>

#include <observation_tasks.h>
#include <approx_point_to_shape.h>
#include <axis_bound.h>
#include <axis_distance.h>
#include <komo_factory.h>

using namespace std;

//===========================================================================

struct CarKinematic:TaskMap{

  CarKinematic( const std::string & object )
    : object_( object )
  {

  }

  virtual mlr::String shortTag(const mlr::KinematicWorld& G)
  {
    return mlr::String("CarKinematic");
  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1) override
  {
    CHECK(order==1,"");

    mlr::Frame *object = G.getFrameByName( object_.c_str() );

    // get speed vector
    arr y_vel,Jvel;
    TaskMap_Default vel(posDiffTMT, object->ID );
    vel.order = 1;
    vel.phi(y_vel, Jvel, G, int( t ));

    // get orientation vector
    arr y_vec,Jvec;
    TaskMap_Default pos(vecTMT, object->ID, mlr::Vector(0,1,0));
    pos.order = 0;
    pos.phi(y_vec, Jvec, G, t);

    // commit results
    const double scale = 10;
    y.resize(1);
    y(0) = scale * scalarProduct(y_vel, y_vec)  ;

    // commit results
    if(&J){
     J = scale * ( ~y_vel * Jvec + ~y_vec * Jvel );
    }
  }

  virtual uint dim_phi(const mlr::KinematicWorld& K) override
  {
    return dim_;
  }

private:
  static const uint dim_ = 1;
  std::string object_;
};



//===========================================================================

void overtake()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( mlr::KinematicWorld( "model_overtake.g" ) );


  // general settings
  komo.setSquaredQAccelerations();
  //komo.setTask( 0.0, -1, new CarKinematic( "car_ego" ), OT_eq, NoArr, 1e2, 1 );

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
  komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );

  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
  komo.setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );

  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "car_op" );

  komo.setCollisions( true, 0.03 );

  //mlr::wait( 30, true );

  // launch komo
  komo.reset();
  komo.run();
  komo.checkGradients();

  Graph result = komo.getReport(true);

  //komo.plotTrajectory();
  //komo.plotVelocity();

  for(;;) komo.displayTrajectory(.05, true);
}

void carkin()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( mlr::KinematicWorld( "model_carkin.g" ) );

  // general settings
  komo.setSquaredQAccelerations();
  komo.setTask( 0.0, -1, new CarKinematic( "car_ego" ), OT_eq, NoArr, 1e2, 1 );

  // road bounds
  komo.setTask( 0.0, -1, new AxisBound( "car_ego", -0.15, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.15, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  // min speed
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // get sight
  komo.setTask( 2.0, 3.0, new AxisBound( "car_ego", 0.0, AxisBound::Y, AxisBound::MIN ), OT_sumOfSqr );

  // overtake constraints
  komo.setPosition( 5.0, -1, "car_ego", "truck", OT_sumOfSqr, { 0.6, 0, 0 } );

  // truck speed
  arr truck_speed{ 0.03, 0, 0 };
  komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );

  // opposite car speed
  arr op_speed{ -0.00, 0, 0 };
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
  komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );

  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
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
  komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );

  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
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
  komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );
  komo.setTask( 0.0, -1, new AxisBound( "truck_2",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // distance between vehicle
  //komo.setTask( 0.0, -1, new AxisDistance( "truck_2", "car_ego", 0.5, AxisDistance::Y, AxisDistance::MIN ), OT_ineq );

  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
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
  //komo.setPosition( 5.0, -1, "car_ego", "truck", OT_sumOfSqr, { 0.4, 0, 0 } );
  komo.setPosition( 5.0, -1, "car_ego", "truck_2", OT_sumOfSqr, { 0.4, 0, 0 } );

  // truck speed
  arr truck_speed{ 0.03, 0, 0 };
  komo.setVelocity( 0.0, 0.1, "truck", NULL, OT_eq, truck_speed );
  komo.setTask( 0.0, -1, new AxisBound( "truck",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.025 }, 1e2, 1 );
  komo.setTask( 0.0, -1, new AxisBound( "truck",  0.00, AxisBound::X, AxisBound::MAX ), OT_ineq,  arr{ 0.04 }, 1e2, 1 );
  komo.setTask( 0.0, -1, new AxisBound( "truck_2",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // distance between vehicle
  //komo.setTask( 0.0, -1, new AxisDistance( "truck_2", "car_ego", 0.5, AxisDistance::Y, AxisDistance::MIN ), OT_ineq );

  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
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

void cooperative_3_bis()
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
  //komo.setPosition( 5.0, -1, "car_ego", "truck", OT_sumOfSqr, { 0.4, 0, 0 } );
  komo.setPosition( 7.0, -1, "car_ego", "truck_2", OT_sumOfSqr, { 0.4, 0, 0 } );

  // truck speed
  arr truck_speed{ 0.03, 0, 0 };
  komo.setVelocity( 0.0, 0.1, "truck", NULL, OT_eq, truck_speed );
  komo.setTask( 0.0, -1, new AxisBound( "truck",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.025 }, 1e2, 1 );
  komo.setTask( 0.0, -1, new AxisBound( "truck",  0.00, AxisBound::X, AxisBound::MAX ), OT_ineq,  arr{ 0.04 }, 1e2, 1 );
  komo.setTask( 0.0, -1, new AxisBound( "truck_2",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // distance between vehicle
  //komo.setTask( 0.0, -1, new AxisDistance( "truck_2", "car_ego", 0.5, AxisDistance::Y, AxisDistance::MIN ), OT_ineq );

  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
  komo.setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );

  komo.activateCollisions( "truck", "truck_2" );
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

void cooperative_4()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( mlr::KinematicWorld( "model_cooperative_4.g" ) );

  //komo.world.watch( true );
  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.setTask( 0.0, -1, new AxisBound( "car_ego", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "car_1", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_1",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "car_2", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_2",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "car_3", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_3",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "truck", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  //komo.setTask( 0.0, -1, new AxisBound( "truck", -0.075, AxisBound::Y, AxisBound::MIN, 0.1 ), OT_sumOfSqr );
  komo.setTask( 0.0, -1, new AxisBound( "truck",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  // min speed
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.06 }, 1e2, 1 );

  // get sight
  //komo.setTask( 3.0, 3.0, new AxisBound( "car_ego", 0.0, AxisBound::Y, AxisBound::MIN ), OT_sumOfSqr );

  // overtake constraints
  //komo.setPosition( 5.0, -1, "car_ego", "truck", OT_sumOfSqr, { 0.4, 0, 0 } );
  //komo.setPosition( 7.0, -1, "car_ego", "truck_2", OT_sumOfSqr, { 0.4, 0, 0 } );

  // vehicle speeds
  komo.setTask( 0.0, -1, new AxisBound( "truck",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.025 }, 1e2, 1 );
  komo.setTask( 0.0, -1, new AxisBound( "truck",  0.00, AxisBound::X, AxisBound::MAX ), OT_ineq,  arr{ 0.045 }, 1e2, 1 );

  komo.setTask( 0.0, -1, new AxisBound( "car_1",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );
  komo.setTask( 0.0, -1, new AxisBound( "car_1",  0.00, AxisBound::X, AxisBound::MAX ), OT_ineq,  arr{ 0.045 }, 1e2, 1 );

  komo.setTask( 0.0, -1, new AxisBound( "car_2",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );
  komo.setTask( 0.0, -1, new AxisBound( "car_2",  0.00, AxisBound::X, AxisBound::MAX ), OT_ineq,  arr{ 0.045 }, 1e2, 1 );

  komo.setTask( 0.0, -1, new AxisBound( "car_3",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );
  komo.setTask( 0.0, -1, new AxisBound( "car_3",  0.00, AxisBound::X, AxisBound::MAX ), OT_ineq,  arr{ 0.04 }, 1e2, 1 );


  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "car_1" );
  komo.activateCollisions( "car_ego", "car_2" );
  komo.activateCollisions( "car_ego", "car_3" );

  komo.activateCollisions( "truck", "car_1" );
  komo.activateCollisions( "truck", "car_2" );
  komo.activateCollisions( "truck", "car_3" );

  komo.activateCollisions( "car_1", "car_2" );
  komo.activateCollisions( "car_1", "car_3" );

  komo.activateCollisions( "car_2", "car_3" );

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

void cooperative_5()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( mlr::KinematicWorld( "model_cooperative_5.g" ) );

  //komo.world.watch( true );
  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.setTask( 0.0, -1, new AxisBound( "car_ego", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "car_1", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_1",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "car_2", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_2",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "car_3", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_3",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "truck", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  //komo.setTask( 0.0, -1, new AxisBound( "truck", -0.075, AxisBound::Y, AxisBound::MIN, 0.1 ), OT_sumOfSqr );
  komo.setTask( 0.0, -1, new AxisBound( "truck",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  // min speed
  //komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.06 }, 1e2, 1 );
  komo.setVelocity( 0.0, -1, "car_ego", NULL, OT_eq, { 0.07, 0, 0 } );

  // get sight
  //komo.setTask( 3.0, 3.0, new AxisBound( "car_ego", 0.0, AxisBound::Y, AxisBound::MIN ), OT_sumOfSqr );

  // overtake constraints
  //komo.setPosition( 5.0, -1, "car_ego", "truck", OT_sumOfSqr, { 0.4, 0, 0 } );
  //komo.setPosition( 7.0, -1, "car_ego", "truck_2", OT_sumOfSqr, { 0.4, 0, 0 } );

  // vehicle speeds
  komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, { 0.03, 0, 0 } );

  //komo.setVelocity( 0.0, -1, "car_1", NULL, OT_sumOfSqr, { 0.03, 0, 0 } );
  //komo.setVelocity( 0.0, -1, "car_2", NULL, OT_sumOfSqr, { 0.03, 0, 0 } );
  //komo.setVelocity( 0.0, -1, "car_3", NULL, OT_sumOfSqr, { 0.03, 0, 0 } );


  komo.setTask( 0.0, -1, new AxisBound( "car_1",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.025 }, 1e2, 1 );
  komo.setTask( 0.0, -1, new AxisBound( "car_1",  0.00, AxisBound::X, AxisBound::MAX ), OT_ineq,  arr{ 0.045 }, 1e2, 1 );

  komo.setTask( 0.0, -1, new AxisBound( "car_2",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );
  komo.setTask( 0.0, -1, new AxisBound( "car_2",  0.00, AxisBound::X, AxisBound::MAX ), OT_ineq,  arr{ 0.045 }, 1e2, 1 );

  komo.setTask( 0.0, -1, new AxisBound( "car_3",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );
  komo.setTask( 0.0, -1, new AxisBound( "car_3",  0.00, AxisBound::X, AxisBound::MAX ), OT_ineq,  arr{ 0.045 }, 1e2, 1 );


  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "car_1" );
  komo.activateCollisions( "car_ego", "car_2" );
  komo.activateCollisions( "car_ego", "car_3" );

  komo.activateCollisions( "truck", "car_1" );
  komo.activateCollisions( "truck", "car_2" );
  komo.activateCollisions( "truck", "car_3" );

  komo.activateCollisions( "car_1", "car_2" );
  komo.activateCollisions( "car_1", "car_3" );

  komo.activateCollisions( "car_2", "car_3" );

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

void cooperative_flying()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( mlr::KinematicWorld( "model_cooperative_flying.g" ) );

  //komo.world.watch( true );
  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.setTask( 0.0, -1, new AxisBound( "car_ego", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "car_1", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_1",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "car_2", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_2",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "car_3", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_3",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  komo.setTask( 0.0, -1, new AxisBound( "truck", -0.12, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  //komo.setTask( 0.0, -1, new AxisBound( "truck", -0.075, AxisBound::Y, AxisBound::MIN, 0.1 ), OT_sumOfSqr );
  komo.setTask( 0.0, -1, new AxisBound( "truck",  0.12, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  // min speed
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.05 }, 1e2, 1 );

  // get sight
  //komo.setTask( 3.0, 3.0, new AxisBound( "car_ego", 0.0, AxisBound::Y, AxisBound::MIN ), OT_sumOfSqr );

  // overtake constraints
  //komo.setPosition( 5.0, -1, "car_ego", "truck", OT_sumOfSqr, { 0.4, 0, 0 } );
  //komo.setPosition( 7.0, -1, "car_ego", "truck_2", OT_sumOfSqr, { 0.4, 0, 0 } );

  // vehicle speeds
  komo.setTask( 0.0, -1, new AxisBound( "truck",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.025 }, 1e2, 1 );
  komo.setTask( 0.0, -1, new AxisBound( "truck",  0.00, AxisBound::X, AxisBound::MAX ), OT_ineq,  arr{ 0.04 }, 1e2, 1 );

  komo.setTask( 0.0, -1, new AxisBound( "car_1",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.025 }, 1e2, 1 );
  //komo.setTask( 0.0, -1, new AxisBound( "car_1",  0.00, AxisBound::X, AxisBound::MAX ), OT_ineq,  arr{ 0.04 }, 1e2, 1 );

  komo.setTask( 0.0, -1, new AxisBound( "car_2",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.025 }, 1e2, 1 );
  //komo.setTask( 0.0, -1, new AxisBound( "car_2",  0.00, AxisBound::X, AxisBound::MAX ), OT_ineq,  arr{ 0.04 }, 1e2, 1 );

  komo.setTask( 0.0, -1, new AxisBound( "car_3",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.025 }, 1e2, 1 );
  //komo.setTask( 0.0, -1, new AxisBound( "car_3",  0.00, AxisBound::X, AxisBound::MAX ), OT_ineq,  arr{ 0.04 }, 1e2, 1 );


  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "car_1" );
  komo.activateCollisions( "car_ego", "car_2" );
  komo.activateCollisions( "car_ego", "car_3" );

  komo.activateCollisions( "truck", "car_1" );
  komo.activateCollisions( "truck", "car_2" );
  komo.activateCollisions( "truck", "car_3" );

  komo.activateCollisions( "car_1", "car_2" );
  komo.activateCollisions( "car_1", "car_3" );

  komo.activateCollisions( "car_2", "car_3" );

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

void lane_insertion()
{
  mp::ExtensibleKOMO komo;
  komo.setConfigFromFile();
  komo.setModel( mlr::KinematicWorld( "model_lane_insertion.g" ) );

  //komo.world.watch( true );
  // general settings
  komo.setSquaredQAccelerations();

  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.setTask( 0.0, -1, new AxisBound( "car_ego", -0.15, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.15, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  // min speed
  komo.setTask( 0.0, -1, new AxisBound( "car_ego",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.025}, 1e2, 1 );

  // overtake constraints
  komo.setPosition( 7.0, -1, "car_ego", "car_1", OT_sumOfSqr, { 0.225, 0, 0 } );
  //komo.setPosition( 10, -1, "car_ego", "truck", OT_sumOfSqr, { 0.475, 0, 0 } );

  // other vehicles speeds
  arr k_speed{ 0.03, 0, 0 };
  komo.setVelocity( 0.0, -1, "truck", NULL, OT_eq, k_speed );
  komo.setVelocity( 0.0, -1, "car_1", NULL, OT_eq, k_speed );
  komo.setVelocity( 0.0, -1, "car_2", NULL, OT_eq, k_speed );
  komo.setVelocity( 0.0, -1, "car_3", NULL, OT_eq, k_speed );

  komo.activateCollisions( "car_ego", "truck" );
  komo.activateCollisions( "car_ego", "car_1" );
  komo.activateCollisions( "car_ego", "car_2" );
  komo.activateCollisions( "car_ego", "car_3" );

  komo.setCollisions( true, 0.03 );

  // launch komo
  komo.reset();
  komo.run();
  //komo.checkGradients();

  Graph result = komo.getReport(true);

  komo.plotTrajectory();
  komo.plotVelocity();

  for(;;) komo.displayTrajectory(.05, true);
}

void test_config()
{
  //mlr::KinematicWorld G( "model_cooperative_5.g" );
  mlr::KinematicWorld G( "model_attempt_paper.g" );
  //std::cout << G.q.d0 << std::endl;
  G.watch( true );
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  //carkin();

  //overtake();

  //attempt();

  //cooperative();

  //cooperative_2();

  //cooperative_3();

  //cooperative_3_bis(); // strange

  //cooperative_4();

  //cooperative_5();


  //cooperative_flying();

  //lane_insertion();

  test_config();

  return 0;
}
