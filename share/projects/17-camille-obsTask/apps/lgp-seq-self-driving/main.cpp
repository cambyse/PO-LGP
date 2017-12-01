#include <KOMO/komo.h>

#include <observation_tasks.h>
#include <approx_point_to_shape.h>

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

struct Axisbound:TaskMap{

  enum Axis
  {
    X = 0,
    Y
  };

  enum BoundType
  {
    MIN = 0,
    MAX
  };

  Axisbound( const std::string & object, double bound, const enum Axis & axis, const enum BoundType & boundType )
    : object_( object )
    , bound_( bound )
    , boundType_( boundType )
    , id_( axis == X ? 0 : 1 )
  {

  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1)
  {
    mlr::Frame *object = G.getFrameByName( object_.c_str() );
    arr posObject, posJObject;
    G.kinematicsPos(posObject, posJObject, object);    // get function to minimize and its jacobian in state G

    const double sign = ( ( boundType_ == MIN ) ? 1 : -1 );

    arr tmp_y = zeros( dim_ );
    tmp_y( 0 ) = - sign * ( posObject( id_ ) - bound_ );

    arr tmp_J = zeros( dim_, posJObject.dim(1) );
    tmp_J.setMatrixBlock( - sign * posJObject.row( id_ ), 0 , 0 );    // jacobian

    // commit results
    y = tmp_y;
    if(&J) J = tmp_J;

    // v
    // w
    // ->R

    // v = cos(theta) x' + sin(theta) y'

    //v = x' + y'
    //x' * cos(theta) = v
    //y' * sin(theta) = w
  }

  virtual uint dim_phi(const mlr::KinematicWorld& G)
  {
    return dim_;
  }

  virtual mlr::String shortTag(const mlr::KinematicWorld& G)
  {
    return mlr::String("AxisBound");
  }

private:
  static const uint dim_ = 1;
  std::string object_;
  const double bound_;
  const BoundType boundType_;
  std::size_t id_;
};

//===========================================================================

void overtake()
{
  KOMO komo;
  komo.setConfigFromFile();

  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.setTask( 0.0, -1, new Axisbound( "car_ego", -0.15, Axisbound::Y, Axisbound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new Axisbound( "car_ego",  0.15, Axisbound::Y, Axisbound::MAX ), OT_ineq );

  // min speed
  komo.setTask( 0.0, -1, new Axisbound( "car_ego",  0.00, Axisbound::X, Axisbound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // get sight
  komo.setTask( 3.0, 4.0, new Axisbound( "car_ego", 0.0, Axisbound::Y, Axisbound::MIN ), OT_sumOfSqr );

  // overtake constraints
  komo.setPosition( 5.0, -1, "car_ego", "truck", OT_sumOfSqr, { 0.6, 0, 0 } );

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
  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.1, true);
}

void attempt()
{
  KOMO komo;
  komo.setConfigFromFile();

  // general settings
  komo.setSquaredQAccelerations();

  // road bounds
  komo.setTask( 0.0, -1, new Axisbound( "car_ego", -0.15, Axisbound::Y, Axisbound::MIN ), OT_ineq );
  komo.setTask( 0.0, -1, new Axisbound( "car_ego",  0.15, Axisbound::Y, Axisbound::MAX ), OT_ineq );

  // min speed
  komo.setTask( 0.0, -1, new Axisbound( "car_ego",  0.00, Axisbound::X, Axisbound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // get sight
  komo.setTask( 3.0, 4.0, new Axisbound( "car_ego", 0.0, Axisbound::Y, Axisbound::MIN ), OT_sumOfSqr );

  // overtake constraints
  komo.setPosition( 5.0, -1, "car_ego", "truck", OT_sumOfSqr, { -0.6, 0, 0 } );

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
  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.1, true);
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  //move_1();

  //move_blocks();

  //overtake();

  attempt();

  return 0;
}
