#pragma one

#include "komo_planner.h"
#include <komo_wrapper.h>

#include <gtest/gtest.h>
#include <functional>

using namespace std;
using namespace mp;

/*
 *
 * Tests of various komo configurations komo with car examples
 *
 */

using W = mp::KomoWrapper;

/////////////////Tasks////////////////////////
struct AxisBound:Feature{

  enum Axis
  {
    X = 0,
    Y,
    Z
  };

  enum BoundType
  {
    MIN = 0,
    MAX
  };

  AxisBound( const std::string & object, double bound, const enum Axis & axis, const enum BoundType & boundType, const double k = 1.0 )
    : object_( object )
    , bound_( bound )
    , boundType_( boundType )
    , k_( k )
  {
    if( axis == X ) id_ = 0;
    else if( axis == Y ) id_ = 1;
    else if( axis == Z ) id_ = 2;
  }

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G)
  {
    rai::Frame *object = G.getFrameByName( object_.c_str() );
    arr posObject, posJObject;
    G.kinematicsPos(posObject, posJObject, object);    // get function to minimize and its jacobian in state G

    const double sign = ( ( boundType_ == MIN ) ? 1 : -1 );

    arr tmp_y = zeros( dim_ );
    tmp_y( 0 ) = - k_ * sign * ( posObject( id_ ) - bound_ );

    arr tmp_J = zeros( dim_, posJObject.dim(1) );
    tmp_J.setMatrixBlock( - k_ * sign * posJObject.row( id_ ), 0 , 0 );    // jacobian

    // commit results
    y = tmp_y;
    if(&J) J = tmp_J;
  }

  virtual uint dim_phi(const rai::KinematicWorld& G)
  {
    return dim_;
  }

  virtual rai::String shortTag(const rai::KinematicWorld& G)
  {
    return rai::String("AxisBound");
  }

private:
  static const uint dim_ = 1;
  std::string object_;
  const double bound_;
  const BoundType boundType_;
  const double k_;
  std::size_t id_= 0;
};


/////////////////Grounders////////////////////
class InitialGrounder
{
public:
  void init( KOMO_ext* komo, int verbose )
  {
    mp::Interval always{{0, -1}, {0, 1}};

    // road bounds
    komo->addObjective( 0.0, -1, new AxisBound( "car_ego", -0.15, AxisBound::Y, AxisBound::MIN ), OT_ineq );
    komo->addObjective( 0.0, -1, new AxisBound( "car_ego",  0.15, AxisBound::Y, AxisBound::MAX ), OT_ineq );

    // min speed
    komo->addObjective( 0.0, -1, new AxisBound( "car_ego",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

    // truck speed
    arr truck_speed{ 0.03, 0, 0 };
    truck_speed( 0 ) = 0.03;
    komo->setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );

    // min speed
    komo->addObjective( 0.0, 1.0, new AxisBound( "car_ego", -0.1, AxisBound::Y, AxisBound::MAX ), OT_sos );
    komo->addObjective( 0.0, -1, new AxisBound( "car_ego",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

    // collision
    komo->activateCollisions( "car_ego", "truck" );
    komo->activateCollisions( "car_ego", "car_op" );
    komo->add_collision( true );
  }

  virtual void groundInitSingleAgent( const TreeBuilder& tree, KOMO_ext* komo, int verbose )
  {
    init( komo, verbose );

    // opposite car speed
    arr op_speed{ -0.03, 0, 0 };
    komo->setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );
  }

  virtual void groundInitDoubleAgent( const TreeBuilder& tree, KOMO_ext* komo, int verbose )
  {
    init( komo, verbose );

    arr op_speed{ -0.04, 0, 0 };
    komo->setVelocity( 0, 1.5, "car_op", NULL, OT_eq, op_speed );
  }
};

class InitGrounderMock : public InitialGrounder
{
public:
  virtual void groundInitSingleAgent( const TreeBuilder& tree, KOMO_ext* komo, int verbose )
  {
    InitialGrounder::groundInitSingleAgent( tree, komo, verbose );

    nInitSingleAgent++;
  }

  virtual void groundInitDoubleAgent( const TreeBuilder& tree, KOMO_ext* komo, int verbose )
  {
    InitialGrounder::groundInitDoubleAgent( tree, komo, verbose );

    nInitDoubleAgent++;
  }

  uint nInitSingleAgent = 0;
  uint nInitDoubleAgent = 0;
};

void groundLook( const mp::Interval& phase, const mp::TreeBuilder& tree, const std::vector< std::string > & args, KOMO_ext* komo, int verbose )
{
  //
  const double t_start = phase.time.from;
  const double t_end =   phase.time.to;
  //

  // look
  komo->addObjective( t_start + 0.9, t_end, new AxisBound( "car_ego", 0.0, AxisBound::Y, AxisBound::MIN ), OT_sos );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " look " << " at " << args[0] << std::endl;
  }
}

void groundOvertake( const mp::Interval& phase, const mp::TreeBuilder& tree, const std::vector< std::string > & args, KOMO_ext* komo, int verbose )
{
  //
  const double t_start = phase.time.from;
  const double t_end =   phase.time.to;
  //

  // overtake
  //komo->addObjective( t_start -0.5, t_start + 0.5, new AxisBound( "car_ego", 0.05, AxisBound::Y, AxisBound::MIN ), OT_sos );
  komo->setPosition( t_end, -1, "car_ego", args[0].c_str(), OT_sos, { 0.45, 0, 0 } );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " overtake " << args[0] << std::endl;
  }
}

void groundFollow( const mp::Interval& phase, const mp::TreeBuilder& tree, const std::vector< std::string > & args, KOMO_ext* komo, int verbose )
{
  //
  const double t_start = phase.time.from;
  const double t_end =   phase.time.to;
  //

  // overtake
  komo->setPosition( t_end, -1, "car_ego", "truck", OT_sos, { -0.7, 0, 0 } ); // -0.55

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " follow " << args[0] << std::endl;
  }
}

void groundAccelerate( const mp::Interval& phase, const mp::TreeBuilder& tree, const std::vector< std::string > & args, KOMO_ext* komo, int verbose )
{
  //
  const double t_start = phase.time.from;
  const double t_end =   phase.time.to;
  //

  // opposite car speed
  arr op_speed{ -0.07, 0, 0 };
  komo->setVelocity( t_start, -1, "car_op", NULL, OT_eq, op_speed );
}

void groundContinue( const mp::Interval& phase, const mp::TreeBuilder& tree, const std::vector< std::string > & args, KOMO_ext* komo, int verbose )
{
  //
  const double t_start = phase.time.from;
  const double t_end =   phase.time.to;
  //

  // opposite car speed
  arr op_speed{ -0.04, 0, 0 };
  komo->setVelocity( t_start, -1, "car_op", NULL, OT_eq, op_speed );
}

void groundSlowDown( const mp::Interval& phase, const mp::TreeBuilder& tree, const std::vector< std::string > & args, KOMO_ext* komo, int verbose )
{
  //
  const double t_start = phase.time.from;
  const double t_end =   phase.time.to;
  //

  // opposite car speed
  arr op_speed{ -0.015, 0, 0 };
  komo->setVelocity( t_start, -1, "car_op", NULL, OT_eq, op_speed );
}

void groundMergeBetween( const mp::Interval& phase, const mp::TreeBuilder& tree, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose )
{
  //
  const double t_start = phase.time.from;
  const double t_end =   phase.time.to;
  //
  auto car_before = facts[0];
  auto car_next = facts[1];

  komo->setPosition( t_start+1, -1, "car_ego", car_next.c_str(), OT_sos, {-0.7, 0, 0} );
  komo->setPosition( t_start+1, -1, car_before.c_str(), "car_ego", OT_sos, {-0.7, 0, 0} );

  //setKeepDistanceTask( phase+1, -1, komo, car_successors );
  //std::cout << "merge between " << car_before << " and " << car_next << std::endl;
}

//////////////Fixture////////////////
struct KomoPlannerFixture : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    // register symbols
    planner.registerTask( "__AGENT_0__look"      , groundLook );
    planner.registerTask( "__AGENT_0__overtake"  , groundOvertake );
    planner.registerTask( "__AGENT_0__follow"    , groundFollow );

    planner.registerTask( "__AGENT_1__accelerate", groundAccelerate );
    planner.registerTask( "__AGENT_1__continue"  , groundContinue );
    planner.registerTask( "__AGENT_1__slow_down" , groundSlowDown );

    planner.registerTask( "merge_between" , groundMergeBetween );
  }

  virtual void TearDown()
  {

  }

  InitGrounderMock initGrounder;
  mp::KOMOPlanner planner;
};

struct KomoPlannerSingleAgentFixture : public KomoPlannerFixture
{
protected:
  virtual void SetUp()
  {
    KomoPlannerFixture::SetUp();

    using namespace std::placeholders;

    // register symbols
    planner.registerInit( std::bind( &InitialGrounder::groundInitSingleAgent, &initGrounder, _1, _2, _3 ) );
  }
};

struct KomoPlannerDoubleAgentFixture : public KomoPlannerFixture
{
protected:
  virtual void SetUp()
  {
    KomoPlannerFixture::SetUp();

    using namespace std::placeholders;

    // register symbols
    planner.registerInit( std::bind( &InitialGrounder::groundInitDoubleAgent, &initGrounder, _1, _2, _3 ) );
  }
};
