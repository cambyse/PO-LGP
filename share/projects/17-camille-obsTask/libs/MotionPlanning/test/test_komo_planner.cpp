#include "komo_planner.h"
#include <gtest/gtest.h>

using namespace std;

using namespace mp;

/////////////////Tasks////////////////////////
struct AxisBound:TaskMap{

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

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1)
  {
//    for( auto p : G.proxies )
//    {
//      std::cout << p->a << " " << p->b << std::endl;
//    }

    mlr::Frame *object = G.getFrameByName( object_.c_str() );
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
  const double k_;
  std::size_t id_= 0;
};


/////////////////Grounders////////////////////
void init( double phase, const std::vector< std::string > & args, mp::ExtensibleKOMO * komo, int verbose )
{
  // road bounds
  komo->setTask( 0.0, -1, new AxisBound( "car_ego", -0.15, AxisBound::Y, AxisBound::MIN ), OT_ineq );
  komo->setTask( 0.0, -1, new AxisBound( "car_ego",  0.15, AxisBound::Y, AxisBound::MAX ), OT_ineq );

  // min speed
  komo->setTask( 0.0, -1, new AxisBound( "car_ego",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // truck speed
  arr truck_speed{ 0.03, 0, 0 };
  truck_speed( 0 ) = 0.03;
  komo->setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );

  // min speed
  komo->setTask( 0.0, 1.0, new AxisBound( "car_ego", -0.1, AxisBound::Y, AxisBound::MAX ), OT_sumOfSqr );
  komo->setTask( 0.0, -1, new AxisBound( "car_ego",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

  // collision
  komo->activateCollisions( "car_ego", "truck" );
  komo->activateCollisions( "car_ego", "car_op" );
  komo->setCollisions( true );
}

void groundInitSingleAgent( double phase, const std::vector< std::string > & args, mp::ExtensibleKOMO * komo, int verbose )
{
  init( phase, args, komo, verbose );

  // opposite car speed
  arr op_speed{ -0.03, 0, 0 };
  komo->setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );
}

void groundInitDoubleAgent( double phase, const std::vector< std::string > & args, mp::ExtensibleKOMO * komo, int verbose )
{
  init( phase, args, komo, verbose );

  arr op_speed{ -0.04, 0, 0 };
  komo->setVelocity( 0, 1.5, "car_op", NULL, OT_eq, op_speed );
}

void groundLook( double phase, const std::vector< std::string > & args, mp::ExtensibleKOMO * komo, int verbose )
{
  //
  const double t_start = phase;
  const double t_end =   phase + 1;
  //

  // look
  komo->setTask( t_start + 0.9, t_end, new AxisBound( "car_ego", 0.0, AxisBound::Y, AxisBound::MIN ), OT_sumOfSqr );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " look " << args[0] << " at " << args[1] << std::endl;
  }
}

void groundOvertake( double phase, const std::vector< std::string > & args, mp::ExtensibleKOMO * komo, int verbose )
{
  //
  const double t_start = phase;
  const double t_end =   phase + 1.0;
  //

  // overtake
  //komo->setTask( t_start -0.5, t_start + 0.5, new AxisBound( "car_ego", 0.05, AxisBound::Y, AxisBound::MIN ), OT_sumOfSqr );
  komo->setPosition( t_end, -1, "car_ego", args[0].c_str(), OT_sumOfSqr, { 0.45, 0, 0 } );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " overtake " << args[0] << std::endl;
  }
}

void groundFollow( double phase, const std::vector< std::string > & args, mp::ExtensibleKOMO * komo, int verbose )
{
  //
  const double t_start = phase;
  const double t_end =   phase + 1.0;
  //

  // overtake
  komo->setPosition( t_end, -1, "car_ego", "truck", OT_sumOfSqr, { -0.7, 0, 0 } ); // -0.55

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " follow " << args[0] << " at " << args[1] << std::endl;
  }
}

void groundAccelerate( double phase, const std::vector< std::string > & args, mp::ExtensibleKOMO * komo, int verbose )
{
  //
  const double t_start = phase;
  const double t_end =   phase + 1.0;
  //

  // opposite car speed
  arr op_speed{ -0.07, 0, 0 };
  komo->setVelocity( t_start, -1, "car_op", NULL, OT_eq, op_speed );
}

void groundContinue( double phase, const std::vector< std::string > & args, mp::ExtensibleKOMO * komo, int verbose )
{
  //
  const double t_start = phase;
  const double t_end =   phase + 1.0;
  //

  // opposite car speed
  arr op_speed{ -0.04, 0, 0 };
  komo->setVelocity( t_start, -1, "car_op", NULL, OT_eq, op_speed );
}

void groundSlowDown( double phase, const std::vector< std::string > & args, mp::ExtensibleKOMO * komo, int verbose )
{
  //
  const double t_start = phase;
  const double t_end =   phase + 1.0;
  //

  // opposite car speed
  arr op_speed{ -0.015, 0, 0 };
  komo->setVelocity( t_start, -1, "car_op", NULL, OT_eq, op_speed );
}

//////////////Fixture////////////////
struct KomoPlannerFixture : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    // register symbols
    planner.registerTask( "initSingleAgent"      , groundInitSingleAgent );
    planner.registerTask( "initDoubleAgent"      , groundInitDoubleAgent );

    planner.registerTask( "__AGENT_0__look"      , groundLook );
    planner.registerTask( "__AGENT_0__overtake"  , groundOvertake );
    planner.registerTask( "__AGENT_0__follow"    , groundFollow );

    planner.registerTask( "__AGENT_1__accelerate", groundAccelerate );
    planner.registerTask( "__AGENT_1__continue"  , groundContinue );
    planner.registerTask( "__AGENT_1__slow_down" , groundSlowDown );
  }

  virtual void TearDown()
  {

  }

  mp::KOMOPlanner planner;
};

/////////////////////////////////////

TEST_F(KomoPlannerFixture, ParseKinFileDoesntThrow1w)
{
  EXPECT_NO_THROW( planner.setKin( "data/LGP-overtaking-kin.g" ) );
}

TEST_F(KomoPlannerFixture, ParseKinFileDoesntThrow2w)
{
  EXPECT_NO_THROW( planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" ) );
}

/////////////////////SINGLE AGENT OBSERVABLE/////////////////////////////
TEST_F(KomoPlannerFixture, PlanSingleAgent1WMarkovianPath)
{
  planner.setKin( "data/LGP-overtaking-kin.g" );

  Skeleton policy;
  policy.load( "data/LGP-overtaking-single-agent-1w-policy.po" );

  // add set up
  policy.root()->data().leadingKomoArgs = {"initSingleAgent"};

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "markovJointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Skeleton::INFORMED );
}

TEST_F(KomoPlannerFixture, PlanSingleAgent1WJointPath)
{
  planner.setKin( "data/LGP-overtaking-kin.g" );

  Skeleton policy;
  policy.load( "data/LGP-overtaking-single-agent-1w.po" );

  // add set up
  policy.root()->data().leadingKomoArgs = {"initSingleAgent"};

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Skeleton::INFORMED );
}

TEST_F(KomoPlannerFixture, PlanSingleAgent1WDisplay)
{
  planner.setKin( "data/LGP-overtaking-kin.g" );

  Skeleton policy;
  policy.load( "data/LGP-overtaking-single-agent-1w.po" );

  // add set up
  policy.root()->data().leadingKomoArgs = {"initSingleAgent"};

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.display( policy, 10.0 ) );
}

/////////////////////SINGLE AGENT PARTIALLY OBSERVABLE/////////////////////////////
TEST_F(KomoPlannerFixture, PlanSingleAgent2WMarkovianPath)
{
  planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" );

  Skeleton policy;
  policy.load( "data/LGP-overtaking-single-agent-2w.po" );

  // add set up
  policy.root()->data().leadingKomoArgs = {"initSingleAgent"};

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "markovJointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Skeleton::INFORMED );
}

TEST_F(KomoPlannerFixture, PlanSingleAgent2WJointPath)
{
  planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" );

  Skeleton policy;
  policy.load( "data/LGP-overtaking-single-agent-2w.po" );

  // add set up
  policy.root()->data().leadingKomoArgs = {"initSingleAgent"};

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Skeleton::INFORMED );
}

TEST_F(KomoPlannerFixture, PlanSingleAgent2WDisplay)
{
  planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" );

  Skeleton policy;
  policy.load( "data/LGP-overtaking-single-agent-2w.po" );

  // add set up
  policy.root()->data().leadingKomoArgs = {"initSingleAgent"};

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.display( policy, 10.0 ) );
}

/////////////////////TWO AGENTS FULLY OBSERVABLE/////////////////////////////
TEST_F(KomoPlannerFixture, PlanTwoAgents1WMarkovianPath)
{
  planner.setKin( "data/LGP-overtaking-kin.g" );

  Skeleton policy;
  policy.load( "data/LGP-overtaking-double-agent-1w.po" );

  // add set up
  policy.root()->data().leadingKomoArgs = {"initDoubleAgent"};

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "markovJointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Skeleton::INFORMED );
}

TEST_F(KomoPlannerFixture, PlanTwoAgents1WJointPath)
{
  planner.setKin( "data/LGP-overtaking-kin.g" );

  Skeleton policy;
  policy.load( "data/LGP-overtaking-double-agent-1w.po" );

  // add set up
  policy.root()->data().leadingKomoArgs = {"initDoubleAgent"};

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Skeleton::INFORMED );
}

TEST_F(KomoPlannerFixture, PlanTwoAgents1WDisplay)
{
  planner.setKin( "data/LGP-overtaking-kin.g" );

  Skeleton policy;
  policy.load( "data/LGP-overtaking-double-agent-1w.po" );

  // add set up
  policy.root()->data().leadingKomoArgs = {"initDoubleAgent"};

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.display( policy, 10.0 ) );
}

TEST_F(KomoPlannerFixture, PlanTwoAgents1WTweakedDisplay)
{
  planner.setKin( "data/LGP-overtaking-kin.g" );

  Skeleton policy;
  policy.load( "data/LGP-overtaking-double-agent-1w-tweaked.po" );

  // add set up
  policy.root()->data().leadingKomoArgs = {"initDoubleAgent"};

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.display( policy, 10.0 ) );
}

/////////////////////TWO AGENTS PARTIALLY OBSERVABLE/////////////////////////////
TEST_F(KomoPlannerFixture, PlanTwoAgents2WMarkovianPath)
{
  planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" );

  Skeleton policy;
  policy.load( "data/LGP-overtaking-double-agent-2w.po" );

  // add set up
  policy.root()->data().leadingKomoArgs = {"initDoubleAgent"};

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "markovJointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Skeleton::INFORMED );
}

TEST_F(KomoPlannerFixture, PlanTwoAgents2WJointPath)
{
  planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" );

  Skeleton policy;
  policy.load( "data/LGP-overtaking-double-agent-2w.po" );

  // add set up
  policy.root()->data().leadingKomoArgs = {"initDoubleAgent"};

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Skeleton::INFORMED );
}

TEST_F(KomoPlannerFixture, PlanTwoAgents2WDisplay)
{
  planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" );

  Skeleton policy;
  policy.load( "data/LGP-overtaking-double-agent-2w.po" );

  // add set up
  policy.root()->data().leadingKomoArgs = {"initDoubleAgent"};

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.display( policy, 10.0 ) );
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

