#include "komo_planner.h"
#include <gtest/gtest.h>
#include <functional>

using namespace std;
using namespace mp;

/*
 *
 * Tests of classical linear traj komo with car examples
 *
 */

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

  virtual void groundInitSingleAgent( KOMO_ext* komo, int verbose )
  {
    init( komo, verbose );

    // opposite car speed
    arr op_speed{ -0.03, 0, 0 };
    komo->setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );
  }

  virtual void groundInitDoubleAgent( KOMO_ext* komo, int verbose )
  {
    init( komo, verbose );

    arr op_speed{ -0.04, 0, 0 };
    komo->setVelocity( 0, 1.5, "car_op", NULL, OT_eq, op_speed );
  }
};

class InitGrounderMock : public InitialGrounder
{
public:
  virtual void groundInitSingleAgent( KOMO_ext* komo, int verbose )
  {
    InitialGrounder::groundInitSingleAgent( komo, verbose );

    nInitSingleAgent++;
  }

  virtual void groundInitDoubleAgent( KOMO_ext* komo, int verbose )
  {
    InitialGrounder::groundInitDoubleAgent( komo, verbose );

    nInitDoubleAgent++;
  }

  uint nInitSingleAgent = 0;
  uint nInitDoubleAgent = 0;
};

void groundLook( double phase, const std::vector< std::string > & args, KOMO_ext* komo, int verbose )
{
  //
  const double t_start = phase;
  const double t_end =   phase + 1;
  //

  // look
  komo->addObjective( t_start + 0.9, t_end, new AxisBound( "car_ego", 0.0, AxisBound::Y, AxisBound::MIN ), OT_sos );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " look " << args[0] << " at " << args[1] << std::endl;
  }
}

void groundOvertake( double phase, const std::vector< std::string > & args, KOMO_ext* komo, int verbose )
{
  //
  const double t_start = phase;
  const double t_end =   phase + 1.0;
  //

  // overtake
  //komo->addObjective( t_start -0.5, t_start + 0.5, new AxisBound( "car_ego", 0.05, AxisBound::Y, AxisBound::MIN ), OT_sos );
  komo->setPosition( t_end, -1, "car_ego", args[0].c_str(), OT_sos, { 0.45, 0, 0 } );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " overtake " << args[0] << std::endl;
  }
}

void groundFollow( double phase, const std::vector< std::string > & args, KOMO_ext* komo, int verbose )
{
  //
  const double t_start = phase;
  const double t_end =   phase + 1.0;
  //

  // overtake
  komo->setPosition( t_end, -1, "car_ego", "truck", OT_sos, { -0.7, 0, 0 } ); // -0.55

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " follow " << args[0] << " at " << args[1] << std::endl;
  }
}

void groundAccelerate( double phase, const std::vector< std::string > & args, KOMO_ext* komo, int verbose )
{
  //
  const double t_start = phase;
  const double t_end =   phase + 1.0;
  //

  // opposite car speed
  arr op_speed{ -0.07, 0, 0 };
  komo->setVelocity( t_start, -1, "car_op", NULL, OT_eq, op_speed );
}

void groundContinue( double phase, const std::vector< std::string > & args, KOMO_ext* komo, int verbose )
{
  //
  const double t_start = phase;
  const double t_end =   phase + 1.0;
  //

  // opposite car speed
  arr op_speed{ -0.04, 0, 0 };
  komo->setVelocity( t_start, -1, "car_op", NULL, OT_eq, op_speed );
}

void groundSlowDown( double phase, const std::vector< std::string > & args, KOMO_ext* komo, int verbose )
{
  //
  const double t_start = phase;
  const double t_end =   phase + 1.0;
  //

  // opposite car speed
  arr op_speed{ -0.015, 0, 0 };
  komo->setVelocity( t_start, -1, "car_op", NULL, OT_eq, op_speed );
}

void groundMergeBetween( double phase, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose )
{
  auto car_before = facts[0];
  auto car_next = facts[1];

  komo->setPosition( phase+1, -1, "car_ego", car_next.c_str(), OT_sos, {-0.7, 0, 0} );
  komo->setPosition( phase+1, -1, car_before.c_str(), "car_ego", OT_sos, {-0.7, 0, 0} );

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
    planner.registerInit( std::bind( &InitialGrounder::groundInitSingleAgent, &initGrounder, _1, _2 ) );
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
    planner.registerInit( std::bind( &InitialGrounder::groundInitDoubleAgent, &initGrounder, _1, _2 ) );
  }
};


/////////////////////////////////////

TEST_F(KomoPlannerSingleAgentFixture, ParseKinFileDoesntThrow1w)
{
  EXPECT_NO_THROW( planner.setKin( "data/LGP-overtaking-kin.g" ) );
}

TEST_F(KomoPlannerSingleAgentFixture, ParseKinFileDoesntThrow2w)
{
  EXPECT_NO_THROW( planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" ) );
}

TEST_F(KomoPlannerSingleAgentFixture, InitialGroundingIsCalledWithAtEachStageForMarkovianPaths)
{
  planner.setKin( "data/LGP-overtaking-kin.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-single-agent-1w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "markovJointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( initGrounder.nInitSingleAgent, 6 );  // 3 (pose) + 3(markov)
}

TEST_F(KomoPlannerSingleAgentFixture, InitialGroundingIsCalledWithAtEachStageForJointPaths)
{
  planner.setKin(  "data/LGP-overtaking-kin-2w_bis.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-single-agent-2w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( initGrounder.nInitSingleAgent, 10 );  // 6 (pose) + 2 (paths) + 2(joint paths)
}

TEST_F(KomoPlannerSingleAgentFixture, InitialGroundingIsCalledWithNoRandomVector)
{
  planner.setKin(  "data/LGP-overtaking-kin-2w_bis.g" );
  auto randomVec = planner.drawRandomVector();

  EXPECT_EQ( randomVec.size(), 0 );
}


TEST_F(KomoPlannerSingleAgentFixture, InitialGroundingIsCalledWithRandomVectorOfCorrectSize)
{
  planner.setKin( "data/LGP-merging-kin.g" );
  auto randomVec = planner.drawRandomVector();

  EXPECT_EQ( randomVec.size(), 2 );
}

TEST_F(KomoPlannerSingleAgentFixture, TestRandomVectorOverride)
{
  planner.setKin( "data/LGP-merging-kin.g" );
  std::vector<double> randomVec = planner.drawRandomVector({1.0, 0.5});

  EXPECT_EQ( randomVec, std::vector<double>({1.0, 0.5}) );
}

/////////////////////SINGLE AGENT OBSERVABLE/////////////////////////////
TEST_F(KomoPlannerSingleAgentFixture, PlanSingleAgent1WMarkovianPath)
{
  planner.setKin( "data/LGP-overtaking-kin.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-single-agent-1w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "markovJointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Policy::INFORMED );
}

TEST_F(KomoPlannerSingleAgentFixture, PlanSingleAgent1WJointPath)
{
  planner.setKin( "data/LGP-overtaking-kin.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-single-agent-1w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Policy::INFORMED );
}

TEST_F(KomoPlannerSingleAgentFixture, PlanSingleAgent1WDisplay)
{
  planner.setKin( "data/LGP-overtaking-kin.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-single-agent-1w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.display( policy, 3.0 ) );
}

/////////////////////SINGLE AGENT PARTIALLY OBSERVABLE/////////////////////////////
TEST_F(KomoPlannerSingleAgentFixture, PlanSingleAgent2WMarkovianPath)
{
  planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-single-agent-2w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "markovJointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Policy::INFORMED );
}

TEST_F(KomoPlannerSingleAgentFixture, PlanSingleAgent2WMarkovianPath_NodeSetAsPlanned)
{
  planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-single-agent-2w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "markovJointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Policy::INFORMED );

  for(const auto & leaf: policy.leaves())
  {
    auto path = getPathTo( leaf );
    for( const auto & n: path )
    {
      if(n->data().markovianReturn != 0)
        EXPECT_EQ( n->data().status, PolicyNodeData::INFORMED );
      else
        EXPECT_EQ( n->data().status, PolicyNodeData::UNPLANNED );
    }
  }
}

TEST_F(KomoPlannerSingleAgentFixture, PlanSingleAgent2WJointPath)
{
  planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-single-agent-2w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Policy::INFORMED );
}

TEST_F(KomoPlannerSingleAgentFixture, PlanSingleAgent2WDisplay)
{
  planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-single-agent-2w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.display( policy, 10.0 ) );
}

/////////////////////TWO AGENTS FULLY OBSERVABLE/////////////////////////////
TEST_F(KomoPlannerDoubleAgentFixture, PlanTwoAgents1WMarkovianPath)
{
  planner.setKin( "data/LGP-overtaking-kin.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-double-agent-1w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "markovJointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Policy::INFORMED );
}

TEST_F(KomoPlannerDoubleAgentFixture, PlanTwoAgents1WJointPath)
{
  planner.setKin( "data/LGP-overtaking-kin.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-double-agent-1w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Policy::INFORMED );
}

TEST_F(KomoPlannerDoubleAgentFixture, PlanTwoAgents1WDisplay)
{
  planner.setKin( "data/LGP-overtaking-kin.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-double-agent-1w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.display( policy, 10.0 ) );
}

TEST_F(KomoPlannerDoubleAgentFixture, PlanTwoAgents1WTweakedDisplay)
{
  planner.setKin( "data/LGP-overtaking-kin.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-double-agent-1w-tweaked.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.display( policy, 10.0 ) );
}

/////////////////////TWO AGENTS PARTIALLY OBSERVABLE/////////////////////////////
TEST_F(KomoPlannerDoubleAgentFixture, PlanTwoAgents2WMarkovianPath)
{
  planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-double-agent-2w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "markovJointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Policy::INFORMED );
}

TEST_F(KomoPlannerDoubleAgentFixture, PlanTwoAgents2WJointPath)
{
  planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-double-agent-2w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Policy::INFORMED );
}

TEST_F(KomoPlannerDoubleAgentFixture, PlanTwoAgents2WDisplay)
{
  planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-double-agent-2w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.display( policy, 10.0 ) );
}

//TEST_F(KomoPlannerSingleAgentFixture, QResultSetAfterMarkovianPlanning)
//{
//  planner.setKin( "data/LGP-overtaking-kin.g" );

//  Skeleton policy;
//  policy.load( "data/LGP-overtaking-single-agent-1w.po" );

//  MotionPlanningParameters po( policy.id() );
//  po.setParam( "type", "markovJointPath" );

//  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
//  EXPECT_EQ( policy.status(), Skeleton::INFORMED );
//  EXPECT_EQ( policy.qresult().nWorlds(), 1 );
//  EXPECT_GE( policy.qresult().nSteps(0), 1 );
//}

TEST_F(KomoPlannerSingleAgentFixture, QResultSetAfterJointPathPlanning1W)
{
  planner.setKin( "data/LGP-overtaking-kin.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-single-agent-1w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Policy::INFORMED );
  EXPECT_EQ( policy.qresult().nWorlds(), 1 );
  EXPECT_GE( policy.qresult().nSteps(0), 1 );
}


TEST_F(KomoPlannerSingleAgentFixture, QResultSetAfterJointPathPlanning2W)
{
  planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-single-agent-2w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Policy::INFORMED );
  EXPECT_EQ( policy.qresult().nWorlds(), 2 );
  EXPECT_GE( policy.qresult().nSteps(1), 1 );
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

