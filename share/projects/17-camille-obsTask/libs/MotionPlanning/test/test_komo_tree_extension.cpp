#include "komo_planner.h"
#include <gtest/gtest.h>
#include <functional>

#include <Core/graph.h>
#include <Core/array.h>

#include <Kin/taskMap_default.h>
#include <Kin/taskMap_transition.h>

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
class InitialGrounder
{
public:
  void init( mp::ExtensibleKOMO * komo, int verbose )
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

  virtual void groundInitSingleAgent( mp::ExtensibleKOMO * komo, int verbose )
  {
    init( komo, verbose );

    // opposite car speed
    arr op_speed{ -0.03, 0, 0 };
    komo->setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );
  }

  virtual void groundInitDoubleAgent( mp::ExtensibleKOMO * komo, int verbose )
  {
    init( komo, verbose );

    arr op_speed{ -0.04, 0, 0 };
    komo->setVelocity( 0, 1.5, "car_op", NULL, OT_eq, op_speed );
  }
};

class InitGrounderMock : public InitialGrounder
{
public:
  virtual void groundInitSingleAgent( mp::ExtensibleKOMO * komo, int verbose )
  {
    InitialGrounder::groundInitSingleAgent( komo, verbose );

    nInitSingleAgent++;
  }

  virtual void groundInitDoubleAgent( mp::ExtensibleKOMO * komo, int verbose )
  {
    InitialGrounder::groundInitDoubleAgent( komo, verbose );

    nInitDoubleAgent++;
  }

  uint nInitSingleAgent = 0;
  uint nInitDoubleAgent = 0;
};

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

void groundMergeBetween( double phase, const std::vector< std::string >& facts, mp::ExtensibleKOMO * komo, int verbose )
{
  auto car_before = facts[0];
  auto car_next = facts[1];

  komo->setPosition( phase+1, -1, "car_ego", car_next.c_str(), OT_sumOfSqr, {-0.7, 0, 0} );
  komo->setPosition( phase+1, -1, car_before.c_str(), "car_ego", OT_sumOfSqr, {-0.7, 0, 0} );

  //setKeepDistanceTask( phase+1, -1, komo, car_successors );
  //std::cout << "merge between " << car_before << " and " << car_next << std::endl;
}

//////////////Fixture////////////////
struct KomoTreeExtensionFixture : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    kin.init("data/LGP-mono-car.g");
    komo.setModel(kin);
  }

  virtual void TearDown()
  {

  }

  mlr::KinematicWorld kin;
  ExtensibleKOMO komo;
};

/////////////////////////////////////

TEST(RowShifting, Understanding_RowShifting)
{
  arr J;
  uint xN = 20;
  uint band_size = 6;// (k+1)*dim_xmax
  uint phiN = xN * 3; // xN * nTask * taskDim
  RowShifted *Jaux = makeRowShifted(J, phiN, band_size, xN);

  EXPECT_EQ(J.d0, phiN);
  EXPECT_EQ(J.d1, band_size);
  EXPECT_EQ(Jaux->real_d1, xN);

  // assign
  J(0, 0) = 1; // store phiN * band_size elements
  J(1, Jaux->rowShift(1)) = 1;

  //J(30, 1) = 1;

  // necessary?
  Jaux->reshift();
  Jaux->computeColPatches(true);

  // unshift
  auto Junshifted = unpack(J); // store phiN * xN

  EXPECT_EQ(Junshifted.d0, phiN);
  EXPECT_EQ(Junshifted.d1, xN);
  EXPECT_EQ(Junshifted(0, 0), 1);
  EXPECT_EQ(Junshifted(1, 1), 1);
}

TEST_F(KomoTreeExtensionFixture, SimpleTwoBranchesTreePlanning)
{
  //komo->setTiming( 1., 2, 5., 1/*, true*/ );
  komo.setTiming( 10, 10, 1.0, 2 );

  arr op_speed_1{ 0.5, 0, 0 };
  arr op_speed_2{ 1.5, 0, 0 };

  //komo.setVelocity( 0, -1, "car_ego", NULL, OT_sumOfSqr, op_speed );

  auto path_1 = uintA(52); // n phase + 2 steps (prefix)
  for(uint t=0; t < 52; t++)
  {
    path_1(t)=t;
  }

  auto path_2 = uintA(52);
  for(uint t=0; t < 10; t++)
  {
    path_2(t)=t;
  }
  for(uint t=10; t < 52; t++)
  {
    path_2(t)=40+t+2;
  }

  komo.setTreeTask(0, -1, path_1, new TaskMap_Transition(komo.world), OT_sumOfSqr, NoArr, 1.0, 2);
  komo.setTreeTask(0,  5, path_1, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_1, 1.0, 1);

  komo.setTreeTask(0, -1, path_2, new TaskMap_Transition(komo.world), OT_sumOfSqr, NoArr, 1.0, 2);
  komo.setTreeTask(3,  5, path_2, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_2, 1.0, 1);

  komo.reset();
  komo.checkGradients();
  komo.run();

  //Graph g = komo.getReport(true);
  //komo.plotVelocity();
  komo.displayTrajectory(0.1, true);
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

