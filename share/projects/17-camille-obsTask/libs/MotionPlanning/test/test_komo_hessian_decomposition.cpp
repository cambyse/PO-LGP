#include "komo_planner_fixture.h"

/////////////////////SINGLE AGENT PARTIALLY OBSERVABLE/////////////////////////////

TEST_F(KomoPlannerSingleAgentFixture, PlanSingleAgent2WJointPath)
{
  planner.setKin( "data/LGP-overtaking-kin-2w_bis.g" );

  Policy policy;
  policy.load( "data/LGP-overtaking-single-agent-2w.po" );

  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointSparse" );

  EXPECT_NO_THROW( planner.solveAndInform( po, policy ) );
  EXPECT_EQ( policy.status(), Policy::INFORMED );
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

