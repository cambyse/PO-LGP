#include <graph_planner.h>

#include <gtest/gtest.h>

using namespace matp;

class GraphPlannerTest : public ::testing::Test {
 protected:
  GraphPlanner tp;
};

class WorldsTest : public ::testing::Test {
 protected:
  Worlds w;
};

// Worlds
TEST_F(WorldsTest, ThrowIfFileNotFound) {
  ASSERT_THROW( w.setFol( "data/nofile.g" ), FolFileNotFound );
}

//TEST_F(WorldsTest, FolEnginesInitializedSingleAgent1W) {
//  w.setFol( "data/LGP-overtaking-single-agent-1w.g" );
//  ASSERT_EQ( w.enginesInitialized(), true );
//}

//TEST_F(WorldsTest, FolEnginesInitializedSingleAgent2W) {
//  w.setFol( "data/LGP-overtaking-single-agent-2w.g" );
//  ASSERT_EQ( w.enginesInitialized(), true );
//}

//TEST_F(WorldsTest, FolEnginesInitializedDoubleAgent1W) {
//  w.setFol( "data/LGP-overtaking-double-agent-1w.g" );
//  ASSERT_EQ( w.enginesInitialized(), true );
//}

//TEST_F(WorldsTest, FolEnginesInitializedDoubleAgent2W) {
//  w.setFol( "data/LGP-overtaking-double-agent-2w.g" );
//  ASSERT_EQ( w.enginesInitialized(), true );
//}

// Agent Number
TEST_F(WorldsTest, AgentNumberSingleAgent1W) {
  w.setFol( "data/LGP-overtaking-single-agent-1w.g" );
  ASSERT_EQ( w.agentNumber(), 1 );
}

TEST_F(WorldsTest, AgentNumberDoubleAgent1W) {
  w.setFol( "data/LGP-overtaking-double-agent-1w.g" );
  ASSERT_EQ( w.agentNumber(), 2 );
}

TEST_F(WorldsTest, AgentNumberSingleAgent2W) {
  w.setFol( "data/LGP-overtaking-single-agent-2w.g" );
  ASSERT_EQ( w.agentNumber(), 1 );
}

TEST_F(WorldsTest, AgentNumberDoubleAgent2W) {
  w.setFol( "data/LGP-overtaking-double-agent-2w.g" );
  ASSERT_EQ( w.agentNumber(), 2 );
}

// Total Number Of Worlds
TEST_F(WorldsTest, StartStateNumberSingleAgent1W) {
  w.setFol( "data/LGP-overtaking-single-agent-1w.g" );
  ASSERT_EQ( w.possibleStartStates().size(), 1 );
}

TEST_F(WorldsTest, StartStateNumberSingleAgent2W) {
  w.setFol( "data/LGP-overtaking-single-agent-2w.g" );
  ASSERT_EQ( w.possibleStartStates().size(), 2 );
}

// Start state content
TEST_F(WorldsTest, StartStatesSizeSingleAgent1W) {
  w.setFol( "data/LGP-overtaking-single-agent-1w.g" );
  ASSERT_GT( w.possibleStartStates()[ 0 ].size(), 1 );
}

TEST_F(WorldsTest, StartStatesSizeSingleAgent2W) {
  w.setFol( "data/LGP-overtaking-single-agent-2w.g" );
  ASSERT_GT( w.possibleStartStates()[ 0 ].size(), 1 );
  ASSERT_GT( w.possibleStartStates()[ 1 ].size(), 1 );
}

//TEST_F(WorldsTest, StartStatesSizeSingleAgent2W) {
//  w.setFol( "data/LGP-overtaking-single-agent-2w.g" );
//  ASSERT_GT( w.possibleStartStates()[ 0 ].size(), 1 );
//  ASSERT_GT( w.possibleStartStates()[ 1 ].size(), 1 );
//}

// Agent Actions

/*TEST_F(WorldsTest, FolBeliefStateSize1) {
  w.setFol( "data/LGP-overtaking-agent-1.g" );
  ASSERT_EQ( w.beliefStateSize(), 1 );
}

TEST_F(WorldsTest, FolBeliefState1) {
  w.setFol( "data/LGP-overtaking-agent-1.g" );
  ASSERT_EQ( w.beliefState()[ 0 ], 1.0 );
}

TEST_F(WorldsTest, FolBeliefStateSize2) {
  w.setFol( "data/LGP-overtaking-agent-1-bs-2.g" );
  ASSERT_EQ( w.beliefStateSize(), 2 );
}

TEST_F(WorldsTest, FolBeliefState2) {
  w.setFol( "data/LGP-overtaking-agent-1-bs-2.g" );
  ASSERT_EQ( ag.beliefState()[ 1 ], 0.95 );
}*/

// GraphPlanner
TEST_F(GraphPlannerTest, ThrowIfFileNotFound) {
  ASSERT_THROW( tp.setFol( "data/nofile.g" ), FolFileNotFound );
}

TEST_F(GraphPlannerTest, ThrowIfNoAgentFiles) {
  ASSERT_THROW( tp.setFol( "" ), FolFileNotFound );
}

//TEST_F(GraphPlannerTest, AgentNumber) {
//  tp.setFol( "data/LGP-overtaking-single-agent-1w.g" );
//  ASSERT_EQ( tp.agentNumber(), 1 );
//}

//TEST_F(GraphPlannerTest, AgentNumber2) {
//  tp.setFols( {"data/LGP-overtaking-agent-1.g",
//               "data/LGP-overtaking-agent-2.g" } );
//  ASSERT_EQ( tp.agentNumber(), 2 );
//}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
