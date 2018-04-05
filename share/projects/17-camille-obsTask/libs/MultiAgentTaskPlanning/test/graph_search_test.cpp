#include <graph_planner.h>

#include <gtest/gtest.h>

using namespace matp;

class GraphPlannerTest : public ::testing::Test {
 protected:
  GraphPlanner tp;
};

class AgentTest : public ::testing::Test {
 protected:
  Agent ag;
};

// Agent
TEST_F(AgentTest, ThrowIfFileNotFound) {
  ASSERT_THROW( ag.setFols( {"data/nofile.g"} ), FolFileNotFound );
}

TEST_F(AgentTest, FolEnginesInitialized) {
  ag.setFols( "data/LGP-overtaking-agent-1.g" );
  ASSERT_EQ( ag.enginesInitialized(), true );
}

TEST_F(AgentTest, FolBeliefStateSize1) {
  ag.setFols( "data/LGP-overtaking-agent-1.g" );
  ASSERT_EQ( ag.beliefStateSize(), 1 );
}

TEST_F(AgentTest, FolBeliefState1) {
  ag.setFols( "data/LGP-overtaking-agent-1.g" );
  ASSERT_EQ( ag.beliefState()[ 0 ], 1.0 );
}

TEST_F(AgentTest, FolBeliefStateSize2) {
  ag.setFols( "data/LGP-overtaking-agent-1-bs-2.g" );
  ASSERT_EQ( ag.beliefStateSize(), 2 );
}

TEST_F(AgentTest, FolBeliefState2) {
  ag.setFols( "data/LGP-overtaking-agent-1-bs-2.g" );
  ASSERT_EQ( ag.beliefState()[ 1 ], 0.95 );
}

// GraphPlanner
TEST_F(GraphPlannerTest, ThrowIfFileNotFound) {
  ASSERT_THROW( tp.setFols( {"data/nofile.g"} ), FolFileNotFound );
}

TEST_F(GraphPlannerTest, ThrowIfNoAgentFiles) {
  ASSERT_THROW( tp.setFols( {} ), MissingArgument );
}

TEST_F(GraphPlannerTest, AgentNumber) {
  tp.setFols( {"data/LGP-overtaking-agent-1.g"} );
  ASSERT_EQ( tp.agentNumber(), 1 );
}

TEST_F(GraphPlannerTest, AgentNumber2) {
  tp.setFols( {"data/LGP-overtaking-agent-1.g",
               "data/LGP-overtaking-agent-2.g" } );
  ASSERT_EQ( tp.agentNumber(), 2 );
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
