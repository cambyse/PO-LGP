#include <graph_planner.h>

#include <gtest/gtest.h>

using namespace matp;

class GraphPlannerTest : public ::testing::Test {
 protected:
  GraphPlanner tp;
};

// GraphPlanner
TEST_F(GraphPlannerTest, ThrowIfFileNotFound) {
  ASSERT_THROW( tp.setFol( "data/nofile.g" ), FolFileNotFound );
}

TEST_F(GraphPlannerTest, ThrowIfNoAgentFiles) {
  ASSERT_THROW( tp.setFol( "" ), FolFileNotFound );
}

TEST_F(GraphPlannerTest, AgentNumber) {
  tp.setFol( "data/LGP-overtaking-double-agent-2w.g" );
  ASSERT_EQ( tp.agentNumber(), 2 );
}

TEST_F(GraphPlannerTest, emptyGraph) {
  tp.buildGraph();
  auto graph = tp.decisionGraph();
  ASSERT_TRUE( graph.empty() );
}

TEST_F(GraphPlannerTest, OneNodeIfCorrectFile) {
  tp.setFol( "data/LGP-overtaking-single-agent-1w.g" );
  auto graph = tp.decisionGraph();
  ASSERT_TRUE( graph.size() == 1 );
}

TEST_F(GraphPlannerTest, buildGraph) {
  tp.setFol( "data/LGP-overtaking-single-agent-1w.g" );
  tp.buildGraph();
  auto graph = tp.decisionGraph();
  ASSERT_TRUE( graph.size() > 1 );
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
