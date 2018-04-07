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

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
