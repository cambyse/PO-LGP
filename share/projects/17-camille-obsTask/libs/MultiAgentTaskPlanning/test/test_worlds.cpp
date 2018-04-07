#include <worlds.h>

#include <gtest/gtest.h>

using namespace matp;

class WorldsTest : public ::testing::Test {
 protected:
  Worlds w;
};

// Worlds
TEST_F(WorldsTest, ThrowIfFileNotFound) {
  ASSERT_THROW( w.setFol( "data/nofile.g" ), FolFileNotFound );
}

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
TEST_F(WorldsTest, StartStatesContentSizeSingleAgent1W) {
  w.setFol( "data/LGP-overtaking-single-agent-1w.g" );
  ASSERT_GT( w.possibleStartStates()[ 0 ].size(), 1 );
}

TEST_F(WorldsTest, StartStatesContentSizeSingleAgent2W) {
  w.setFol( "data/LGP-overtaking-single-agent-2w.g" );
  ASSERT_GT( w.possibleStartStates()[ 0 ].size(), 1 );
  ASSERT_GT( w.possibleStartStates()[ 1 ].size(), 1 );
}

// Ego Agent belief state
TEST_F(WorldsTest, BeliefStateSizeSingleAgent1W) {
  w.setFol( "data/LGP-overtaking-single-agent-1w.g" );
  ASSERT_EQ( w.egoBeliefState().size(), 1 );
}

TEST_F(WorldsTest, BeliefStateSizeSingleAgent2W) {
  w.setFol( "data/LGP-overtaking-single-agent-2w.g" );
  ASSERT_EQ( w.egoBeliefState().size(), 2 );
}

TEST_F(WorldsTest, BeliefStateSizeDoubleAgent1W) {
  w.setFol( "data/LGP-overtaking-double-agent-1w.g" );
  ASSERT_EQ( w.egoBeliefState().size(), 1 );
}

TEST_F(WorldsTest, BeliefStateSizeDoubleAgent2W) {
  w.setFol( "data/LGP-overtaking-double-agent-2w.g" );
  ASSERT_EQ( w.egoBeliefState().size(), 2 );
}

// Ill defined files
TEST_F(WorldsTest, WrongBeliefStateSize) {
  ASSERT_THROW( w.setFol( "data/LGP-overtaking-single-agent-2w-wrong-belief-state-size.g" ), IncoherentDefinition );
}

TEST_F(WorldsTest, WrongBeliefProbabilities) {
  ASSERT_THROW( w.setFol( "data/LGP-overtaking-single-agent-2w-wrong-probabilities.g" ), IncoherentDefinition );
}

// Agent Actions Number
TEST_F(WorldsTest, FirstAgentActionsNumber) {
  w.setFol( "data/LGP-overtaking-single-agent-1w.g" );
  ASSERT_EQ( w.actionsNumber( 0 ), 3 );
}

TEST_F(WorldsTest, SecondAgentActionsNumber) {
  w.setFol( "data/LGP-overtaking-double-agent-1w.g" );
  ASSERT_EQ( w.actionsNumber( 1 ), 3 );
}

TEST_F(WorldsTest, ThrowIfActionsWithoutPrefix) {
  ASSERT_THROW( w.setFol( "data/LGP-overtaking-double-agent-2w-wrong-action-definitions.g" ), IncoherentDefinition );
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
