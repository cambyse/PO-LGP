#include <graph_planner.h>

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

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
  tp.setFol( "data/LGP-overtaking-single-agent-2w.g" );
  tp.setMaxDepth( 3 );
  tp.buildGraph();
  tp.saveGraphToFile( "LGP-overtaking-single-agent-2w.gv" );
  auto graph = tp.decisionGraph();
  ASSERT_TRUE( graph.size() > 1 );
}

TEST_F(GraphPlannerTest, buildGraphDoubleAgent1w) {
  tp.setFol( "data/LGP-overtaking-double-agent-1w.g" );
  tp.setMaxDepth( 2 );
  tp.buildGraph();
  tp.saveGraphToFile( "LGP-overtaking-double-agent-1w.gv" );
  auto graph = tp.decisionGraph();
  ASSERT_TRUE( graph.size() > 1 );
}

//TEST_F(GraphPlannerTest, solveDoubleAgent1w) {
//  tp.setFol( "data/LGP-overtaking-double-agent-1w.g" );
//  tp.solve();
//  auto policy = tp.getPolicy();
//  ASSERT_NE( policy, nullptr );
//}

TEST_F(GraphPlannerTest, solveDoubleAgent2w) {
  tp.setFol( "data/LGP-overtaking-double-agent-2w.g" );
  tp.solve();
  auto policy = tp.getNewPolicy();
  ASSERT_FALSE( policy.empty() );
}

TEST_F(GraphPlannerTest, DecisionArtifactToKomoTag) {
  auto leadingArtifact = "(actionName X Y Z)";
  auto args = decisionArtifactToKomoArgs( leadingArtifact );

  ASSERT_EQ( args[0]    , "actionName" );
  ASSERT_EQ( args[1]    , "X" );
  ASSERT_EQ( args[2]    , "Y" );
  ASSERT_EQ( args[3]    , "Z" );
}

TEST_F(GraphPlannerTest, DecisionGraphNodeToPolicyNode) {
  auto r0 = -2;
  auto agentId = 5;
  auto beliefState = std::vector< double >( { 0.4, 0.6 } );
  auto leadingArtifact = "(actionName X Y Z)";
  auto nodeType = NodeData::NodeType::ACTION;
  auto p = 0.5;
  auto states = std::vector< std::string >( { "", "" } );
  auto terminal = false;

  tp.setR0( r0 );

  NodeData nData;
  nData.agentId = agentId;
  nData.beliefState = beliefState;
  nData.leadingArtifact = leadingArtifact;
  nData.nodeType = nodeType;
  nData.p = p;
  nData.states = states;
  nData.terminal = terminal;

  auto pData = tp.decisionGraphtoPolicyData( nData );

  ASSERT_EQ( pData.beliefState    , beliefState );
  ASSERT_EQ( pData.markovianReturn, r0 );
  ASSERT_EQ( pData.leadingKomoArgs.size(), 4 );
  ASSERT_EQ( pData.p, p );
}

TEST_F(GraphPlannerTest, PolicySave) {
  tp.setFol( "data/LGP-overtaking-single-agent-2w.g" );
  tp.solve();
  auto policy = tp.getNewPolicy();
  const std::string policyFileName( "saved_policy.po" );
  policy.save( policyFileName );
  ASSERT_TRUE( boost::filesystem::exists( policyFileName ) );
}

TEST_F(GraphPlannerTest, PolicySaveToGraph) {
  tp.setFol( "data/LGP-overtaking-double-agent-2w.g" );
  tp.setMaxDepth( 2 );
  tp.solve();
  auto policy = tp.getNewPolicy();
  const std::string policyFileName( "LGP-overtaking-double-agent-2w.gv" );
  policy.saveToGraphFile( policyFileName );
  ASSERT_TRUE( boost::filesystem::exists( policyFileName ) );
}

TEST_F(GraphPlannerTest, PolicyLeafs) {
  tp.setFol( "data/LGP-overtaking-single-agent-1w.g" );
  tp.setMaxDepth( 2 );
  tp.solve();
  tp.saveGraphToFile( "LGP-overtaking-single-agent-1w-decision-graph.gv" );
  tp.saveDecidedGraphToFile( "LGP-overtaking-single-agent-1w-decided-decision-graph.gv" );
  auto policy = tp.getNewPolicy();
  const std::string policyFileName( "LGP-overtaking-single-agent-1w-policy.gv" );
  policy.saveToGraphFile( policyFileName );
  auto leafs = policy.leafs();
  auto leaf = leafs.front();
  EXPECT_EQ( leafs.size(), 1 );
  EXPECT_EQ( leaf.lock()->id(), 2 );
}

//TEST_F(GraphPlannerTest, solveDoubleAgent1w) {
//  tp.setFol( "data/LGP-overtaking-single-agent-2w.g" );
//  tp.solve();
//  auto policy = tp.getPolicy();
//  ASSERT_NE( policy, nullptr );
//}
// - early stop in VI if stable

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
