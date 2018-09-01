#include <graph_planner.h>

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

using namespace matp;

// DecideOnGraph
TEST(DecideOnGraph, CorrectDecisionEvenWithInfiniteReward) {
  LogicEngine engine;
  std::vector< std::string > startStates;
  std::vector< double > egoBeliefState;
  DecisionGraph graph( engine, startStates, egoBeliefState );

  auto root = graph.root();

  // branch 1
  auto child_o_1 = root->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, false, 0, NodeData::NodeType::OBSERVATION) );
  graph._addNode( child_o_1 );
  graph._addEdge( child_o_1->id(), root->id(), 1.0, "" );
  auto child_a_1 = child_o_1->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, false, 0, NodeData::NodeType::ACTION) );
  graph._addNode( child_a_1 );
  graph._addEdge( child_a_1->id(), child_o_1->id(), 1.0, "" );
  auto child_o_2 = child_a_1->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, false, 0, NodeData::NodeType::OBSERVATION) );
  graph._addNode( child_o_2 );
  graph._addEdge( child_o_2->id(), child_a_1->id(), 1.0, "" );
  auto child_a_2 = child_o_2->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, true, 0, NodeData::NodeType::ACTION) );
  graph._addNode( child_a_2 );
  graph._addEdge( child_a_2->id(), child_o_2->id(), 1.0, "" );

  // branch 2
  auto child_o_10 = root->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, false, 0, NodeData::NodeType::OBSERVATION) );
  graph._addNode( child_o_10 );
  graph._addEdge( child_o_10->id(), root->id(), 1.0, "" );
  auto child_a_10 = child_o_10->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, false, 0, NodeData::NodeType::ACTION) );
  graph._addNode( child_a_10 );
  graph._addEdge( child_a_10->id(), child_o_10->id(), 1.0, "" );
  auto child_o_20 = child_a_10->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, false, 0, NodeData::NodeType::OBSERVATION) );
  graph._addNode( child_o_20 );
  graph._addEdge( child_o_20->id(), child_a_10->id(), 1.0, "" );
  auto child_a_20 = child_o_20->makeChild( DecisionGraph::GraphNodeDataType({""}, {1.0}, true, 0, NodeData::NodeType::ACTION) );
  graph._addNode( child_a_20 );
  graph._addEdge( child_a_20->id(), child_o_20->id(), 1.0, "" );

  graph.saveGraphToFile( "CorrectDecisionEvenWithInfiniteReward.gv" );

  std::vector< double > rewards =  { -1.0, -1.0, -1.0, -1.0, -1.0, -1000.0, -0.1, -0.1, -0.1 };

//  /////
  auto values = ValueIterationAlgorithm::process( graph, rewards );
  auto decided = DecideOnGraphAlgorithm::process( graph, values, rewards );
//  /////

  decided.saveGraphToFile( "CorrectDecisionEvenWithInfiniteRewardDecided.gv" );

  EXPECT_EQ( decided.terminalNodes().size(), 1 );
  EXPECT_EQ( decided.terminalNodes().back().lock()->id(), 4 );
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
