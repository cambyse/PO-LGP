#include <graph_planner.h>

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

using namespace matp;

// ValueIteration

TEST(ValueIteration, OnUnsolvedRootCyclicGraphValuesAreInfinity) {
  LogicEngine engine;
  std::vector< std::string > startStates;
  std::vector< double > egoBeliefState;
  DecisionGraph graph( engine, startStates, egoBeliefState );
  auto root = graph.root();
  DecisionGraph::GraphNodeDataType data_observation( {""}, {1.0}, false, 0, NodeData::NodeType::OBSERVATION );
  DecisionGraph::GraphNodeDataType data_action( {""}, {1.0}, false, 0, NodeData::NodeType::ACTION );
  DecisionGraph::GraphNodeDataType not_terminal( {""}, {1.0}, false, 0, NodeData::NodeType::ACTION );

  auto child_o_1 = root->makeChild( DecisionGraph::GraphNodeDataType(data_observation) );
  auto child_a_2 = child_o_1->makeChild( DecisionGraph::GraphNodeDataType(data_action) );

  auto child_o_3 = child_a_2->makeChild( DecisionGraph::GraphNodeDataType(data_observation) );
  child_o_3->addExistingChild( root );

  graph._addNode( child_o_1 );
  graph._addEdge( child_o_1->id(), root->id(), 1.0, "" );
  graph._addNode( child_a_2 );
  graph._addEdge( child_a_2->id(), child_o_1->id(), 1.0, "" );
  graph._addNode( child_o_3 );
  graph._addEdge( child_o_3->id(), child_a_2->id(), 1.0, "" );
  graph._addEdge( root->id(), child_o_3->id(), 1.0, "" );

  graph.saveGraphToFile( "OnUnsolvedRootCyclicGraphValuesAreInfinity.gv" );

  std::vector< double > rewards =  { -1.0, -1.0, -1.0, -1.0, -1.0 };

  /////
  auto values = ValueIterationAlgorithm::process( graph, rewards );
  /////

  EXPECT_LE( values[0], -200.0 );

  EXPECT_LE( values[1], -200.0 );
  EXPECT_LE( values[2], -200.0 );

  EXPECT_LE( values[3], -200.0 );
}

TEST(ValueIteration, OnUnsolvedCyclicGraphValuesAreInfinity) {
  LogicEngine engine;
  std::vector< std::string > startStates;
  std::vector< double > egoBeliefState;
  DecisionGraph graph( engine, startStates, egoBeliefState );
  auto root = graph.root();
  DecisionGraph::GraphNodeDataType data_observation( {""}, {1.0}, false, 0, NodeData::NodeType::OBSERVATION );
  DecisionGraph::GraphNodeDataType data_action( {""}, {1.0}, false, 0, NodeData::NodeType::ACTION );
  DecisionGraph::GraphNodeDataType not_terminal( {""}, {1.0}, false, 0, NodeData::NodeType::ACTION );

  auto child_o_1 = root->makeChild( DecisionGraph::GraphNodeDataType(data_observation) );
  auto child_a_2 = child_o_1->makeChild( DecisionGraph::GraphNodeDataType(data_action) );

  auto child_o_3 = child_a_2->makeChild( DecisionGraph::GraphNodeDataType(data_observation) );
  auto child_a_4 = child_o_3->makeChild( DecisionGraph::GraphNodeDataType(data_action) );

  auto child_o_5 = child_a_4->makeChild( DecisionGraph::GraphNodeDataType(data_observation) );
  child_o_5->addExistingChild( child_a_2 );

  graph._addNode( child_o_1 );
  graph._addEdge( child_o_1->id(), root->id(), 1.0, "a" );
  graph._addNode( child_a_2 );
  graph._addEdge( child_a_2->id(), child_o_1->id(), 1.0, "a" );
  graph._addNode( child_o_3 );
  graph._addEdge( child_o_3->id(), child_a_2->id(), 1.0, "a" );
  graph._addNode( child_a_4 );
  graph._addEdge( child_a_4->id(), child_o_3->id(), 1.0, "a" );

  graph._addNode( child_o_5 );
  graph._addEdge( child_o_5->id(), child_a_4->id(), 1.0, "a" );
  graph._addEdge( child_a_2->id(), child_o_5->id(), 1.0, "a" );

  graph.saveGraphToFile( "OnUnsolvedCyclicGraphValuesAreInfinity.gv" );

  std::vector< double > rewards =  { -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0 };

  /////
  auto values = ValueIterationAlgorithm::process( graph, rewards );
  /////

  EXPECT_LE( values[0], -200.0 );

  EXPECT_LE( values[1], -200.0 );
  EXPECT_LE( values[2], -200.0 );

  EXPECT_LE( values[3], -200.0 );
  EXPECT_LE( values[4], -200.0 );

  EXPECT_LE( values[5], -200.0 );
}

TEST(ValueIteration, OnUnsolvedBranchValuesAreInfinity) {
  LogicEngine engine;
  std::vector< std::string > startStates;
  std::vector< double > egoBeliefState;
  DecisionGraph graph( engine, startStates, egoBeliefState );
  auto root = graph.root();

  auto child_o_1 = root->makeChild( DecisionGraph::GraphNodeDataType({"", ""}, {0.5, 0.5}, false, 0, NodeData::NodeType::OBSERVATION) );
  // branch 1
  auto child_a_2  = child_o_1->makeChild( DecisionGraph::GraphNodeDataType({"", ""}, {1.0, 0}, false, 0, NodeData::NodeType::ACTION) );
  auto child_o_3 = child_a_2->makeChild( DecisionGraph::GraphNodeDataType({"", ""}, {1.0, 0}, false, 0, NodeData::NodeType::OBSERVATION) );
  auto child_a_4 = child_o_3->makeChild( DecisionGraph::GraphNodeDataType({"", ""}, {1.0, 0}, false, 0, NodeData::NodeType::ACTION) );

  auto child_o_5 = child_a_4->makeChild( DecisionGraph::GraphNodeDataType({"", ""}, {1.0, 0}, false, 0, NodeData::NodeType::OBSERVATION) );
  child_o_5->addExistingChild( child_a_2 );
  // branch 2
  auto child_a_20 = child_o_1->makeChild( DecisionGraph::GraphNodeDataType({"", ""}, {0.0, 1.0}, false, 0, NodeData::NodeType::ACTION) );
  auto child_o_30 = child_a_20->makeChild( DecisionGraph::GraphNodeDataType({"", ""}, {0.0, 1.0}, false, 0, NodeData::NodeType::OBSERVATION) );
  auto child_a_40 = child_o_30->makeChild( DecisionGraph::GraphNodeDataType({"", ""}, {0.0, 1.0}, true, 0, NodeData::NodeType::ACTION) );

  graph._addNode( child_o_1 );
  graph._addEdge( child_o_1->id(), root->id(), 1.0, "" );

  graph._addNode( child_a_2 );
  graph._addEdge( child_a_2->id(), child_o_1->id(), 0.5, "" );
  graph._addNode( child_o_3 );
  graph._addEdge( child_o_3->id(), child_a_2->id(), 1.0, "" );
  graph._addNode( child_a_4 );
  graph._addEdge( child_a_4->id(), child_o_3->id(), 1.0, "" );
  graph._addNode( child_o_5 );
  graph._addEdge( child_o_5->id(), child_a_4->id(), 1.0, "" );
  graph._addEdge( child_a_2->id(), child_o_5->id(), 1.0, "" );

  graph._addNode( child_a_20 );
  graph._addEdge( child_a_20->id(), child_o_1->id(), 0.5, "" );
  graph._addNode( child_o_30 );
  graph._addEdge( child_o_30->id(), child_a_20->id(), 0.5, "" );
  graph._addNode( child_a_40 );
  graph._addEdge( child_a_40->id(), child_o_30->id(), 0.5, "" );

  graph.saveGraphToFile( "OnUnsolvedBranchValuesAreInfinity.gv" );

  std::vector< double > rewards =  { -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0 };

  /////
  auto values = ValueIterationAlgorithm::process( graph, rewards );
  /////

  EXPECT_LE( values[0], -100.0 );

  EXPECT_LE( values[1], -100.0 );
  EXPECT_LE( values[2], -100.0 );

  EXPECT_LE( values[3], -100.0 );
  EXPECT_LE( values[4], -100.0 );

  EXPECT_LE( values[5], -100.0 );
}

TEST(ValueIteration, OnSolvedAcyclicGraph) {
  LogicEngine engine;
  std::vector< std::string > startStates;
  std::vector< double > egoBeliefState;
  DecisionGraph graph( engine, startStates, egoBeliefState );
  auto root = graph.root();
  DecisionGraph::GraphNodeDataType data_observation( {""}, {1.0}, false, 0, NodeData::NodeType::OBSERVATION );
  DecisionGraph::GraphNodeDataType data_action( {""}, {1.0}, false, 0, NodeData::NodeType::ACTION );
  DecisionGraph::GraphNodeDataType terminal( {""}, {1.0}, true, 0, NodeData::NodeType::ACTION );

  auto child_o_1 = root->makeChild( DecisionGraph::GraphNodeDataType(data_observation) );
  auto child_a_2 = child_o_1->makeChild( DecisionGraph::GraphNodeDataType(data_action) );

  auto child_o_3 = child_a_2->makeChild( DecisionGraph::GraphNodeDataType(data_observation) );
  auto child_a_4 = child_o_3->makeChild( DecisionGraph::GraphNodeDataType(terminal) );

  graph._addNode( child_o_1 );
  graph._addEdge( child_o_1->id(), root->id(), 1.0, "" );

  graph._addNode( child_a_2 );
  graph._addEdge( child_a_2->id(), child_o_1->id(), 1.0, "" );

  graph._addNode( child_o_3 );
  graph._addEdge( child_o_3->id(), child_a_2->id(), 1.0, "" );

  graph._addNode( child_a_4 );
  graph._addEdge( child_a_4->id(), child_o_3->id(), 1.0, "" );

  graph.saveGraphToFile( "OnSolvedAcyclicGraph.gv" );

  std::vector< double > rewards =  { -1.0, -1.0, -1.0, -1.0, -1.0 };

  /////
  auto values = ValueIterationAlgorithm::process( graph, rewards );
  /////

  EXPECT_NEAR( values[0], -2.0, 0.01 );

  EXPECT_NEAR( values[1], -1.0, 0.01 );
  EXPECT_NEAR( values[2], -1.0, 0.01 );

  EXPECT_NEAR( values[3], 0.0, 0.01 );
  EXPECT_NEAR( values[4], 0.0, 0.01 );
}

TEST(ValueIteration, OnUnsolvedAcyclicGraphValueAreInfinity) {
  LogicEngine engine;
  std::vector< std::string > startStates;
  std::vector< double > egoBeliefState;
  DecisionGraph graph( engine, startStates, egoBeliefState );
  auto root = graph.root();
  DecisionGraph::GraphNodeDataType data_observation( {""}, {1.0}, false, 0, NodeData::NodeType::OBSERVATION );
  DecisionGraph::GraphNodeDataType data_action( {""}, {1.0}, false, 0, NodeData::NodeType::ACTION );
  DecisionGraph::GraphNodeDataType not_terminal( {""}, {1.0}, false, 0, NodeData::NodeType::ACTION );

  auto child_o_1 = root->makeChild( DecisionGraph::GraphNodeDataType(data_observation) );
  auto child_a_2 = child_o_1->makeChild( DecisionGraph::GraphNodeDataType(data_action) );

  auto child_o_3 = child_a_2->makeChild( DecisionGraph::GraphNodeDataType(data_observation) );
  auto child_a_4 = child_o_3->makeChild( DecisionGraph::GraphNodeDataType(not_terminal) );

  graph._addNode( child_o_1 );
  graph._addEdge( child_o_1->id(), root->id(), 1.0, "" );

  graph._addNode( child_a_2 );
  graph._addEdge( child_a_2->id(), child_o_1->id(), 1.0, "" );

  graph._addNode( child_o_3 );
  graph._addEdge( child_o_3->id(), child_a_2->id(), 1.0, "" );

  graph._addNode( child_a_4 );
  graph._addEdge( child_a_4->id(), child_o_3->id(), 1.0, "" );

  graph.saveGraphToFile( "OnUnsolvedAcyclicGraph.gv" );

  std::vector< double > rewards =  { -1.0, -1.0, -1.0, -1.0, -1.0 };

  /////
  auto values = ValueIterationAlgorithm::process( graph, rewards );
  /////

  EXPECT_LE( values[0], -1000.0 );

  EXPECT_LE( values[1], -1000.0 );
  EXPECT_LE( values[2], -1000.0 );

  EXPECT_LE( values[3], -1000.0 );
  EXPECT_LE( values[4], -1000.0 );
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
