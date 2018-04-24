#include <decision_graph.h>
#include <logic_parser.h>

#include <gtest/gtest.h>

using namespace matp;

// DecisionGraph
TEST(DecisionGraph, DefaultIsEmpty) {
  DecisionGraph graph;
  ASSERT_EQ( graph.empty(), true );
}

// Actions
TEST(DecisionGraph, getCommonPossibleActions1W) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-1w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  ASSERT_EQ( graph.getCommonPossibleActions( graph.root(), 0 ).size(), p.engine().getPossibleActions( 0 ).size() );
}

TEST(DecisionGraph, getCommonPossibleActions2W) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" ); // one action not possible because we don't know if the lane is free or not
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  ASSERT_EQ( graph.getCommonPossibleActions( graph.root(), 0 ).size(), 2 );
}

TEST(DecisionGraph, NoCommonActions) {
  LogicParser p;
  p.parse( "data/LGP-single-agent-2w-no-common-actions.g" ); // one action not possible because we don't know if the lane is free or not
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );

  ASSERT_EQ( graph.getCommonPossibleActions( graph.root(), 0 ).size(), 0 );
}

// Apply states PO
TEST(DecisionGraph, applyPOStates) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  auto e = p.engine();
  auto startState = graph.root()->data().states[ 1 ];
  ASSERT_NE( startState, e.getState() );
  e.setState( startState );
  ASSERT_EQ( startState, e.getState() );
}

// Outcomes
TEST(DecisionGraph, getPossibleOutcomes1W) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-1w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  auto actions = graph.getCommonPossibleActions( graph.root(), 0 );
  ASSERT_EQ( graph.getPossibleOutcomes( graph.root(), actions[ 1 ] ).size(), 1 );
}

TEST(DecisionGraph, getPossibleOutcomes2W) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  auto actions = graph.getCommonPossibleActions( graph.root(), 0 );
  ASSERT_EQ( graph.getPossibleOutcomes( graph.root(), actions[ 0 ] ).size(), 2 );
}

// Expansion
TEST(DecisionGraph, expandFromRoot2W) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  auto actions = graph.getCommonPossibleActions( graph.root(), 0 );
  graph.expand( graph.root() );
  auto rootChildren = graph.root()->children();
  ASSERT_EQ( graph.size(), 6 );
  ASSERT_EQ( rootChildren.size(), actions.size() );
}

TEST(DecisionGraph, expandFromRootDouble2W) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  auto actions = graph.getCommonPossibleActions( graph.root(), 0 );
  graph.expand( graph.root() );
  auto rootChildren = graph.root()->children();
  ASSERT_EQ( graph.size(), 24 );
  ASSERT_EQ( rootChildren.size(), actions.size() );
}

TEST(DecisionGraph, expandFromRootDouble2WFringeSize) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  auto queue = graph.expand( graph.root() );
  ASSERT_EQ( queue.size(), 9 );
}

// Agent Id
TEST(DecisionGraph, agentId) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.expand( graph.root() );
  auto observationNodes = graph.root()->children();
  auto actionNodes = graph.root()->children().back()->children();

  ASSERT_EQ( observationNodes.back()->data().agentId, 0 );
  ASSERT_EQ( actionNodes.back()->data().agentId, 1 );
}

// Node types
TEST(DecisionGraph, expandedNodesTypes) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.expand( graph.root() );
  for( auto child : graph.root()->children() )
  {
    ASSERT_EQ( child->data().nodeType, NodeData::NodeType::OBSERVATION );

    for( auto childChild : child->children() )
    {
      ASSERT_EQ( childChild->data().nodeType, NodeData::NodeType::ACTION );
    }
  }
}

// Artifacts
TEST(DecisionGraph, expandedLeadingAction) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.expand( graph.root() );

  auto leadingAction = graph.root()->children().front()->data().leadingArtifact;
  ASSERT_NE( leadingAction.find( "look lanes" ), std::string::npos );
}

TEST(DecisionGraph, expandedLeadingObservation0) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.expand( graph.root() );
  auto childChild = graph.root()->children().back()->children().back();

  auto leadingObservation = childChild->data().leadingArtifact;
  ASSERT_EQ( leadingObservation, "" );
}

TEST(DecisionGraph, expandedLeadingObservation1) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.expand( graph.root() );
  auto childChild = graph.root()->children().front()->children().front();

  auto leadingObservation = childChild->data().leadingArtifact;
  ASSERT_NE( leadingObservation.find( "free lane_2" ), std::string::npos );
}

TEST(DecisionGraph, expandedLeadingObservation2) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.expand( graph.root() );
  auto childChild = graph.root()->children().front()->children().back();

  auto leadingObservation = childChild->data().leadingArtifact;
  ASSERT_EQ( leadingObservation, "" );
}

// Build
TEST(DecisionGraph, buildFromRootDouble2WD3) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.build(1);
  ASSERT_EQ( graph.size(), 24 );
}

TEST(DecisionGraph, buildCheckNodeDepth) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.build(1);
  for( auto node : graph.nodes() )
  {
    ASSERT_LE( node->depth(), 4 );
  }
}


// Terminal label
TEST(DecisionGraph, terminalNodes) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.build(2);
  auto leafs = graph.terminalNodes();

  bool found = false;
  int agentId = -1;
  for( auto l : leafs )
  {
    if( l->id() == 25 )
    {
      found = true;
      agentId = l->data().agentId;
    }
  }

  ASSERT_TRUE( found );
  ASSERT_EQ( agentId, 1 );// terminal a last ego action
}

TEST(DecisionGraph, terminalNodesHaveNoChildren) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.build(2);
  auto leafs = graph.terminalNodes();

  for( auto l : leafs )
  {
    ASSERT_EQ( l->children().size(), 0 );
  }
}

// Print graph
TEST(DecisionGraph, buildGraphAndPrintD1) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.build(1);
  graph.saveGraphToFile( "buildGraphAndPrintD1.gv" );
}

TEST(DecisionGraph, buildGraphAndPrintD2) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-double-agent-2w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  graph.build(2);
  graph.saveGraphToFile( "buildGraphAndPrintD2.gv" );
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
