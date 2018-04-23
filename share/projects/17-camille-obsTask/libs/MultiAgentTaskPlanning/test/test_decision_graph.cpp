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
  graph.expand( graph.root() );
  ASSERT_EQ( graph.size(), 6 );
}
//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
