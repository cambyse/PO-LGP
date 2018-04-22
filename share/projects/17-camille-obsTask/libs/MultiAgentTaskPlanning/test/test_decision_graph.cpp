#include <decision_graph.h>
#include <logic_parser.h>

#include <gtest/gtest.h>

using namespace matp;

// DecisionGraph
TEST(DecisionGraph, DefaultIsEmpty) {
  DecisionGraph graph;
  ASSERT_EQ( graph.empty(), true );
}

TEST(DecisionGraph, getCommonPossibleActions1W) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-1w.g" );
  DecisionGraph graph( p.engine(), p.possibleStartStates(), p.egoBeliefState() );
  ASSERT_EQ( graph.getCommonPossibleActions( graph.root(), 0 ).size(), p.engine().getPossibleActions( 0 ).size() );
}

TEST(DecisionGraph, getCommonPossibleActions2W) {
  LogicParser p;
  p.parse( "data/LGP-overtaking-single-agent-1w.g" );
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
