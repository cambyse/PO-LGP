#include <new_policy.h>

#include <gtest/gtest.h>

#include <fstream>

#include <boost/filesystem.hpp>

// GraphNode
TEST(NewPolicy, Save) {
  NewPolicy p;
  //
  const std::string filename( "policy" );
  const double value = 1.0;
  const NewPolicy::StatusType status( NewPolicy::INFORMED );
  //

  p.setValue( 1.0 );
  p.setStatus( status );
  p.save( filename );

  NewPolicy pp;
  pp.load( filename );

  ASSERT_EQ( pp.value(), value );
  ASSERT_EQ( pp.status(), status );
}

// GraphNode
TEST(NewPolicy, SaveToGraph) {

  const std::string filename( "policy_graph.gv" );

  // root
  NewPolicyNodeData rootData;
  //rootData.beliefState = { 1.0 };
  NewPolicy::GraphNodeTypePtr root = NewPolicy::GraphNodeType::root( rootData );

  // child 1
  NewPolicyNodeData childData1;
  //childData.beliefState = { 1.0 };
  childData1.leadingKomoArgs = { "komoAction", "X", "Y", "Z" };
  root->makeChild( childData1 );

  // child 1
  NewPolicyNodeData childData2;
  //childData.beliefState = { 1.0 };
  childData2.leadingKomoArgs = { "komoAction", "A", "B", "C" };
  root->makeChild( childData2 );

  // policy
  NewPolicy p( root );
  p.setValue( 1.0 );

  p.saveToGraphFile( "policy_graph.gv" );

  ASSERT_TRUE( boost::filesystem::exists( filename ) );
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
