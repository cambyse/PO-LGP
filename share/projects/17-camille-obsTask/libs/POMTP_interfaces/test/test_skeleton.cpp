#include <skeleton.h>

#include <gtest/gtest.h>

#include <fstream>

#include <boost/filesystem.hpp>

// GraphNode
TEST(Skeleton, SaveLoad) {
  Skeleton p;
  //
  const std::string filename( "policy" );
  const double value = 1.0;
  const Skeleton::StatusType status( Skeleton::INFORMED );
  //

  p.setValue( 1.0 );
  p.setStatus( status );
  p.save( filename );

  Skeleton pp;
  pp.load( filename );

  ASSERT_EQ( pp.value(), value );
  ASSERT_EQ( pp.status(), status );
}

TEST(Skeleton, Equality) {
  // root
  SkeletonNodeData rootData;
  Skeleton::GraphNodeTypePtr root = Skeleton::GraphNodeType::root( rootData );
  Skeleton p( root );
  //
  const std::string filename( "policy" );
  const double value = 1.0;
  const Skeleton::StatusType status( Skeleton::INFORMED );
  //

  p.setValue( 1.0 );
  p.setStatus( status );
  p.save( filename );

  Skeleton pp;
  pp.load( filename );

  ASSERT_EQ( pp, p );
}

TEST(Skeleton, NonEquality) {
  // root
  SkeletonNodeData rootData;
  Skeleton::GraphNodeTypePtr root = Skeleton::GraphNodeType::root( rootData );
  Skeleton p( root );
  //
  const std::string filename( "policy" );
  const double value = 1.0;
  const Skeleton::StatusType status( Skeleton::INFORMED );
  //

  p.setValue( 1.0 );
  p.setStatus( status );
  p.save( filename );

  Skeleton pp;

  ASSERT_NE( pp, p );
}

TEST(Skeleton, HashingEquality) {
  // root
  SkeletonNodeData rootData;
  Skeleton::GraphNodeTypePtr root = Skeleton::GraphNodeType::root( rootData );
  Skeleton p( root );
  //
  const std::string filename( "policy" );
  const double value = 1.0;
  const Skeleton::StatusType status( Skeleton::INFORMED );
  //

  p.setValue( 1.0 );
  p.setStatus( status );
  p.save( filename );

  Skeleton pp;
  pp.load( filename );

  ASSERT_EQ( pp.hash(), p.hash() );
}

TEST(Skeleton, HashingInequality) {
  // root
  SkeletonNodeData rootData;
  Skeleton::GraphNodeTypePtr root = Skeleton::GraphNodeType::root( rootData );
  Skeleton p( root );
  //
  const std::string filename( "policy" );
  const double value = 1.0;
  const Skeleton::StatusType status( Skeleton::INFORMED );
  //

  p.setValue( 1.0 );
  p.setStatus( status );
  p.save( filename );

  Skeleton pp;

  ASSERT_NE( pp.hash(), p.hash() );
}

// GraphNode
TEST(Skeleton, SaveToGraph) {

  const std::string filename( "policy_graph.gv" );

  // root
  SkeletonNodeData rootData;
  //rootData.beliefState = { 1.0 };
  Skeleton::GraphNodeTypePtr root = Skeleton::GraphNodeType::root( rootData );

  // child 1
  SkeletonNodeData childData1;
  //childData.beliefState = { 1.0 };
  childData1.leadingKomoArgs = { "komoAction", "X", "Y", "Z" };
  root->makeChild( childData1 );

  // child 1
  SkeletonNodeData childData2;
  //childData.beliefState = { 1.0 };
  childData2.leadingKomoArgs = { "komoAction", "A", "B", "C" };
  root->makeChild( childData2 );

  // policy
  Skeleton p( root );
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
