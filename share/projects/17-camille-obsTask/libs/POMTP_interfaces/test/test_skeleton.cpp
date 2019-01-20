#include <skeleton.h>

#include <gtest/gtest.h>

#include <fstream>

#include <boost/filesystem.hpp>

// GraphNode
TEST(QResult, DefaultConstruction) {
  QResult result;

  ASSERT_EQ( result.tau(), 0.0 );
  ASSERT_EQ( result.nWorlds(), 0 );
}

TEST(QResult, Construction) {
  QResult result(2, 0.1);

  ASSERT_EQ( result.tau(), 0.1 );
  ASSERT_EQ( result.nWorlds(), 2 );
}

TEST(QResult, CopyConstruction) {
  QResult result(2, 0.1);
  QResult result2(result);

  ASSERT_EQ( result2.tau(), 0.1 );
  ASSERT_EQ( result2.nWorlds(), 2 );
}

TEST(QResult, Copy) {
  QResult result(2, 0.1);
  QResult result2(3, 0.1);

  result2 = result;

  ASSERT_EQ( result2.tau(), 0.1 );
  ASSERT_EQ( result2.nWorlds(), 2 );
}

TEST(QResult, setGetTrajectory) {
  QResult result(2, 0.1);

  result.createTrajectory(0, 2);
  result.createTrajectory(1, 1);

  result.setQ(0, 0, {0.0, 0.0});
  result.setQ(0, 1, {0.1, 0.1});

  result.setQ(1, 0, {1.0, 1.0});

  ASSERT_EQ( result.q(0, 0), std::vector< double >({0.0, 0.0}) );
  ASSERT_EQ( result.q(0, 1), std::vector< double >({0.1, 0.1}) );
  ASSERT_EQ( result.nSteps(0), 2 );

  ASSERT_EQ( result.q(1, 0), std::vector< double >({1.0, 1.0}) );
  ASSERT_EQ( result.nSteps(1), 1 );
}

// GraphNode
TEST(Skeleton, SaveLoad) {
  Skeleton p;
  //
  const std::string filename( "policy" );
  const double value = 1.0;
  const Skeleton::StatusType status( Skeleton::INFORMED );
  QResult qr(2, 0.1);
  qr.createTrajectory(0, 2);
  qr.createTrajectory(1, 1);

  qr.setQ(0, 0, {0.0, 0.0});
  qr.setQ(0, 1, {0.1, 0.1});
  //

  p.setValue( 1.0 );
  p.setQResult( qr );
  p.setStatus( status );
  p.save( filename );

  Skeleton pp;
  pp.load( filename );

  ASSERT_EQ( pp.value(), value );
  ASSERT_EQ( pp.qresult(), qr );
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
