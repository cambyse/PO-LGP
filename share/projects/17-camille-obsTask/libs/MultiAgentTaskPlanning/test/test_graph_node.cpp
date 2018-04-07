#include <graph_node.h>

#include <gtest/gtest.h>

using namespace matp;

// GraphNode
TEST(GraphNode, RootNode) {
  auto root = GraphNode< double >::root( 0.0 );
  ASSERT_EQ( root->isRoot(), true );
}

TEST(GraphNode, ChildNodeNotRoot) {
  auto root = GraphNode< double >::root( 0.0 );
  auto child = root->makeChild( 1.0 );
  ASSERT_EQ( ! child->isRoot(), true );
}

TEST(GraphNode, ChildNodeRegisteredInParent) {
  auto root = GraphNode< double >::root( 0.0 );
  auto child = root->makeChild( 1.0 );
  ASSERT_EQ( root->children().size(), 1 );
}

TEST(GraphNode, ChildOfChild) {
  auto root = GraphNode< double >::root( 0.0 );
  auto child = root->makeChild( 1.0 );
  auto childchild = child->makeChild( 1.0 );
  ASSERT_EQ( child->children().size(), 1 );
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
