#pragma once

#include <fstream>
#include <po_graph_node.h>

namespace tp
{

class GraphPrinter
{
public:
  GraphPrinter( std::ostream & ss )
    : ss_( ss )
  {

  }

  void print( const POGraphNode::ptr & node );

private:
  void saveGraphFrom( const POGraphNode::ptr & node );
  void saveEdge( const POGraphNode::ptr & a, const POGraphNode::ptr & b );

private:
  std::ostream & ss_;
  std::list< std::pair< POGraphNode::ptr, POGraphNode::ptr > > savedEdges_;
};

}
