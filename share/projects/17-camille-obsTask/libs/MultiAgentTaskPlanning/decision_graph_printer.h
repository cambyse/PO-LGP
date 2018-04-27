#pragma once

#include <fstream>

#include <decision_graph.h>

namespace matp
{
class GraphPrinter
{
public:
  GraphPrinter( std::ostream & ss )
    : ss_( ss )
  {

  }

  void print( const DecisionGraph & graph );

private:
  void saveGraphFrom( const DecisionGraph::GraphNodeType::ptr & node );

private:
  std::ostream & ss_;
};

}