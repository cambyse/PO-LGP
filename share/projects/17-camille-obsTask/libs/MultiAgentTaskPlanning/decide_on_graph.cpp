#include <graph_planner.h>

#include <algorithm>    // std::random_shuffle
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

static double m_inf() { return std::numeric_limits< double >::lowest(); }

namespace matp
{

DecisionGraph DecideOnGraphAlgorithm::process( const DecisionGraph & graph, std::vector< double > & values, std::vector< double > & rewards )
{
  std::cout << "GraphPlanner::decideOnDecisionGraphCopy.." << std::endl;

  using NodeTypePtr = std::shared_ptr< DecisionGraph::GraphNodeType >;

  std::vector< bool > toKeep( graph.size(), false );
  std::vector< bool > decided( graph.size(), false );
  std::list< std::weak_ptr< DecisionGraph::GraphNodeType > > toRemove;
  toKeep[0] = true; // always keep root

  auto decidedGraph = graph; // copy

  std::queue< NodeTypePtr > Q;

  Q.push( decidedGraph.root() );

  while( ! Q.empty() )
  {
    auto u = Q.front();
    Q.pop();

    decided[ u->id() ] = true;

    // EGO DECISION : we prune the actions that are sub-optimal
    if( u->data().agentId == 0 )
    {
      if( u->data().nodeType == NodeData::NodeType::ACTION )
      {
        double bestValue = m_inf();
        uint bestId = -1;
        for( auto v : u->children() )
        {
          if( rewards[ v->id() ] + values[ v->id() ] >= bestValue )
          {
            bestValue = rewards[ v->id() ] + values[ v->id() ];
            bestId = v->id();
            //std::cout << "best child of " << u->id() << " is " << v->id() << std::endl;
          }
        }

        for( auto v : u->children() )
        {
          if( v->id() != bestId )
          {
            toKeep[ v->id() ] = false;
            toRemove.push_back( v );
            //std::cout << "remove " << v->id() << std::endl;
          }
          else
          {
            // keep this node and all its observation counterparts
            toKeep[ v->id() ] = true;
            for( auto w : v->children() )
            {
              toKeep[ w->id() ] = true;
            }

            for( auto w : v->children() )
            {
              Q.push( w );
            }
          }
        }
      }
    }
  }

  decidedGraph.purgeNodes( toKeep );

  // remove nodes that don't have to be kept
  for( auto _n : toRemove )
  {
    if( _n.lock() )
    {
      auto n = _n.lock();

      for( auto _p : n->parents() )
      {
        auto p = _p.lock();
        if( p )
        {
          p->removeChild( n );
        }
      }
    }
  }

  std::cout << "decideOnDecisionGraphCopy.. end" << std::endl;

  return decidedGraph;
}

}
