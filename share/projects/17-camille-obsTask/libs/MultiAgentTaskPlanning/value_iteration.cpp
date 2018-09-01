#include <graph_planner.h>

#include <algorithm>    // std::random_shuffle
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

static double m_inf() { return std::numeric_limits< double >::lowest(); }

namespace matp
{

std::vector< double > ValueIterationAlgorithm::process( const DecisionGraph & graph, std::vector< double > & rewards )
{
  std::cout << "GraphPlanner::valueIteration.. start" << std::endl;

  using NodeTypePtr = std::shared_ptr< DecisionGraph::GraphNodeType >;

  double alpha = 0.5;
  const double initValue = -10;

  std::vector< double > values = std::vector< double >( graph.size(), initValue ); // magic value!! distance from root to vertex[i]

  auto comp = [ & ]( const NodeTypePtr & a, const NodeTypePtr & b ) -> bool
  {
    return values[ a->id() ] < values[ b->id() ];
  };

  auto diff_func = []( double a, double b ) -> double
  {
    const double gentle_m_inf = -10e8;
    if( a < gentle_m_inf && b < gentle_m_inf )
    {
      return 0;
    }
    else if( a >= gentle_m_inf && b >= gentle_m_inf )
    {
      return fabs( a -b );
    }
    else
    {
      return std::numeric_limits< double >::infinity();
    }
  };

  // go from leafs to root
  const auto nodes = graph.nodes();
  const auto terminals = graph.terminalNodes();
  auto edges = graph.edges();

  for( auto weakV : terminals )
  {
    auto v = weakV.lock();

    values[ v->id() ] = 0; // all rewards negative
  }

  // expected reward up to terminal nodes
  // add terminal nodes to Q
  uint totalUpdates = 0;
  constexpr double eps = 10e-4;
  bool stable = false;
  for( auto i = 0; ! stable && i < 1000; ++i )
  {
    double maxDiff = 0;

    auto itNodes = nodes;
    //std::random_shuffle ( itNodes.begin(), itNodes.end() );

    for( auto weakU : itNodes )
    {
      auto u = weakU.lock();

      if( u->id() == 3 )
      {
        int a = 0;
      }

      if( u->data().nodeType == NodeData::NodeType::ACTION )
      {
        if( u->data().agentId == 0 )
        {
          if( ! u->data().terminal )
          {
            double newTargetValue = m_inf(); // if no children and not terminal, it means that it is infeasible hence m_inf

            // max operation, choose the best child
            for( auto v : u->children() )
            {
              const auto r = rewards[ v->id() ];

              if( values[ v->id() ] + r > newTargetValue )
              {
                newTargetValue = values[ v->id() ] + r;
              }
            }

            if( newTargetValue == m_inf() ) // there were no children so unfeasible
            {
              const auto diff = diff_func( values[ u->id() ], m_inf() );
              maxDiff = std::max( maxDiff, diff );

              //std::cout << "A update " << u->id() << " old value:" << values_[ u->id() ] << " new value:" << m_inf() << std::endl;
              //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

              values[ u->id() ] = m_inf();
            }
            else
            {
              const auto newValue = values[ u->id() ] * ( 1 - alpha ) + alpha * newTargetValue;
              const auto diff = diff_func( values[ u->id() ], newValue );
              maxDiff = std::max( maxDiff, diff );

              //std::cout << "B update " << u->id() << " old value:" << values[ u->id() ] << " new value:" << newValue << std::endl;
              //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

              values[ u->id() ] = newValue;
            }
          }
        }
        else // other agent
        {
          double newTargetValue = 0;

          uint n = u->children().size();
          for( auto v : u->children() )
          {
            newTargetValue += 1.0 / n * values[ v->id() ] ; // average
          }

          const auto newValue = values[ u->id() ] * ( 1 - alpha ) + alpha * newTargetValue;
          const auto diff = diff_func( values[ u->id() ], newValue );
          maxDiff = std::max( maxDiff, diff );

          //std::cout << "C update " << u->id() << " old value:" << values_[ u->id() ] << " new value:" << newValue << std::endl;
          //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

          values[ u->id() ] = newValue;
        }
      }
      else if( u->data().nodeType == NodeData::NodeType::OBSERVATION )
      {
        double newTargetValue = 0;
        for( auto v : u->children() )
        {
          CHECK_EQ( edges[ v->id() ].count( u->id() ), 1, "corruption in edge data structure" );
          const double p = edges[ v->id() ][ u->id() ].first;
          newTargetValue += p * values[ v->id() ] ;
        }

        const auto newValue = values[ u->id() ] * ( 1 - alpha ) + alpha * newTargetValue;
        const auto diff = diff_func( values[ u->id() ], newValue );
        maxDiff = std::max( maxDiff, diff );

        //std::cout << "D update " << u->id() << " old value:" << values[ u->id() ] << " new value:" << newValue << std::endl;
        //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

        values[ u->id() ] = newValue;
      }
    }

    stable = maxDiff < eps;

    std::cout << "it: " << i << " maxDiff: " << maxDiff << std::endl;
  }

  std::cout << "GraphPlanner::valueIteration.. end" << std::endl;

  return values;
}

}
