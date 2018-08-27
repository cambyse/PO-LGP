#include <graph_planner.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

double m_inf() { return std::numeric_limits< double >::lowest(); }

namespace matp
{
std::vector< std::string > decisionArtifactToKomoArgs( const std::string & _artifact )
{
  std::vector< std::string > args;

  auto artifact = _artifact;

  boost::replace_all(artifact, "(", "");
  boost::replace_all(artifact, ")", "");

  boost::split( args, artifact, boost::is_any_of(" ") );

  return args;
}

// modifiers
void GraphPlanner::setFol( const std::string & descrition )
{
  if( ! boost::filesystem::exists( descrition ) ) throw FolFileNotFound();

  parser_.parse( descrition );
  graph_ = DecisionGraph( parser_.engine(), parser_.possibleStartStates(), parser_.egoBeliefState() );
}

void GraphPlanner::solve()
{
  if( graph_.empty() )
  {
    buildGraph();
  }

  if( rewards_.empty() )
  {
    initializeRewards();
  }

  valueIteration();

  decideOnDecisionGraphCopy();

  decidedGraph_.saveGraphToFile( "decided.gv" );

  buildSkeleton();
}

void GraphPlanner::integrate( const Skeleton & policy )
{
  std::queue< Skeleton::GraphNodeTypePtr > Q;
  Q.push( policy.root() );

  while( ! Q.empty() )
  {
    auto n = Q.front();
    Q.pop();

    rewards_[ n->data().decisionGraphNodeId ] = n->data().markovianReturn;

    for( auto c : n->children() )
    {
      Q.push( c );
    }
  }
}

// getters
bool GraphPlanner::terminated() const
{
  return false;
}

Skeleton GraphPlanner::getPolicy() const
{
  return skeleton_;
}

double GraphPlanner::reward( uint nodeId ) const
{
  return rewards_[ nodeId ];
}

void GraphPlanner::buildGraph( bool graph )
{
  std::cout << "GraphPlanner::buildGraph.." << std::endl;
  if( ! parser_.successfullyParsed() )
  {
    return;
  }

  graph_.build( maxDepth_, graph );

  std::cout << "GraphPlanner::buildGraph.. end" << std::endl;
}

void GraphPlanner::initializeRewards()
{
  rewards_ = std::vector< double >( graph_.nodes().size(), r0_ );

  for( const auto & n : graph_.nodes() )
  {
    rewards_[ n.lock()->id() ] = r0_;
  }
}

SkeletonNodeData GraphPlanner::decisionGraphtoPolicyData( const NodeData & dData, uint id ) const
{
  SkeletonNodeData pData;

  pData.beliefState = dData.beliefState;
  pData.markovianReturn = r0_;
  pData.leadingKomoArgs = decisionArtifactToKomoArgs( dData.leadingArtifact );
  pData.p           = dData.p;
  pData.decisionGraphNodeId = id;

  return pData;
}

void GraphPlanner::valueIteration()
{
  std::cout << "GraphPlanner::valueIteration.. start" << std::endl;

  using NodeTypePtr = std::shared_ptr< DecisionGraph::GraphNodeType >;

  double alpha = 0.5;
  const double initValue = maxDepth_ * r0_;

  values_ = std::vector< double >( graph_.size(), initValue ); // magic value!! distance from root to vertex[i]

  auto comp = [ & ]( const NodeTypePtr & a, const NodeTypePtr & b ) -> bool
  {
    return values_[ a->id() ] < values_[ b->id() ];
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
  const auto nodes = graph_.nodes();
  auto terminals = graph_.terminalNodes();
  for( auto weakV : terminals )
  {
    auto v = weakV.lock();

    values_[ v->id() ] = 0; // all rewards negative
  }

  // expected reward up to terminal nodes
  // add terminal nodes to Q
  uint totalUpdates = 0;
  constexpr double eps = 10e-4;
  bool stable = false;
  for( auto i = 0; ! stable && i < 1000; ++i )
  {
    double maxDiff = 0;

    for( auto weakU : nodes )
    {
      auto u = weakU.lock();

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
              const auto r = rewards_[ v->id() ];

              if( values_[ v->id() ] + r > newTargetValue )
              {
                newTargetValue = values_[ v->id() ] + r;
              }
            }

            if( newTargetValue == m_inf() ) // there were no children so unfeasible
            {
              const auto diff = diff_func( values_[ u->id() ], m_inf() );
              maxDiff = std::max( maxDiff, diff );

              //std::cout << "A update " << u->id() << " old value:" << values_[ u->id() ] << " new value:" << m_inf() << std::endl;
              //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

              values_[ u->id() ] = m_inf();
            }
            else
            {
              const auto newValue = values_[ u->id() ] * ( 1 - alpha ) + alpha * newTargetValue;
              const auto diff = diff_func( values_[ u->id() ], newValue );
              maxDiff = std::max( maxDiff, diff );

              //std::cout << "B update " << u->id() << " old value:" << values_[ u->id() ] << " new value:" << newValue << std::endl;
              //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

              values_[ u->id() ] = newValue;
            }
          }
        }
        else // other agent
        {
          double newTargetValue = 0;

          uint n = u->children().size();
          for( auto v : u->children() )
          {
            newTargetValue += 1.0 / n * values_[ v->id() ] ; // average
          }

          const auto newValue = values_[ u->id() ] * ( 1 - alpha ) + alpha * newTargetValue;
          const auto diff = diff_func( values_[ u->id() ], newValue );
          maxDiff = std::max( maxDiff, diff );

          //std::cout << "C update " << u->id() << " old value:" << values_[ u->id() ] << " new value:" << newValue << std::endl;
          //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

          values_[ u->id() ] = newValue;
        }
      }
      else if( u->data().nodeType == NodeData::NodeType::OBSERVATION )
      {
        double newTargetValue = 0;

        for( auto v : u->children() )
        {
          newTargetValue += v->data().p * values_[ v->id() ] ;
        }

        const auto newValue = values_[ u->id() ] * ( 1 - alpha ) + alpha * newTargetValue;
        const auto diff = diff_func( values_[ u->id() ], newValue );
        maxDiff = std::max( maxDiff, diff );

        //std::cout << "D update " << u->id() << " old value:" << values_[ u->id() ] << " new value:" << newValue << std::endl;
        //std::cout << "diff:" << diff << " maxDiff:" << maxDiff << std::endl;

        values_[ u->id() ] = newValue;
      }
    }

    stable = maxDiff < eps;

    std::cout << "it: " << i << " maxDiff: " << maxDiff << std::endl;
  }

  std::cout << "GraphPlanner::valueIteration.. end" << std::endl;
}

void GraphPlanner::decideOnDecisionGraphCopy()
{
  std::cout << "GraphPlanner::decideOnDecisionGraphCopy.." << std::endl;

  using NodeTypePtr = std::shared_ptr< DecisionGraph::GraphNodeType >;

  std::vector< bool > toKeep( graph_.size(), false );
  std::vector< bool > decided( graph_.size(), false );
  toKeep[0] = true; // always keep root

  decidedGraph_.reset();
  decidedGraph_ = graph_; // copy

  std::queue< NodeTypePtr > Q;

  Q.push( decidedGraph_.root() );

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
          if( values_[ v->id() ] >= bestValue )
          {
            bestValue = values_[ v->id() ];
            bestId = v->id();

            // keep this node and all its observation counterparts
            toKeep[v->id()] = true;
            for( auto w : v->children() )
            {
              toKeep[w->id()] = true;
            }
            //std::cout << "best child of " << u->id() << " is " << v->id() << std::endl;
          }
        }

        // remove other nodes
        while( u->children().size() > 1 )
        {
          for( auto v : u->children() )
          {
            if( v->id() != bestId )
            {
              u->removeChild( v );

              //std::cout << "remove " << v->id() << std::endl;
            }
          }
        }
      }
    }

    // push children on Q
    for( auto v : u->children() )
    {
      if( ! decided[v->id()] )
      {
        Q.push( v );
      }
    }
  }

  // remove nodes that don't have to be kept
  for( auto n : decidedGraph_.nodes() )
  {
    auto nn = n.lock();
    if( nn )
    {
      if( ! toKeep[ nn->id() ] )
      {
        for( auto p : nn->parents() )
        {
          auto pp = p.lock();
          if( pp )
          {
            pp->removeChild( nn );
          }
        }
      }
    }
  }

  std::cout << "GraphPlanner::decideOnDecisionGraphCopy.. end" << std::endl;
}

void GraphPlanner::buildSkeleton()
{
  std::cout << "GraphPlanner::buildSkeleton.." << std::endl;

  using NodeTypePtr = std::shared_ptr< DecisionGraph::GraphNodeType >;

  std::queue< std::pair< NodeTypePtr, Skeleton::GraphNodeTypePtr > > Q;

  // create policy root node from decision graph node
  auto root = decidedGraph_.root();
  SkeletonNodeData rootData;
  rootData.beliefState = root->data().beliefState;

  auto policyRoot = GraphNode< SkeletonNodeData >::root( rootData );

  Q.push( std::make_pair( decidedGraph_.root(), policyRoot ) );

  while( ! Q.empty() )
  {
    auto uPair = Q.front();
    Q.pop();

    auto u     = uPair.first;
    auto uSke = uPair.second;

    for( auto v : u->children() )
    {
      SkeletonNodeData data = decisionGraphtoPolicyData( v->data(), v->id() );

      auto vSke = uSke->makeChild( data );

      for( auto w : v->children() ) // skip obs nodes
      {
        Q.push( std::make_pair( w, vSke ) );
      }
    }
  }

  skeleton_ = Skeleton( policyRoot );
  skeleton_.setValue( values_[ decisionGraph().root()->id() ] );

  std::cout << "GraphPlanner::buildSkeleton.. end" << std::endl;
}

}
