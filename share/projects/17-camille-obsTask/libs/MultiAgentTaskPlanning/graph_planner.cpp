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
  if( rewards_.empty() )
  {
    buildGraph();

    initializeRewards();
  }

  valueIteration();

  decideOnDecisionGraphCopy();

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

    rewards_[ n->id() ] = n->data().markovianReturn;

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
  auto it = rewards_.find( nodeId );

  if( it != rewards_.end() )
  {
    return it->second;
  }

  return m_inf();
}

void GraphPlanner::buildGraph()
{
  if( ! parser_.successfullyParsed() )
  {
    return;
  }

  graph_.build( maxDepth_ );
}

void GraphPlanner::initializeRewards()
{
  for( const auto & n : graph_.nodes() )
  {
    rewards_[ n.lock()->id() ] = r0_;
  }
}

SkeletonNodeData GraphPlanner::decisionGraphtoPolicyData( const NodeData & dData ) const
{
  SkeletonNodeData pData;

  pData.beliefState = dData.beliefState;
  pData.markovianReturn = r0_;
  pData.leadingKomoArgs = decisionArtifactToKomoArgs( dData.leadingArtifact );
  pData.p           = dData.p;

  return pData;
}

void GraphPlanner::valueIteration()
{
  std::cout << "GraphPlanner::valueIteration.. start" << std::endl;

  using NodeTypePtr = std::shared_ptr< DecisionGraph::GraphNodeType >;

  double alpha = 0.5;
  constexpr double initValue = -10.0;

  values_ = std::vector< double >( graph_.size(), initValue ); // magic value!! distance from root to vertex[i]

  auto comp = [ & ]( const NodeTypePtr & a, const NodeTypePtr & b ) -> bool
  {
    return values_[ a->id() ] < values_[ b->id() ];
  };

  // go from leafs to root
  auto nodes = graph_.nodes();
  auto terminals = graph_.terminalNodes();
  for( auto weakV : terminals )
  {
    auto v = weakV.lock();

    values_[ v->id() ] = 0; // all rewards negative
  }

  // expected reward up to terminal nodes
  // add terminal nodes to Q
  uint totalUpdates = 0;
  constexpr double eps = 0.01;
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
            double newValue = m_inf(); // if no children and not terminal, it means that it is infeasible hence m_inf

            // max operation, choose the best child
            for( auto v : u->children() )
            {
              if( values_[ v->id() ] + r0_ > newValue )
              {
                auto r = rewards_[ v->id() ];
                newValue = values_[ v->id() ] + r;
              }
            }

            if( newValue == m_inf() )
            {
              values_[ u->id() ] = newValue;
            }
            else
            {
              values_[ u->id() ] = values_[ u->id() ] * ( 1 - alpha ) + alpha * newValue;
            }

            maxDiff = std::max( maxDiff, std::abs( values_[ u->id() ] - newValue ) );
          }
        }
        else // other agent
        {
          double newValue = 0;

          uint n = u->children().size();
          for( auto v : u->children() )
          {
            newValue += 1.0 / n * values_[ v->id() ] ; // average
          }

          maxDiff = std::max( maxDiff, std::abs( values_[ u->id() ] - newValue ) );
          values_[ u->id() ] = values_[ u->id() ] * ( 1 - alpha ) + alpha * newValue;
        }
      }
      else if( u->data().nodeType == NodeData::NodeType::OBSERVATION )
      {
        double newValue = 0;

        for( auto v : u->children() )
        {
          newValue += v->data().p * values_[ v->id() ] ;
        }

        maxDiff = std::max( maxDiff, std::abs( values_[ u->id() ] - newValue ) );
        values_[ u->id() ] = values_[ u->id() ] * ( 1 - alpha ) + alpha * newValue;
      }
    }

    stable = maxDiff < eps;
  }

  std::cout << "GraphPlanner::valueIteration.. end" << std::endl;
}

void GraphPlanner::decideOnDecisionGraphCopy()
{
  using NodeTypePtr = std::shared_ptr< DecisionGraph::GraphNodeType >;

  decidedGraph_ = graph_; // copy

  std::queue< NodeTypePtr > Q;

  Q.push( decidedGraph_.root() );

  while( ! Q.empty() )
  {
    auto u = Q.front();
    Q.pop();

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
      Q.push( v );
    }
  }
}

void GraphPlanner::buildSkeleton()
{
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
    auto uCopy = uPair.second;

    for( auto v : u->children() )
    {
      SkeletonNodeData data = decisionGraphtoPolicyData( v->data() );

      auto vCopy = uCopy->makeChild( data );

      for( auto w : v->children() ) // skip obs nodes
      {
        Q.push( std::make_pair( w, vCopy ) );
      }
    }
  }

  skeleton_ = Skeleton( policyRoot );
  skeleton_.setValue( values_[ decisionGraph().root()->id() ] );
}

}
