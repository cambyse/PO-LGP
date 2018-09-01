#include <graph_planner.h>

#include <algorithm>    // std::random_shuffle
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

static double m_inf() { return std::numeric_limits< double >::lowest(); }

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

  //decidedGraph_.saveGraphToFile( "decided.gv" );

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
  pData.decisionGraphNodeId = id;

  return pData;
}

void GraphPlanner::valueIteration()
{
  values_ = ValueIterationAlgorithm::process( graph_, rewards_ );

}

void GraphPlanner::decideOnDecisionGraphCopy()
{
  decidedGraph_.reset();
  decidedGraph_ = DecideOnGraphAlgorithm::process( graph_, values_, rewards_ );
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

    //std::cout << "u->id()" << u->id() << " action:" << u->data().leadingArtifact << std::endl;

    for( auto v : u->children() )
    {
      auto edge = decidedGraph_.edges()[ v->id() ][ u->id() ];
      SkeletonNodeData data = decisionGraphtoPolicyData( v->data(), v->id() );
      data.p = edge.first;
      data.leadingKomoArgs = decisionArtifactToKomoArgs( edge.second );

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
