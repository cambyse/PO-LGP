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

  //graph_.saveGraphToFile( "graph.gv" );

  if( rewards_.empty() )
  {
    initializeRewards();
  }

  valueIteration();

  decideOnDecisionGraphCopy();

  //decidedGraph_.saveGraphToFile( "decided.gv" );

  buildPolicy();
}

void GraphPlanner::integrate( const Policy & policy )
{
  std::queue< Policy::GraphNodeTypePtr > Q;
  Q.push( policy.root() );

  auto decisionGraphNodes = graph_.nodes();

  while( ! Q.empty() )
  {
    auto n = Q.front();
    Q.pop();

    if( n->id() == 0 )
    {
      CHECK_EQ( n->children().size(), 1, "wrong Policy" );

      for( auto c : n->children() )
      {
        //std::cout << "integrate from " << n->data().decisionGraphNodeId << " to " << c->data().decisionGraphNodeId << " = " << c->data().markovianReturn << std::endl;

        rewards_[ fromToIndex( n->data().decisionGraphNodeId, c->data().decisionGraphNodeId ) ] = c->data().markovianReturn;
        Q.push( c );
      }
    }
    else
    {
      // we have to skip the observation node
      for( auto c : n->children() )
      {
        // find correct parent in decision graph
        // 1 - get similar nod in graph
        auto c_g = decisionGraphNodes[ c->data().decisionGraphNodeId ];

        auto id_right_parent = -1;
        for( auto p_g : c_g.lock()->parents() )
        {
          for( auto p_p_g : p_g.lock()->parents() )
          {
            if( p_p_g.lock()->id() == n->data().decisionGraphNodeId )
            {
              id_right_parent = p_g.lock()->id();

              //std::cout << "integrate from " << id_right_parent << " to " << c->data().decisionGraphNodeId << " = " << c->data().markovianReturn << std::endl;

              rewards_[ fromToIndex( id_right_parent, c->data().decisionGraphNodeId ) ] = c->data().markovianReturn;

              Q.push( c );

              break;
            }
          }
        }
      }
    }

//    auto nn = decisionGraphNodes[ n->id() ];

//    for( auto nnn : nn.lock()->children() ) // observation
//    {
//    for( auto cc : nnn->children() )
//    {
//    for( auto c : n->children() )
//    {
//      std::cout << "integrate from " << n->data().decisionGraphNodeId << " to " << c->data().decisionGraphNodeId << " = " << c->data().markovianReturn << std::endl;

//      //rewards_[ fromToIndex( nnn->id(), c->data().decisionGraphNodeId ) ] = c->data().markovianReturn;
//      Q.push( c );
//    }
//    }
//    }
  }

  // go through
}

// getters
bool GraphPlanner::terminated() const
{
  return false;
}

Policy GraphPlanner::getPolicy() const
{
  return policy_;
}

double GraphPlanner::reward( uint from, uint to ) const
{
  return rewards_[ fromToIndex( from, to ) ];
}

void GraphPlanner::buildGraph( bool graph )
{
  std::cout << "GraphPlanner::buildGraph.." << std::endl;
  if( ! parser_.successfullyParsed() )
  {
    return;
  }

  graph_.build( maxDepth_, graph );

  std::cout << "GraphPlanner::buildGraph.. end, size of graph:" << graph_.size() << std::endl;
}

void GraphPlanner::initializeRewards()
{
  rewards_ = std::vector< double >( graph_.nodes().size() * graph_.nodes().size(), r0_ );

  for( const auto & n : graph_.nodes() )
  {
    for( const auto & m : graph_.nodes() )
    {
      rewards_[ fromToIndex( n.lock()->id(), m.lock()->id() ) ] = r0_;
    }
  }
}

PolicyNodeData GraphPlanner::decisionGraphtoPolicyData( const NodeData & dData, uint id ) const
{
  PolicyNodeData pData;

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

void GraphPlanner::buildPolicy()
{
  std::cout << "GraphPlanner::buildPolicy.." << std::endl;

  using NodeTypePtr = std::shared_ptr< DecisionGraph::GraphNodeType >;

  std::queue< std::pair< NodeTypePtr, Policy::GraphNodeTypePtr > > Q;

  // create policy root node from decision graph node
  auto root = decidedGraph_.root();
  PolicyNodeData rootData;
  rootData.beliefState = root->data().beliefState;

  auto policyRoot = GraphNode< PolicyNodeData >::root( rootData );

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
      PolicyNodeData data = decisionGraphtoPolicyData( v->data(), v->id() );
      data.p = edge.first;
      data.leadingKomoArgs = decisionArtifactToKomoArgs( edge.second );
      data.markovianReturn = rewards_[ fromToIndex( u->id(), v->id() ) ];

      auto vSke = uSke->makeChild( data );

      //std::cout << "build ske from " << uSke->id() << "(" << u->id() << ") to " << vSke->id()<< " (" << v->id() << ") = " << data.markovianReturn << std::endl;

      for( auto w : v->children() ) // skip obs nodes
      {
        Q.push( std::make_pair( w, vSke ) );
      }
    }
  }

  policy_ = Policy( policyRoot );
  policy_.setValue( values_[ decisionGraph().root()->id() ] );

  std::cout << "GraphPlanner::buildPolicy.. end (value=" << policy_.value() << ")" << std::endl;
}

uint GraphPlanner::fromToIndex( uint from, uint to ) const
{
  return from * graph_.size() + to;
}

}
