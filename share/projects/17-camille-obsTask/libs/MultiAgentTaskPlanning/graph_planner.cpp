#include <graph_planner.h>

#include <boost/filesystem.hpp>

double m_inf() { return std::numeric_limits< double >::lowest(); }

namespace matp
{
// modifiers
void GraphPlanner::setFol( const std::string & descrition )
{
  if( ! boost::filesystem::exists( descrition ) ) throw FolFileNotFound();

  parser_.parse( descrition );
  graph_ = DecisionGraph( parser_.engine(), parser_.possibleStartStates(), parser_.egoBeliefState() );
}

void GraphPlanner::solve()
{
  uint maxSteps = 3;

  buildGraph( maxSteps );

  valueIteration();

  decideOnDecisionGraphCopy();

  buildPolicy();
}

void GraphPlanner::integrate( const Policy::ptr & policy )
{

}

// getters
bool GraphPlanner::terminated() const
{
  return false;
}

Policy::ptr GraphPlanner::getPolicy() const
{
  return nullptr;
}

NewPolicy GraphPlanner::getNewPolicy() const
{
  return policy_;
}

MotionPlanningOrder GraphPlanner::getPlanningOrder() const
{
  return MotionPlanningOrder( 0 );
}

void GraphPlanner::buildGraph( int maxSteps )
{
  if( ! parser_.successfullyParsed() )
  {
    return;
  }

  graph_.build( maxSteps );
}

void GraphPlanner::valueIteration()
{
  using NodeTypePtr = std::shared_ptr< DecisionGraph::GraphNodeType >;

  double alpha = 0.5;

  values_ = std::vector< double >( graph_.size(), 0 ); // distance from root to vertex[i]


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
  bool stable = false;
  for( auto i = 0; ! stable && i < 1000; ++i )
  {
    for( auto weakU : nodes )
    {
      auto u = weakU.lock();

      if( u->data().nodeType == NodeData::NodeType::ACTION )
      {
        if( u->data().agentId == 0 )
        {
          if( ! u->data().terminal )
          {
            double newValue = m_inf();

            // max operation, choose the best child
            for( auto v : u->children() )
            {
              if( values_[ v->id() ] - 1 > newValue )
              {
                newValue = values_[ v->id() ] - 1;
              }
            }

            values_[ u->id() ] = values_[ u->id() ] * ( 1 - alpha ) + alpha * newValue;
          }
        }
        else
        {
          double newValue = 0;

          uint n = u->children().size();
          for( auto v : u->children() )
          {
            newValue += 1.0 / n * values_[ v->id() ] ;
          }

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

        values_[ u->id() ] = values_[ u->id() ] * ( 1 - alpha ) + alpha * newValue;
      }
    }
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

  decidedGraph_.saveGraphToFile( "lastSolution.gv" );
}

void GraphPlanner::buildPolicy()
{
  using NodeTypePtr = std::shared_ptr< DecisionGraph::GraphNodeType >;

  std::queue< std::pair< NodeTypePtr, NewPolicy::GraphNodeTypePtr > > Q;

  // create policy root node from decision graph node
  auto root = decidedGraph_.root();
  NewPolicyNodeData rootData;
  rootData.beliefState = root->data().beliefState;

  auto policyRoot = GraphNode< NewPolicyNodeData >::root( rootData );

  Q.push( std::make_pair( decidedGraph_.root(), policyRoot ) );

  while( ! Q.empty() )
  {
    auto uPair = Q.front();
    Q.pop();

    auto u     = uPair.first;
    auto uCopy = uPair.second;

    for( auto v : u->children() )
    {
      NewPolicyNodeData data;

      data.beliefState = v->data().beliefState;
      data.startTime   = v->depth() / 2;
      data.endTime     = data.startTime + 1;
      data.markovianReturn = -1;
      data.terminal    = v->data().terminal;

      // get komo tag

      auto vCopy = uCopy->makeChild( data );

      Q.push( std::make_pair( v, vCopy ) );
    }
  }

  policy_ = NewPolicy( policyRoot );
}

}
