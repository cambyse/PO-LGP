#include <decision_graph.h>

#include <set>
#include <queue>
#include <algorithm>

#include <decision_graph_printer.h>

namespace matp
{
std::vector < double > normalizeBs( const std::vector < double > & bs )
{
  std::vector < double > newBs = bs;

  double sum = 0;
  for( auto p : bs )
  {
    sum += p;
  }

  for( auto w = 0; w < bs.size(); ++w )
  {
    newBs[ w ] = bs[ w ] / sum;
  }

  return newBs;
}

// DecisionGraph
DecisionGraph::DecisionGraph( const LogicEngine & engine, const std::vector< std::string > & startStates, const std::vector< double > & egoBeliefState )
  : engine_( engine )
  , root_( GraphNode< NodeData >::root( NodeData( { startStates, egoBeliefState } ) ) )
{
  nodes_.push_back( root_ );
}

void DecisionGraph::build( int maxSteps )
{
  std::queue< GraphNode< NodeData >::ptr > queue;
  queue.push( root_ );

  uint step = 0;

  while( ! queue.empty() )
  {
    auto node = queue.front();
    queue.pop();

    step = node->depth() / 2 / engine_.agentNumber();

    if( step < maxSteps )
    {
      auto queueExtension = expand( node );

      while( ! queueExtension.empty() )
      {
        queue.push( std::move( queueExtension.front() ) );
        queueExtension.pop();
      }
    }
    else
    {
      break;
    }
  }
}

std::queue< GraphNode< NodeData >::ptr > DecisionGraph::expand( const GraphNode< NodeData >::ptr & node )
{
  auto bs     = node->data().beliefState;
  auto states = node->data().states;

  std::queue< GraphNode< NodeData >::ptr > nextQueue;

  nextQueue.push( node );

  // for each agent
  for( uint agentId = 0; agentId < engine_.agentNumber(); ++agentId )
  {
    auto queue = nextQueue; // copy
    nextQueue = std::queue< GraphNode< NodeData >::ptr >();

    while( ! queue.empty() )
    {
      auto node = queue.front();
      queue.pop();

      // for each agent
      auto actions = getCommonPossibleActions( node, agentId );

      for( auto action : actions )
      {
        auto child = node->makeChild( { states, bs, action, false, 1.0, agentId, NodeData::NodeType::OBSERVATION } );
        nodes_.push_back( child );

        auto outcomes = getPossibleOutcomes( node, action );

        // for each outcome
        for( auto outcome : outcomes )
        {
          CHECK( node->data().agentId == agentId, "Corruption in the queue!" );
          auto nextAgentId = ( agentId + 1 ) % engine_.agentNumber();
          auto childChild = child->makeChild( { outcome.states, outcome.beliefState, outcome.leadingArtifact, outcome.terminal, outcome.p, nextAgentId, NodeData::NodeType::ACTION } );

          nodes_.push_back( childChild );

          if( outcome.terminal )
          {
            terminalNodes_.push_back( childChild );
          }
          else
          {
            nextQueue.push( childChild );
          }
        }
      }
    }
  }

  return nextQueue;
}

void DecisionGraph::saveGraphToFile( const std::string & filename ) const
{
  if( ! root_ )
  {
    return;
  }

  std::ofstream file;
  file.open( filename );

  GraphPrinter printer( file );
  printer.print( *this );

  file.close();

  // png
  std::string nameCopy( filename );
  const std::string ext( ".gv" );
  std::string newName = nameCopy.replace( nameCopy.find( ext ), ext.length(), ".png" );

  std::stringstream ss;
  ss << "dot"   << " ";
  ss << "-Tpng" << " ";
  ss << "-o"    << " ";
  ss << newName << " ";
  ss << filename;

  system( ss.str().c_str() );
}

std::vector< std::string > DecisionGraph::getCommonPossibleActions( const GraphNode< NodeData >::ptr & node, uint agentId ) const
{
  auto engine = engine_; // copy to be const
  auto bs     = node->data().beliefState;
  auto states = node->data().states;

  /// get possible actions
  std::vector< std::string > possibleActions;
  for( auto w = 0; w < bs.size(); ++w )
  {
    if( bs[ w ] > 0 )
    {
      auto startState = states[ w ];
      engine.setState( startState );

      auto newActions = engine.getPossibleActions( agentId );

      if( possibleActions.empty() )
      {
        possibleActions = newActions;
      }
      else
      {
        std::vector< std::string > newPossibleActions;

        // each action in the new possible actions is has to be previously in the previous possible actions
        std::set_intersection( possibleActions.begin(), possibleActions.end(),
                               newActions.begin(), newActions.end(),
                               std::back_inserter( newPossibleActions ) );

        possibleActions = newPossibleActions;
      }
    }
  }

  return possibleActions;
}

std::vector< NodeData > DecisionGraph::getPossibleOutcomes( const GraphNode< NodeData >::ptr & node, const std::string & action ) const
{
  std::vector< NodeData > outcomes;

  auto engine = engine_; // copy to be const
  auto bs     = node->data().beliefState;
  auto states = node->data().states;

  //        observable facts                                    world state
  std::map< std::set< std::string >, std::vector< std::pair< uint, std::string > > > observableStatesToStates;
  std::map< std::set< std::string >, bool > terminalOutcome;
  std::set< std::string > factIntersection;

  for( auto w = 0; w < bs.size(); ++w )
  {
    if( bs[ w ] > 0 )
    {
      auto startState = states[ w ];
      engine.setState( startState );

      engine.transition( action );

      auto result           = engine.getState();
      auto facts            = getFacts( result );
      auto observableFacts  = getObservableFacts( facts );
      auto terminal         = engine.isTerminal();

      std::set< std::string > newIntersection;

      if( factIntersection.empty() )
      {
        newIntersection = facts;
      }
      else
      {
        std::set_intersection( facts.begin(), facts.end(), factIntersection.begin(), factIntersection.end(),
                               std::inserter( newIntersection, newIntersection.begin() ) );
      }

      // store results
      factIntersection = newIntersection;
      observableStatesToStates[ observableFacts ].push_back( std::make_pair( w, result ) );
      terminalOutcome         [ observableFacts ] = terminal;
    }
  }

  //
  for( auto observableResultPair : observableStatesToStates )
  {
    std::vector< std::string > states( bs.size(), "" );
    std::vector< double > newBs( bs.size(), 0 );

    const auto & worldToOutcomes = observableResultPair.second;
    auto observationFacts = getEmergingFacts( factIntersection, observableResultPair.first );
    auto observation      = concatenateFacts( observationFacts );
    auto terminal         = terminalOutcome[ observableResultPair.first ];

    double p = 0;
    for( const auto & worldOutcome :  worldToOutcomes )
    {
      auto w = worldOutcome.first;
      auto state = worldOutcome.second;

      p += bs[ w ];
      states[ w ] = state;
      newBs[ w ] = bs[ w ];
    }

    newBs = normalizeBs( newBs );

    outcomes.push_back( { states, newBs, observation, terminal, p } );
  }

  return outcomes;
}

} // namespace matp
