#include <decision_graph.h>

#include <set>
#include <algorithm>

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

void DecisionGraph::build()
{

}

void DecisionGraph::expand( const GraphNode< NodeData >::ptr & node )
{
  auto bs     = node->data().beliefState;
  auto states = node->data().states;

  for( auto agentId = 0; agentId < engine_.agentNumber(); ++agentId )
  {
    auto actions = getCommonPossibleActions( node, agentId );

    for( auto action : actions )
    {
      auto child = node->makeChild( { states, bs, action, NodeData::NodeType::OBSERVATION } );
      nodes_.push_back( child );

      auto outcomes = getPossibleOutcomes( node, action );

      for( auto outcome : outcomes )
      {
        auto childChild = child->makeChild( { outcome.states, outcome.beliefState, action, NodeData::NodeType::ACTION } );
        nodes_.push_back( childChild );
      }
    }
  }
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

  std::map< std::string, std::vector< std::pair< uint, std::string > > > observableStatesToStates;

  for( auto w = 0; w < bs.size(); ++w )
  {
    if( bs[ w ] > 0 )
    {
      auto startState = states[ w ];
      engine.setState( startState );

      engine.transition( action );

      auto result           = engine.getState();
      auto observableResult = getObservableState( result );

      observableStatesToStates[ observableResult ].push_back( std::make_pair( w, result ) );
    }
  }

  for( auto observableResultPair : observableStatesToStates )
  {
    std::vector< std::string > states;
    std::vector< double > newBs( bs.size(), 0 );

    const auto & worldToOutcomes = observableResultPair.second;

    for( const auto & worldOutcome :  worldToOutcomes )
    {
      auto w = worldOutcome.first;
      auto state = worldOutcome.second;

      states.push_back( state );
      newBs[ w ] = bs[ w ];
    }

    newBs = normalizeBs( newBs );

    outcomes.push_back( { states, newBs } );
  }

  return outcomes;
}

} // namespace matp
