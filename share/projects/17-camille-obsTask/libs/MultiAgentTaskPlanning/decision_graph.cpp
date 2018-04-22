#include <decision_graph.h>

#include <set>

namespace matp
{

DecisionGraph::DecisionGraph( const LogicEngine & engine, const std::vector< std::string > & startStates, const std::vector< double > & egoBeliefState )
  : engine_( engine )
  , root_( GraphNode< NodeData >::root( NodeData( { startStates, egoBeliefState } ) ) )
{
  nodes_.push_back( root_ );
}

void DecisionGraph::build()
{
  auto bs     = root_->data().beliefState;
  auto states = root_->data().states;

  auto node = root_;

  for( auto agentId = 0; agentId < engine_.agentNumber(); ++agentId )
  {
    auto actions = getCommonPossibleActions( node, agentId );
    /// apply actions
    for( auto a : actions )
    {
      for( auto w = 0; w < bs.size(); ++w )
      {
        if( bs[ w ] > 0 )
        {
          auto startState = states[ w ];
          engine_.setState( startState );

          engine_.transition( a );
        }
      }
    }
  }
}

std::set< FOL_World::Handle > DecisionGraph::getCommonPossibleActions( const GraphNode< NodeData >::ptr & node, uint agentId ) const
{
  auto engine = engine_; // copy to be const
  auto bs     = root_->data().beliefState;
  auto states = root_->data().states;

  /// get possible actions
  std::set< FOL_World::Handle > actions;
  for( auto w = 0; w < bs.size(); ++w )
  {
    if( bs[ w ] > 0 )
    {
      auto startState = states[ w ];
      engine.setState( startState );

      auto actions_vec = engine.getPossibleActions( agentId );

      for( auto action : actions_vec )
      {
        actions.insert( action );
      }
    }
  }

  return actions;
}

} // namespace matp
