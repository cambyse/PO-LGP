#pragma once

#include <set>
#include <vector>
#include <string>

#include <logic_engine.h>

#include <graph_node.h>

namespace matp
{
struct NodeData
{
  std::vector< std::string > states;
  std::vector< double      > beliefState;
};

class DecisionGraph
{
public:
  DecisionGraph() = default;

  DecisionGraph( const LogicEngine &, const std::vector< std::string > & startStates, const std::vector< double > & egoBeliefState );
  bool empty() const { return nodes_.size() == 0; }
  std::size_t size() const { return nodes_.size(); }
  void build();
  GraphNode< NodeData >::ptr root() const { return root_; }

  // public for testing purpose
  std::set< FOL_World::Handle > getCommonPossibleActions( const GraphNode< NodeData >::ptr & node, uint agentId ) const;
private:
  LogicEngine engine_;
  GraphNode< NodeData >::ptr root_;
  std::list< GraphNode< NodeData >::ptr > nodes_;
};
} // namespace matp
