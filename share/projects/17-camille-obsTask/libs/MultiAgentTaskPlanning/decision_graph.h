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
  enum class NodeType
  {
    ACTION = 0, // an action has to be taken at this node
    OBSERVATION
  };

  std::vector< std::string > states;
  std::vector< double      > beliefState;
  std::string leadingArtifact; // leading action of leading observation
  //
  NodeType nodeType;
};

class DecisionGraph
{
public:
  DecisionGraph() = default;

  DecisionGraph( const LogicEngine &, const std::vector< std::string > & startStates, const std::vector< double > & egoBeliefState );
  bool empty() const { return nodes_.size() == 0; }
  std::size_t size() const { return nodes_.size(); }
  void build();
  void expand( const GraphNode< NodeData >::ptr & node );
  GraphNode< NodeData >::ptr root() const { return root_; }

  // public for testing purpose
  std::vector< std::string > getCommonPossibleActions( const GraphNode< NodeData >::ptr & node, uint agentId ) const;
  std::vector< NodeData > getPossibleOutcomes( const GraphNode< NodeData >::ptr & node, const std::string & action ) const;

private:
  LogicEngine engine_;
  GraphNode< NodeData >::ptr root_;
  std::list< GraphNode< NodeData >::ptr > nodes_;
};
} // namespace matp
