#pragma once

#include <set>
#include <queue>
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
  bool terminal;
  double p; // probability to reach this node given the parent
  uint agentId;
  NodeType nodeType;
};

class DecisionGraph
{
public:
  using GraphNodeType = GraphNode< NodeData >;
public:
  DecisionGraph() = default;

  DecisionGraph( const LogicEngine &, const std::vector< std::string > & startStates, const std::vector< double > & egoBeliefState );
  bool empty() const { return nodes_.size() == 0; }
  std::size_t size() const { return nodes_.size(); }
  void build( int maxSteps );
  std::queue< GraphNodeType::ptr > expand( const GraphNodeType::ptr & node );
  GraphNodeType::ptr root() const { return root_; }
  std::list< std::weak_ptr< GraphNodeType > > nodes() const { return nodes_; }
  std::list< std::weak_ptr< GraphNodeType > > terminalNodes() const { return terminalNodes_; }

  void saveGraphToFile( const std::string & filename ) const;

  // public for testing purpose
  std::vector< std::string > getCommonPossibleActions( const GraphNodeType::ptr & node, uint agentId ) const;
  std::vector< NodeData > getPossibleOutcomes( const GraphNodeType::ptr & node, const std::string & action ) const;

private:
  LogicEngine engine_;
  GraphNodeType::ptr root_;
  std::list< std::weak_ptr< GraphNodeType > > nodes_;
  std::list< std::weak_ptr< GraphNodeType > > terminalNodes_;
};
} // namespace matp
