#pragma once

#include <set>
#include <queue>
#include <vector>
#include <string>
#include <unordered_map>

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

  NodeData()
    : states()
    , beliefState()
    , leadingArtifact()
    , terminal( false )
    , p( 0.0 )
    , agentId( 0 )
    , nodeType( NodeType::ACTION )
  {
    computeHash();
  }

  NodeData( const std::vector< std::string > & states,
            const std::vector< double      > & beliefState,
            const std::string leadingArtifact,
            bool terminal,
            double p,
            uint agentId,
            NodeType nodeType )
    : states( states )
    , beliefState( beliefState )
    , leadingArtifact( leadingArtifact )
    , terminal( terminal )
    , p( p )
    , agentId( agentId )
    , nodeType( nodeType)
  {
    computeHash();
  }

  std::vector< std::string > states;
  std::vector< double      > beliefState;
  std::string leadingArtifact; // leading action of leading observation
  //
  bool terminal;
  double p; // probability to reach this node given the parent
  uint agentId;
  NodeType nodeType;

  std::size_t hash() const
  {
    return hash_;
  }

private:
  void computeHash()  // element depending on the parent are not included into the hash
  {
    hash_ = 0;
    for( const auto s : states )
    {
      hash_+=std::hash<std::string>()(s);
    }
    for( const auto p : beliefState )
    {
      hash_+=std::hash<double>()(p);
    }
    hash_+=std::hash<bool>()(terminal);
    hash_+=agentId;
    hash_+=(int)nodeType;
  }

  std::size_t hash_;
};

bool sameState ( const NodeData & a, const NodeData & b );

std::ostream& operator<<(std::ostream& stream, NodeData const& data);

class DecisionGraph
{
public:
  using GraphNodeDataType = NodeData;
  using GraphNodeType = GraphNode< NodeData >;
public:
  DecisionGraph() = default;

  DecisionGraph( const DecisionGraph & ); // copy ctor
  DecisionGraph& operator= ( const DecisionGraph & ); // assignment operator

  DecisionGraph( const LogicEngine &, const std::vector< std::string > & startStates, const std::vector< double > & egoBeliefState );
  bool empty() const { return nodes_.size() <= 1; } // root node
  std::size_t size() const { return nodes_.size(); }
  void build( int maxSteps, bool graph = false );
  std::queue< GraphNodeType::ptr > expand( const GraphNodeType::ptr & node, bool graph = false );
  GraphNodeType::ptr root() const { return root_; }
  std::vector< std::weak_ptr< GraphNodeType > > nodes() const { return nodes_; }
  std::list< std::weak_ptr< GraphNodeType > > terminalNodes() const { return terminalNodes_; }

  void saveGraphToFile( const std::string & filename ) const;

  // public for testing purpose
  std::vector< std::string > getCommonPossibleActions( const GraphNodeType::ptr & node, uint agentId ) const;
  std::vector< NodeData > getPossibleOutcomes( const GraphNodeType::ptr & node, const std::string & action ) const;

private:
  void copy( const DecisionGraph & );

private:
  mutable LogicEngine engine_;
  GraphNodeType::ptr root_;
  std::vector< std::weak_ptr< GraphNodeType > > nodes_;
  std::unordered_map< std::size_t, uint > hash_to_id_;
  std::list< std::weak_ptr< GraphNodeType > > terminalNodes_;
};
} // namespace matp
