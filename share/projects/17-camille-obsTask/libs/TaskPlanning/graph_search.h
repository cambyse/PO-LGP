#pragma once

#include <queue>
#include <string>
#include <Logic/fol.h>
#include <Logic/fol_mcts_world.h>

#include <policy.h>
#include <task_planner.h>

#include <po_graph_node.h>
#include <node_visitors.h>

double m_inf();

namespace tp
{

class POGraph
{
public:
  using ptr = std::shared_ptr< POGraph >;

  POGraph( const POGraphNode::ptr & root, const std::list < POGraphNode::ptr > & terminals )
    : root_( root )
    , terminals_( terminals )
    , indexedNodes_( root_->graph().size() )
  {
    for( auto n : root_->graph() )
    {
      indexedNodes_[ n->id() ] = n;
    }
  }

  POGraphNode::ptr root() const
  {
    return root_;
  }

  std::list < POGraphNode::ptr > terminals() const
  {
    return terminals_;
  }

  POGraphNode::ptr getNode( const std::size_t i ) const
  {
    return indexedNodes_[ i ];
  }

  std::size_t size() const { return indexedNodes_.size(); }

private:
  POGraphNode::ptr root_;
  std::list  < POGraphNode::ptr > terminals_;
  std::vector< POGraphNode::ptr > indexedNodes_;
};

class GraphEdgeRewards
{
public:
  using ptr = std::shared_ptr< GraphEdgeRewards >;

public:
  GraphEdgeRewards( const POGraph::ptr & graph )
    : graph_( graph )
    , size_( graph->size() )
    , rewards_( size_ * size_, -1 )
  {

  }

  void reset()
  {
    rewards_ = std::vector< double >( size_ * size_, 0 );
  }

  void removeEdge( const std::size_t parent, const std::size_t child )
  {
    auto i = index( parent, child );

    rewards_[ i ] = m_inf();
  }

  void removeNode( const std::size_t nodeId )
  {
    auto n = graph_->getNode( nodeId );

    for( auto p : n->parents() )
    {
      removeEdge( p->id(), n->id() );
    }
  }

  double reward( const std::size_t parent, const std::size_t child ) const
  {
    auto i = index( parent, child );

    return rewards_[ i ];
  }

private:
  std::size_t index( const std::size_t parent, const std::size_t child ) const
  {
    return size_ * parent + child;
  }

private:
  POGraph::ptr graph_;
  const std::size_t size_;
  std::vector< double > rewards_;
};

class GraphSearchPlanner : public TaskPlanner
{
public:

  // modifiers
  void setFol( const std::string & folDescription ) override;
  void solve() override;
  void integrate( const Policy::ptr & policy ) override;

  // getters
  Policy::ptr getPolicy() const override;
  MotionPlanningOrder getPlanningOrder() const override;
  bool      terminated () const override { return policy_ != nullptr; }

  // utility
  void saveGraphToFile( const std::string & filename );

private:
  void buildGraph();
  void yen( uint k );   // generates a set of policies

  uint n_exp_ = 0;

private:
  void checkIntegrity();

private:
  // state
  mlr::Array< std::shared_ptr<FOL_World> > folEngines_;
  arr bs_;
  POGraph::ptr graph_;

  // dijkstra
  std::vector< double > expectedReward_;
  // policy reconstruction
  std::vector< int >   bestFamily_;     // action to take in this bs and i
  std::vector< POGraphNode::ptr > parents_;
  Policy::ptr policy_;
  std::map< POGraphNode::ptr, PolicyNode::ptr > PO2Policy_;

  // constants
  const mlr::String beliefStateTag_  = "BELIEF_START_STATE";
};

class Dijkstra
{
public:
  Dijkstra( const mlr::Array< std::shared_ptr<FOL_World> > & folEngines );

  Policy::ptr solve( const POGraph::ptr & graph, const POGraphNode::ptr & from, GraphEdgeRewards::ptr = nullptr );

private:
  void dijkstra( const std::list < POGraphNode::ptr > & terminals, GraphEdgeRewards::ptr );
  void extractSolutionFrom( const POGraphNode::ptr & );
  void buildPolicy( const POGraphNode::ptr & );
  void buildPolicyFrom( const POGraphNode::ptr & );

private:
  mlr::Array< std::shared_ptr< FOL_World > > folEngines_;
  std::vector< double > expectedReward_;
  // policy reconstruction
  POGraph::ptr graph_;
  std::vector< int >   bestFamily_;     // action to take in this bs and i
  std::vector< POGraphNode::ptr > parents_;
  Policy::ptr policy_;
  std::map< POGraphNode::ptr, PolicyNode::ptr > PO2Policy_;
  std::map< PolicyNode::ptr, POGraphNode::ptr > Policy2PO_;
};

class Yens
{
public:
  Yens( const mlr::Array< std::shared_ptr<FOL_World> > & folEngines );

  std::list< Policy::ptr > solve( const POGraph::ptr & graph, uint k );

private:
  POGraph::ptr graph_;
  GraphEdgeRewards::ptr mask_;

  mlr::Array< std::shared_ptr<FOL_World> > folEngines_;

  Dijkstra dijkstra_;
};

}
